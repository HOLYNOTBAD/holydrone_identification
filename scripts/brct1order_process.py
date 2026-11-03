#!/usr/bin/env python3
"""从两次独立 PX4 ULog 提取完整脉冲，拟合并验证 BRCT 一阶响应。
将 FRD 日志转换为 FLU，按零阶保持对齐指令；输出报告，不修改控制参数。"""
import argparse
import csv
import hashlib
from pathlib import Path
import tempfile
import sys

# Support the package source tree inside the analysis Conda environment.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np
import yaml

from response import identify_response


CHANNELS = ('roll', 'pitch', 'yaw', 'thrust')


def topic(ulog, name):
    matches = [item for item in ulog.data_list if item.name == name and item.multi_id == 0]
    if not matches:
        raise ValueError('ULog is missing required topic: ' + name)
    return matches[0].data


def field(data, name, index=None):
    candidates = [name] if index is None else [f'{name}[{index}]', f'{name}_{index}']
    for candidate in candidates:
        if candidate in data:
            return np.asarray(data[candidate], float)
    raise ValueError('ULog topic is missing field: ' + candidates[0])


def previous_sample(source_t, values, target_t):
    indices = np.searchsorted(source_t, target_t, side='right') - 1
    return values[np.clip(indices, 0, len(values)-1)]


def pulse_groups(t, commands):
    """识别连续的等幅反向平台，排除回中闭环产生的平滑角速度。

    角速度激励期间允许定高推力变化。推力组仍要求零角速度，并以整段
    悬停推力中位数确定正负方向；过大的基准漂移会保守地丢弃该组。
    """
    t, commands = np.asarray(t), np.asarray(commands)
    airborne = commands[:, 3][commands[:, 3] > .05]
    if not len(airborne):
        return []
    hover = float(np.median(airborne))
    groups = []
    for channel in range(4):
        other_axes = [axis for axis in range(3) if axis != channel]
        isolated = np.max(np.abs(commands[:, other_axes]), axis=1) < .005
        signal = commands[:, channel] - (hover if channel == 3 else 0)
        minimum = .003 if channel == 3 else .03
        tolerance = .0005 if channel == 3 else .002
        eligible = isolated & (np.abs(signal) > minimum) & (commands[:, 3] > .05)
        # 一个平台从首次采样开始计时；不能靠逐点微小变化把回中曲线误认为常值。
        plateaus = []
        start = None
        for index in range(len(t)+1):
            same = (index < len(t) and eligible[index] and
                    (start is None or (t[index]-t[index-1] <= .08 and
                                      abs(signal[index]-signal[start]) <= tolerance)))
            if not same and start is not None:
                if t[index-1]-t[start] >= .12:
                    plateaus.append((start, index-1, float(np.median(signal[start:index]))))
                start = None
            if index < len(t) and eligible[index] and start is None:
                start = index
        index = 0
        while index+1 < len(plateaus):
            a, b = plateaus[index:index+2]
            da, db = t[a[1]]-t[a[0]], t[b[1]]-t[b[0]]
            equal = abs(abs(a[2])-abs(b[2])) <= max(tolerance*2, .3*max(abs(a[2]),abs(b[2])))
            if (a[2]*b[2] < 0 and equal and t[b[0]]-t[a[1]] <= .08 and
                    min(da,db) >= .7*max(da,db) and t[b[1]]-t[a[0]] >= .4):
                groups.append((channel, t[a[0]], t[b[1]]))
                index += 2
            else:
                index += 1
    return sorted(groups, key=lambda group: group[1])


def ulog_to_csv(ulog_path, csv_path, padding=.25):
    """将已解锁 OFFBOARD 的 ULog 片段转换为拟合 CSV，保留采样间隔与脉冲边界。"""
    try:
        from pyulog import ULog
    except ImportError as error:
        raise SystemExit('pyulog is required: python3 -m pip install pyulog') from error
    ulog = ULog(str(ulog_path.expanduser()))
    setpoint = topic(ulog, 'vehicle_rates_setpoint')
    angular = topic(ulog, 'vehicle_angular_velocity')
    sensor = topic(ulog, 'sensor_combined')
    control_mode = topic(ulog, 'vehicle_control_mode')

    command_t = field(setpoint, 'timestamp') * 1e-6
    rate_t = field(angular, 'timestamp_sample') * 1e-6
    accel_t = (field(sensor, 'timestamp') +
               field(sensor, 'accelerometer_timestamp_relative')) * 1e-6
    # ULog 使用 FRD：pitch/yaw 及负 Z 推力取反后形成 FLU [wx, wy, wz, thrust]。
    raw_commands = np.column_stack([
        field(setpoint, 'roll'), -field(setpoint, 'pitch'), -field(setpoint, 'yaw'),
        -field(setpoint, 'thrust_body', 2)])
    commands = previous_sample(command_t, raw_commands, rate_t)
    rates = np.column_stack([field(angular, 'xyz', 0),
                             -field(angular, 'xyz', 1),
                             -field(angular, 'xyz', 2)])
    # FLU Z 比力包含悬停支撑分量，拟合 hover_thrust 时以 9.81 m/s² 为目标。
    fz = np.interp(rate_t, accel_t, -field(sensor, 'accelerometer_m_s2', 2))
    mode_t = field(control_mode, 'timestamp') * 1e-6
    offboard = previous_sample(mode_t,
                               field(control_mode, 'flag_control_offboard_enabled'), rate_t) > .5
    armed = previous_sample(mode_t, field(control_mode, 'flag_armed'), rate_t) > .5

    # 只保留所有源流覆盖且已解锁 OFFBOARD 的样本，不外推缺失飞行阶段。
    valid_time = ((rate_t >= command_t[0]) & (rate_t <= command_t[-1]) &
                  (rate_t >= accel_t[0]) & (rate_t <= accel_t[-1]) & offboard & armed)
    rate_t, commands, rates, fz = (value[valid_time] for value in
                                   (rate_t, commands, rates, fz))
    groups = pulse_groups(rate_t, commands)
    if not groups or set(channel for channel, _, _ in groups) != set(range(4)):
        raise ValueError('Could not find complete roll/pitch/yaw/thrust pulses in ULog')

    names = ['t', 'cmd_wx', 'cmd_wy', 'cmd_wz', 'cmd_thrust',
             'wx', 'wy', 'wz', 'fz', 'segment', 'channel', 'valid',
             'phase', 'sequence_complete']
    rows = []
    for segment, (channel, begin, end) in enumerate(groups):
        left, right = begin-padding, end+padding
        if segment:
            left = max(left, (groups[segment-1][2]+begin)/2)
        if segment+1 < len(groups):
            right = min(right, (end+groups[segment+1][1])/2)
        indices = np.flatnonzero((rate_t >= left) & (rate_t < right))
        if len(indices) < 20:
            continue
        for index in indices:
            rows.append([rate_t[index], *commands[index], *rates[index], fz[index],
                         segment, CHANNELS[channel], 1, 'pulse', 0])
    if rows:
        rows[-1][-1] = 1
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True,
                        help='Training .ulg copied from the PX4 SD card')
    parser.add_argument('--validation', type=Path, required=True,
                        help='Independent validation .ulg copied from the PX4 SD card')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; use a new path')
    for path in (args.train, args.validation):
        if path.suffix.lower() != '.ulg' or not path.is_file():
            parser.error(str(path)+' must be an existing PX4 .ulg file')
    with tempfile.TemporaryDirectory() as directory:
        training_csv = Path(directory) / 'training.csv'
        validation_csv = Path(directory) / 'validation.csv'
        ulog_to_csv(args.train, training_csv)
        ulog_to_csv(args.validation, validation_csv)
        report = identify_response(training_csv, validation_csv)
    report['training_sha256'] = hashlib.sha256(args.train.read_bytes()).hexdigest()
    report['validation_sha256'] = hashlib.sha256(args.validation.read_bytes()).hexdigest()
    report['source_logs'] = [str(args.train.resolve()), str(args.validation.resolve())]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print('BRCT first-order report:', args.output, 'passed:', report['passed'])


if __name__ == '__main__':
    main()
