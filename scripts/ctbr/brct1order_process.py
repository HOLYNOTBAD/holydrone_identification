#!/usr/bin/env python3
"""从一次 PX4 ULog 提取完整脉冲，按组划分训练/验证并拟合 BRCT 一阶响应。
将 FRD 日志转换为 FLU，按零阶保持对齐指令；输出报告，不修改控制参数。"""
import argparse
import csv
import hashlib
from pathlib import Path
import tempfile
import sys

# 支持 Python -I 隔离模式下导入同目录回归模块。
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import yaml

from brct1order_regression import identify_response


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


def pulse_groups(t, commands, rate_control=None):
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
    separate = rate_control is not None
    for channel in range(4):
        other_axes = [axis for axis in range(3) if axis != channel]
        isolated = np.max(np.abs(commands[:, other_axes]), axis=1) < .005
        signal = commands[:, channel] - (hover if channel == 3 else 0)
        minimum = .003 if channel == 3 else .03
        tolerance = .0005 if channel == 3 else .002
        eligible = isolated & (np.abs(signal) > minimum) & (commands[:, 3] > .05)
        if separate:
            eligible &= np.asarray(rate_control, bool)
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
            if (a[2]*b[2] < 0 and equal and t[b[0]]-t[a[1]] <= (15. if separate else .08) and
                    min(da,db) >= .7*max(da,db) and (separate or t[b[1]]-t[a[0]] >= .4)):
                if separate:
                    # 每个平台独立拟合，不把位置制动过程拼进响应模型。
                    groups.extend([(channel, t[a[0]], t[a[1]]), (channel, t[b[0]], t[b[1]])])
                else:
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
    # Offboard 请求记录比低频 vehicle_control_mode 更准确地标记交接边界。
    offboard_modes = [d.data for d in ulog.data_list if d.name == 'offboard_control_mode' and d.multi_id == 0]
    rate_control = None
    if offboard_modes:
        mode = offboard_modes[0]
        mode_time = field(mode, 'timestamp')*1e-6
        rate_control = previous_sample(mode_time, field(mode, 'body_rate'), rate_t) > .5
        for key in ('position', 'velocity', 'acceleration', 'attitude'):
            rate_control &= previous_sample(mode_time, field(mode, key), rate_t) < .5
    groups = pulse_groups(rate_t, commands, rate_control)
    if not groups or set(channel for channel, _, _ in groups) != set(range(4)):
        raise ValueError('Could not find complete roll/pitch/yaw/thrust pulses in ULog')

    names = ['t', 'cmd_wx', 'cmd_wy', 'cmd_wz', 'cmd_thrust',
             'wx', 'wy', 'wz', 'fz', 'segment', 'channel', 'valid',
             'phase', 'sequence_complete', 'pulse_hold_s']
    rows = []
    for segment, (channel, begin, end) in enumerate(groups):
        left, right = begin-padding, end+padding
        if segment:
            left = max(left, (groups[segment-1][2]+begin)/2)
        if segment+1 < len(groups):
            right = min(right, (end+groups[segment+1][1])/2)
        if rate_control is not None:
            center = np.searchsorted(rate_t, begin)
            before = np.flatnonzero(~rate_control[:center])
            after = np.flatnonzero(~rate_control[center:])
            if len(before): left = max(left, rate_t[before[-1]+1])
            if len(after): right = min(right, rate_t[center+after[0]])
        indices = np.flatnonzero((rate_t >= left) & (rate_t < right))
        if len(indices) < 20:
            continue
        for index in indices:
            rows.append([rate_t[index], *commands[index], *rates[index], fz[index],
                         segment, CHANNELS[channel], 1, 'pulse', 0,
                         (end-begin+float(np.median(np.diff(rate_t[indices]))))/(1 if rate_control is not None else 2)])
    if rows:
        rows[-1][-2] = 1
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(rows)


def split_pulse_groups(source_csv, train_csv, validation_csv, validation_fraction=1/3):
    """按通道/时长留出时间靠后的完整双脉冲；两集合不共享组或采样。"""
    if not 0 < validation_fraction < 1:
        raise ValueError('validation_fraction must be between 0 and 1')
    with source_csv.open(newline='') as stream:
        reader = csv.DictReader(stream)
        names, rows = reader.fieldnames, list(reader)
    groups = {}
    for row in rows:
        groups.setdefault(int(row['segment']), []).append(row)
    strata = {}
    for segment, samples in groups.items():
        channel = samples[0]['channel']
        if any(row['channel'] != channel for row in samples):
            raise ValueError('Mixed channels in pulse group')
        # 以 50 ms 为时长档位，消除日志采样与定时器带来的微小差异。
        duration_bin = int(round(float(samples[0]['pulse_hold_s'])/.05))
        strata.setdefault((channel, duration_bin), []).append(segment)
    if {channel for channel, _ in strata} != set(CHANNELS):
        raise ValueError('Single-log split needs all four channels')
    train_ids, validation_ids = set(), set()
    for (channel, duration_bin), ids in sorted(strata.items()):
        ids.sort(key=lambda segment: float(groups[segment][0]['t']))
        if len(ids) < 2:
            raise ValueError(f'Need at least two complete {channel} groups at '
                             f'{duration_bin*.05:.2f} s; collect repetitions >= 2')
        count = min(len(ids)-1, max(1, int(np.ceil(len(ids)*validation_fraction))))
        train_ids.update(ids[:-count])
        validation_ids.update(ids[-count:])
    train_times = {row['t'] for segment in train_ids for row in groups[segment]}
    validation_times = {row['t'] for segment in validation_ids for row in groups[segment]}
    if train_ids & validation_ids or train_times & validation_times:
        raise ValueError('Training and validation overlap')
    for path, selected in ((train_csv, train_ids), (validation_csv, validation_ids)):
        selected_rows = [dict(row) for row in rows if int(row['segment']) in selected]
        for row in selected_rows:
            row['sequence_complete'] = '0'
        selected_rows[-1]['sequence_complete'] = '1'
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=names)
            writer.writeheader(); writer.writerows(selected_rows)
    def describe(selected):
        return [dict(segment=segment, channel=groups[segment][0]['channel'],
                     pulse_hold_s=float(groups[segment][0]['pulse_hold_s']),
                     start_s=float(groups[segment][0]['t']), end_s=float(groups[segment][-1]['t']),
                     samples=len(groups[segment])) for segment in sorted(selected)]
    return dict(method='chronological_holdout_by_channel_and_duration',
                validation_fraction_requested=validation_fraction,
                scope='Disjoint pulse groups from one flight; not cross-flight validation',
                training_groups=describe(train_ids), validation_groups=describe(validation_ids))


def run_identification(identify, order):
    parser = argparse.ArgumentParser(description=f'BRCT {order}: split one ULog into training and validation pulse groups')
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--ulog', type=Path, help='One .ulg containing repeated complete pulse groups')
    inputs.add_argument('--train', type=Path, help='Optional legacy mode: training .ulg from a separate flight')
    parser.add_argument('--validation', type=Path, help='Validation .ulg for legacy --train mode')
    parser.add_argument('--validation-fraction', type=float, default=1/3,
                        help='Held-out fraction per channel/duration; at least one group (default: 1/3)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if bool(args.train) != bool(args.validation):
        parser.error('Use --ulog for one flight, or --train and --validation together')
    if not 0 < args.validation_fraction < 1:
        parser.error('--validation-fraction must be between 0 and 1')
    sources = [args.ulog] if args.ulog else [args.train, args.validation]
    for path in sources:
        if path.suffix.lower() != '.ulg' or not path.is_file():
            parser.error(str(path)+' must be an existing PX4 .ulg file')
    datasets = [args.output.with_name(args.output.stem+suffix) for suffix in ('_train.csv', '_validation.csv')]
    plot_path = args.output.with_suffix('.png')
    for path in [args.output, *datasets, plot_path]:
        if path.exists():
            parser.error('Output already exists; use a new path: '+str(path))
    with tempfile.TemporaryDirectory() as directory:
        train_csv, validation_csv = (Path(directory)/name for name in ('training.csv','validation.csv'))
        if args.ulog:
            source_csv = Path(directory)/'all.csv'
            ulog_to_csv(args.ulog, source_csv)
            split = split_pulse_groups(source_csv, train_csv, validation_csv, args.validation_fraction)
        else:
            ulog_to_csv(args.train, train_csv)
            ulog_to_csv(args.validation, validation_csv)
            split = dict(method='separate_flights', scope='Cross-flight validation')
        report = identify(train_csv, validation_csv)
        report['data_split'] = split
        report['provenance'] = ('One flight; held-out complete pulse groups' if args.ulog
                                else 'Separate training and validation flights')
        report['source_logs'] = [str(path.resolve()) for path in sources]
        report['source_log_sha256'] = [hashlib.sha256(path.read_bytes()).hexdigest() for path in sources]
        report['datasets'] = dict(training=str(datasets[0].resolve()), validation=str(datasets[1].resolve()))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        for source, target in zip((train_csv, validation_csv), datasets):
            with target.open('xb') as stream:
                stream.write(source.read_bytes())
    with args.output.open('x') as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    from brct1order_plot import plot_response
    plot_response(*datasets, report, plot_path)
    print('Response plot:', plot_path)
    print(f'BRCT {order} report:', args.output, 'passed:', report['passed'])
    if args.ulog:
        print('Pulse groups: training=', len(split['training_groups']),
              'validation=', len(split['validation_groups']))


def main():
    run_identification(identify_response, 'first-order')


if __name__ == '__main__':
    main()
