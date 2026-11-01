#!/usr/bin/env python3
"""Fit and validate the BRCT first-order model from two PX4 SD-card ULogs."""
import argparse
import csv
import hashlib
from pathlib import Path
import tempfile

import numpy as np
import yaml

from holydrone_identification.response import identify_response


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
    airborne_thrust = commands[:, 3][commands[:, 3] > .05]
    if not len(airborne_thrust):
        return []
    thrust_hover = float(np.median(airborne_thrust))
    rate_size = np.abs(commands[:, :3])
    dominant_rate = rate_size.argmax(axis=1)
    ordered_rates = np.sort(rate_size, axis=1)
    rate_pulse = ((ordered_rates[:, 2] > .03) & (ordered_rates[:, 1] < .005) &
                  (np.abs(commands[:, 3]-thrust_hover) < .006))
    thrust_pulse = ((rate_size.max(axis=1) < .005) &
                    (np.abs(commands[:, 3]-thrust_hover) > .003) &
                    (commands[:, 3] > .05))
    active = rate_pulse | thrust_pulse
    selected = np.where(thrust_pulse, 3, dominant_rate)
    groups = []
    start = None
    channel = None
    for index in range(len(t)):
        if active[index] and (start is None or selected[index] == channel):
            if start is None:
                start, channel = index, int(selected[index])
        else:
            if start is not None:
                groups.append((channel, t[start], t[index-1]))
                start = None
            if active[index]:
                start, channel = index, int(selected[index])
    if start is not None:
        groups.append((channel, t[start], t[-1]))
    complete = []
    for channel, begin, end in groups:
        values = commands[(t >= begin) & (t <= end), channel]
        minimum_span = .008 if channel == 3 else .08
        if end-begin >= .4 and np.ptp(values) >= minimum_span:
            complete.append((channel, begin, end))
    return complete


def ulog_to_csv(ulog_path, csv_path, padding=.25):
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
    raw_commands = np.column_stack([
        field(setpoint, 'roll'), -field(setpoint, 'pitch'), -field(setpoint, 'yaw'),
        -field(setpoint, 'thrust_body', 2)])
    commands = previous_sample(command_t, raw_commands, rate_t)
    rates = np.column_stack([field(angular, 'xyz', 0),
                             -field(angular, 'xyz', 1),
                             -field(angular, 'xyz', 2)])
    fz = np.interp(rate_t, accel_t, -field(sensor, 'accelerometer_m_s2', 2))
    mode_t = field(control_mode, 'timestamp') * 1e-6
    offboard = previous_sample(mode_t,
                               field(control_mode, 'flag_control_offboard_enabled'), rate_t) > .5
    armed = previous_sample(mode_t, field(control_mode, 'flag_armed'), rate_t) > .5

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
        indices = np.flatnonzero((rate_t >= left) & (rate_t <= right))
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
