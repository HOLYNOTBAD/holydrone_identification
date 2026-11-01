#!/usr/bin/env python3
"""Identify aggregate Gazebo physics from two independent PX4 ULogs."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear
from scipy.signal import savgol_filter
import yaml


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


def longest_contiguous(t, arrays):
    typical = float(np.median(np.diff(t)))
    cuts = np.flatnonzero(np.diff(t) > max(.05, 5*typical)) + 1
    blocks = np.split(np.arange(len(t)), cuts)
    selected = max(blocks, key=len)
    return (t[selected]-t[selected][0],) + tuple(value[selected] for value in arrays)


def read_ulog(path):
    try:
        from pyulog import ULog
    except ImportError as error:
        raise SystemExit('pyulog is required: python3 -m pip install pyulog') from error
    ulog = ULog(str(path.expanduser()))
    motors = topic(ulog, 'actuator_motors')
    angular = topic(ulog, 'vehicle_angular_velocity')
    sensor = topic(ulog, 'sensor_combined')
    control_mode = topic(ulog, 'vehicle_control_mode')

    motor_t = field(motors, 'timestamp_sample') * 1e-6
    rate_t = field(angular, 'timestamp_sample') * 1e-6
    accel_t = (field(sensor, 'timestamp') +
               field(sensor, 'accelerometer_timestamp_relative')) * 1e-6
    mode_t = field(control_mode, 'timestamp') * 1e-6
    raw_commands = np.column_stack([field(motors, 'control', index) for index in range(4)])
    commands = previous_sample(motor_t, raw_commands, rate_t)
    rates = np.column_stack([field(angular, 'xyz', 0),
                             -field(angular, 'xyz', 1),
                             -field(angular, 'xyz', 2)])
    acceleration = np.column_stack([
        np.interp(rate_t, accel_t, field(sensor, 'accelerometer_m_s2', 0)),
        -np.interp(rate_t, accel_t, field(sensor, 'accelerometer_m_s2', 1)),
        -np.interp(rate_t, accel_t, field(sensor, 'accelerometer_m_s2', 2))])
    offboard = previous_sample(mode_t,
                               field(control_mode, 'flag_control_offboard_enabled'), rate_t) > .5
    armed = previous_sample(mode_t, field(control_mode, 'flag_armed'), rate_t) > .5
    valid = ((rate_t >= motor_t[0]) & (rate_t <= motor_t[-1]) &
             (rate_t >= accel_t[0]) & (rate_t <= accel_t[-1]) & offboard & armed &
             np.isfinite(np.column_stack((commands, rates, acceleration))).all(axis=1) &
             (commands >= 0.).all(axis=1) & (commands <= 1.).all(axis=1))
    if valid.sum() < 400:
        raise ValueError('Need at least 400 armed OFFBOARD samples with four valid motor commands')
    t, commands, rates, acceleration = longest_contiguous(
        rate_t[valid], (commands[valid], rates[valid], acceleration[valid]))
    if len(t) < 400:
        raise ValueError('Need one contiguous identification interval of at least 400 samples')
    return dict(t=t, commands=commands, rates=rates, acceleration=acceleration)


def motor_state(t, commands, tau):
    state = np.empty_like(commands)
    state[0] = commands[0]
    for index in range(1, len(t)):
        alpha = np.exp(-(t[index]-t[index-1])/tau)
        state[index] = alpha*state[index-1] + (1-alpha)*commands[index-1]
    return state


def rotor_speed(state, scaling, zero, maximum):
    return np.clip(zero + scaling*state, 0., maximum)


def fit_propulsion(data, mass, tau_min, tau_max, scaling, zero, maximum):
    target = mass * data['acceleration'][:, 2]
    candidates = np.geomspace(tau_min, tau_max, 80)
    fits = []
    for tau in candidates:
        state = motor_state(data['t'], data['commands'], tau)
        speed = rotor_speed(state, scaling, zero, maximum)
        regressor = np.sum(speed**2, axis=1)
        coefficient = float(regressor @ target / (regressor @ regressor))
        error = coefficient*regressor-target
        fits.append((float(np.sqrt(np.mean(error**2))), tau, coefficient, state))
    rmse, tau, coefficient, state = min(fits, key=lambda row: row[0])
    if coefficient <= 0 or np.ptp(np.sum(state**2, axis=1)) < .02:
        raise ValueError('Collective excitation is insufficient for a positive thrust curve')
    boundary = tau in (candidates[0], candidates[-1])
    return dict(tau=float(tau), coefficient=coefficient, state=state,
                rmse=rmse, at_boundary=boundary)


def propulsion_score(data, tau, coefficient, mass, scaling, zero, maximum):
    state = motor_state(data['t'], data['commands'], tau)
    observed = mass*data['acceleration'][:, 2]
    speed = rotor_speed(state, scaling, zero, maximum)
    predicted = coefficient*np.sum(speed**2, axis=1)
    return state, float(np.sqrt(np.mean((predicted-observed)**2)))


def rotational_system(data, state, motor_constant, config):
    positions = np.asarray(config['rotor_positions_flu_m'], float)
    signs = np.asarray(config['yaw_torque_signs'], float)
    if positions.shape != (4, 3) or signs.shape != (4,) or not np.all(np.isin(signs, [-1, 1])):
        raise ValueError('Need four FLU rotor positions and four body yaw-torque signs')
    speed = rotor_speed(state, float(config['gazebo_input_scaling_rad_s']),
                        float(config.get('gazebo_zero_position_armed_rad_s', 0.)),
                        float(config['gazebo_max_rot_velocity_rad_s']))
    forces = motor_constant*speed**2
    torque_x = forces @ positions[:, 1]
    torque_y = -forces @ positions[:, 0]
    yaw_basis = forces @ signs

    t, rates = data['t'], data['rates']
    dt = float(np.median(np.diff(t)))
    window = min(21, (len(t)//2)*2-1)
    if window < 7:
        raise ValueError('Not enough angular-rate samples')
    smooth = savgol_filter(rates, window, 3, axis=0)
    derivative = savgol_filter(rates, window, 3, deriv=1, delta=dt, axis=0)
    keep = slice(window//2, -window//2)
    smooth, derivative = smooth[keep], derivative[keep]
    torque_x, torque_y, yaw_basis = torque_x[keep], torque_y[keep], yaw_basis[keep]

    x, y, z = smooth.T
    dx, dy, dz = derivative.T
    count = len(smooth)
    matrix = np.zeros((count, 3, 7))
    target = np.zeros((count, 3))
    # Parameters: Ixx, Iyy, Izz, momentConstant, Dx, Dy, Dz.
    matrix[:, 0, 0] = dx
    matrix[:, 0, 1] = -y*z
    matrix[:, 0, 2] = y*z
    matrix[:, 0, 4] = x
    matrix[:, 1, 0] = x*z
    matrix[:, 1, 1] = dy
    matrix[:, 1, 2] = -x*z
    matrix[:, 1, 5] = y
    matrix[:, 2, 0] = -x*y
    matrix[:, 2, 1] = x*y
    matrix[:, 2, 2] = dz
    matrix[:, 2, 3] = -yaw_basis
    matrix[:, 2, 6] = z
    target[:, 0], target[:, 1] = torque_x, torque_y
    return matrix.reshape(-1, 7), target.reshape(-1)


def fit_rotation(matrix, target):
    scale = np.linalg.norm(matrix, axis=0)
    if np.any(scale < 1e-10):
        raise ValueError('Rotational excitation leaves an unobservable parameter')
    normalized = matrix/scale
    condition = float(np.linalg.cond(normalized))
    if np.linalg.matrix_rank(normalized) < 7 or condition > 1e5:
        raise ValueError('Inertia, yaw torque and damping are not independently observable')
    result = lsq_linear(normalized, target, bounds=(0., np.inf), lsmr_tol='auto')
    if not result.success:
        raise ValueError('Bounded rotational fit did not converge')
    values = result.x/scale
    inertia = values[:3]
    if np.any(inertia <= 0) or np.any(2*inertia > inertia.sum()):
        raise ValueError('Identified inertia violates rigid-body triangle inequalities')
    return values, condition


def rotation_score(matrix, target, values):
    residual = (matrix@values-target).reshape(-1, 3)
    return np.sqrt(np.mean((residual/values[:3])**2, axis=0))


def identify(training, validation, config):
    mass = float(config.get('mass_kg', 0.))
    scaling = float(config.get('gazebo_input_scaling_rad_s', 0.))
    zero = float(config.get('gazebo_zero_position_armed_rad_s', 0.))
    maximum = float(config.get('gazebo_max_rot_velocity_rad_s', 0.))
    tau_min = float(config.get('motor_tau_min_s', .003))
    tau_max = float(config.get('motor_tau_max_s', .15))
    if mass <= 0 or scaling <= 0 or maximum <= 0 or zero < 0 or not 0 < tau_min < tau_max:
        raise ValueError('Set positive mass, Gazebo motor mapping and motor tau bounds')
    propulsion = fit_propulsion(training, mass, tau_min, tau_max,
                                scaling, zero, maximum)
    validation_state, validation_force_rmse = propulsion_score(
        validation, propulsion['tau'], propulsion['coefficient'], mass,
        scaling, zero, maximum)
    train_matrix, train_target = rotational_system(
        training, propulsion['state'], propulsion['coefficient'], config)
    values, condition = fit_rotation(train_matrix, train_target)
    validation_matrix, validation_target = rotational_system(
        validation, validation_state, propulsion['coefficient'], config)
    angular_rmse = rotation_score(validation_matrix, validation_target, values)
    force_limit = mass*9.81*.12
    angular_limit = float(config.get('max_angular_accel_rmse_rad_s2', 3.0))
    passed = (not propulsion['at_boundary'] and validation_force_rmse < force_limit and
              np.all(angular_rmse < angular_limit))
    return dict(
        passed=bool(passed),
        method='Grey-box fit inspired by Eschmann, Albani and Loianno (2024)',
        assumptions=['known mass and rotor geometry', 'vertical identical rotors',
                     'diagonal aggregate inertia', 'common quadratic thrust curve',
                     'common first-order motor delay', 'low-speed flight'],
        measured=dict(mass_kg=mass,
                      rotor_positions_flu_m=config['rotor_positions_flu_m'],
                      yaw_torque_signs=config['yaw_torque_signs']),
        propulsion=dict(
            common_motor_time_constant_s=propulsion['tau'],
            gazebo_time_constant_up_s=propulsion['tau'],
            gazebo_time_constant_down_s=propulsion['tau'],
            at_search_boundary=propulsion['at_boundary'],
            gazebo_input_scaling_rad_s=scaling,
            gazebo_zero_position_armed_rad_s=zero,
            gazebo_max_rot_velocity_rad_s=maximum,
            gazebo_motor_constant_n_per_rad_s_squared=propulsion['coefficient']),
        rotation=dict(
            aggregate_inertia_diagonal_kg_m2=values[:3].tolist(),
            gazebo_moment_constant_m=float(values[3]),
            angular_damping_nm_per_rad_s=values[4:].tolist(),
            condition_number=condition),
        validation=dict(
            force_rmse_n=validation_force_rmse,
            force_rmse_limit_n=force_limit,
            angular_acceleration_rmse_rad_s2=angular_rmse.tolist(),
            angular_acceleration_rmse_limit_rad_s2=angular_limit),
        limitations=[
            'No RPM is measured; command-to-speed scaling must be supplied for Gazebo conversion',
            'A single delay is used for motor rise and fall',
            'Aggregate inertia is not a drop-in base_link inertia when other SDF links have mass'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; use a new path')
    for path in (args.train, args.validation):
        if path.suffix.lower() != '.ulg' or not path.is_file():
            parser.error(str(path) + ' must be an existing PX4 .ulg file')
    if args.train.read_bytes() == args.validation.read_bytes():
        parser.error('Training and validation must be independent ULogs')
    with args.config.open() as stream:
        config = yaml.safe_load(stream) or {}
    report = identify(read_ulog(args.train), read_ulog(args.validation), config)
    report['training_sha256'] = hashlib.sha256(args.train.read_bytes()).hexdigest()
    report['validation_sha256'] = hashlib.sha256(args.validation.read_bytes()).hexdigest()
    report['source_logs'] = [str(args.train.resolve()), str(args.validation.resolve())]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print('Gazebo physics report:', args.output, 'passed:', report['passed'])


if __name__ == '__main__':
    main()
