#!/usr/bin/env python3
"""从独立训练与验证 ULog 辨识聚合 Gazebo 物理参数。
已知质量和 FLU 旋翼几何，估计共同电机时常、二次推力系数、对角惯量与阻尼。
仅输出报告，不自动覆盖 SDF；电机转速映射必须由配置提供。"""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear, least_squares
from scipy.integrate import cumulative_trapezoid
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
    """按大于 50 ms 或五倍典型间隔的断点分段，仅保留最长连续采集。"""
    typical = float(np.median(np.diff(t)))
    cuts = np.flatnonzero(np.diff(t) > max(.05, 5*typical)) + 1
    blocks = np.split(np.arange(len(t)), cuts)
    selected = max(blocks, key=len)
    return (t[selected]-t[selected][0],) + tuple(value[selected] for value in arrays)


def read_ulog(path, rpm_csv=None, ros_minus_px4_s=None, esc_indices=None):
    """对齐电机指令、FLU 角速度和比力，筛选至少 400 个连续已解锁 OFFBOARD 样本。"""
    try:
        from pyulog import ULog
    except ImportError as error:
        raise SystemExit('pyulog is required: python3 -m pip install pyulog') from error
    ulog = ULog(str(path.expanduser()))
    motors = topic(ulog, 'actuator_motors')
    angular = topic(ulog, 'vehicle_angular_velocity')
    sensor = topic(ulog, 'sensor_combined')
    control_mode = topic(ulog, 'vehicle_control_mode')

    # Use publication time: timestamp_sample is the upstream IMU sample, not actuation.
    motor_t = field(motors, 'timestamp') * 1e-6
    measured = rpm_csv is not None or esc_indices is not None
    if not measured and np.median(np.diff(motor_t)) > .02:
        raise ValueError('Motor logging is too slow; enable SDLOG_PROFILE high-rate bit before logger starts')
    rate_t = field(angular, 'timestamp_sample') * 1e-6
    accel_t = (field(sensor, 'timestamp') +
               field(sensor, 'accelerometer_timestamp_relative')) * 1e-6
    mode_t = field(control_mode, 'timestamp') * 1e-6
    raw_commands = np.column_stack([field(motors, 'control', index) for index in range(4)])
    factor = None
    speed_commands = raw_commands
    if not measured:
        # PX4 FunctionMotors applies THR_MDL_FAC before PWMSim/Gazebo.
        # Only support the verified four non-reversible, directly ordered SITL outputs.
        params = ulog.initial_parameters
        for i in range(4):
            if params.get(f'PWM_MAIN_FUNC{i+1}') != 101+i:
                raise ValueError('Expected direct PWM_MAIN_FUNC1..4 motor mapping')
        if np.any(field(motors, 'reversible_flags') != 0):
            raise ValueError('Reversible motors are unsupported')
        factor = float(params['THR_MDL_FAC'])
        if not 0 <= factor <= 1:
            raise ValueError('Invalid THR_MDL_FAC')
        if any(name == 'THR_MDL_FAC' for _, name, _ in ulog.changed_parameters):
            raise ValueError('THR_MDL_FAC changed during recording')
        speed_commands = thrust_to_speed_command(raw_commands, factor)
        speed_commands[(raw_commands < 0) | (raw_commands > 1)] = np.nan
    commands = previous_sample(motor_t, speed_commands, rate_t)
    # PX4 FRD 机体系转换为 FLU：X 不变，Y/Z 取反。
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
    t, commands, rates, acceleration, absolute_t = longest_contiguous(
        rate_t[valid], (commands[valid], rates[valid], acceleration[valid], rate_t[valid]))
    if len(t) < 400:
        raise ValueError('Need one contiguous identification interval of at least 400 samples')
    result = dict(t=t, commands=commands, rates=rates, acceleration=acceleration,
                  input_mapping=dict(source='actuator_motors + PX4 inverse thrust curve',
                                     thrust_model_factor=factor, motor_median_dt_s=float(np.median(np.diff(motor_t)))))
    if rpm_csv is not None:
        if ros_minus_px4_s is None or not np.isfinite(ros_minus_px4_s):
            raise ValueError('Explicit --rpm-ros-minus-px4-s required; never infer clock alignment from fit error')
        rows = np.atleast_1d(np.genfromtxt(rpm_csv, delimiter=',', names=True))
        source_t = rows['t_ros_s'] - ros_minus_px4_s
        speed = np.column_stack([rows[f'omega_{i}_rad_s'] for i in range(4)])
        result['rotor_speed'] = align_rotor_speed(source_t, speed, absolute_t)
        result['input_mapping'] = dict(source='measured mechanical rad/s CSV',
            csv=str(Path(rpm_csv).resolve()), sha256=hashlib.sha256(Path(rpm_csv).read_bytes()).hexdigest(),
            ros_minus_px4_s=ros_minus_px4_s, median_dt_s=float(np.median(np.diff(source_t))))
    elif esc_indices is not None:
        if len(esc_indices) != 4 or len(set(esc_indices)) != 4 or min(esc_indices)<0 or max(esc_indices)>7:
            raise ValueError('Supply four distinct ESC indices in rotor geometry order')
        esc = topic(ulog, 'esc_status')
        channels = []
        for index in esc_indices:
            stamp = field(esc, f'esc[{index}].timestamp')*1e-6
            rpm = field(esc, f'esc[{index}].esc_rpm')
            # ESC status repeats the other ESCs' last samples: use each ESC timestamp.
            take = np.r_[True, np.diff(stamp)!=0] & (stamp>0)
            channels.append(align_rotor_speed(stamp[take], (rpm[take]*2*np.pi/60)[:,None], absolute_t)[:,0])
        result['rotor_speed'] = np.column_stack(channels)
        result['input_mapping'] = dict(source='PX4 esc_status mechanical RPM', esc_indices=esc_indices,
            motor_pole_count=ulog.initial_parameters.get('MOT_POLE_COUNT'))
    return result


def align_rotor_speed(source_t, speed, target_t, max_gap=.02):
    """Interpolate measured speed only inside contiguous, finite, monotonic telemetry."""
    source_t, speed, target_t = np.asarray(source_t), np.asarray(speed), np.asarray(target_t)
    if len(source_t)<2 or np.any(np.diff(source_t)<=0) or not np.isfinite(source_t).all():
        raise ValueError('RPM timestamps must be finite and strictly increasing')
    if not np.isfinite(speed).all() or np.any(speed<0):
        raise ValueError('Invalid/reversed mechanical rotor speed')
    if target_t[0]<source_t[0] or target_t[-1]>source_t[-1]:
        raise ValueError('RPM telemetry does not cover the selected Offboard interval')
    right = np.clip(np.searchsorted(source_t, target_t, side='right'), 1, len(source_t)-1)
    if np.any(source_t[right]-source_t[right-1]>max_gap):
        raise ValueError('RPM telemetry gap exceeds 20 ms; do not interpolate missing dynamics')
    result = np.column_stack([np.interp(target_t,source_t,channel) for channel in speed.T])
    if np.any(result<=0):
        raise ValueError('Stopped motor in selected armed Offboard interval')
    return result


def thrust_to_speed_command(control, factor):
    """PX4 FunctionMotors inverse thrust curve, followed by 1000..2000 PWM quantization."""
    if factor > 1e-7:
        b = (1-factor)/(2*factor)
        value = -b + np.sqrt(b*b + np.maximum(control, 0)/factor)
    else:
        value = control.copy()
    return np.floor(np.clip(value, 0, 1)*1000 + .5)/1000


def motor_state(t, commands, tau):
    """在不等间隔采样上对上一时刻电机指令作一阶零阶保持响应，tau 单位 s。"""
    state = np.empty_like(commands)
    state[0] = commands[0]
    for index in range(1, len(t)):
        alpha = np.exp(-(t[index]-t[index-1])/tau)
        state[index] = alpha*state[index-1] + (1-alpha)*commands[index-1]
    return state


def rotor_speed(state, scaling, zero, maximum):
    return np.clip(zero + scaling*state, 0., maximum)


def fit_propulsion(data, mass, tau_min, tau_max, scaling, zero, maximum):
    """网格搜索共同电机时常并拟合 F=k·Σω²，返回最小推力 RMSE 解和边界标志。"""
    # 低速、竖直同构旋翼假设下，FLU Z 比力乘质量给出总推力目标。
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
    """构造 FLU 欧拉刚体方程及线性阻尼的七参数回归矩阵。

    Returns:
        惯量 Ixx/Iyy/Izz、偏航力矩系数和三轴阻尼对应的矩阵与力矩目标。"""
    positions = np.asarray(config['rotor_positions_flu_m'], float)
    signs = np.asarray(config['yaw_torque_signs'], float)
    if positions.shape != (4, 3) or signs.shape != (4,) or not np.all(np.isin(signs, [-1, 1])):
        raise ValueError('Need four FLU rotor positions and four body yaw-torque signs')
    speed = data.get('rotor_speed')
    if speed is None:
        speed = rotor_speed(state, float(config['gazebo_input_scaling_rad_s']),
                            float(config.get('gazebo_zero_position_armed_rad_s', 0.)),
                            float(config['gazebo_max_rot_velocity_rad_s']))
    forces = motor_constant*speed**2
    torque_x = forces @ positions[:, 1]
    torque_y = -forces @ positions[:, 0]
    yaw_basis = forces @ signs

    t, rates = data['t'], data['rates']
    # Integrate both sides over the same ~80 ms window. This avoids applying
    # a derivative smoother to gyro data while leaving torque unfiltered.
    half = max(2, int(round(.04 / np.median(np.diff(t)))))
    if len(t) <= 2*half:
        raise ValueError('Not enough angular-rate samples')
    span = (t[2*half:]-t[:-2*half])[:, None]
    def average(values):
        integral = cumulative_trapezoid(values, t, axis=0, initial=0)
        result = integral[2*half:]-integral[:-2*half]
        return result/span if result.ndim == 2 else result/span[:, 0]
    smooth = average(rates)
    derivative = (rates[2*half:]-rates[:-2*half])/span
    torque_x, torque_y, yaw_basis = map(average, (torque_x, torque_y, yaw_basis))
    x, y, z = smooth.T
    dx, dy, dz = derivative.T
    xy = average(rates[:, 0]*rates[:, 1])
    xz = average(rates[:, 0]*rates[:, 2])
    yz = average(rates[:, 1]*rates[:, 2])
    count = len(smooth)
    matrix = np.zeros((count, 3, 7))
    target = np.zeros((count, 3))
    # 欧拉方程 τ=I·ω̇+ω×(Iω)+Dω；列为 Ixx、Iyy、Izz、偏航系数、Dx、Dy、Dz。
    matrix[:, 0, 0] = dx
    matrix[:, 0, 1] = -yz
    matrix[:, 0, 2] = yz
    matrix[:, 0, 4] = x
    matrix[:, 1, 0] = xz
    matrix[:, 1, 1] = dy
    matrix[:, 1, 2] = -xz
    matrix[:, 1, 5] = y
    matrix[:, 2, 0] = -xy
    matrix[:, 2, 1] = xy
    matrix[:, 2, 2] = dz
    matrix[:, 2, 3] = -yaw_basis
    matrix[:, 2, 6] = z
    target[:, 0], target[:, 1] = torque_x, torque_y
    return matrix.reshape(-1, 7), target.reshape(-1)


def fit_rotation(matrix, target, strict=True):
    """列归一化后做非负最小二乘，拒绝秩亏、病态系统及不满足刚体三角不等式的惯量。"""
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
    # 撤销列缩放；正惯量还必须满足每一轴不大于另外两轴之和。
    values = result.x/scale
    inertia = values[:3]
    if strict and (np.any(inertia <= 0) or np.any(2*inertia > inertia.sum())):
        raise ValueError('Identified inertia violates rigid-body triangle inequalities')
    return values, condition


def validate_inertia_prior(config):
    """Validate measured inertia and its uncertainty before loading flight data."""
    mean = config.get('inertia_prior_kg_m2')
    sigma = config.get('inertia_prior_std_kg_m2')
    if (mean is None) != (sigma is None):
        raise ValueError('Inertia prior requires both measured mean and standard deviation')
    raw_axes = config.get('inertia_prior_axes',[0,1,2])
    if not isinstance(raw_axes,list) or any(type(axis) is not int for axis in raw_axes):
        raise ValueError('Inertia prior axes must be integer indices 0, 1, 2')
    axes = np.asarray(raw_axes,int)
    if mean is not None:
        mean, sigma = np.asarray(mean,float), np.asarray(sigma,float)
        if (axes.ndim!=1 or len(axes)==0 or len(set(axes.tolist()))!=len(axes)
                or np.any(axes<0) or np.any(axes>2) or mean.shape!=axes.shape or sigma.shape!=axes.shape
                or not np.isfinite([mean,sigma]).all() or np.any(mean<=0) or np.any(sigma<=0)
                or (len(axes)==3 and np.any(2*mean>=mean.sum())) or not isinstance(config.get('inertia_prior_source'),str)
                or not config['inertia_prior_source'].strip()):
            raise ValueError('Invalid inertia prior; supply axis indices, physical values, positive uncertainties and source')
    return mean, sigma, axes


def load_parameter_priors(config, path):
    """An explicitly selected prior file replaces any embedded inertia prior."""
    content = path.read_bytes()
    prior = yaml.safe_load(content)
    keys = {'inertia_prior_axes', 'inertia_prior_kg_m2', 'inertia_prior_std_kg_m2', 'inertia_prior_source'}
    if not isinstance(prior,dict) or type(prior.get('enabled')) is not bool:
        raise ValueError('Prior file requires enabled: true or false')
    if set(prior)-keys-{'enabled'}:
        raise ValueError('Unknown parameter prior keys: '+str(sorted(set(prior)-keys-{'enabled'})))
    merged = {key:value for key,value in config.items() if key not in keys}
    if prior['enabled']:
        if prior.get('inertia_prior_kg_m2') is None or prior.get('inertia_prior_std_kg_m2') is None:
            raise ValueError('Enabled prior requires measured inertia and standard deviation')
        merged.update({key:prior[key] for key in keys if key in prior})
        validate_inertia_prior(merged)
        if merged.get('rotation_fit_method') != 'angular_acceleration':
            raise ValueError('Enabled prior requires rotation_fit_method: angular_acceleration')
    return merged, dict(path=str(path.resolve()),sha256=hashlib.sha256(content).hexdigest(),enabled=prior['enabled'])


def fit_angular_acceleration(matrix, target, config):
    """Fit acceleration residuals, not homogeneous torque residuals.

    Positive second moments enforce all inertia triangle inequalities. Optional
    independent inertia measurements enter as a Gaussian prior with explicit sigma.
    """
    mean, sigma, axes = validate_inertia_prior(config)
    def decode(log_values):
        v = np.exp(log_values)
        j = v[:3]
        v[:3] = [j[1]+j[2],j[0]+j[2],j[0]+j[1]]
        return v
    noise = float(config.get('angular_accel_noise_rad_s2',1.0))
    if not np.isfinite(noise) or noise<=0:
        raise ValueError('angular_accel_noise_rad_s2 must be positive')
    def residual(log_values):
        v = decode(log_values)
        residual = ((matrix@v-target).reshape(-1,3)/v[:3]/noise).ravel()
        if mean is not None:
            residual = np.r_[residual,(v[:3][axes]-mean)/sigma]
        return residual
    lower = np.log([1e-6,1e-6,1e-6,1e-5,1e-8,1e-8,1e-8])
    upper = np.log([.1,.1,.1,1.,1.,1.,1.])
    fits = []
    for scale in (.5,1.,2.):
        inertia = np.array([.005,.006,.008])*scale
        if mean is not None:
            inertia[axes] = mean
        j = np.maximum(inertia.sum()/2-inertia,1e-5)
        initial = np.log(np.r_[j,.05,.001,.001,.001])
        fit = least_squares(residual,np.clip(initial,lower+1e-5,upper-1e-5),
                            bounds=(lower,upper),max_nfev=400)
        fits.append(fit)
    fit = min(fits,key=lambda x:np.sum(x.fun**2))
    if not fit.success:
        raise ValueError('Angular acceleration fit did not converge')
    v = decode(fit.x)
    # Damping may legitimately approach zero; only core inertia/moment bounds invalidate a fit.
    boundary = bool(np.any((fit.x[:4]-lower[:4]<.01)|(upper[:4]-fit.x[:4]<.01)))
    condition = float(np.linalg.cond(fit.jac[:,:4]))
    return v, condition, dict(method='angular_acceleration', core_parameter_at_boundary=boundary,
        inertia_prior_used=mean is not None, inertia_prior_axes=axes.tolist() if mean is not None else [], inertia_prior_source=config.get('inertia_prior_source'),
        inertia_prior_kg_m2=None if mean is None else mean.tolist(),
        inertia_prior_std_kg_m2=None if sigma is None else sigma.tolist(),
        note='Physical constraints and low residuals do not prove unique or accurate parameters')


def rotation_score(matrix, target, values):
    residual = (matrix@values-target).reshape(-1, 3)
    return np.sqrt(np.mean((residual/values[:3])**2, axis=0))


def identify(training, validation, config):
    """先拟合共同推进模型再拟合转动参数；仅用独立验证数据判断误差阈值。"""
    mass = float(config.get('mass_kg', 0.))
    scaling = float(config.get('gazebo_input_scaling_rad_s', 0.))
    zero = float(config.get('gazebo_zero_position_armed_rad_s', 0.))
    maximum = float(config.get('gazebo_max_rot_velocity_rad_s', 0.))
    tau_min = float(config.get('motor_tau_min_s', .003))
    tau_max = float(config.get('motor_tau_max_s', .15))
    if mass <= 0 or scaling <= 0 or maximum <= 0 or zero < 0 or not 0 < tau_min < tau_max:
        raise ValueError('Set positive mass, Gazebo motor mapping and motor tau bounds')
    measured = 'rotor_speed' in training
    if measured != ('rotor_speed' in validation):
        raise ValueError('Training and validation must use the same rotor input source')
    if measured:
        regressor = np.sum(training['rotor_speed']**2, axis=1)
        target = mass*training['acceleration'][:,2]
        coefficient = float(regressor@target/(regressor@regressor))
        if coefficient<=0 or np.ptp(regressor)/np.mean(regressor)<.01:
            raise ValueError('Insufficient measured collective excitation')
        propulsion = dict(tau=None, coefficient=coefficient, state=None, at_boundary=False)
        validation_state = None
        predicted = coefficient*np.sum(validation['rotor_speed']**2,axis=1)
        validation_force_rmse = float(np.sqrt(np.mean((predicted-mass*validation['acceleration'][:,2])**2)))
    else:
        propulsion = fit_propulsion(training, mass, tau_min, tau_max, scaling, zero, maximum)
        validation_state, validation_force_rmse = propulsion_score(
            validation, propulsion['tau'], propulsion['coefficient'], mass, scaling, zero, maximum)
    train_matrix, train_target = rotational_system(
        training, propulsion['state'], propulsion['coefficient'], config)
    method = config.get('rotation_fit_method','torque_residual')
    if method == 'angular_acceleration':
        values, condition, fit_info = fit_angular_acceleration(train_matrix, train_target, config)
    elif method == 'torque_residual':
        values, condition = fit_rotation(train_matrix, train_target, strict=False)
        fit_info = dict(method=method, core_parameter_at_boundary=False, inertia_prior_used=False)
        if config.get('inertia_prior_kg_m2') is not None:
            raise ValueError('Inertia priors require rotation_fit_method: angular_acceleration')
    else:
        raise ValueError('Unknown rotation_fit_method')
    physical = bool(np.all(values[:3] > 0) and np.all(2*values[:3] <= values[:3].sum()))
    validation_matrix, validation_target = rotational_system(
        validation, validation_state, propulsion['coefficient'], config)
    angular_rmse = rotation_score(validation_matrix, validation_target, values)
    force_limit = mass*9.81*.12
    angular_limit = float(config.get('max_angular_accel_rmse_rad_s2', 3.0))
    passed = (physical and not fit_info['core_parameter_at_boundary'] and condition < 1e5 and not propulsion['at_boundary'] and validation_force_rmse < force_limit and
              np.all(angular_rmse < angular_limit))
    return dict(
        passed=bool(passed),
        physical_consistency=dict(inertia_triangle_inequality=physical,
            failure_reason='' if physical else 'Candidate inertia violates rigid-body triangle inequalities; do not apply'),
        method='Grey-box fit inspired by Eschmann, Albani and Loianno (2024)',
        rotation_fit=fit_info,
        rotor_input='measured' if measured else 'command_model',
        assumptions=['known mass and rotor geometry', 'vertical identical rotors',
                     'diagonal aggregate inertia', 'common quadratic thrust curve',
                     'low-speed flight'] + ([] if measured else ['common first-order motor delay']),
        parameter_accuracy_validated=False,
        validation_scope='passed tests held-out dynamic residuals and physical constraints, not parameter accuracy',
        measured=dict(mass_kg=mass,
                      rotor_positions_flu_m=config['rotor_positions_flu_m'],
                      yaw_torque_signs=config['yaw_torque_signs']),
        propulsion=dict(
            common_motor_time_constant_s=propulsion['tau'],
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
            ('Measured RPM used directly; motor time constants are not identified' if measured else
             'No RPM measured; common rise/fall delay and supplied command-to-speed scaling'),
            'Aggregate inertia is not a drop-in base_link inertia when other SDF links have mass'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train', type=Path, required=True)
    parser.add_argument('--validation', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--priors', type=Path, help='Optional parameter prior YAML; replaces embedded inertia priors')
    parser.add_argument('--train-rpm', type=Path)
    parser.add_argument('--validation-rpm', type=Path)
    parser.add_argument('--rpm-ros-minus-px4-s', type=float)
    parser.add_argument('--esc-indices', type=int, nargs=4, help='ULog ESC slots in rotor geometry order; verify pole count')
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
    prior_metadata = None
    if args.priors is not None:
        try:
            config, prior_metadata = load_parameter_priors(config, args.priors)
        except (ValueError, TypeError, OSError, yaml.YAMLError) as error:
            parser.error(str(error))
    if bool(args.train_rpm) != bool(args.validation_rpm) or (args.train_rpm and args.esc_indices):
        parser.error('Supply both RPM CSVs or --esc-indices, not a mixture')
    training = read_ulog(args.train,args.train_rpm,args.rpm_ros_minus_px4_s,args.esc_indices)
    validation = read_ulog(args.validation,args.validation_rpm,args.rpm_ros_minus_px4_s,args.esc_indices)
    report = identify(training, validation, config)
    report['parameter_prior_file'] = prior_metadata
    report['input_mapping'] = dict(training=training['input_mapping'], validation=validation['input_mapping'])
    report['training_sha256'] = hashlib.sha256(args.train.read_bytes()).hexdigest()
    report['validation_sha256'] = hashlib.sha256(args.validation.read_bytes()).hexdigest()
    report['source_logs'] = [str(args.train.resolve()), str(args.validation.resolve())]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        yaml.safe_dump(report, stream, sort_keys=False)
    print('Gazebo physics report:', args.output, 'passed:', report['passed'])


if __name__ == '__main__':
    main()
