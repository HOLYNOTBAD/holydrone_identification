"""一阶惯性串联二阶环节、带纯时延的三阶闭环响应辨识。"""
import hashlib
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from brct1order_regression import CHANNELS, read_csv, segments


PARAMETER_KEYS = ('tau_s', 'natural_frequency_rad_s', 'damping_ratio', 'delay_s', 'gain', 'bias')


def predict(t, u, initial, tau, omega_n, damping, delay, gain, bias):
    """精确 ZOH 阶跃叠加；每段 y(0) 为实测值，y'(0)=y''(0)=0。

    G(s)=K*wn²*exp(-d*s)/((tau*s+1)*(s²+2*zeta*wn*s+wn²))。
    首个样本前输入延拓为 u[0]；偏置为常数输出偏置。
    状态 [一阶输出, y, y'/wn] 使矩阵量级接近，减少特征向量病态。
    """
    t, u = np.asarray(t), np.asarray(u)
    a = np.array([[-1/tau, 0, 0], [0, 0, omega_n],
                  [omega_n, -omega_n, -2*damping*omega_n]])
    poles, vectors = np.linalg.eig(a)
    equilibrium = np.array([1., 1., 0.])
    if np.linalg.cond(vectors) < 1e6:
        weights = vectors[1] * np.linalg.solve(vectors, equilibrium)

        def homogeneous(x):
            return np.real(np.sum(np.exp(x[..., None]*poles)*weights, axis=-1))
    else:
        # 临界阻尼或重合极点不能可靠地特征分解；矩阵指数也覆盖三重极点。
        def homogeneous(x):
            times, inverse = np.unique(x, return_inverse=True)
            values = np.array([(expm(a*dt) @ equilibrium)[1] for dt in times])
            return values[inverse].reshape(x.shape)

    changes = np.flatnonzero(np.diff(u) != 0)+1
    delta = t[:, None]-t[changes][None, :]-delay
    step = np.where(delta > 0, 1-homogeneous(np.maximum(delta, 0)), 0)
    base = gain*u[0]+bias
    return base+(initial-base)*homogeneous(t-t[0])+gain*(step @ np.diff(u)[changes-1])


def fit_channel(pulses, channel):
    """六参数多初值鲁棒拟合；正时间常数和正阻尼保证动态稳定。"""
    max_dt = max(float(np.median(np.diff(t))) for t, _, _ in pulses)
    upper_frequency = min(80., .45/max_dt)
    bounds = ([.003, 2*np.pi, .1, 0., .01, -20.],
              [.5, 2*np.pi*upper_frequency, 2., .15, 100., 20.])

    def residual(p):
        return np.concatenate([(predict(t, u, y[0], *p)-y)[1:] for t, u, y in pulses])

    candidates = []
    starts = [(.015, 40., .5, .005), (.04, 25., .5, .02),
              (.1, 15., 1.2, .04), (.03, 80., 1.2, .01)]
    for tau, wn, damping, delay in starts:
        initial = [tau, min(wn, bounds[1][1]*.8), damping, delay, 30. if channel == 3 else 1., 0.]
        fit = least_squares(residual, initial, bounds=bounds, loss='soft_l1',
                            f_scale=.1, x_scale='jac', max_nfev=400)
        if fit.success:
            candidates.append(fit)
    if not candidates:
        raise ValueError('Third-order optimizer did not converge: '+CHANNELS[channel])
    fit = min(candidates, key=lambda f: np.mean(residual(f.x)**2))
    lower, upper = np.asarray(bounds[0]), np.asarray(bounds[1])
    margin = (upper-lower)*.001
    # 零时延合法；其余参数贴近边界通常意味着模型/采样不足，报告未通过。
    interior = [0, 1, 2, 4, 5]
    boundary = np.any(fit.x >= upper-margin) or np.any(fit.x[interior] <= (lower+margin)[interior])
    return fit.x, bool(boundary)


def score(pulses, params):
    observed = np.concatenate([y[1:] for _, _, y in pulses])
    error = np.concatenate([(predict(t, u, y[0], *params)-y)[1:] for t, u, y in pulses])
    rmse = float(np.sqrt(np.mean(error**2)))
    span = float(np.percentile(observed, 95)-np.percentile(observed, 5))
    return dict(rmse=rmse, normalized_rmse=rmse/max(span, 1e-6), output_span=span,
                samples=len(error), pulses=len(pulses))


def sampling_resolved(pulses, params):
    """极点模长/(2π) 小于 0.45 倍采样率；只作分辨率筛查，不保证可辨识性。"""
    tau, wn, damping = params[:3]
    poles = np.r_[-1/tau, np.roots([1., 2*damping*wn, wn*wn])]
    max_dt = max(float(np.median(np.diff(t))) for t, _, _ in pulses)
    return bool(max(abs(poles))/(2*np.pi) < .45/max_dt)


def identify_response(train_path, validation_path, max_nrmse=.2, max_rate_rmse=.15,
                      max_thrust_rmse=1.):
    training_bytes, validation_bytes = Path(train_path).read_bytes(), Path(validation_path).read_bytes()
    if Path(train_path).resolve() == Path(validation_path).resolve() or training_bytes == validation_bytes:
        raise ValueError('Validation must contain held-out pulse groups, not the training data')
    train, validation = read_csv(train_path), read_csv(validation_path)
    channels = []
    for i, name in enumerate(CHANNELS):
        training, testing = segments(train, i), segments(validation, i)
        params, boundary = fit_channel(training, i)
        metrics = score(testing, params)
        train_u = np.concatenate([u for _, u, _ in training])
        test_u = np.concatenate([u for _, u, _ in testing])
        common_range = [float(max(train_u.min(), test_u.min())), float(min(train_u.max(), test_u.max()))]
        train_resolved, test_resolved = sampling_resolved(training, params), sampling_resolved(testing, params)
        passed = (not boundary and train_resolved and test_resolved
                  and metrics['output_span'] > (.1 if i < 3 else .3)
                  and metrics['normalized_rmse'] <= max_nrmse
                  and metrics['rmse'] <= (max_rate_rmse if i < 3 else max_thrust_rmse)
                  and common_range[0] < common_range[1]
                  and (i == 3 or common_range[0] < 0 < common_range[1]))
        tau, wn, damping, _, gain, _ = params
        channels.append(dict(channel=name, **dict(zip(PARAMETER_KEYS, map(float, params))),
            natural_frequency_hz=float(wn/(2*np.pi)),
            denominator_coefficients=[float(v) for v in (tau, 1+2*damping*wn*tau,
                                                         2*damping*wn+wn*wn*tau, wn*wn)],
            numerator_coefficient=float(gain*wn*wn),
            at_parameter_boundary=boundary, training_sampling_resolved=train_resolved,
            validation_sampling_resolved=test_resolved, validated_input_range=common_range,
            passed=bool(passed), training=score(training, params), validation=metrics))
    hover = (9.81-channels[3]['bias'])/channels[3]['gain']
    return dict(model='third_order_with_delay',
        equation='tau*y_dddot + (1+2*zeta*omega_n*tau)*y_ddot + '
                 '(2*zeta*omega_n+omega_n^2*tau)*y_dot + omega_n^2*y = omega_n^2*(gain*u(t-delay)+bias)',
        initial_condition='Measured initial output; zero first and second derivatives per pulse segment',
        passed=bool(all(c['passed'] for c in channels) and 0 < hover < 1
                    and channels[3]['validated_input_range'][0] < hover < channels[3]['validated_input_range'][1]),
        limitation='Local closed-loop all-pole approximation, not motor constants. '
                   'Passing prediction checks does not establish unique parameters; '
                   'real poles can have multiple equivalent first/second-order factorizations.',
        training_sha256=hashlib.sha256(training_bytes).hexdigest(),
        validation_sha256=hashlib.sha256(validation_bytes).hexdigest(),
        validation_thresholds=dict(max_nrmse=max_nrmse, max_rate_rmse=max_rate_rmse, max_thrust_rmse=max_thrust_rmse),
        hover_thrust=float(hover), channels=channels)
