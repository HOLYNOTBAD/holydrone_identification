"""带纯时延二阶闭环响应；复用一阶模型的数据筛选与独立验证口径。"""
import hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from response import CHANNELS, read_csv, segments


def predict(t, u, initial, omega_n, damping, delay, gain, bias):
    """二阶稳定系统的精确 ZOH 阶跃叠加；初始输出已知，初始导数为零。

    y'' + 2*zeta*wn*y' + wn²*y = wn²*(K*u(t-d)+b)。
    omega_n 单位 rad/s；支持欠阻尼、临界阻尼、过阻尼。
    """
    t, u = np.asarray(t), np.asarray(u)
    def homogeneous(x):
        if abs(damping-1) < 1e-7:
            return np.exp(-omega_n*x)*(1+omega_n*x)
        if damping < 1:
            wd = omega_n*np.sqrt(1-damping*damping)
            return np.exp(-damping*omega_n*x)*(np.cos(wd*x)+damping*omega_n/wd*np.sin(wd*x))
        root = np.sqrt(damping*damping-1)
        slow, fast = -omega_n*(damping-root), -omega_n*(damping+root)
        return (fast*np.exp(slow*x)-slow*np.exp(fast*x))/(fast-slow)
    # 只保留实际输入跳变，避免为长平台建立不必要的 N×N 矩阵。
    changes = np.flatnonzero(np.diff(u) != 0)+1
    delta = t[:, None]-t[changes][None, :]-delay
    step = np.where(delta > 0, 1-homogeneous(np.maximum(delta, 0)), 0)
    base = gain*u[0]+bias
    return base+(initial-base)*homogeneous(t-t[0])+gain*(step @ (u[changes]-u[changes-1]))


def fit_channel(pulses, channel):
    """多初值鲁棒拟合五参数；自然频率上界同时受观测采样率限制。"""
    max_dt = max(float(np.median(np.diff(t))) for t, _, _ in pulses)
    upper_frequency = min(80., .45/max_dt)
    bounds = ([2*np.pi, .1, 0., .01, -20.],
              [2*np.pi*upper_frequency, 2., .15, 100., 20.])
    def residual(p):
        return np.concatenate([(predict(t, u, y[0], *p)-y)[1:] for t, u, y in pulses])
    candidates = []
    for wn, damping, delay in [(15., .5, .01), (40., .5, .02), (80., 1.2, .04)]:
        initial = [min(wn, bounds[1][0]*.8), damping, delay, 30. if channel == 3 else 1., 0.]
        fit = least_squares(residual, initial, bounds=bounds, loss='soft_l1',
                            f_scale=.1, max_nfev=250)
        if fit.success:
            candidates.append(fit)
    if not candidates:
        raise ValueError('Second-order optimizer did not converge: '+CHANNELS[channel])
    fit = min(candidates, key=lambda f: np.mean(residual(f.x)**2))
    # d=0 是合法模型，其余参数接近边界应明确标为未通过。
    lower, upper = np.asarray(bounds[0]), np.asarray(bounds[1])
    margin = (upper-lower)*.001
    boundary = np.any(fit.x >= upper-margin) or np.any(fit.x[[0, 1, 3, 4]] <= (lower+margin)[[0, 1, 3, 4]])
    return fit.x, bool(boundary)


def score(pulses, params):
    observed = np.concatenate([y[1:] for _, _, y in pulses])
    error = np.concatenate([(predict(t, u, y[0], *params)-y)[1:] for t, u, y in pulses])
    rmse = float(np.sqrt(np.mean(error**2)))
    span = float(np.percentile(observed, 95)-np.percentile(observed, 5))
    return dict(rmse=rmse, normalized_rmse=rmse/max(span, 1e-6), output_span=span,
                samples=len(error), pulses=len(pulses))


def identify_response(train_path, validation_path, max_nrmse=.2, max_rate_rmse=.15,
                      max_thrust_rmse=1.):
    training_bytes, validation_bytes = Path(train_path).read_bytes(), Path(validation_path).read_bytes()
    if Path(train_path).resolve() == Path(validation_path).resolve() or training_bytes == validation_bytes:
        raise ValueError('Validation must be a different acquisition, not the training data')
    train, validation = read_csv(train_path), read_csv(validation_path)
    channels = []
    for i, name in enumerate(CHANNELS):
        training, testing = segments(train, i), segments(validation, i)
        params, boundary = fit_channel(training, i)
        metrics = score(testing, params)
        train_u = np.concatenate([u for _, u, _ in training])
        test_u = np.concatenate([u for _, u, _ in testing])
        common_range = [float(max(train_u.min(), test_u.min())), float(min(train_u.max(), test_u.max()))]
        resolved = params[0]/(2*np.pi) < .45/max(np.median(np.diff(t)) for t, _, _ in testing)
        passed = (not boundary and resolved and metrics['output_span'] > (.1 if i < 3 else .3)
                  and metrics['normalized_rmse'] <= max_nrmse
                  and metrics['rmse'] <= (max_rate_rmse if i < 3 else max_thrust_rmse)
                  and common_range[0] < common_range[1]
                  and (i == 3 or common_range[0] < 0 < common_range[1]))
        channels.append(dict(channel=name,
            **dict(zip(('natural_frequency_rad_s', 'damping_ratio', 'delay_s', 'gain', 'bias'), map(float, params))),
            natural_frequency_hz=float(params[0]/(2*np.pi)),
            at_parameter_boundary=boundary, validation_sampling_resolved=bool(resolved),
            validated_input_range=common_range, passed=bool(passed),
            training=score(training, params), validation=metrics))
    hover = (9.81-channels[3]['bias'])/channels[3]['gain']
    return dict(model='second_order_with_delay',
        equation='y_ddot + 2*zeta*omega_n*y_dot + omega_n^2*y = omega_n^2*(gain*u(t-delay)+bias)',
        initial_condition='Measured initial output and zero initial derivative per pulse group',
        passed=bool(all(c['passed'] for c in channels) and 0 < hover < 1
                    and channels[3]['validated_input_range'][0] < hover < channels[3]['validated_input_range'][1]),
        limitation='Local closed-loop approximation; parameters are not Gazebo motor constants',
        training_sha256=hashlib.sha256(training_bytes).hexdigest(),
        validation_sha256=hashlib.sha256(validation_bytes).hexdigest(),
        validation_thresholds=dict(max_nrmse=max_nrmse, max_rate_rmse=max_rate_rmse, max_thrust_rmse=max_thrust_rmse),
        hover_thrust=float(hover), channels=channels)
