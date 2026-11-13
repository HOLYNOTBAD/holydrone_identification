"""对连续完整脉冲拟合带时延的一阶闭环响应，并使用留出的完整脉冲验证。
输入 CSV 的角速度为 FLU rad/s、推力为归一化值、fz 为 FLU Z 比力 m/s²。
仅认可训练/验证共同覆盖的输入范围；不把局部闭环近似视为全域物理模型。"""
import hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

CHANNELS = ('roll', 'pitch', 'yaw', 'thrust')
INPUTS = ('cmd_wx', 'cmd_wy', 'cmd_wz', 'cmd_thrust')
OUTPUTS = ('wx', 'wy', 'wz', 'fz')


def read_csv(path):
    data = np.genfromtxt(str(path), delimiter=',', names=True, dtype=None, encoding='utf-8')
    data = np.atleast_1d(data)
    if len(data)<20 or 't' not in data.dtype.names:
        raise ValueError('Need at least 20 timestamped samples')
    if not np.isfinite(data['t']).all() or np.any(np.diff(data['t'])<=0):
        raise ValueError('Timestamps must be finite and strictly increasing; split clock resets')
    return data


def segments(data, channel):
    """筛选指定通道的完整连续脉冲，拒绝无效反馈、断采和激励不足的片段。"""
    names = data.dtype.names
    required = ('t', INPUTS[channel], OUTPUTS[channel], 'segment', 'channel', 'valid')
    if any(k not in names for k in required):
        raise ValueError('Response CSV needs '+str(required))
    result = []
    # 不拼接分离脉冲，也不跨无效反馈插值，以免虚构连续响应。
    for segment in np.unique(data['segment']):
        rows = data[data['segment']==segment]
        if len(rows)<20 or not np.all(rows['channel']==CHANNELS[channel]):
            continue
        if 'phase' in names:
            sequence_complete = ('sequence_complete' in names and
                                 np.any(data['sequence_complete']==1))
            if (np.any(np.isin(rows['phase'], ('fault', 'aborted'))) or
                    (not np.any(data['segment']>segment) and not sequence_complete)):
                continue
        good=np.flatnonzero(rows['valid']==1)
        if len(good)<20:continue
        rows=rows[good[0]:good[-1]+1]
        if not np.all(rows['valid']==1):continue
        t,u,y = (np.asarray(rows[k], float) for k in ('t', INPUTS[channel], OUTPUTS[channel]))
        if not np.isfinite(np.column_stack((t,u,y))).all():
            continue
        dt = np.diff(t)
        if dt.max()>.05 or dt.max()>3*np.median(dt):
            continue
        if np.ptp(u)<(.008 if channel==3 else .08):
            continue
        result.append((t-t[0],u,y))
    if not result:
        raise ValueError('No complete, sufficiently excited pulses for '+CHANNELS[channel])
    return result


def predict(t, u, initial, tau, delay, gain, bias):
    # 指令为零阶保持；时延后的跳变可能落在两个观测时刻之间，须分段积分。
    """按延迟指令的零阶保持分段积分一阶响应。

    Args:
        t: 脉冲相对时间，s。
        u: 分段保持的通道指令。
        initial: 首个实测输出。
        tau: 一阶时间常数，s。
        delay: 输入时延，s。
        gain: 稳态增益。
        bias: 输出偏置。

    Returns:
        与 t 对齐的预测输出。"""
    y = np.empty(len(t)); y[0] = initial
    transitions = t+delay
    for i in range(1,len(t)):
        cuts = transitions[(transitions>t[i-1]) & (transitions<t[i])]
        edges = np.r_[t[i-1], cuts, t[i]]
        value = y[i-1]
        for start,end in zip(edges[:-1],edges[1:]):
            index = max(0, np.searchsorted(t, (start+end)/2-delay, side='right')-1)
            alpha = np.exp(-(end-start)/tau)
            value = alpha*value+(1-alpha)*(gain*u[index]+bias)
        y[i] = value
    return y


def fit_channel(pulses, channel):
    """从三个时延初值进行有界鲁棒拟合，返回 [tau, delay, gain, bias] 与边界标志。"""
    def residual(p):
        return np.concatenate([predict(t,u,y[0],*p)-y for t,u,y in pulses])
    gain = 30. if channel==3 else 1.
    bounds = ([.003,0.,.01,-20.], [.5,.15,100.,20.])
    candidates = [least_squares(residual,[.04,d,gain,0.],bounds=bounds,
                                loss='soft_l1', f_scale=.1, max_nfev=180)
                  for d in (.005,.03,.08)]
    fit = min(candidates,key=lambda f: np.mean(residual(f.x)**2))
    if not fit.success:
        raise ValueError('Optimizer did not converge: '+CHANNELS[channel])
    # 触及搜索边界说明模型或参数范围不足，验证结果中单独标记。
    boundary = bool(np.any(fit.x[[0,2]]<=np.array(bounds[0])[[0,2]]*1.01) or
                    np.any(fit.x[:3]>=np.array(bounds[1])[:3]*.99))
    return fit.x, boundary


def score(pulses, params):
    """按全部脉冲的非初始样本计算 RMSE，并用实测 5–95 百分位跨度归一化。"""
    observed = np.concatenate([y[1:] for _,_,y in pulses])
    errors = np.concatenate([(predict(t,u,y[0],*params)-y)[1:] for t,u,y in pulses])
    rmse = float(np.sqrt(np.mean(errors**2)))
    span = float(np.percentile(observed,95)-np.percentile(observed,5))
    return dict(rmse=rmse, normalized_rmse=rmse/max(span,1e-6), output_span=span,
                samples=int(len(errors)), pulses=len(pulses))


def identify_response(train_path, validation_path, max_nrmse=.2, max_rate_rmse=.15,
                      max_thrust_rmse=1.):
    """使用分离的训练/验证 CSV，拒绝同一路径或相同内容。"""
    train_bytes, test_bytes = Path(train_path).read_bytes(), Path(validation_path).read_bytes()
    if Path(train_path).resolve()==Path(validation_path).resolve() or train_bytes==test_bytes:
        raise ValueError('Validation must contain held-out pulse groups, not the training data')
    train, test = read_csv(train_path), read_csv(validation_path)
    channels = []
    for i,name in enumerate(CHANNELS):
        training, validation = segments(train,i), segments(test,i)
        params,boundary = fit_channel(training,i)
        metrics = score(validation,params)
        passed = (not boundary and metrics['output_span']>(.1 if i<3 else .3) and
                  metrics['normalized_rmse']<=max_nrmse and
                  metrics['rmse']<=(max_rate_rmse if i<3 else max_thrust_rmse))
        train_u=np.concatenate([u for _,u,_ in training]);val_u=np.concatenate([u for _,u,_ in validation])
        validated_range=[float(max(train_u.min(),val_u.min())),float(min(train_u.max(),val_u.max()))]
        passed=passed and validated_range[0]<validated_range[1] and (i==3 or validated_range[0]<0<validated_range[1])
        row = dict(channel=name, validated_input_range=validated_range, **dict(zip(('tau_s','delay_s','gain','bias'),map(float,params))),
                   passed=bool(passed), at_parameter_boundary=boundary,
                   training=score(training,params), validation=metrics)
        channels.append(row)
    hover = (9.81-channels[3]['bias'])/channels[3]['gain']
    return dict(passed=bool(all(c['passed'] for c in channels) and 0<hover<1 and
                            channels[3]['validated_input_range'][0]<hover<channels[3]['validated_input_range'][1]),
                provenance='Disjoint training/validation data; ZOH first-order fit',
                limitation='Local closed-loop approximation; no guarantee outside measured excitation range',
                training_sha256=hashlib.sha256(train_bytes).hexdigest(),
                validation_sha256=hashlib.sha256(test_bytes).hexdigest(),
                validation_thresholds=dict(max_nrmse=max_nrmse,max_rate_rmse=max_rate_rmse,
                                           max_thrust_rmse=max_thrust_rmse),
                hover_thrust=float(hover), channels=channels)
