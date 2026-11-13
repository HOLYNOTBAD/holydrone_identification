#!/usr/bin/env python3
"""从现有 escmotor_dataset 生成稳态映射、逐工况启动模型和离线报告。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from escmotor_identification import fit, predict

COLORS = ['#087f8c', '#6561c5', '#d47a26']


def read_csv(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_plot(fig, path):
    fig.tight_layout()
    for ext in ('svg', 'png', 'pdf'):
        fig.savefig(path.with_suffix('.'+ext), dpi=160, bbox_inches='tight')
    plt.close(fig)


def metrics(errors, scale):
    errors = np.asarray(errors)
    rmse = float(np.sqrt(np.mean(errors**2)))
    return dict(rmse_rpm=rmse, nrmse=rmse/max(abs(scale), 1.), max_abs_error_rpm=float(np.max(np.abs(errors))))


def load_pulses(dataset):
    valid, excluded = [], []
    for row in read_csv(dataset/'pulses.csv'):
        reason = None
        if row['phase'] != 'experiment':
            reason = '试转，不参与正式拟合'
        elif row['quality'] != 'pass':
            reason = row['reasons'] or '数据质量未通过'
        if reason:
            excluded.append(dict(pulse_id=row['pulse_id'], reason=reason))
            continue
        records = read_csv(dataset/row['samples_path'])
        t = np.array([float(r['time_s']) for r in records])
        y = np.array([float(r['rpm']) for r in records])
        before = np.array([r['phase'] == 'before' for r in records])
        on = np.array([r['phase'] == 'hold' for r in records])
        if not np.isfinite(np.r_[t, y]).all() or np.any(np.diff(t) <= 0) or not before.any() or on.sum() < 5:
            excluded.append(dict(pulse_id=row['pulse_id'], reason='缺少初值或有效过渡样本'))
            continue
        try:
            voltage, u, steady = float(row['target_voltage_v']), float(row['command']), float(row['steady_rpm'])
            measured_voltage = float(row['mean_voltage_v'])
            if not np.isfinite([voltage, u, steady, measured_voltage]).all() or min(voltage, u, steady, measured_voltage) <= 0:
                raise ValueError
        except ValueError:
            excluded.append(dict(pulse_id=row['pulse_id'], reason='工况参数无效'))
            continue
        valid.append(dict(**row, u=u, target_v=voltage, steady=steady, voltage=measured_voltage,
                          t=t, y=y, before=before, on=on, initial=float(y[before][-1]),
                          sample_sha256=hashlib.sha256((dataset/row['samples_path']).read_bytes()).hexdigest()))
    return valid, excluded


def split_pulses(items):
    items = sorted(items, key=lambda p: (p['experiment_id'], float(p['start_fcu_s'])))
    training = items[:-1] if len(items) > 1 else items
    validation = items[-1:] if len(items) > 1 else []
    return training, validation


def fit_startup(items):
    """有重复时最后一脉冲留出；单脉冲只计算样本内误差，不伪造验证。"""
    training, validation = split_pulses(items)
    items = training + validation
    steady = float(np.mean([p['steady'] for p in training]))
    triples = [(p['t'][p['on']], p['y'][p['on']], p['initial']) for p in training]
    tau, delay = map(float, fit(triples, steady))
    train_errors, val_errors, traces = [], [], []
    for p in items:
        t, y = p['t'][p['on']], p['y'][p['on']]
        estimate = predict(t, p['initial'], steady, tau, delay)
        split = 'validation' if any(p is v for v in validation) else 'train'
        (val_errors if split == 'validation' else train_errors).extend(estimate-y)
        traces.append(dict(pulse_id=p['pulse_id'], experiment_id=p['experiment_id'], split=split,
                           initial_rpm=p['initial'], measured_voltage_v=p['voltage'], sample_sha256=p['sample_sha256'],
                           raw_time_s=p['t'].tolist(), raw_rpm=p['y'].tolist(), time_s=t.tolist(),
                           measured_rpm=y.tolist(), predicted_rpm=estimate.tolist(), residual_rpm=(estimate-y).tolist()))
    dt = max(float(np.median(np.diff(p['t'][p['on']]))) for p in items)
    flags = []
    if tau <= .00101 or tau >= 1.998 or delay >= .4995:
        flags.append('参数接近优化边界')
    if tau < 3*dt:
        flags.append('采样不足以分辨时间常数')
    scale = steady-float(np.mean([p['initial'] for p in training]))
    validation_metrics = metrics(val_errors, steady-float(np.mean([p['initial'] for p in validation]))) if validation else None
    if validation_metrics and validation_metrics['nrmse'] > .2:
        flags.append('验证归一化误差超过 20%')
    if not validation:
        flags.append('单次脉冲，无独立验证')
    return dict(command=items[0]['u'], target_voltage_v=items[0]['target_v'], steady_rpm=steady,
                tau_s=tau, delay_s=delay, training_count=len(training), validation_count=len(validation),
                training_metrics=metrics(train_errors, scale), validation_metrics=validation_metrics,
                sampling_interval_s=dt, flags=flags, traces=traces)


def fit_unified(pulses, conditions):
    """先用训练末段转速拟合 f(u)，再只用训练保持段估计共享 τ 和延迟。"""
    strata = {}
    for p in pulses:
        strata.setdefault(p['u'], []).append(p)
    training, validation = [], []
    for items in strata.values():
        train, val = split_pulses(items)
        training.extend(train)
        validation.extend(val)
    u = np.array([p['u'] for p in training])
    y = np.array([p['steady'] for p in training])
    degree = min(2, len(np.unique(u))-1)
    coefficients = np.pad(np.polyfit(u, y, degree), (2-degree, 0))
    def residual(x):
        return np.concatenate([predict(p['t'][p['on']], p['initial'],
                                       np.polyval(coefficients, p['u']), *x)-p['y'][p['on']]
                               for p in training])
    scale = max(float(np.median(y))*.02, 1.)
    candidates = [least_squares(residual, [.05, delay], bounds=([.001, 0], [2., .5]),
                                loss='soft_l1', f_scale=scale) for delay in [0., .02, .1]]
    candidates = [c for c in candidates if c.success]
    if not candidates:
        raise ValueError('统一启动模型优化未收敛')
    best = min(candidates, key=lambda c: np.mean(residual(c.x)**2))
    tau, delay = map(float, best.x)
    val_ids = {p['pulse_id'] for p in validation}
    baseline = {t['pulse_id']: t for c in conditions for t in c['traces']}
    traces = []
    for p in sorted(pulses, key=lambda p: (p['u'], p['pulse_id'])):
        t, measured = p['t'][p['on']], p['y'][p['on']]
        steady = float(np.polyval(coefficients, p['u']))
        predicted = predict(t, p['initial'], steady, tau, delay)
        ref = baseline.get(p['pulse_id'])
        split = 'validation' if p['pulse_id'] in val_ids else 'train'
        if ref and (ref['split'] != split or not np.array_equal(t, ref['time_s'])):
            raise ValueError('统一模型和逐档模型的划分或样本时间不一致')
        traces.append(dict(pulse_id=p['pulse_id'], experiment_id=p['experiment_id'],
                           command=p['u'], split=split, initial_rpm=p['initial'],
                           steady_rpm=steady, measured_steady_rpm=p['steady'],
                           measured_voltage_v=p['voltage'], sample_sha256=p['sample_sha256'],
                           time_s=t.tolist(), measured_rpm=measured.tolist(), predicted_rpm=predicted.tolist(),
                           residual_rpm=(predicted-measured).tolist(),
                           local_predicted_rpm=ref['predicted_rpm'] if ref else None,
                           local_residual_rpm=ref['residual_rpm'] if ref else None))
    def evaluate(items, field):
        available = [t for t in items if t[field] is not None]
        if not available:
            return None
        errors = np.concatenate([t[field] for t in available])
        return dict(pulse_count=len(available), sample_count=len(errors),
                    rmse_rpm=float(np.sqrt(np.mean(errors**2))),
                    max_abs_error_rpm=float(np.max(np.abs(errors))))
    comparison = []
    for split in ('train', 'validation'):
        # 比较只在两种模型都有预测的相同脉冲上进行。
        subset = [t for t in traces if t['split'] == split and t['local_residual_rpm'] is not None]
        comparison.append(dict(split=split, unified=evaluate(subset, 'residual_rpm'),
                               local=evaluate(subset, 'local_residual_rpm')))
    by_command = []
    for command in sorted(strata):
        subset = [t for t in traces if t['command'] == command]
        by_command.append(dict(command=command, rows=[dict(split=split,
            unified=evaluate([t for t in subset if t['split'] == split and t['local_residual_rpm'] is not None], 'residual_rpm'),
            local=evaluate([t for t in subset if t['split'] == split and t['local_residual_rpm'] is not None], 'local_residual_rpm'))
            for split in ('train', 'validation')]))
    dt = max(float(np.median(np.diff(p['t'][p['on']]))) for p in pulses)
    flags = []
    if tau <= .00101 or tau >= 1.998 or delay >= .4995:
        flags.append('参数接近优化边界')
    if tau < 3*dt:
        flags.append('采样不足以分辨时间常数')
    if not validation:
        flags.append('本组无独立验证脉冲')
    if any(t['local_residual_rpm'] is None for t in traces):
        flags.append('部分逐档模型拟合失败，对比仅使用双方均有结果的脉冲')
    return dict(coefficients=coefficients.tolist(), polynomial_degree=degree,
                tau_s=tau, delay_s=delay, training_count=len(training), validation_count=len(validation),
                training_pulse_ids=[p['pulse_id'] for p in training], validation_pulse_ids=sorted(val_ids),
                command_range=[float(u.min()), float(u.max())],
                validation_commands=sorted(set(p['u'] for p in validation)),
                training_metrics=evaluate([t for t in traces if t['split'] == 'train'], 'residual_rpm'),
                validation_metrics=evaluate([t for t in traces if t['split'] == 'validation'], 'residual_rpm'),
                comparison=comparison, by_command=by_command, traces=traces, flags=flags,
                loss='soft_l1', loss_scale_rpm=scale,
                method='two-stage: training-only steady polynomial, then shared dynamics; equal raw-RPM residual weight per training sample')


def save_unified(report, folder):
    (folder/'unified_model.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    rows = []
    for trace in report['traces']:
        for i, t in enumerate(trace['time_s']):
            rows.append(dict(pulse_id=trace['pulse_id'], command=trace['command'], split=trace['split'],
                             time_s=t, measured_rpm=trace['measured_rpm'][i], predicted_rpm=trace['predicted_rpm'][i],
                             residual_rpm=trace['residual_rpm'][i],
                             local_predicted_rpm=trace['local_predicted_rpm'][i] if trace['local_predicted_rpm'] is not None else '',
                             local_residual_rpm=trace['local_residual_rpm'][i] if trace['local_residual_rpm'] is not None else ''))
    write_csv(folder/'unified_predictions.csv', rows)
    comparison = []
    for group in report['by_command']:
        for r in group['rows']:
            if r['unified']:
                comparison.append(dict(command=group['command'], split=r['split'],
                    pulse_count=r['unified']['pulse_count'], unified_rmse_rpm=r['unified']['rmse_rpm'],
                    local_rmse_rpm=r['local']['rmse_rpm']))
    write_csv(folder/'unified_comparison.csv', comparison)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, split, label in zip(axes, ('train', 'validation'), ('Training', 'Held-out validation')):
        selected = [r for r in comparison if r['split'] == split]
        if selected:
            ax.plot([r['command'] for r in selected], [r['unified_rmse_rpm'] for r in selected], 'o-', color='#087f8c', label='Shared model')
            ax.plot([r['command'] for r in selected], [r['local_rmse_rpm'] for r in selected], 's--', color='#6561c5', label='Per-command model')
            ax.legend()
        else:
            ax.text(.5, .5, 'No held-out validation pulses', ha='center', transform=ax.transAxes)
        ax.set(title=label, ylabel='RMSE (RPM)')
        ax.grid(alpha=.2)
    axes[-1].set_xlabel('Normalized command')
    save_plot(fig, folder/'unified_comparison')


def make_group(pulses, key, title, folder, experiment_names):
    folder.mkdir(parents=True, exist_ok=True)
    source_ids = list(dict.fromkeys(p['experiment_id'] for p in pulses))
    colors = {exp: COLORS[i % len(COLORS)] for i, exp in enumerate(source_ids)}
    u = np.array([p['u'] for p in pulses])
    steady = np.array([p['steady'] for p in pulses])
    # 每个有效正式脉冲等权；仅在采样油门区间内使用，不固定零截距。
    degree = min(2, len(np.unique(u))-1)
    coefficients = np.pad(np.polyfit(u, steady, degree), (2-degree, 0))
    estimates = np.polyval(coefficients, u)
    err = estimates-steady
    ss = float(np.sum((steady-steady.mean())**2))
    steady_report = dict(coefficients=coefficients.tolist(), degree=degree,
                         rmse_rpm=float(np.sqrt(np.mean(err**2))),
                         max_abs_error_rpm=float(np.max(np.abs(err))),
                         r_squared=1-float(np.sum(err**2))/ss if ss else None,
                         command_range=[float(u.min()), float(u.max())],
                         points=[dict(pulse_id=p['pulse_id'], experiment_id=p['experiment_id'], command=p['u'],
                                      measured_voltage_v=p['voltage'], steady_rpm=p['steady'],
                                      predicted_rpm=float(pred), residual_rpm=float(pred-p['steady'])) for p, pred in zip(pulses, estimates)])
    write_csv(folder/'steady_points.csv', steady_report['points'])
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [3, 1]})
    grid = np.linspace(min(u), max(u), 400)
    axes[0].plot(grid, np.polyval(coefficients, grid), color='#243b53', label='Quadratic fit')
    for i, exp in enumerate(source_ids):
        mask = [p['experiment_id'] == exp for p in pulses]
        axes[0].scatter(u[mask], steady[mask], color=colors[exp], label=experiment_names[exp])
        axes[1].scatter(u[mask], err[mask], color=colors[exp])
    axes[0].set_ylabel('Steady speed (RPM)')
    axes[0].legend()
    axes[1].axhline(0, color='#8898a8', lw=1)
    axes[1].set(xlabel='Normalized command', ylabel='Predicted - measured (RPM)')
    for ax in axes:
        ax.grid(alpha=.2)
    save_plot(fig, folder/'steady')
    strata = {}
    for p in pulses:
        strata.setdefault(p['u'], []).append(p)
    conditions, failures = [], []
    for i, (command, items) in enumerate(sorted(strata.items()), 1):
        try:
            c = fit_startup(items)
        except ValueError as error:
            failures.append(dict(command=command, reason=str(error)))
            continue
        c['key'] = f'condition_{i:03d}'
        conditions.append(c)
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [3, 1]})
        trace_rows = []
        for j, trace in enumerate(c['traces']):
            color = COLORS[j % len(COLORS)]
            label = f'{trace["split"]} / pulse {j+1}'
            axes[0].plot(trace['raw_time_s'], trace['raw_rpm'], color=color, alpha=.7, label=label+' measured')
            axes[0].plot(trace['time_s'], trace['predicted_rpm'], '--', color=color, label=label+' model')
            axes[1].plot(trace['time_s'], trace['residual_rpm'], color=color, label=label)
            trace_rows.extend(dict(pulse_id=trace['pulse_id'], split=trace['split'], time_s=t,
                                   measured_rpm=y, predicted_rpm=pred, residual_rpm=e)
                              for t, y, pred, e in zip(trace['time_s'], trace['measured_rpm'], trace['predicted_rpm'], trace['residual_rpm']))
        axes[0].set(title=f'Command {command:g} | tau={c["tau_s"]:.5f} s | delay={c["delay_s"]:.5f} s', ylabel='Speed (RPM)')
        axes[0].legend(fontsize=9)
        axes[1].axhline(0, color='#8898a8', lw=1)
        axes[1].set(xlabel='Time since command enabled (s)', ylabel='Predicted - measured (RPM)')
        for ax in axes:
            ax.grid(alpha=.2)
        save_plot(fig, folder/c['key'])
        write_csv(folder/(c['key']+'.csv'), trace_rows)
    summary = []
    for c in conditions:
        summary.append(dict(command=c['command'], steady_rpm=c['steady_rpm'], tau_s=c['tau_s'], delay_s=c['delay_s'],
                            training_count=c['training_count'], validation_count=c['validation_count'],
                            training_rmse_rpm=c['training_metrics']['rmse_rpm'],
                            validation_rmse_rpm=c['validation_metrics']['rmse_rpm'] if c['validation_metrics'] else '',
                            flags=';'.join(c['flags'])))
    write_csv(folder/'startup_parameters.csv', summary)
    unified = fit_unified(pulses, conditions)
    save_unified(unified, folder)
    report = dict(key=key, title=title, pulse_count=len(pulses), target_voltage_v=pulses[0]['target_v'],
                  measured_voltage_range=[min(p['voltage'] for p in pulses), max(p['voltage'] for p in pulses)],
                  experiments=source_ids, steady=steady_report, conditions=conditions, unified=unified, failures=failures)
    (folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return report


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=root/'data/escmotor/dataset')
    parser.add_argument('--output', type=Path, default=root/'data/escmotor/fit')
    args = parser.parse_args()
    experiments = read_csv(args.dataset/'experiments.csv')
    pulses, excluded = load_pulses(args.dataset)
    if not pulses:
        raise ValueError('无可拟合的正式脉冲')
    names = {e['experiment_id']: f'Session {i+1}' for i, e in enumerate(experiments)}
    groups = []
    args.output.mkdir(parents=True, exist_ok=True)
    for voltage in sorted(set(p['target_v'] for p in pulses)):
        at_voltage = [p for p in pulses if p['target_v'] == voltage]
        selections = []
        for i, experiment in enumerate(experiments):
            session = experiment['session']
            if Path(session).name != session or session in ('.', '..'):
                raise ValueError(f'实验目录名无效：{session}')
            experiment_pulses = [p for p in pulses if p['experiment_id'] == experiment['experiment_id']]
            # 单电压会话沿用原始日期时间目录名；多电压会话额外区分工况。
            key = session if len({p['target_v'] for p in experiment_pulses}) == 1 else session+f'_v{voltage:g}'
            selections.append((key, f'第 {i+1} 次实验', [p for p in experiment_pulses if p['target_v'] == voltage]))
        selections.append((f'combined_v{voltage:g}', '所有数据共同拟合', at_voltage))
        for key, label, items in selections:
            if not items:
                continue
            report = make_group(items, key, label+f' · {voltage:g} V', args.output/key, names)
            groups.append(report)
            print(f'{report["title"]}：{len(items)} 个正式脉冲，{len(report["conditions"])} 个启动工况；稳态 RMSE={report["steady"]["rmse_rpm"]:.3f} RPM')
            print(f'统一模型：τ={report["unified"]["tau_s"]:.6f} s，延迟={report["unified"]["delay_s"]:.6f} s')
    bundle = dict(schema='escmotor_fit_v2', experiments=experiments, groups=groups, excluded=excluded,
                  source_dataset=str(args.dataset.resolve()),
                  index_sha256={name: hashlib.sha256((args.dataset/name).read_bytes()).hexdigest() for name in ['experiments.csv', 'pulses.csv']},
                  methods=dict(steady='n = a*u^2 + b*u + c; equal weight per formal pulse; in-sample error',
                               startup='reuse escmotor_identification.fit/predict; fixed training steady speed; robust soft_l1',
                               unified='training-only quadratic steady map, then shared tau and delay across training pulses; compared on identical train/validation pulses',
                               split='last pulse per command held out when repeated; singleton is training only',
                               bounds=dict(tau_s=[.001, 2.], delay_s=[0, .5])),
                  limitations='Startup from rest, not running-point dynamics; delay includes command and telemetry effects; supply voltage is not a fitted input.')
    payload = json.dumps(bundle, ensure_ascii=False, allow_nan=False)
    (args.output/'results.json').write_text(payload+'\n')
    template = Path(__file__).with_name('escmotor_fit_report.html').read_text()
    (args.output/'index.html').write_text(template.replace('__RESULTS_JSON__', payload.replace('<', '\\u003c')))
    (args.output/'README.md').write_text('''# ESC / 电机拟合\n\n打开 `index.html` 离线查看，按实验及模型类型切换。\n\n```bash\nconda run --no-capture-output -n holydrone_autotune python scripts/escmotor/escmotor_fit.py\n```\n\n输入为现有 `escmotor_dataset`，不连接飞控、不修改原始数据，也不需要重新导入 ULog。每个实验和联合数据分别输出，目标电压不同则分组，不跨电压合并。\n\n稳态映射使用各正式有效脉冲的末段转速，拟合 n=a u²+b u+c；每脉冲等权，只有测量区间内有效，不强制通过零点。这是实测供电条件下的经验曲线，没有消除压降。\n\n启动模型复用 `escmotor_identification.py` 的带延迟一阶响应，以训练脉冲末段均值固定稳态转速，稳态多项式不参与动态拟合。每工况有多个脉冲时最后一个留出验证，单脉冲只显示样本内误差。原始停转前后样本可见，但仅保持阶段参与启动拟合。初始化使用激励前最后一个转速样本。\n\n优化沿用 soft L1 损失及原参数边界；参数触边、采样不足、验证误差偏大分别提示。启动时间常数不等同于旋转工作点附近的时间常数，延迟包含输出与遥测影响。联合会话的硬件型号仍需核对。\n\n统一模型先只使用训练脉冲的末段转速拟合 f(u)，再用训练保持段拟合共同 τ 和延迟；验证脉冲既不进入 f(u)，也不进入动态参数估计。统一模型的多项式因此可能与“稳态油门—转速”面板的全数据拟合略有不同。动态目标采用原始 RPM 残差的 soft L1 损失，每个采样点等权，尺度为训练稳态转速中位数的 2%。与逐档模型的比较在相同脉冲和时间点上进行，差异同时包含稳态映射近似与共享动态参数的影响。只验证有重复的油门档位，不能将低油门验证推广为全区间独立验证。模型仅用于已测范围的启动响应，不用于零油门稳态或停机。统一结果另存 unified_model.json、unified_predictions.csv、unified_comparison.csv 和对比图。\n\n每组保存 report.json、steady_points.csv、startup_parameters.csv、逐工况采样预测 CSV，以及 SVG/PNG/PDF 图。results.json 包含来源索引指纹及参与动态拟合的样本文件指纹。排除的试转和质量异常记录在 results.json 中。\n''')
    print(f'离线报告：{args.output/"index.html"}')


if __name__ == '__main__':
    main()
