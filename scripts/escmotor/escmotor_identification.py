#!/usr/bin/env python3
"""从台架 dataset 拟合逐工况稳态转速与带延迟的一阶启动响应。"""
import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
import yaml


def predict(t, initial, steady, tau, delay):
    return initial+(steady-initial)*(1-np.exp(-np.maximum(t-delay, 0)/tau))


def fit(pulses, steady):
    def residual(x):
        return np.concatenate([predict(t, initial, steady, *x)-y for t, y, initial in pulses])
    candidates = [least_squares(residual, [.05, delay], bounds=([.001, 0], [2., .5]),
                                loss='soft_l1', f_scale=max(abs(steady)*.02, 1.)) for delay in [0., .02, .1]]
    candidates = [f for f in candidates if f.success]
    if not candidates: raise ValueError('启动模型优化未收敛')
    best = min(candidates, key=lambda f: np.mean(residual(f.x)**2))
    return best.x


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--max-nrmse', type=float, default=.2)
    args = p.parse_args()
    if not np.isfinite(args.max_nrmse) or args.max_nrmse <= 0:
        p.error('--max-nrmse 必须为有限正数')
    if args.output.exists(): p.error('输出已存在，请换一个报告路径')
    meta = yaml.safe_load((args.dataset/'dataset.yaml').read_text())
    if meta['schema'] != 'escmotor_dataset_v1': raise ValueError('不支持的数据集格式')
    with (args.dataset/'samples.csv').open() as f: rows = list(csv.DictReader(f))
    def pulse(group):
        records = [r for r in rows if int(r['segment']) == group['segment']]
        if any(r['split'] != group['split'] for r in records): raise ValueError('数据集划分不一致')
        pre = [float(r['omega_rad_s']) for r in records if r['phase'] == 'pre']
        on = [r for r in records if r['phase'] == 'on']
        if not pre or len(on) < 5: raise ValueError('脉冲缺少初值或有效过渡样本')
        t = np.array([float(r['t_s']) for r in on])
        y = np.array([float(r['omega_rad_s']) for r in on])
        if not np.isfinite(np.r_[t,y,pre]).all() or np.any(np.diff(t) <= 0): raise ValueError('样本时间或数值无效')
        return t, y, pre[-1]
    strata = {}
    ids = [g['segment'] for g in meta['groups']]
    if len(ids) != len(set(ids)): raise ValueError('重复脉冲编号')
    for g in meta['groups']:
        strata.setdefault((g['target_voltage_v'], g['u']), []).append(g)
    results = []
    for (voltage, u), groups in sorted(strata.items()):
        train = [g for g in groups if g['split'] == 'train']
        validation = [g for g in groups if g['split'] == 'validation']
        if not train or not validation: raise ValueError('每个工况必须有独立训练和验证脉冲')
        steady = float(np.mean([g['omega_ss_rad_s'] for g in train]))
        tau, delay = map(float, fit([pulse(g) for g in train], steady))
        val = [pulse(g) for g in validation]
        error = np.concatenate([predict(t, initial, steady, tau, delay)-y for t,y,initial in val])
        rmse = float(np.sqrt(np.mean(error**2)))
        scale = max(abs(steady-float(np.mean([a for _,_,a in val]))), 1.)
        boundary = tau <= .00101 or tau >= 1.998 or delay >= .4995
        dt = max(float(np.median(np.diff(pulse(g)[0]))) for g in groups)
        resolved = tau >= 3*dt
        results.append(dict(target_voltage_v=voltage, command=u,
                            training_voltage_v=float(np.mean([g['voltage_v'] for g in train])),
                            validation_voltage_v=float(np.mean([g['voltage_v'] for g in validation])),
                            omega_ss_rad_s=steady, startup_tau_s=tau, startup_delay_s=delay,
                            validation_rmse_rad_s=rmse, validation_nrmse=rmse/scale,
                            at_boundary=boundary, sampling_resolved=resolved,
                            passed=bool(not boundary and resolved and rmse/scale <= args.max_nrmse),
                            training_segments=[g['segment'] for g in train], validation_segments=[g['segment'] for g in validation]))
    report = dict(model='per_condition_first_order_startup_with_delay',
                  limitation='Starts from rest; not a running-point motor time constant. Steady map is a table, no voltage extrapolation. Delay includes output/telemetry effects.',
                  input='PX4 actuator_test normalized command', dataset=str(args.dataset.resolve()),
                  max_validation_nrmse=args.max_nrmse, passed=bool(results and all(r['passed'] for r in results)), conditions=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f: yaml.safe_dump(report, f, allow_unicode=True, sort_keys=False)
    print('辨识报告：', args.output, 'passed:', report['passed'])


if __name__ == '__main__':
    main()
