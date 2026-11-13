#!/usr/bin/env python3
"""按 blade 实验拟合静态推力表：查表插值及 F=k_f*omega^2。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def load_table(path):
    review_path = Path(path).with_name('point_reviews.json')
    reviews = json.loads(review_path.read_text()) if review_path.exists() else {}
    with Path(path).open(newline='') as f:
        rows = list(csv.DictReader(f))
    valid, excluded = [], []
    for i, row in enumerate(rows, 1):
        review = reviews.get(str(i), {})
        if review.get('status') in ('pending', 'rejected'):
            excluded.append(dict(row=i, reason=review.get('reason') or review['status']))
            continue
        if row.get('quality', 'pass') != 'pass':
            excluded.append(dict(row=i, reason=row.get('quality_reason') or 'quality_failed'))
            continue
        if not row['rpm'].strip():
            excluded.append(dict(row=i, reason='missing_rpm', thrust_g=float(row['thrust_g'])))
            continue
        values = [float(row[k]) for k in ('rpm', 'thrust_g', 'voltage_v')]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError(f'第 {i} 行转速、推力和电压必须为有限正数')
        valid.append(values)
    if len(valid) < 2:
        raise ValueError('至少需要两个有效转速—推力点')
    data = np.asarray(sorted(valid), float)
    if len(np.unique(data[:, 0])) < 2:
        raise ValueError('至少需要两个不同的有效转速')
    return data[:, 0], data[:, 1], float(np.mean(data[:, 2])), excluded


def fit_square(rpm, thrust_n):
    omega = np.asarray(rpm, float)*2*np.pi/60
    x = omega**2
    kf = float(np.dot(x, thrust_n)/np.dot(x, x))
    predicted = kf*x
    residual = predicted-thrust_n
    ss = float(np.sum((thrust_n-np.mean(thrust_n))**2))
    return kf, predicted, dict(
        rmse_n=float(np.sqrt(np.mean(residual**2))),
        max_abs_error_n=float(np.max(np.abs(residual))),
        max_abs_relative_error=float(np.max(np.abs(residual/thrust_n))),
        r_squared=1-float(np.sum(residual**2))/ss if ss else None)


def lookup(rpm, knots_rpm, thrust_n):
    """仅在已测转速区间内线性插值，禁止外推。"""
    query = np.asarray(rpm, float)
    if not np.isfinite(query).all() or np.any(query < knots_rpm[0]) or np.any(query > knots_rpm[-1]):
        raise ValueError(f'查询转速必须在 [{knots_rpm[0]:g}, {knots_rpm[-1]:g}] rpm 内')
    # 重复测量保留用于拟合；同 RPM 的查表节点取平均推力。
    unique = np.unique(knots_rpm)
    means = [np.mean(np.asarray(thrust_n)[np.asarray(knots_rpm) == r]) for r in unique]
    return np.interp(query, unique, means)


def main():
    root = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__)
    selection = p.add_mutually_exclusive_group()
    selection.add_argument('--experiment', default=None, help='blade_experiment 下的实验目录名，默认 manufacturer')
    selection.add_argument('--table', type=Path, help='自定义 CSV；不套用已有实验的来源或硬件信息')
    p.add_argument('--output', type=Path)
    p.add_argument('--rpm', type=float, help='可选：查询表格有效区间内的推力')
    args = p.parse_args()
    if args.table is None:
        experiment = args.experiment or 'manufacturer'
        folder = root/'data/blade/experiment'/experiment
        if Path(experiment).name != experiment or not (folder/'metadata.json').is_file():
            p.error(f'未知实验：{experiment}')
        metadata = json.loads((folder/'metadata.json').read_text())
        args.table = folder/'static_thrust.csv'
    else:
        experiment = 'custom'
        metadata = dict(source=str(args.table), motor=None, propeller=None)
    if args.output is None:
        args.output = root/'data/blade/dataset/fits'/experiment
    rpm, grams, voltage, excluded = load_table(args.table)
    thrust = grams*9.81/1000
    kf, predicted, metrics = fit_square(rpm, thrust)
    query = None
    if args.rpm is not None:
        query = dict(rpm=args.rpm, lookup_thrust_n=float(lookup(args.rpm, rpm, thrust)),
                     square_thrust_n=float(kf*(args.rpm*2*np.pi/60)**2))
    report = dict(
        experiment_id=experiment, source=metadata['source'],
        source_table_sha256=hashlib.sha256(args.table.read_bytes()).hexdigest(),
        motor=metadata.get('motor'), propeller=metadata.get('propeller'), voltage_v=voltage,
        source_type=metadata.get('source_type'), measurement_date=metadata.get('measurement_date'),
        source_notes=metadata.get('notes'),
        voltage_summary='Mean of valid point voltages; measured supply may vary.',
        applicability='Reference table only; installed motor/propeller specifications must be verified.',
        gravity_m_s2=9.81, valid_rpm=[float(rpm[0]), float(rpm[-1])], used_points=len(rpm), excluded=excluded,
        model='F_N = k_f * omega_rad_s^2', fit_method='unweighted least squares through origin',
        k_f_n_per_rad_s_squared=kf, k_f_n_per_rpm_squared=float(kf*(2*np.pi/60)**2),
        in_sample_metrics=metrics,
        interpolation='piecewise linear in RPM, no extrapolation',
        limitation='No independent validation, dynamic response, throttle mapping or torque coefficient inferred.',
        lookup_table=[dict(rpm=float(r),omega_rad_s=float(r*2*np.pi/60),thrust_n=float(f)) for r,f in zip(rpm,thrust)])
    if query: report['query'] = query
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'report.yaml').write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    with (args.output/'fit_points.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['rpm','omega_rad_s','thrust_n','predicted_thrust_n','predicted_minus_measured_n','relative_error'])
        for r,y,yhat in zip(rpm,thrust,predicted):
            w.writerow([r,r*2*np.pi/60,y,yhat,yhat-y,(yhat-y)/y])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    grid = np.linspace(rpm[0], rpm[-1], 400)
    fig, ax = plt.subplots(2,1,figsize=(8,7),sharex=True,gridspec_kw={'height_ratios':[3,1]})
    ax[0].plot(grid,kf*(grid*2*np.pi/60)**2,label='Least squares: F = k_f omega²',color='#148b82')
    ax[0].plot(grid,lookup(grid,rpm,thrust),'--',label='Piecewise linear lookup',color='#626ac7')
    ax[0].scatter(rpm,thrust,color='#ce7537',label='Source table',zorder=3)
    ax[0].set_ylabel('Thrust (N)');ax[0].legend()
    ax[0].set_title(f'{experiment} / {voltage:g} V\n'
                    f'{metadata.get("motor") or "Motor unknown"} / '
                    f'{metadata.get("propeller") or "Propeller unknown"}')
    ax[1].axhline(0,color='gray',lw=.8)
    ax[1].plot(rpm,predicted-thrust,'o-',color='#148b82')
    ax[1].set_ylabel('Fit error (N)');ax[1].set_xlabel('Mechanical speed (rpm)')
    for a in ax:a.grid(alpha=.25)
    fig.tight_layout()
    for extension in ('png','pdf'):fig.savefig(args.output/f'thrust_fit.{extension}',dpi=180)
    print(f'有效点 {len(rpm)}，缺少 RPM 排除 {len(excluded)}；有效范围 {rpm[0]:g}–{rpm[-1]:g} rpm')
    print(f'k_f = {kf:.9g} N/(rad/s)^2; RMSE = {metrics["rmse_n"]:.6g} N')
    if query: print(query)
    print(f'结果：{args.output}')


if __name__ == '__main__':
    main()
