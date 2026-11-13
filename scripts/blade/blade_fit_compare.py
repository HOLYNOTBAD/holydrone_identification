#!/usr/bin/env python3
"""重建 Blade 数据集，生成四组平方模型拟合及离线 HTML 对比报告。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from blade_catalog import main as rebuild_catalog
from blade_fit import fit_square

COLORS = ['#087f8c', '#d47a26', '#6561c5', '#243b53']


def save_figure(fig, folder, stem):
    for extension in ('svg', 'png', 'pdf'):
        fig.savefig(folder/f'{stem}.{extension}', dpi=180, bbox_inches='tight')
    plt.close(fig)


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--today-experiment', default='bench_20261113_113812', help='今日组对应的明确实验编号，不自动随日期切换')
    parser.add_argument('--output', type=Path, default=root/'data/blade/fit')
    args = parser.parse_args()
    rebuild_catalog()
    source = root/'data/blade/dataset/catalog.json'
    catalog = json.loads(source.read_text())
    experiments = {e['experiment_id']: e for e in catalog['experiments']}
    source_ids = ['manufacturer', 'summer_measurement', args.today_experiment]
    for exp in source_ids:
        if exp not in experiments:
            parser.error(f'找不到实验：{exp}')
    labels = ['仅厂商数据', '仅暑假数据', '仅今日数据', '所有数据共同拟合']
    keys = ['manufacturer', 'summer', 'today', 'combined']
    english = ['Manufacturer', 'Summer measurement', 'Today measurement', 'Combined']
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    selected = [r for r in catalog['samples'] if r['experiment_id'] in source_ids]
    excluded = [r for r in selected if not r['fit_eligible']]
    groups = [[r for r in selected if r['fit_eligible'] and r['experiment_id'] == exp] for exp in source_ids]
    groups.append([r for group in groups for r in group])
    reports = []
    overlay, axis = plt.subplots(figsize=(10, 6))
    for index, group in enumerate(groups):
        if len(group) < 2:
            raise ValueError(f'{labels[index]}有效点不足')
        rpm = np.array([r['rpm'] for r in group], float)
        force = np.array([r['thrust_n'] for r in group], float)
        coefficient, predicted, metrics = fit_square(rpm, force)
        folder = output/keys[index]
        folder.mkdir(exist_ok=True)
        points = [dict(experiment_id=r['experiment_id'], source_row=r['source_row'],
                       rpm=r['rpm'], thrust_g=r['thrust_g'], thrust_n=r['thrust_n'],
                       voltage_v=r['voltage_v'], review_status=r['review_status'],
                       predicted_thrust_n=float(y), residual_n=float(y-r['thrust_n']))
                  for r, y in zip(group, predicted)]
        source_metrics = []
        for exp in source_ids:
            subset = [p for p in points if p['experiment_id'] == exp]
            if subset:
                errors = np.array([p['residual_n'] for p in subset])
                source_metrics.append(dict(experiment_id=exp, count=len(subset),
                                           rmse_n=float(np.sqrt(np.mean(errors**2))),
                                           mean_residual_n=float(np.mean(errors))))
        report = dict(key=keys[index], title=labels[index], count=len(group),
                      model='F_N = k_f * (RPM * 2*pi/60)^2', gravity_m_s2=9.81,
                      method='unweighted least squares through origin; each valid point has equal weight',
                      k_f=coefficient, k_f_units='N/(rad/s)^2',
                      k_rpm=coefficient*(2*np.pi/60)**2,
                      rpm_range=[float(min(rpm)), float(max(rpm))],
                      voltage_range=[min(r['voltage_v'] for r in group), max(r['voltage_v'] for r in group)],
                      metrics=metrics, source_metrics=source_metrics,
                      source_experiments=[experiments[s['experiment_id']] for s in source_metrics],
                      dataset_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        (folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        with (folder/'fit_points.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(points[0]))
            writer.writeheader()
            writer.writerows(points)
        report['points'] = points
        reports.append(report)
        grid = np.linspace(min(rpm), max(rpm), 400)
        curve = coefficient*(grid*2*np.pi/60)**2
        axis.plot(grid, curve, color=COLORS[index], label=english[index], lw=2,
                  linestyle='--' if index == 3 else '-')
        if index < 3:
            axis.scatter(rpm, force, color=COLORS[index], s=24, alpha=.7)
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={'height_ratios': [3, 1]})
        axes[0].plot(grid, curve, color=COLORS[index], lw=2, label='Fit: F = k_f omega^2')
        for j, exp in enumerate(source_ids):
            subset = [p for p in points if p['experiment_id'] == exp]
            if subset:
                axes[0].scatter([p['rpm'] for p in subset], [p['thrust_n'] for p in subset], color=COLORS[j], label=english[j], s=32)
                axes[1].scatter([p['rpm'] for p in subset], [p['residual_n'] for p in subset], color=COLORS[j], s=24)
        axes[0].set_title(f'{english[index]} | n={len(group)} | k_f={coefficient:.7g} N/(rad/s)^2')
        axes[0].set_ylabel('Thrust (N)')
        axes[0].legend()
        axes[1].axhline(0, color='#8898a8', lw=1)
        axes[1].set_ylabel('Predicted - measured (N)')
        axes[1].set_xlabel('Mechanical speed (RPM)')
        for ax in axes:
            ax.grid(alpha=.2)
        fig.tight_layout()
        save_figure(fig, folder, 'fit')
        print(f'{labels[index]}：{len(group)} 点，k_f={coefficient:.9g}，RMSE={metrics["rmse_n"]:.6g} N')
    axis.set(xlabel='Mechanical speed (RPM)', ylabel='Thrust (N)', title='Blade: four fits over their measured RPM ranges')
    axis.grid(alpha=.2)
    axis.legend()
    overlay.tight_layout()
    save_figure(overlay, output, 'comparison')
    bundle = dict(today_experiment=args.today_experiment, reports=reports, excluded=excluded)
    (output/'results.json').write_text(json.dumps(bundle, ensure_ascii=False, indent=2)+'\n')
    template = Path(__file__).with_name('blade_fit_report.html').read_text()
    (output/'index.html').write_text(template.replace('__RESULTS_JSON__', json.dumps(bundle, ensure_ascii=False).replace('<', '\\u003c')))
    (output/'README.md').write_text(
        '# Blade 四组拟合\n\n打开 `index.html` 离线查看四组系数、叠加曲线和逐点残差。今日组固定对应 `'+args.today_experiment+'`。\n\n'
        '在仓库根目录重建：\n\n```bash\nconda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit_compare.py\n```\n\n'
        '脚本先重建 dataset。每组子目录保存 report.json、fit_points.csv 及 SVG/PNG/PDF 图。联合拟合使用所选三个实验的全部有效点，每点等权，不是每个实验等权。\n\n'
        '模型 F=k_f×omega²，F 用 N，omega 用 rad/s。图中只绘制各组已测 RPM 区间；全部误差为样本内误差。厂家缺 RPM 的一行排除。不同来源硬件一致性未确认，联合系数是三组数据的共同近似。\n')
    print(f'离线报告：{output / "index.html"}')


if __name__ == '__main__':
    main()
