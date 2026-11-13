#!/usr/bin/env python3
"""将 blade_experiment 原始表格整理为独立实验索引和统一单位的样本表。"""
import csv
import hashlib
import json
import math
from pathlib import Path


def write_csv(path, fields, rows):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    root = Path(__file__).resolve().parents[2]
    output = root / 'data/blade/dataset'
    experiments, samples = [], []
    fields = ['throttle_percent', 'voltage_v', 'current_a', 'rpm', 'thrust_g',
              'power_w', 'efficiency_g_per_w']
    detail_fields = ['point_id', 'repeat', 'submitted_unix_s', 'window_start_monotonic_s',
                     'window_end_monotonic_s', 'sample_count', 'rpm_std',
                     'rpm_relative_span', 'quality', 'quality_reason']
    for folder in sorted((root / 'data/blade/experiment').iterdir()):
        if not folder.is_dir():
            continue
        meta = json.loads((folder / 'metadata.json').read_text())
        if meta['experiment_id'] != folder.name:
            raise ValueError(f'{folder}: experiment_id 必须与目录名相同')
        table = folder / 'static_thrust.csv'
        with table.open(newline='') as f:
            rows = list(csv.DictReader(f))
        review_file = folder / 'point_reviews.json'
        reviews = json.loads(review_file.read_text()) if review_file.exists() else {}
        valid_count = 0
        for index, row in enumerate(rows, 1):
            values = {key: float(row[key]) if row.get(key, '').strip() else '' for key in fields}
            for key, value in values.items():
                if value != '' and (not math.isfinite(value) or value < 0):
                    raise ValueError(f'{table}:{index}: {key} 无效')
            if any(values[key] == '' or values[key] <= 0 for key in ('voltage_v', 'thrust_g')):
                raise ValueError(f'{table}:{index}: 缺少有效电压或推力')
            valid = values['rpm'] != '' and values['rpm'] > 0 and row.get('quality', 'pass') == 'pass'
            reason = '' if valid else row.get('quality_reason') or 'missing_or_zero_rpm'
            review = reviews.get(str(index), {})
            review_status = review.get('status', 'unreviewed')
            if review_status in ('pending', 'rejected'):
                valid = False
                reason = ';'.join(filter(None, [reason, review.get('reason') or review_status]))
            valid_count += int(valid)
            samples.append(dict(experiment_id=folder.name, source_row=index, **values,
                                omega_rad_s=values['rpm'] * 2 * math.pi / 60 if values['rpm'] != '' else '',
                                thrust_n=values['thrust_g'] * 9.81 / 1000,
                                **{key: row.get(key, '') for key in detail_fields},
                                review_status=review_status, fit_eligible=valid, exclusion_reason=reason))
        status_file = folder / 'status.json'
        status = json.loads(status_file.read_text())['status'] if status_file.exists() else 'source_table'
        status_reason = json.loads(status_file.read_text()).get('reason', '') if status_file.exists() else ''
        experiments.append(dict(**meta, source_table=str(table.relative_to(root)),
                                session_status=status,
                                session_reason=status_reason,
                                source_table_sha256=hashlib.sha256(table.read_bytes()).hexdigest(),
                                sample_count=len(rows), fit_point_count=valid_count))
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / 'experiments.csv', ['experiment_id', 'source_type', 'source', 'motor',
              'propeller', 'measurement_date', 'notes', 'source_table', 'source_table_sha256',
              'sample_count', 'fit_point_count', 'session_status', 'session_reason'], experiments)
    write_csv(output / 'samples.csv', ['experiment_id', 'source_row', *fields, 'omega_rad_s',
              'thrust_n', *detail_fields, 'review_status', 'fit_eligible', 'exclusion_reason'], samples)
    payload = json.dumps(dict(experiments=experiments, samples=samples), ensure_ascii=False, indent=2)
    (output / 'catalog.json').write_text(payload + '\n', encoding='utf-8')
    template = Path(__file__).with_name('blade_overview.html').read_text(encoding='utf-8')
    (output / 'overview.html').write_text(template.replace('__CATALOG_JSON__', payload.replace('<', '\\u003c')), encoding='utf-8')
    print(f'已整理 {len(experiments)} 个实验，{len(samples)} 行样本：{output}')


if __name__ == '__main__':
    main()
