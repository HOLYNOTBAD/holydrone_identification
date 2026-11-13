#!/usr/bin/env python3
"""将同一台架会话的 ULog、events.jsonl 与配置整理为不重叠的脉冲数据集。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def intervals(data, function):
    """重建测试控制的保持区间；续约不能改变幅值或跨越超时。"""
    result, active = [], None
    for j in range(len(data['timestamp'])):
        if int(data['function'][j]) != function:
            continue
        t = float(data['timestamp'][j])*1e-6
        value = float(data['value'][j])
        if active and t > active['expires']:
            active['end'] = active['expires']; result.append(active); active = None
        enabled = int(data['action'][j]) == 1 and np.isfinite(value) and value > 0
        if active and (not enabled or abs(value-active['u']) > 1e-6):
            active['end'] = t; result.append(active); active = None
        if enabled:
            if active is None:
                active = dict(start=t, u=value)
            active['expires'] = t+float(data['timeout_ms'][j])/1000
    # 缺少释放指令的最后一段不视为完整实验。
    return result


def split_groups(groups):
    strata = {}
    for g in groups:
        strata.setdefault((g['target_voltage_v'], round(g['u'], 6)), []).append(g)
    for key, items in strata.items():
        if len(items) < 2:
            raise ValueError(f'{key} 只有 {len(items)} 个有效脉冲；每个组合至少需两个')
        for g in items[:-1]: g['split'] = 'train'
        items[-1]['split'] = 'validation'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ulog', type=Path, required=True)
    p.add_argument('--session', type=Path, required=True, help='含 config.yaml 和 events.jsonl 的采集目录')
    p.add_argument('--output', type=Path, required=True, help='新的 dataset 目录')
    args = p.parse_args()
    from pyulog import ULog
    config = yaml.safe_load((args.session/'config.yaml').read_text())
    events = [json.loads(line) for line in (args.session/'events.jsonl').read_text().splitlines() if line.strip()]
    log = ULog(str(args.ulog))
    if log.dropouts:
        raise ValueError('ULog 存在丢写记录；请先检查日志，当前工具不跨丢写区间拟合')
    def topic(name):
        return log.get_dataset(name, 0).data
    command = topic('actuator_test')
    pulses = intervals(command, 100+config['motor_index'])
    starts = [e for e in events if e['event'] == 'pulse_start']
    if len(starts) != len(pulses):
        raise ValueError('ULog 脉冲数与事件清单不一致；请使用同一完整会话的文件')
    esc = topic('esc_status')
    prefix = f'esc[{config["esc_index"]}].'
    # 使用逐 ESC 样本时间，去除同一遥测被重复发布的记录。
    raw_t = np.asarray(esc[prefix+'timestamp'], dtype=float)*1e-6
    if np.any(np.diff(raw_t) < 0):
        raise ValueError('ESC 样本时钟回退')
    unique = np.r_[True, np.diff(raw_t) > 0] & (raw_t > 0)
    t = raw_t[unique]
    omega = np.asarray(esc[prefix+'esc_rpm'], float)[unique]*2*np.pi/60
    voltage = np.asarray(esc[prefix+'esc_voltage'], float)[unique]
    current = np.asarray(esc[prefix+'esc_current'], float)[unique]
    if config.get('voltage_source', 'esc') == 'battery':
        battery = None
        for instance in range(8):
            try:
                candidate = log.get_dataset('battery_status', instance).data
            except (KeyError, IndexError):
                continue
            if np.all(np.asarray(candidate['id']) == config.get('battery_id', 0)+1):
                battery = candidate
                break
        if battery is None:
            raise ValueError('ULog 缺少对应 battery_status.id 的电源电压')
        bt = np.asarray(battery['timestamp'], float)*1e-6
        if len(bt) == 0 or np.any(np.diff(bt) <= 0):
            raise ValueError('电源时间戳无效')
        indices = np.searchsorted(bt, t, side='right')-1
        previous = np.maximum(indices, 0)
        voltage = np.asarray(battery['voltage_v'], float)[previous]
        valid_voltage = (indices >= 0) & (t-bt[previous] <= config['telemetry_timeout_s'])
    else:
        valid_voltage = np.ones(len(t), dtype=bool)
    checked = [t, omega, voltage]
    if config.get('require_esc_current', True): checked.append(current)
    if not np.isfinite(np.column_stack(checked)).all():
        raise ValueError('ESC 数据含非有限值')
    rows, groups, excluded = [], [], []
    for i, (pulse, event) in enumerate(zip(pulses, starts)):
        if abs(pulse['u']-event['command']) > 1e-5:
            raise ValueError('日志与实验清单的激励幅值不匹配')
        anchor = event.get('last_esc_time_us', 0)*1e-6
        if not 0 <= pulse['start']-anchor <= 3:
            raise ValueError('事件与 ULog 的飞控时间不匹配')
        # 每个 start 后、下一个 start 前必须有 complete；失败段不能进入数据集。
        position = next(k for k, e in enumerate(events) if e is event)
        following = []
        for e in events[position+1:]:
            if e['event'] == 'pulse_start': break
            following.append(e)
        if event.get('phase') != 'experiment' or not any(e['event'] == 'pulse_complete' for e in following):
            excluded.append(dict(segment=i, reason='precheck_or_incomplete')); continue
        begin, end = pulse['start'], pulse['end']
        mask = (t >= begin-config['pre_record_s']) & (t <= end+config['post_record_s'])
        # 不能借用相邻脉冲的样本。
        if i: mask &= t > pulses[i-1]['end']+config['post_record_s']
        if i+1 < len(pulses): mask &= t < pulses[i+1]['start']-config['pre_record_s']
        ids = np.flatnonzero(mask)
        during = ids[(t[ids] >= begin) & (t[ids] < end)]
        tail = during[t[during] >= end-config['steady_window_s']]
        reason = None
        if len(during) < config['min_samples'] or len(tail) < 3 or not np.any(t[ids] < begin):
            reason = 'insufficient_samples'
        elif np.max(np.diff(t[ids])) > config['telemetry_timeout_s']:
            reason = 'telemetry_gap'
        elif not np.all(valid_voltage[ids]) or np.any(voltage[ids] <= 0) or (config.get('require_esc_current', True) and np.any(current[ids] < 0)):
            reason = 'invalid_voltage_or_current'
        elif np.mean(omega[tail]) <= 0 or np.ptp(omega[tail])/np.mean(omega[tail]) > config['steady_relative_span']:
            reason = 'not_steady'
        if reason:
            excluded.append(dict(segment=i, reason=reason)); continue
        groups.append(dict(segment=i, u=pulse['u'], target_voltage_v=event['target_voltage_v'],
                           voltage_v=float(np.mean(voltage[tail])), omega_ss_rad_s=float(np.mean(omega[tail])),
                           duration_s=end-begin, start_fcu_s=begin))
        for j in ids:
            rows.append(dict(segment=i, t_s=float(t[j]-begin), command=pulse['u'] if begin <= t[j] < end else 0.,
                             omega_rad_s=float(omega[j]), voltage_v=float(voltage[j]), current_a=float(current[j]),
                             phase='pre' if t[j] < begin else 'on' if t[j] < end else 'post'))
    if not groups: raise ValueError('没有可用的完整稳态脉冲')
    split_groups(groups)
    lookup = {g['segment']: g for g in groups}
    for row in rows: row['split'] = lookup[row['segment']]['split']
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output/'samples.csv').open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    report = dict(schema='escmotor_dataset_v1', groups=groups, excluded=excluded,
                  voltage_source=config.get('voltage_source', 'esc'), battery_id=config.get('battery_id', 0),
                  current_source='esc_status (optional; zero may mean unavailable)',
                  source_ulog=str(args.ulog.resolve()), source_sha256=hashlib.sha256(args.ulog.read_bytes()).hexdigest(),
                  session=str(args.session.resolve()), motor_index=config['motor_index'], esc_index=config['esc_index'],
                  rpm_assumption='PX4 esc_rpm is mechanical RPM; verify ESC driver before use',
                  split='last complete pulse per target voltage/command is held out')
    (args.output/'dataset.yaml').write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    print(f'数据集：{args.output}；有效 {len(groups)} 组，排除 {len(excluded)} 组')


if __name__ == '__main__':
    main()
