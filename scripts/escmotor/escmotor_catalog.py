#!/usr/bin/env python3
"""增量整理台架会话；保留全部脉冲，不在导入时划分训练/验证集。"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml
from pyulog import ULog
sys.path.insert(0, str(Path(__file__).resolve().parent))
from escmotor_expdataprocess import intervals

VERSION = 1


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False, encoding='utf-8') as f:
        f.write(content)
        temp = Path(f.name)
    temp.replace(path)


def csv_text(rows, fields):
    f = io.StringIO(newline='')
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)
    return f.getvalue()


def align(data, field, times, timeout):
    stamp = np.asarray(data['timestamp'], float)*1e-6
    order = np.argsort(stamp)
    stamp = stamp[order]
    idx = np.searchsorted(stamp, times, side='right')-1
    safe = np.maximum(idx, 0)
    values = np.asarray(data[field], float)[order][safe]
    return np.where((idx >= 0) & (times-stamp[safe] <= timeout), values, np.nan)


def extract(session, ulg, expid, fingerprint):
    c = yaml.safe_load((session/'config.yaml').read_text())
    events = [json.loads(x) for x in (session/'events.jsonl').read_text().splitlines() if x.strip()]
    if not any(e['event'] == 'experiment_complete' for e in events):
        raise ValueError('会话没有 experiment_complete')
    log = ULog(str(ulg))
    pulses = intervals(log.get_dataset('actuator_test').data, 100+c['motor_index'])
    starts = [i for i,e in enumerate(events) if e['event'] == 'pulse_start']
    if len(pulses) != len(starts):
        raise ValueError('ULog 与事件清单脉冲数不一致')
    esc = log.get_dataset('esc_status').data
    prefix = f'esc[{c["esc_index"]}].'
    raw = np.asarray(esc[prefix+'timestamp'], float)*1e-6
    if np.any(np.diff(raw) < 0):
        raise ValueError('ESC 时间戳回退')
    unique = (raw > 0) & np.r_[True, np.diff(raw) > 0]
    t = raw[unique]
    rpm = np.asarray(esc[prefix+'esc_rpm'], float)[unique]
    esc_current = np.asarray(esc[prefix+'esc_current'], float)[unique]
    battery = next((d.data for d in log.data_list if d.name == 'battery_status'
                    and np.all(d.data['id'] == c.get('battery_id',0)+1)), None)
    supply_i = align(battery, 'current_a', t, c['telemetry_timeout_s']) if battery is not None else np.full(t.shape,np.nan)
    v = (align(battery, 'voltage_v', t, c['telemetry_timeout_s']) if battery is not None else np.full(t.shape,np.nan)) if c.get('voltage_source') == 'battery' else np.asarray(esc[prefix+'esc_voltage'],float)[unique]
    summaries, samples = [], {}
    for n,(p,pos) in enumerate(zip(pulses,starts),1):
        ev=events[pos]
        if abs(p['u']-ev['command']) > 1e-5 or not 0 <= p['start']-ev.get('last_esc_time_us',0)*1e-6 <= 3:
            raise ValueError('ULog 与事件清单幅值/时间不匹配')
        following=events[pos+1:starts[n] if n<len(starts) else len(events)]
        begin,end=p['start'],p['end']
        left,right=begin-c['pre_record_s'],end+c['post_record_s']
        if n>1: left=max(left,(pulses[n-2]['end']+begin)/2)
        if n<len(pulses): right=min(right,(end+pulses[n]['start'])/2)
        ids=np.flatnonzero((t>=left)&(t<right))
        during=ids[(t[ids]>=begin)&(t[ids]<end)]
        tail=during[t[during]>=end-c['steady_window_s']]
        reasons=[]
        if not any(e['event']=='pulse_complete' for e in following): reasons.append('incomplete')
        if log.dropouts: reasons.append('ulog_dropout')
        if len(during)<c['min_samples'] or len(tail)<3: reasons.append('insufficient_samples')
        if not np.any(t[ids]<begin) or not np.any(t[ids]>=end): reasons.append('missing_before_or_after')
        if len(ids)>1 and np.max(np.diff(t[ids]))>c['telemetry_timeout_s']: reasons.append('telemetry_gap')
        if not np.all(np.isfinite(rpm[ids])) or not np.all(np.isfinite(v[ids]) & (v[ids]>0)): reasons.append('invalid_rpm_or_voltage')
        if len(during) and np.nanmax(np.abs(rpm[during]))<c['min_running_rpm']: reasons.append('not_running')
        steady=float(np.mean(rpm[tail])) if len(tail) else None
        if len(tail)>=3 and (steady<=0 or np.ptp(rpm[tail])/steady>c['steady_relative_span']): reasons.append('not_steady')
        pid=f'{expid}_p{n:03d}'
        rows=[]
        for j in ids:
            active=begin<=t[j]<end
            rows.append(dict(time_s=round(float(t[j]-begin),6),test_active=int(active),command=p['u'] if active else None,
                             phase='before' if t[j]<begin else 'hold' if active else 'after',rpm=float(rpm[j]),
                             voltage_v=float(v[j]) if np.isfinite(v[j]) else None,
                             supply_current_a=float(supply_i[j]) if np.isfinite(supply_i[j]) else None,
                             esc_current_a=float(esc_current[j]) if np.isfinite(esc_current[j]) else None))
        samples[pid]=rows
        summaries.append(dict(experiment_id=expid,pulse_id=pid,phase=ev.get('phase',''),motor_index=c['motor_index'],esc_index=c['esc_index'],
                              command=round(p['u'],6),target_voltage_v=ev.get('target_voltage_v'),repeat=ev.get('repeat'),
                              start_fcu_s=begin,duration_s=end-begin,sample_count=len(ids),steady_rpm=steady,
                              mean_voltage_v=float(np.mean(v[during])) if len(during) and np.all(np.isfinite(v[during])) else None,
                              quality='flagged' if reasons else 'pass',reasons=';'.join(reasons),samples_path=f'samples/{pid}.csv'))
    experiment=dict(experiment_id=expid,session=session.name,source_ulog=ulg.name,fingerprint=fingerprint,processing_version=VERSION,
                    complete=True,pulse_count=len(summaries),dropouts=len(log.dropouts),motor_index=c['motor_index'],esc_index=c['esc_index'],
                    motor_model=c.get('motor_model',''),propeller_model=c.get('propeller_model',''),esc_model=c.get('esc_model',''))
    return dict(experiment=experiment,pulses=summaries,samples=samples)


def main():
    root=Path(__file__).resolve().parents[2]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=root/'data/escmotor/experiment')
    parser.add_argument('--output',type=Path,default=root/'data/escmotor/dataset')
    args=parser.parse_args()
    out=args.output
    cache=out/'catalog.json'
    catalog=json.loads(cache.read_text()) if cache.exists() else dict(schema='escmotor_catalog_v1',entries={})
    errors=[]
    for session in sorted(args.source.iterdir()):
        if not session.is_dir() or not (session/'config.yaml').exists(): continue
        logs=sorted(session.glob('*.ulg'))
        if len(logs)!=1:
            errors.append(f'{session.name}: 需要唯一 ULog，当前 {len(logs)} 个');continue
        try:
            fingerprint=hashlib.sha256(b''.join(hashlib.sha256(p.read_bytes()).digest() for p in [logs[0],session/'config.yaml',session/'events.jsonl'])).hexdigest()
            expid='exp_'+hashlib.sha256(session.name.encode()).hexdigest()[:12]
            previous=catalog['entries'].get(expid)
            if previous and previous['experiment']['fingerprint']==fingerprint and previous['experiment']['processing_version']==VERSION:
                print('跳过',session.name);continue
            entry=extract(session,logs[0],expid,fingerprint)
            catalog['entries'][expid]=entry
            print('导入',session.name,len(entry['pulses']),'个脉冲')
        except Exception as e: errors.append(f'{session.name}: {e}')
    entries=list(catalog['entries'].values())
    if not entries: raise SystemExit('没有可导入会话：'+str(errors))
    experiments=[e['experiment'] for e in entries]
    pulses=[p for e in entries for p in e['pulses']]
    for e in entries:
        for pid,rows in e['samples'].items():
            if rows: write(out/'samples'/f'{pid}.csv',csv_text(rows,list(rows[0])))
    write(out/'experiments.csv',csv_text(experiments,list(experiments[0])))
    write(out/'pulses.csv',csv_text(pulses,list(pulses[0])))
    write(cache,json.dumps(catalog,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
    payload=json.dumps(entries,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    template=Path(__file__).with_name('escmotor_overview.html').read_text()
    write(out/'overview.html',template.replace('__CATALOG_DATA__',payload))
    write(out/'import_report.json',json.dumps(dict(errors=errors,experiments=len(entries),pulses=len(pulses)),ensure_ascii=False,indent=2)+'\n')
    print(f'共 {len(entries)} 次实验、{len(pulses)} 个脉冲；总览：{out}/overview.html')
    if errors: print('未导入：\n'+'\n'.join(errors));raise SystemExit(1)


if __name__=='__main__': main()
