#!/usr/bin/env python3
"""PX4 油门斜坡 + 手动推力读数；本机即时保存，异常退出释放测试输出。"""
import argparse
import csv
from datetime import datetime
import glob
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import sys
import time

import yaml

# 支持与 escmotor 一致的 python -I 启动入口。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from blade_bench import Bench


POINT_FIELDS = ['point_id', 'repeat', 'throttle_percent', 'submitted_unix_s',
                'window_start_monotonic_s', 'window_end_monotonic_s', 'sample_count',
                'rpm', 'rpm_std', 'rpm_relative_span', 'voltage_v', 'current_a',
                'thrust_g', 'quality', 'quality_reason']
RAW_FIELDS = ['received', 'time_us', 'rpm', 'voltage', 'current', 'command', 'phase']


def validate(c):
    for key in ('connection_timeout_s', 'telemetry_timeout_s', 'heartbeat_timeout_s',
                'voltage_v', 'voltage_tolerance_v', 'ramp_up_per_s', 'ramp_down_per_s',
                'settle_s', 'steady_window_s', 'steady_relative_span', 'input_timeout_s',
                'min_running_rpm', 'stop_rpm', 'stop_timeout_s'):
        value = c[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} 必须为有限正数')
    for key in ('repetitions', 'min_samples', 'motor_index'):
        if type(c[key]) is not int or c[key] < 1:
            raise ValueError(f'{key} 必须为正整数')
    if c['motor_index'] > 12 or type(c['esc_index']) is not int or not 0 <= c['esc_index'] < 12:
        raise ValueError('motor_index 为 1–12；esc_index 为 0–11')
    if type(c['battery_id']) is not int or not 0 <= c['battery_id'] <= 255:
        raise ValueError('battery_id 必须为 0–255 整数')
    if c['voltage_source'] not in ('battery', 'esc'):
        raise ValueError('voltage_source 必须为 battery 或 esc')
    for key in ('require_esc_current', 'record_ulog'):
        if type(c[key]) is not bool:
            raise ValueError(f'{key} 必须为布尔值')
    if not isinstance(c['commands'], list) or not c['commands']:
        raise ValueError('请先在 config/blade/blade.yaml 填写工作区间 commands；例如 [0.25, 0.30, 0.35]，仅为格式示例')
    for u in c['commands']:
        if isinstance(u, bool) or not isinstance(u, (int, float)) or not math.isfinite(u) or not 0 < u <= 1:
            raise ValueError('commands 必须为 (0,1] 内的有限数值')
    if c['steady_window_s'] > c['settle_s'] or c['min_samples'] < 3:
        raise ValueError('steady_window_s 必须 <= settle_s，min_samples 必须 >= 3')
    if max(c['ramp_up_per_s'], c['ramp_down_per_s']) > 1:
        raise ValueError('油门变化率不能超过 1/s')
    return c


def summarize(samples, end, c):
    start = end-c['steady_window_s']
    window = [s for s in samples if start <= s['received'] <= end]
    reason = []
    if len(window) < c['min_samples']:
        reason.append('insufficient_samples')
    gaps = [b['received']-a['received'] for a, b in zip(window, window[1:])]
    if not window or max([window[0]['received']-start, end-window[-1]['received'], *gaps]) > c['telemetry_timeout_s']:
        reason.append('telemetry_gap')
    rpms = [abs(s['rpm']) for s in window]
    volts = [s['voltage'] for s in window]
    mean_rpm = statistics.mean(rpms) if rpms else 0
    span = (max(rpms)-min(rpms))/mean_rpm if mean_rpm > 0 else 0
    if not all(math.isfinite(x) for x in rpms) or mean_rpm < c['min_running_rpm']:
        reason.append('not_running_or_invalid_rpm')
    if span > c['steady_relative_span']:
        reason.append('unstable_rpm')
    valid_volts = volts and all(math.isfinite(x) and x > 0 for x in volts)
    if not valid_volts:
        reason.append('invalid_voltage')
    currents = [s['current'] for s in window]
    # 可选电流全零可能是未回传；不要把它记录为实测零电流。
    valid_current = currents and all(math.isfinite(x) and x >= 0 for x in currents) and any(x > 0 for x in currents)
    if c['require_esc_current'] and not valid_current:
        reason.append('invalid_current')
    return dict(window_start_monotonic_s=start, window_end_monotonic_s=end,
                sample_count=len(window), rpm=mean_rpm if math.isfinite(mean_rpm) else '',
                rpm_std=statistics.pstdev(rpms) if rpms and all(math.isfinite(x) for x in rpms) else '',
                rpm_relative_span=span if math.isfinite(span) else '',
                voltage_v=statistics.mean(volts) if valid_volts else '',
                current_a=statistics.mean(currents) if valid_current else '',
                quality='fail' if reason else 'pass', quality_reason=';'.join(reason))


class BladeBench(Bench):
    def __init__(self, link, config, events, raw, clock=time.monotonic):
        super().__init__(link, config, events, clock)
        self.raw = raw
        self.raw_writer = csv.DictWriter(raw, fieldnames=RAW_FIELDS)
        self.raw_writer.writeheader()
        self.u = 0.0
        self.phase = 'idle'
        self.next_send = 0.0

    def poll(self, timeout=.05):
        previous = self.esc['time_us'] if self.esc else None
        m = super().poll(timeout)
        if self.esc and self.esc['time_us'] != previous:
            sample = self.samples[-1]
            if self.c['voltage_source'] == 'battery' and (
                    not self.battery or self.clock()-self.battery['received'] > self.c['telemetry_timeout_s']):
                sample['voltage'] = float('nan')
            row = dict(sample, command=self.u if self.active else '', phase=self.phase)
            self.raw_writer.writerow({k: '' if isinstance(v, float) and not math.isfinite(v) else v for k, v in row.items()})
            self.raw.flush()
        return m

    def tick(self):
        self.poll(.01)
        self.healthy()
        if abs(self.supply_voltage()-self.c['voltage_v']) > self.c['voltage_tolerance_v']:
            raise RuntimeError('带载电压偏离目标，停止实验')
        if self.clock() >= self.next_send:
            self.active = True
            self.command(310, [self.u, .6, 0, 0, self.c['motor_index']], timeout=.35)
            self.next_send = self.clock()+.1

    def ramp(self, target):
        self.phase = 'ramp'
        self.event('ramp_start', initial=self.u, target=target)
        previous = self.clock()
        while abs(target-self.u) > 1e-10:
            now = self.clock()
            # 卡顿后不补发大幅跳变；单步最多按 0.1 秒的斜率变化。
            dt = min(max(now-previous, 0), .1)
            previous = now
            rate = self.c['ramp_up_per_s'] if target > self.u else self.c['ramp_down_per_s']
            self.u += math.copysign(min(abs(target-self.u), rate*dt), target-self.u)
            self.tick()
        # 确认目标指令已经实际发送，再开始稳定等待。
        self.next_send = 0
        self.tick()
        self.event('ramp_complete', command=self.u)

    def hold(self, duration):
        self.phase = 'settle'
        deadline = self.clock()+duration
        while self.clock() < deadline:
            self.tick()

    def read_thrust(self):
        self.phase = 'measurement'
        self.samples.clear()  # 统计窗口只使用提示开始后的测量阶段
        deadline = self.clock()+self.c['input_timeout_s']
        print(f'油门 {self.u*100:g}%｜RPM {abs(self.esc["rpm"]):.0f}；请观察推力稳定后输入克数（g）。'
              '\n输入 s 跳过，q 停止；回车提交：', flush=True)
        while self.clock() < deadline:
            self.tick()
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            line = sys.stdin.readline()
            if not line or line.strip().lower() == 'q':
                raise KeyboardInterrupt
            if line.strip().lower() == 's':
                return None
            try:
                thrust = float(line.strip())
                if not math.isfinite(thrust) or thrust <= 0:
                    raise ValueError
            except ValueError:
                print('请输入有限正数，单位 g（不带单位字符）；s 跳过，q 停止：', flush=True)
                continue
            end = self.clock()
            stats = summarize(self.samples, end, self.c)
            if stats['quality'] != 'pass':
                self.event('reading_rejected', thrust_g=thrust, **stats)
                print(f'当前窗口不可用：{stats["quality_reason"]}。请等待稳定后重新读数输入：', flush=True)
                continue
            return dict(thrust_g=thrust, submitted_unix_s=time.time(), **stats)
        raise TimeoutError('等待推力输入超时')


def run(bench, c, points, point_file):
    if c['record_ulog']:
        bench.start_log(Path(__file__).with_name('blade_logger_topics.txt').read_text())
    if c['voltage_source'] == 'battery':
        bench.command(511, [147, 100000])
    bench.command(511, [291, 20000])
    deadline = bench.clock()+c['connection_timeout_s']
    while (bench.esc is None or (c['voltage_source'] == 'battery' and bench.battery is None)) and bench.clock() < deadline:
        bench.poll()
    bench.healthy()
    bench.wait_stopped()
    point_id = 0
    for repeat in range(1, c['repetitions']+1):
        for u in c['commands']:
            point_id += 1
            print(f'第 {repeat}/{c["repetitions"]} 轮，第 {point_id} 个测量点，目标油门 {u*100:g}%', flush=True)
            bench.ramp(u)
            bench.hold(c['settle_s'])
            result = bench.read_thrust()
            if result is None:
                bench.event('point_skipped', point_id=point_id, command=u, repeat=repeat)
                continue
            row = dict(point_id=point_id, repeat=repeat, throttle_percent=u*100, **result)
            points.writerow(row)
            point_file.flush()
            os.fsync(point_file.fileno())
            bench.event('point_saved', **row)
            print(f'已保存：{result["thrust_g"]:g} g，{result["rpm"]:.0f} rpm', flush=True)
    bench.ramp(0)
    bench.stop()
    bench.wait_stopped()


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root/'config/blade/blade.yaml')
    parser.add_argument('--output', type=Path, help='新会话目录；默认 blade_experiment/bench_时间')
    parser.add_argument('--check-config', action='store_true')
    args = parser.parse_args()
    c = validate(yaml.safe_load(args.config.read_text()))
    if args.check_config:
        print('配置检查通过；未连接飞控。')
        return
    output = args.output or root/'data/blade/experiment'/datetime.now().strftime('bench_%Y%m%d_%H%M%S')
    if output.exists():
        raise FileExistsError(f'实验目录已存在：{output}')
    print(f'电机 {c["motor_index"]} / ESC {c["esc_index"]}；目标 {c["voltage_v"]} V；'
          f'油门档位 {c["commands"]}；爬升 {c["ramp_up_per_s"]*100:g} 百分点/秒。')
    if input('确认台架和推力读数准备好，按 Enter 开始；其他输入退出：').strip():
        return
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil
    from blade_mavlink import enable_esc_status
    enable_esc_status(mavutil)
    port = c['port']
    if port == 'auto':
        ports = glob.glob('/dev/serial/by-id/*') or glob.glob('/dev/ttyACM*')
        if len(ports) != 1:
            raise RuntimeError(f'无法唯一确定 USB 端口：{ports}；请在 YAML 指定 port')
        port = ports[0]
    output.mkdir(parents=True)
    (output/'config.yaml').write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
    metadata = dict(experiment_id=output.name, source_type='measurement',
                    source='PX4 ESC_STATUS + terminal manual thrust (g)',
                    motor=c.get('motor_model'), propeller=c.get('propeller_model'),
                    measurement_date=datetime.now().astimezone().isoformat(),
                    notes='手工推力与提交前遥测窗口配对；电流为 ESC 电流，空值表示无有效反馈。')
    (output/'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    status = dict(status='incomplete')
    (output/'status.json').write_text(json.dumps(status)+'\n')
    bench = link = None
    with (output/'events.jsonl').open('w') as events, (output/'telemetry.csv').open('w', newline='') as raw, (output/'static_thrust.csv').open('w', newline='') as point_file:
        points = csv.DictWriter(point_file, fieldnames=POINT_FIELDS)
        points.writeheader()
        point_file.flush()
        try:
            link = mavutil.mavlink_connection(port, baud=115200, source_system=255, dialect='common')
            bench = BladeBench(link, c, events, raw)
            deadline = time.monotonic()+c['connection_timeout_s']
            while time.monotonic() < deadline:
                m = link.recv_match(type='HEARTBEAT', blocking=True, timeout=.2)
                if m and m.autopilot == 12:
                    link.target_system, link.target_component = m.get_srcSystem(), m.get_srcComponent()
                    bench.hb = (time.monotonic(), m)
                    break
            bench.healthy(False)
            bench.event('connected', port=port)
            run(bench, c, points, point_file)
            bench.event('experiment_complete')
            status = dict(status='complete')
        except BaseException as error:
            status = dict(status='aborted', reason=str(error) or type(error).__name__)
            if bench:
                bench.event('aborted', reason=status['reason'])
            raise
        finally:
            try:
                if bench and bench.hb:
                    try:
                        bench.stop()
                    finally:
                        if bench.logger_owned:
                            bench.shell('logger off')
            except BaseException as error:
                status = dict(status='aborted', reason='cleanup: '+(str(error) or type(error).__name__))
                raise
            finally:
                if link:
                    link.close()
                (output/'status.json').write_text(json.dumps(status, ensure_ascii=False)+'\n')
                print(f'记录已保留：{output}', flush=True)


if __name__ == '__main__':
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n实验中止。')
        raise SystemExit(130)
    except Exception as error:
        print(f'实验终止：{error}')
        raise SystemExit(1)
