#!/usr/bin/env python3
"""USB/PX4 台架交互激励；原始数据由飞控 ULog 保存，不自动解锁。"""
import argparse
from collections import deque
import glob
import io
import json
import math
import os
from pathlib import Path
import re
import select
import sys
import tempfile
import time
import uuid

import yaml


def validate(c):
    def positive(key):
        x = c[key]
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0:
            raise ValueError(f'{key} 必须为有限正数')
    for k in ('connection_timeout_s', 'telemetry_timeout_s', 'heartbeat_timeout_s',
              'voltage_tolerance_v', 'precheck_duration_s', 'pulse_duration_s',
              'pre_record_s', 'post_record_s', 'min_running_rpm', 'stop_rpm',
              'stop_timeout_s', 'steady_window_s', 'steady_relative_span'):
        positive(k)
    for key in ('motor_index', 'repetitions', 'min_samples'):
        if type(c[key]) is not int or c[key] < 1:
            raise ValueError(f'{key} 必须为正整数')
    if c['motor_index'] > 12 or type(c['esc_index']) is not int or not 0 <= c['esc_index'] < 12:
        raise ValueError('电机编号为 1–12，ESC 遥测索引为 0–11')
    if not c['voltages_v'] or not c['commands']:
        raise ValueError('请先在 YAML 中填写 voltages_v 和 commands')
    for v in c['voltages_v']:
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('电压列表必须为有限正数')
    for u in [c['precheck_command'], *c['commands']]:
        if isinstance(u, bool) or not isinstance(u, (int, float)) or not math.isfinite(u) or not 0 < u <= 1:
            raise ValueError('请填写 precheck_command；所有激励值必须在 (0, 1]')
    if c['steady_window_s'] > min(c['pulse_duration_s'], c['precheck_duration_s']):
        raise ValueError('steady_window_s 不能大于激励时长')
    if c.get('voltage_source', 'esc') not in ('esc', 'battery'):
        raise ValueError('voltage_source 必须为 esc 或 battery')
    if type(c.get('require_esc_current', True)) is not bool:
        raise ValueError('require_esc_current 必须为布尔值')
    if type(c.get('battery_id', 0)) is not int or not 0 <= c.get('battery_id', 0) <= 255:
        raise ValueError('battery_id 必须为 0–255 的整数')
    return c


class Bench:
    def __init__(self, link, config, events, clock=time.monotonic):
        self.link, self.c, self.events, self.clock = link, config, events, clock
        self.hb = None
        self.esc = None
        self.battery = None
        self.samples = deque(maxlen=100000)
        self.logger_owned = False
        self.active = False
        self.last_host_heartbeat = -1e9

    def event(self, kind, **fields):
        row = dict(event=kind, host_monotonic_s=self.clock(), host_unix_s=time.time(), **fields)
        if self.esc:
            row['last_esc_time_us'] = self.esc['time_us']
        self.events.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        self.events.flush()

    def poll(self, timeout=.05):
        now = self.clock()
        if now-self.last_host_heartbeat > 1:
            self.link.mav.heartbeat_send(6, 8, 0, 0, 0)  # GCS / invalid autopilot
            self.last_host_heartbeat = now
        # pymavlink 的 timeout=0 会在读取串口前直接超时；排空缓冲时必须
        # 使用不带 timeout 的非阻塞读取，否则等待 Enter 期间收不到任何数据。
        if timeout <= 0:
            m = self.link.recv_match(blocking=False)
        else:
            m = self.link.recv_match(blocking=True, timeout=timeout)
        if m is None or m.get_srcSystem() != self.link.target_system:
            return None
        if m.get_type() == 'HEARTBEAT' and m.get_srcComponent() == self.link.target_component:
            self.hb = (self.clock(), m)
        if m.get_type() == 'BATTERY_STATUS' and m.id == self.c.get('battery_id', 0):
            cells = [v for v in m.voltages if v != 65535]
            cells += [v for v in getattr(m, 'voltages_ext', []) if v not in (0, 65535)]
            self.battery = dict(received=self.clock(), voltage=sum(cells)/1000.)
        if m.get_type() == 'ESC_STATUS':
            i = self.c['esc_index']-m.index
            if 0 <= i < len(m.rpm):
                stamp = int(m.time_usec)
                if self.esc and stamp < self.esc['time_us']:
                    raise RuntimeError('飞控时间回退，结束本次记录')
                if self.esc is None or stamp > self.esc['time_us']:
                    self.esc = dict(received=self.clock(), time_us=stamp,
                                    rpm=float(m.rpm[i]), voltage=float(m.voltage[i]), current=float(m.current[i]))
                    sample = self.esc.copy()
                    if self.c.get('voltage_source', 'esc') == 'battery':
                        sample['voltage'] = self.battery['voltage'] if self.battery else float('nan')
                    self.samples.append(sample)
        return m

    def healthy(self, telemetry=True):
        if not self.hb or self.clock()-self.hb[0] > self.c['heartbeat_timeout_s']:
            raise RuntimeError('飞控心跳缺失或过期')
        if self.hb[1].base_mode & 128:
            raise RuntimeError('飞控已解锁；台架执行器测试要求未解锁')
        if telemetry:
            e = self.esc
            if not e or self.clock()-e['received'] > self.c['telemetry_timeout_s']:
                detail = '未收到数据' if not e else f'数据年龄={self.clock()-e["received"]:.2f} s，最后 RPM={e["rpm"]}'
                raise RuntimeError(f'ESC_STATUS 转速回传缺失或过期：esc_index={self.c["esc_index"]}；{detail}')
            if not math.isfinite(e['rpm']):
                raise RuntimeError(f'ESC_STATUS.rpm 无效：esc_index={self.c["esc_index"]}，值={e["rpm"]}')
            self.supply_voltage()
            if self.c.get('require_esc_current', True) and (not math.isfinite(e['current']) or e['current'] < 0):
                raise RuntimeError(f'ESC_STATUS.current 无效：esc_index={self.c["esc_index"]}，值={e["current"]} A；require_esc_current=true')

    def supply_voltage(self):
        source = self.c.get('voltage_source', 'esc')
        sample = self.battery if source == 'battery' else self.esc
        field = (f'BATTERY_STATUS.voltages（battery_id={self.c.get("battery_id", 0)}）'
                 if source == 'battery' else f'ESC_STATUS.voltage（esc_index={self.c["esc_index"]}）')
        if sample is None or self.clock()-sample['received'] > self.c['telemetry_timeout_s']:
            detail = '未收到数据' if sample is None else f'数据年龄={self.clock()-sample["received"]:.2f} s，最后电压={sample["voltage"]} V'
            raise RuntimeError(f'{field} 电压回传缺失或过期：{detail}')
        voltage = sample['voltage']
        if not math.isfinite(voltage) or voltage <= 0:
            raise RuntimeError(f'{field} 电压无效：{voltage} V')
        return voltage

    def observe(self, duration, telemetry=True):
        end = self.clock()+duration
        while self.clock() < end:
            self.poll(min(.05, max(0, end-self.clock())))
            self.healthy(telemetry)

    def command(self, command, params, timeout=1.0):
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component,
                                        command, 0, *(list(params)+[0]*(7-len(params))))
        end = self.clock()+timeout
        while self.clock() < end:
            m = self.poll()
            self.healthy(False)
            if m and m.get_type() == 'COMMAND_ACK' and m.command == command and m.get_srcComponent() == self.link.target_component:
                if m.result != 0:
                    raise RuntimeError(f'命令 {command} 被拒绝，MAV_RESULT={m.result}')
                return
        raise RuntimeError(f'命令 {command} 未收到确认')

    def stop(self):
        # NaN = disarmed output；timeout=0 释放测试控制。不要用 u=0 代替停止。
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component,
                                        310, 0, float('nan'), 0, 0, 0, self.c['motor_index'], 0, 0)
        self.active = False
        self.event('stop_sent')

    def wait_stopped(self):
        end = self.clock()+self.c['stop_timeout_s']
        fresh_after = self.clock()
        while self.clock() < end:
            self.poll()
            self.healthy()
            if self.esc['received'] >= fresh_after and abs(self.esc['rpm']) <= self.c['stop_rpm']:
                return
        raise RuntimeError('停止后转速仍未降到停止门槛')

    def shell(self, command):
        # 使用串口控制通道调用 PX4 NSH；不与本机 shell 拼接用户输入。
        token = 'BENCH_'+uuid.uuid4().hex[:10]
        payload = ('\n'+command+'\necho '+token+'\n').encode()
        for start in range(0, len(payload), 70):
            chunk = payload[start:start+70]
            self.link.mav.serial_control_send(10, 6, 0, 0, len(chunk), list(chunk)+[0]*(70-len(chunk)))
        out = ''
        end = self.clock()+5
        while self.clock() < end:
            m = self.poll()
            if m and m.get_type() == 'SERIAL_CONTROL':
                out += bytes(m.data[:m.count]).decode(errors='replace').replace('\r', '')
                if re.search(r'(?m)^'+token+r'\s*$', out):
                    self.event('shell', command=command, response=out)
                    return out
        raise RuntimeError(f'PX4 shell 响应超时：{command}')

    def start_log(self, topics):
        content = self.shell('cat /fs/microsd/etc/logging/logger_topics.txt')
        for line in topics.splitlines():
            if line.strip() and not line.startswith('#') and not re.search(r'(?m)^'+re.escape(line.strip())+r'\s*$', content):
                raise RuntimeError('SD 卡缺少台架日志配置：请按 test/ESC_MOTOR.md 安装 logger_topics.txt')
        status = self.shell('logger status')
        if 'File Logging Running' in status:
            raise RuntimeError('已有日志正在记录；请结束原记录后再运行，避免截断已有实验')
        # 重启 logger 以加载已经安装的 topic 文件，不修改飞控参数。
        self.shell('logger stop')
        self.shell('logger start -b 64')
        self.logger_owned = True
        self.shell('logger on')
        # logger on 只设置覆盖标志；日志线程收到后续状态更新后才打开文件。
        self.check_log(wait_s=5.)

    def check_log(self, wait_s=0.):
        deadline = self.clock()+wait_s
        while True:
            status = self.shell('logger status')
            if 'Full File Logging Running' in status:
                print(status.strip())
                return
            if self.clock() >= deadline:
                raise RuntimeError('飞控未确认完整 ULog 正在写入 SD 卡；'
                                   f'logger status 原始回复：\n{status.strip()}')
            self.observe(min(.25, max(0., deadline-self.clock())), False)

    def pulse(self, u, duration, **labels):
        self.healthy()
        self.check_log()
        self.observe(self.c['pre_record_s'])
        self.samples.clear()
        self.event('pulse_start', command=u, duration_s=duration, **labels)
        start = self.clock()
        next_send = start
        self.active = True
        try:
            while self.clock()-start < duration:
                self.healthy()
                if self.clock() >= next_send:
                    # 每 0.2 s 续约；命令失联后 0.6 s 内由飞控超时释放。
                    self.command(310, [u, .6, 0, 0, self.c['motor_index']], timeout=.35)
                    next_send = self.clock()+.2
                self.poll()
        finally:
            self.stop()
        running = list(self.samples)
        self.observe(self.c['post_record_s'])
        self.wait_stopped()
        if len(running) < self.c['min_samples']:
            raise RuntimeError('独立 ESC 样本不足，检查遥测刷新率')
        if max(abs(s['rpm']) for s in running) < self.c['min_running_rpm']:
            raise RuntimeError('未检测到指定 ESC 转速上升，请核对电机与遥测索引')
        if self.c.get('require_esc_current', True) and max(s['current'] for s in running) <= 0:
            raise RuntimeError('运行期间电流始终为零，无法确认电流回传')
        tail = [s for s in running if s['received'] >= start+duration-self.c['steady_window_s']]
        mean = sum(abs(s['rpm']) for s in tail)/len(tail) if tail else 0
        stable = len(tail) >= 3 and mean > 0 and (max(abs(s['rpm']) for s in tail)-min(abs(s['rpm']) for s in tail))/mean <= self.c['steady_relative_span']
        self.event('pulse_complete', samples=len(running), steady=stable,
                   rpm_mean_tail=mean, voltage_min=min(s['voltage'] for s in running),
                   voltage_max=max(s['voltage'] for s in running), **labels)
        print(f'完成：{len(running)} 个遥测样本，末段平均 {mean:.0f} rpm，稳态检查：{stable}')


def prompt(message, bench=None):
    print(message+'\n按 Enter 继续，输入 q 退出：', flush=True)
    if bench:
        # 等待操作者时仍处理遥测，避免把缓冲的旧数据当成新反馈。
        while not select.select([sys.stdin], [], [], .05)[0]:
            for _ in range(500):
                if bench.poll(0) is None:
                    break
            bench.healthy(False)
    line = sys.stdin.readline()
    if not line or line.strip():
        raise KeyboardInterrupt


def run(bench, c):
    if c.get('voltage_source', 'esc') == 'battery':
        print(f'供电电压来源：飞控 BATTERY_STATUS（battery_id={c.get("battery_id", 0)}）；不检查 ESC 电压。')
    else:
        print('供电电压来源：ESC_STATUS。')
    print(f'转速来源：ESC_STATUS（esc_index={c["esc_index"]}）；'
          f'ESC 电流检查：{"开启" if c.get("require_esc_current", True) else "关闭（不作为实验准入条件）"}。')
    bench.start_log(Path(__file__).with_name('escmotor_logger_topics.txt').read_text())
    # 请求只用于在线检查；ULog 的记录频率独立配置。
    if c.get('voltage_source', 'esc') == 'battery':
        bench.command(511, [147, 100000])  # BATTERY_STATUS，10 Hz 在线电压检查
    bench.command(511, [291, 20000])  # MAVLINK_MSG_ID_ESC_STATUS
    end = bench.clock()+c['connection_timeout_s']
    while (bench.esc is None or (c.get('voltage_source', 'esc') == 'battery' and bench.battery is None)) and bench.clock() < end:
        bench.poll()
    bench.healthy()
    bench.wait_stopped()
    prompt(f'将试转电机 {c["motor_index"]}，u={c["precheck_command"]}，持续 {c["precheck_duration_s"]} s', bench)
    bench.observe(.2)
    bench.pulse(c['precheck_command'], c['precheck_duration_s'], phase='precheck')
    for v in c['voltages_v']:
        prompt(f'电机已停止。请将电源调整为 {v} V', bench)
        bench.observe(.5)
        for u in c['commands']:
            for repeat in range(1, c['repetitions']+1):
                prompt(f'目标 {v} V；电机 {c["motor_index"]}；u={u}；{c["pulse_duration_s"]} s；第 {repeat}/{c["repetitions"]} 次', bench)
                bench.observe(.3)
                bench.wait_stopped()
                if abs(bench.supply_voltage()-v) > c['voltage_tolerance_v']:
                    raise RuntimeError(f'实测 {bench.supply_voltage():.2f} V，与目标 {v} V 不符')
                bench.pulse(u, c['pulse_duration_s'], phase='experiment', target_voltage_v=v, repeat=repeat)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='新的实验目录，保存配置与事件清单')
    parser.add_argument('--check-config', action='store_true', help='只检查配置，不连接飞控')
    args = parser.parse_args()
    c = validate(yaml.safe_load(args.config.read_text()))
    if args.check_config:
        print('配置检查通过；未连接飞控。')
        return
    os.environ["MAVLINK20"] = "1"  # ESC_STATUS 的消息 ID 大于 255
    from pymavlink import mavutil
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from escmotor_mavlink import enable_esc_status
    enable_esc_status(mavutil)
    if args.output.exists():
        raise FileExistsError(f'实验目录已存在：{args.output}')
    prompt('请用 USB 将飞控连接到电脑；关闭占用该串口的其他程序。')
    port = c['port']
    if port == 'auto':
        ports = glob.glob('/dev/serial/by-id/*') or glob.glob('/dev/ttyACM*')
        if len(ports) != 1:
            raise RuntimeError(f'无法唯一确定 USB 端口：{ports}；请在 YAML 指定 port')
        port = ports[0]
    link = mavutil.mavlink_connection(port, baud=115200, source_system=255, dialect='common')
    bench = None
    # 实验期间仅在内存中保存清单，成功完成并关闭日志后才发布目录。
    with io.StringIO() as events:
        try:
            deadline = time.monotonic()+c['connection_timeout_s']
            heartbeat = None
            while time.monotonic() < deadline:
                m = link.recv_match(type='HEARTBEAT', blocking=True, timeout=.2)
                if m and m.autopilot == 12:  # MAV_AUTOPILOT_PX4
                    heartbeat = m
                    break
            if heartbeat is None:
                raise RuntimeError('未收到 PX4 飞控心跳')
            link.target_system = heartbeat.get_srcSystem()
            link.target_component = heartbeat.get_srcComponent()
            bench = Bench(link, c, events)
            bench.hb = (time.monotonic(), heartbeat)
            bench.healthy(False)
            bench.event('connected', port=port, system=link.target_system, component=link.target_component)
            print(f'已连接 PX4：system={link.target_system}, component={link.target_component}')
            run(bench, c)
        except BaseException as error:
            if bench:
                bench.event('aborted', reason=str(error) or type(error).__name__)
            raise
        finally:
            try:
                if bench:
                    try:
                        bench.stop()
                    finally:
                        if bench.logger_owned:
                            bench.shell('logger off')
            finally:
                link.close()
        bench.event('experiment_complete')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # 在同一文件系统中准备完整文件，再一次性发布实验目录。
        with tempfile.TemporaryDirectory(prefix='.escmotor-', dir=args.output.parent.parent) as staging:
            ready = Path(staging)/'session'
            ready.mkdir()
            (ready/'config.yaml').write_text(yaml.safe_dump(c, allow_unicode=True))
            (ready/'events.jsonl').write_text(events.getvalue())
            if args.output.exists():
                raise FileExistsError(f'实验目录已存在：{args.output}')
            ready.rename(args.output)
        print(f'全部实验完成，清单已保存至 {args.output}。原始数据在飞控 SD 卡，日志路径见 events.jsonl。')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n实验已中止。')
        raise SystemExit(130)
    except Exception as error:
        print(f'实验终止：{error}')
        raise SystemExit(1)
