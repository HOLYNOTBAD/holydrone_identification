"""拷贝自 escmotor_excite 的 PX4 通信、遥测、停转与 ULog 支持；独立维护。"""
from collections import deque
import json
import math
import re
import time
import uuid


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

