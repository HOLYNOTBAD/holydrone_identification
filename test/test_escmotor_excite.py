"""台架交互工具的离线故障路径测试；无真实串口或飞控。"""
import importlib.util
import io
from pathlib import Path
import unittest
import tempfile
from types import SimpleNamespace
from unittest.mock import patch, MagicMock
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('escmotor_excite', ROOT/'scripts/escmotor/escmotor_excite.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def config():
    c = yaml.safe_load((ROOT/'config/escmotor/escmotor.yaml').read_text())
    c.update(motor_index=1, voltage_source='esc', require_esc_current=True, precheck_command=.1, voltages_v=[12.], commands=[.2], voltage_tolerance_v=.2)
    return c


class FakeMav:
    def __init__(self):
        self.sent = []
    def command_long_send(self, *args):
        self.sent.append(args)
    def heartbeat_send(self, *args):
        pass


class FakeLink:
    target_system = 1
    target_component = 1
    def __init__(self):
        self.mav = FakeMav()


class Tests(unittest.TestCase):
    def test_publish_only_after_successful_completion(self):
        for failure in ('none', 'run', 'stop', 'logger', 'interrupt'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp)/'escmotor_experiment'/'session'
                link = MagicMock()
                link.recv_match.return_value = SimpleNamespace(
                    autopilot=12, get_srcSystem=lambda: 1, get_srcComponent=lambda: 1)
                bench = MagicMock()
                bench.logger_owned = True
                if failure == 'stop':
                    bench.stop.side_effect = RuntimeError('stop failed')
                if failure == 'logger':
                    bench.shell.side_effect = RuntimeError('logger failed')
                def run(*args):
                    self.assertFalse(output.exists())
                    if failure == 'run':
                        raise RuntimeError('experiment failed')
                    if failure == 'interrupt':
                        raise KeyboardInterrupt()
                modules = {
                    'pymavlink': SimpleNamespace(mavutil=SimpleNamespace(mavlink_connection=lambda *a, **k: link)),
                    'escmotor_mavlink': SimpleNamespace(enable_esc_status=lambda _: None),
                }
                argv = ['excite', '--config', str(ROOT/'config/escmotor/escmotor.yaml'), '--output', str(output)]
                with patch.dict('sys.modules', modules), patch.object(m.sys, 'argv', argv), \
                     patch.object(m, 'prompt'), patch.object(m.glob, 'glob', return_value=['fake-port']), \
                     patch.object(m, 'Bench', return_value=bench), patch.object(m, 'run', side_effect=run):
                    if failure == 'none':
                        m.main()
                        self.assertTrue((output/'config.yaml').is_file())
                        self.assertTrue((output/'events.jsonl').is_file())
                    else:
                        with self.assertRaises(KeyboardInterrupt if failure == 'interrupt' else RuntimeError):
                            m.main()
                        self.assertFalse(output.exists())
                        self.assertFalse(output.parent.exists())
                link.close.assert_called_once()

    def test_enter_wait_drains_telemetry_without_zero_timeout(self):
        c = config()
        link = FakeLink()
        b = m.Bench(link, c, io.StringIO(), clock=lambda: 10.)
        b.hb = (10., type('HB', (), {'base_mode': 0})())
        b.esc = dict(received=0., rpm=0., voltage=12., current=0., time_us=1)
        class Esc:
            index = 0
            time_usec = 2
            rpm = [0.]
            voltage = [12.]
            current = [0.]
            def get_type(self): return 'ESC_STATUS'
            def get_srcSystem(self): return 1
        pending = [Esc()]
        def recv_match(**kw):
            # Match pymavlink: zero timeout expires before it reads buffered data.
            if kw.get('timeout') == 0:
                return None
            self.assertEqual(kw, {'blocking': False})
            return pending.pop(0) if pending else None
        link.recv_match = recv_match
        with patch.object(m.select, 'select', side_effect=[([], [], []), ([object()], [], [])]), \
                patch.object(m.sys, 'stdin', io.StringIO('\n')):
            m.prompt('test', b)
        b.healthy()
        self.assertEqual(b.esc['time_us'], 2)
        self.assertEqual(b.esc['received'], 10.)

    def test_sequence_order_and_voltage_rejection(self):
        c = config()
        c.update(voltages_v=[12., 14.], commands=[.2, .3], repetitions=2)
        class SequenceBench:
            def __init__(self):
                self.calls = []
                self.esc = dict(voltage=12.)
            def start_log(self, topics): pass
            def command(self, *args): pass
            def clock(self): return 0
            def healthy(self): pass
            def supply_voltage(self): return self.esc["voltage"]
            def wait_stopped(self): pass
            def observe(self, *args): pass
            def pulse(self, u, duration, **labels): self.calls.append((u, labels))
        b = SequenceBench()
        def answer(message, bench):
            if '调整为 14.0' in message:
                bench.esc['voltage'] = 14.
        with patch.object(m, 'prompt', answer):
            m.run(b, c)
        self.assertEqual(len(b.calls), 9)
        self.assertEqual([(u, fields['target_voltage_v']) for u, fields in b.calls[1:]],
                         [(u, v) for v in [12., 14.] for u in [.2, .3] for _ in range(2)])
        b = SequenceBench()
        b.esc['voltage'] = 10.
        with patch.object(m, 'prompt', lambda *args: None):
            with self.assertRaisesRegex(RuntimeError, '不符'):
                m.run(b, c)
        self.assertEqual(len(b.calls), 1)  # only precheck, no experiment pulse

    def test_bdshot_voltage_source_and_optional_current(self):
        c = config(); c.update(voltage_source='battery', require_esc_current=False)
        b = m.Bench(FakeLink(), c, io.StringIO(), clock=lambda: 10.)
        b.hb = (10., type('HB', (), {'base_mode':0})())
        b.esc = dict(received=10., rpm=0., voltage=0., current=float('nan'), time_us=1)
        b.battery = dict(received=10., voltage=14.)
        b.healthy()
        self.assertEqual(b.supply_voltage(), 14.)
        b.battery['received'] = 0.
        with self.assertRaisesRegex(RuntimeError, '电压回传缺失或过期'):
            b.healthy()

    def test_config_requires_explicit_experiment(self):
        empty = config()
        empty['commands'] = []
        with self.assertRaises(ValueError):
            m.validate(empty)
        self.assertIs(m.validate(config())['motor_index'], 1)

    def test_battery_message_selection_and_voltage_diagnostics(self):
        c = config(); c.update(voltage_source='battery', require_esc_current=False)
        link = FakeLink()
        b = m.Bench(link, c, io.StringIO(), clock=lambda: 10.)
        b.hb = (10., type('HB', (), {'base_mode': 0})())
        b.esc = dict(received=10., rpm=0., voltage=12., current=0., time_us=1)
        class Battery:
            id = c['battery_id'] + 1
            voltages = [16000] + [65535] * 9
            voltages_ext = [0] * 4
            def get_type(self): return 'BATTERY_STATUS'
            def get_srcSystem(self): return 1
        message = Battery()
        link.recv_match = lambda **kw: message
        b.poll()
        # An unrelated battery and valid ESC voltage must not mask missing selected battery data.
        with self.assertRaisesRegex(RuntimeError, 'BATTERY_STATUS.*未收到数据'):
            b.healthy()
        message.id = c['battery_id']
        b.poll()
        b.healthy()
        self.assertEqual(b.supply_voltage(), 16.)
        message.voltages = [65535] * 10
        b.poll()
        with self.assertRaisesRegex(RuntimeError, 'BATTERY_STATUS.*电压无效：0.0 V'):
            b.healthy()
        b.battery['voltage'] = 16.
        b.esc['rpm'] = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'ESC_STATUS.rpm 无效'):
            b.healthy()

    def test_reject_nonfinite_and_out_of_range(self):
        for key, value in [('commands', [1.1]), ('voltages_v', [float('nan')]),
                           ('esc_index', -1), ('pulse_duration_s', -1), ('precheck_command', None)]:
            c = config(); c[key] = value
            with self.assertRaises(ValueError, msg=key):
                m.validate(c)

    def test_armed_and_stale_feedback_block(self):
        b = m.Bench(FakeLink(), config(), io.StringIO(), clock=lambda: 10.)
        hb = type('Heartbeat', (), {'base_mode': 128})()
        b.hb = (10., hb)
        with self.assertRaisesRegex(RuntimeError, '已解锁'):
            b.healthy(False)
        hb.base_mode = 0
        b.esc = dict(received=0, rpm=500., voltage=12., current=1., time_us=1)
        with self.assertRaisesRegex(RuntimeError, '过期'):
            b.healthy()

    def test_rejected_test_always_releases_motor(self):
        link = FakeLink()
        b = m.Bench(link, config(), io.StringIO(), clock=lambda: 10.)
        b.healthy = lambda *a: None
        b.check_log = lambda: None
        b.observe = lambda *a: None
        def reject(*a, **kw):
            raise RuntimeError('rejected')
        b.command = reject
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            b.pulse(.2, 1.)
        release = link.mav.sent[-1]
        self.assertEqual(release[2], 310)
        self.assertEqual(release[5], 0)  # timeout=0 releases control
        self.assertFalse(b.active)

    def test_preexisting_log_not_stopped(self):
        b = m.Bench(FakeLink(), config(), io.StringIO())
        calls = []
        def shell(cmd):
            calls.append(cmd)
            return 'Full File Logging Running:' if cmd == 'logger status' else 'esc_status 0\n'
        b.shell = shell
        with self.assertRaisesRegex(RuntimeError, '已有日志'):
            b.start_log('esc_status 0')
        self.assertNotIn('logger stop', calls)

    def test_missing_logger_topic_blocks(self):
        b = m.Bench(FakeLink(), config(), io.StringIO())
        b.shell = lambda cmd: 'esc_status 100\n'
        with self.assertRaisesRegex(RuntimeError, '日志配置'):
            b.start_log('esc_status 0')

    def test_logger_start_waits_for_file_backend(self):
        now = [0.]
        b = m.Bench(FakeLink(), config(), io.StringIO(), clock=lambda: now[0])
        replies = iter(['Not logging', 'Mavlink Logging Running (Full log)',
                        'Full File Logging Running: /fs/microsd/log/test.ulg'])
        b.shell = lambda cmd: next(replies)
        def observe(duration, telemetry):
            self.assertFalse(telemetry)
            now[0] += duration
        b.observe = observe
        b.check_log(wait_s=5.)
        self.assertEqual(now[0], .5)

    def test_logger_timeout_reports_status_and_runtime_check_is_immediate(self):
        now = [0.]
        b = m.Bench(FakeLink(), config(), io.StringIO(), clock=lambda: now[0])
        b.shell = lambda cmd: 'INFO [logger] Not logging'
        b.observe = lambda duration, telemetry: now.__setitem__(0, now[0]+duration)
        with self.assertRaisesRegex(RuntimeError, '原始回复：\\nINFO.*Not logging'):
            b.check_log(wait_s=5.)
        self.assertEqual(now[0], 5.)
        with self.assertRaisesRegex(RuntimeError, 'Not logging'):
            b.check_log()
        self.assertEqual(now[0], 5.)


if __name__ == '__main__':
    unittest.main()
