"""不连接硬件的斜坡、交互、数据配对与退出路径验证。"""
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/blade'))
import blade_excite as blade
from blade_fit import load_table, lookup


def config():
    c = yaml.safe_load((ROOT/'config/blade/blade.yaml').read_text())
    c['commands'] = [.10, .12]
    return c


class SimBench(blade.BladeBench):
    def __init__(self, c):
        self.t = 0.0
        super().__init__(Mock(), c, io.StringIO(), io.StringIO(), clock=lambda: self.t)
        self.sent = []
        self.stop_count = 0
        self.esc = dict(rpm=0, time_us=0)

    def tick(self):
        self.t += .05
        self.sent.append((self.t, self.u, self.phase))
        self.esc = dict(rpm=self.u*20000, time_us=int(self.t*1e6))
        self.samples.append(dict(received=self.t, time_us=int(self.t*1e6),
                                 rpm=self.esc['rpm'], voltage=16.8, current=0))

    def poll(self, timeout=.05):
        self.t += .05

    def healthy(self, telemetry=True):
        pass

    def command(self, *args, **kwargs):
        pass

    def wait_stopped(self):
        pass

    def stop(self):
        self.stop_count += 1
        self.active = False


class BladeTests(unittest.TestCase):
    def test_configuration_requires_work_range(self):
        c = config()
        self.assertIs(blade.validate(c), c)
        for key, value in [('commands', []), ('commands', [float('nan')]),
                           ('ramp_up_per_s', 0), ('steady_window_s', 20),
                           ('motor_index', True)]:
            with self.subTest(key=key, value=value):
                bad = dict(c, **{key: value})
                with self.assertRaises(ValueError):
                    blade.validate(bad)

    def test_ramp_both_directions_and_no_jump(self):
        bench = SimBench(config())
        bench.ramp(.12)
        bench.ramp(.08)
        bench.ramp(0)
        previous_t, previous_u = 0, 0
        for t, u, _ in bench.sent:
            self.assertLessEqual(abs(u-previous_u), .02*(t-previous_t)+1e-9)
            self.assertGreaterEqual(u, 0)
            self.assertLessEqual(u, .12)
            previous_t, previous_u = t, u
        self.assertAlmostEqual(bench.u, 0)
        self.assertGreaterEqual(bench.t, 12)

    def test_window_excludes_old_points_and_zero_current_is_missing(self):
        c = config()
        samples = [dict(received=i*.1, rpm=6000, voltage=16.8, current=0) for i in range(101)]
        samples[0]['rpm'] = 90000
        stats = blade.summarize(samples, 10, c)
        self.assertEqual(stats['quality'], 'pass')
        self.assertEqual(stats['rpm'], 6000)
        self.assertEqual(stats['current_a'], '')
        self.assertEqual(stats['sample_count'], 21)
        bad = blade.summarize(samples[::10], 10, c)
        self.assertEqual(bad['quality'], 'fail')
        self.assertIn('telemetry_gap', bad['quality_reason'])

    def test_interaction_rejects_bad_input_then_saves_window(self):
        bench = SimBench(config())
        bench.u = .3
        text = io.StringIO('nan\n-1\nabc\n300\n')
        with patch.object(blade.sys, 'stdin', text), patch.object(blade.select, 'select', return_value=([text], [], [])):
            # 前几个错误输入不停止电机；最后一行提交时已有完整窗口。
            original = bench.tick
            def tick():
                for _ in range(41):
                    original()
            bench.tick = tick
            result = bench.read_thrust()
        self.assertEqual(result['thrust_g'], 300)
        self.assertAlmostEqual(result['rpm'], 6000)

    def test_timeout_skip_quit_eof(self):
        for text, expected in [('s\n', None), ('q\n', KeyboardInterrupt), ('', KeyboardInterrupt)]:
            bench = SimBench(config())
            inp = io.StringIO(text)
            with patch.object(blade.sys, 'stdin', inp), patch.object(blade.select, 'select', return_value=([inp], [], [])):
                if expected is None:
                    self.assertIsNone(bench.read_thrust())
                else:
                    with self.assertRaises(expected):
                        bench.read_thrust()
        bench = SimBench(dict(config(), input_timeout_s=.2))
        with patch.object(blade.select, 'select', return_value=([], [], [])):
            with self.assertRaises(TimeoutError):
                bench.read_thrust()

    def test_complete_sequence_persists_and_stops(self):
        bench = SimBench(config())
        with tempfile.TemporaryFile(mode='w+', newline='') as file:
            points = csv.DictWriter(file, fieldnames=blade.POINT_FIELDS)
            points.writeheader()
            inp = io.StringIO('42\n60\n')
            def ready(*args):
                return ([inp], [], []) if len(bench.samples) >= 45 else ([], [], [])
            with patch.object(blade.sys, 'stdin', inp), patch.object(blade.select, 'select', side_effect=ready):
                blade.run(bench, bench.c, points, file)
            file.seek(0)
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 2)
        self.assertEqual([r['thrust_g'] for r in rows], ['42.0', '60.0'])
        self.assertEqual(bench.stop_count, 1)
        self.assertAlmostEqual(bench.u, 0)

    def test_tick_rejects_voltage_and_renews_lease(self):
        bench = blade.BladeBench(Mock(), config(), io.StringIO(), io.StringIO(), clock=lambda: 5)
        bench.poll = Mock()
        bench.healthy = Mock()
        bench.supply_voltage = Mock(return_value=16.8)
        bench.command = Mock()
        bench.u = .25
        bench.tick()
        bench.command.assert_called_once_with(310, [.25, .6, 0, 0, 2], timeout=.35)
        bench.supply_voltage.return_value = 14
        with self.assertRaises(RuntimeError):
            bench.tick()
        self.assertEqual(bench.command.call_count, 1)

    def test_fit_measured_voltage_variation_and_duplicate_rpm(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'points.csv'
            path.write_text('rpm,thrust_g,voltage_v\n6000,200,16.7\n6000,220,16.8\n8000,400,16.6\n')
            rpm, grams, voltage, excluded = load_table(path)
            self.assertAlmostEqual(voltage, 16.7)
            self.assertEqual(float(lookup(6000, rpm, grams)), 210)
            with self.assertRaises(ValueError):
                lookup(9000, rpm, grams)

    def test_main_abort_stops_and_retains_saved_points(self):
        import blade_mavlink
        from pymavlink import mavutil
        for error in (KeyboardInterrupt(), TimeoutError('input timeout'), RuntimeError('feedback lost')):
            with self.subTest(error=type(error).__name__), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp)/'bench_test'
                cfg = Path(tmp)/'config.yaml'
                cfg.write_text(yaml.safe_dump(dict(config(), port='/dev/fake')))
                link = Mock()
                hb = Mock(autopilot=12, base_mode=0)
                hb.get_srcSystem.return_value = 1
                hb.get_srcComponent.return_value = 1
                link.recv_match.return_value = hb
                def abort(bench, c, writer, file):
                    writer.writerow(dict(point_id=1, rpm=6000, thrust_g=300, voltage_v=16.8))
                    file.flush()
                    raise error
                with patch.object(sys, 'argv', ['blade_excite', '--config', str(cfg), '--output', str(output)]), \
                     patch('builtins.input', return_value=''), \
                     patch.object(blade_mavlink, 'enable_esc_status'), \
                     patch.object(mavutil, 'mavlink_connection', return_value=link), \
                     patch.object(blade, 'run', side_effect=abort):
                    with self.assertRaises(type(error)):
                        blade.main()
                stop_args = link.mav.command_long_send.call_args.args
                self.assertEqual(stop_args[2], 310)
                self.assertTrue(math.isnan(stop_args[4]))
                self.assertEqual(stop_args[5], 0)
                link.close.assert_called_once()
                self.assertEqual(json.loads((output/'status.json').read_text())['status'], 'aborted')
                with (output/'static_thrust.csv').open() as file:
                    self.assertEqual(list(csv.DictReader(file))[0]['thrust_g'], '300')


if __name__ == '__main__':
    unittest.main()
