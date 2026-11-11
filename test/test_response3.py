"""用独立三阶微分方程验证预测、参数恢复和单航次训练/验证流程。"""
import csv
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from scipy.linalg import expm
from scipy.signal import StateSpace, lsim

from brct1order_regression import CHANNELS, INPUTS, OUTPUTS, read_csv
from brct1order_process import split_pulse_groups
from brct3order_regression import (PARAMETER_KEYS, predict, fit_channel, score,
                                   identify_response, sampling_resolved)


def independent(t, u, initial, tau, wn, zeta, delay, gain, bias):
    """直接积分展开后的 y''' 方程，不使用被测代码的串联状态或特征分解。"""
    a = np.array([[0., 1, 0], [0, 0, 1],
                  [-wn*wn/tau, -(2*zeta*wn/tau+wn*wn), -(1/tau+2*zeta*wn)]])
    b = np.array([[0.], [0.], [wn*wn/tau]])
    dt = t[1]-t[0]
    shift = round(delay/dt)
    if np.allclose(np.diff(t), dt, atol=1e-13) and abs(shift*dt-delay) < 1e-12:
        delayed = np.r_[np.full(shift, u[0]), u[:-shift]] if shift else u
        return lsim(StateSpace(a, b, [[1., 0, 0]], [[0.]]), gain*delayed+bias,
                    t-t[0], X0=[initial, 0, 0], interp=False)[1]
    # 非均匀观测及非整数采样时延：以输入跳变为边界精确分段积分。
    events = t[1:][np.diff(u) != 0]+delay
    knots = np.unique(np.r_[t, events[(events > t[0]) & (events < t[-1])]])
    augmented = np.zeros((4, 4)); augmented[:3, :3] = a; augmented[:3, 3:] = b
    state = np.array([initial, 0., 0., gain*u[0]+bias]); values = [initial]
    for start, end in zip(knots[:-1], knots[1:]):
        index = max(0, np.searchsorted(t, (start+end)/2-delay, side='right')-1)
        state[3] = gain*u[index]+bias
        state = expm(augmented*(end-start)) @ state
        values.append(state[0])
    return np.array(values)[np.searchsorted(knots, t)]


class ThirdOrderTest(unittest.TestCase):
    def test_prediction_including_repeated_poles(self):
        t = np.arange(301)*.004
        u = np.where(t < .2, .1, np.where(t < .6, .3, -.2))
        for tau, damping in [(.04, .3), (.04, 1.), (.04, 1.5),
                             (1/30, 1.), (1/30, 1.+1e-9), (1/(30*(1.5+np.sqrt(1.25))), 1.5)]:
            with self.subTest(tau=tau, damping=damping):
                params = (tau, 30., damping, .024, 1.1, .02)
                np.testing.assert_allclose(predict(t, u, .08, *params),
                                           independent(t, u, .08, *params), atol=1e-8)

    def test_fractional_delay_irregular_sampling_and_constant_input(self):
        rng = np.random.default_rng(8)
        t = np.r_[0., np.cumsum(rng.uniform(.003, .006, 240))]+4.
        for u in [np.where(t < 4.25, .1, np.where(t < 4.7, -.2, .3)), np.full(len(t), .2)]:
            params = (.04, 25., .4, .0173, 1.05, .01)
            np.testing.assert_allclose(predict(t, u, -.08, *params),
                                       independent(t, u, -.08, *params), atol=1e-9)

    def test_fit_and_independent_holdout(self):
        truth = (.04, 25., .35, .024, 1.05, .01)
        def pulse(width):
            t = np.arange(351)*.004
            u = np.where(t < .2, 0., np.where(t < .2+width, .25,
                                             np.where(t < .2+2*width, -.25, 0.)))
            return t, u, independent(t, u, .01, *truth)
        fit, boundary = fit_channel([pulse(.2), pulse(.35)], 2)
        self.assertFalse(boundary)
        np.testing.assert_allclose(fit, truth, rtol=.01, atol=.001)
        self.assertLess(score([pulse(.28)], fit)['normalized_rmse'], .005)

    def test_sampling_must_resolve_extra_pole_and_overdamped_fast_pole(self):
        t = np.arange(100)*.01; pulses = [(t, t*0, t*0)]
        self.assertTrue(sampling_resolved(pulses, (.04, 25., .35)))
        self.assertFalse(sampling_resolved(pulses, (.003, 25., .35)))
        self.assertFalse(sampling_resolved(pulses, (.04, 100., 2.)))

    def test_same_data_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            a, b = Path(root)/'a.csv', Path(root)/'b.csv'
            a.write_text('identical'); b.write_text('identical')
            with self.assertRaises(ValueError):
                identify_response(a, b)

    def test_single_flight_split_fit_report_and_plot(self):
        rng = np.random.default_rng(7)
        rows, truths, group = [], [], 0
        for axis in range(4):
            truths.append((.04+.005*axis, 25.+axis, .35, .02,
                           32. if axis == 3 else 1.05, .21 if axis == 3 else .01))
        for repeat in range(3):
            for width in (.2, .35, .5):
                for axis, channel in enumerate(CHANNELS):
                    for sign in (1, -1):
                        t = np.arange(round((width+.5)/.01)+1)*.01
                        base, amp = (.3, .015) if axis == 3 else (0., .2)
                        u = base+sign*amp*((t >= .25) & (t < .25+width))
                        truth = truths[axis]
                        y = independent(t, u, truth[4]*base+truth[5], *truth)
                        y += rng.normal(0., .0003, len(t))
                        for i, stamp in enumerate(t):
                            row = dict(t=group*2.+stamp, cmd_wx=0., cmd_wy=0., cmd_wz=0., cmd_thrust=.3,
                                       wx=0., wy=0., wz=0., fz=9.81, segment=group, channel=channel,
                                       valid=1, phase='pulse', sequence_complete=0, pulse_hold_s=width)
                            row[INPUTS[axis]], row[OUTPUTS[axis]] = u[i], y[i]
                            rows.append(row)
                        group += 1
        rows[-1]['sequence_complete'] = 1
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source, train, validation = [root/name for name in ('all.csv', 'train.csv', 'validation.csv')]
            with source.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
            split = split_pulse_groups(source, train, validation)
            self.assertEqual(len(split['training_groups']), 48)
            self.assertEqual(len(split['validation_groups']), 24)
            self.assertFalse(set(read_csv(train)['t']) & set(read_csv(validation)['t']))
            # 只替代未改动的 ULog 提取层，实际执行新命令行入口、划分、拟合与落盘。
            from brct3order_process import main
            from brct1order_plot import predict_third
            ulog = root/'synthetic.ulg'; ulog.write_bytes(b'synthetic acquisition fixture')
            output = root/'report.yaml'
            with patch('sys.argv', ['brct3order_process.py', '--ulog', str(ulog), '--output', str(output)]), \
                    patch('brct1order_process.ulog_to_csv', side_effect=lambda _, target: shutil.copyfile(source, target)), \
                    patch('brct1order_plot.predict_third', wraps=predict_third) as prediction:
                main()
                self.assertEqual(prediction.call_count, 72)
            report = yaml.safe_load(output.read_text())
            self.assertEqual(report['model'], 'third_order_with_delay')
            self.assertEqual(report['data_split'], split)
            self.assertEqual(report['source_logs'], [str(ulog)])
            for suffix in ('_train.csv', '_validation.csv'):
                self.assertTrue((root/('report'+suffix)).is_file())
            self.assertTrue(report['passed'], report)
            for row, truth in zip(report['channels'], truths):
                np.testing.assert_allclose([row[key] for key in PARAMETER_KEYS], truth, rtol=.08, atol=.006)
            self.assertGreater((root/'report.png').stat().st_size, 1000)
            # 验证误差小也不能掩盖参数触边或未分辨出的额外极点。
            with patch('brct3order_regression.fit_channel', side_effect=lambda p, i: (truths[i], True)):
                self.assertFalse(identify_response(train, validation)['passed'])
            with patch('brct3order_regression.sampling_resolved', return_value=False), \
                    patch('brct3order_regression.fit_channel', side_effect=lambda p, i: (truths[i], False)):
                self.assertFalse(identify_response(train, validation)['passed'])


if __name__ == '__main__':
    unittest.main()
