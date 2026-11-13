"""离线验证启动参数恢复和留出验证的隔离。"""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/escmotor'))
from escmotor_fit import fit_startup, fit_unified, load_pulses
from escmotor_identification import predict


def pulse(number, steady=5000):
    t = np.arange(-.1, 1.001, .005)
    return dict(experiment_id='test', start_fcu_s=number*2, pulse_id=f'test_p{number:03}',
                u=.3, target_v=16.8, steady=steady, voltage=16.7,
                t=t, y=predict(t, 0, steady, .08, .025), on=t>=0, before=t<0,
                initial=0, sample_sha256='test')


class FitTests(unittest.TestCase):
    def test_unified_recovers_shared_model_and_excludes_validation_from_map(self):
        pulses = []
        strata = {}
        for i, command in enumerate([.2, .4, .6, .8]):
            steady = -2000*command**2+10000*command+1000
            items = [pulse(i*2+j+1, steady) for j in range(2)]
            for p in items:
                p['u'] = command
            strata[command] = items
            pulses.extend(items)
        refs = [fit_startup(items) for items in strata.values()]
        baseline = fit_unified(pulses, refs)
        np.testing.assert_allclose(baseline['coefficients'], [-2000, 10000, 1000], rtol=1e-10)
        self.assertAlmostEqual(baseline['tau_s'], .08, places=5)
        self.assertAlmostEqual(baseline['delay_s'], .025, places=5)
        self.assertEqual(baseline['training_count'], 4)
        self.assertEqual(baseline['validation_count'], 4)
        for items in strata.values():
            items[-1]['steady'] *= 3
            items[-1]['y'] *= 3
        altered = fit_unified(pulses, [fit_startup(items) for items in strata.values()])
        np.testing.assert_array_equal(altered['coefficients'], baseline['coefficients'])
        self.assertEqual(altered['tau_s'], baseline['tau_s'])
        self.assertEqual(altered['delay_s'], baseline['delay_s'])
        self.assertGreater(altered['validation_metrics']['rmse_rpm'], 1000)
        self.assertEqual(altered['comparison'][1]['unified']['sample_count'], altered['comparison'][1]['local']['sample_count'])

    def test_unified_singletons_have_no_validation(self):
        pulses = [pulse(i+1, 3000+1000*i) for i in range(3)]
        for i, p in enumerate(pulses):
            p['u'] = .2+.1*i
        report = fit_unified(pulses, [fit_startup([p]) for p in pulses])
        self.assertEqual(report['validation_count'], 0)
        self.assertIsNone(report['validation_metrics'])
        self.assertIsNone(report['comparison'][1]['unified'])
        self.assertIn('本组无独立验证脉冲', report['flags'])

    def test_recover_and_holdout_is_not_used(self):
        first = pulse(1)
        baseline = fit_startup([first])
        second = pulse(2, steady=9000)
        result = fit_startup([first, second])
        self.assertAlmostEqual(result['tau_s'], .08, places=5)
        self.assertAlmostEqual(result['delay_s'], .025, places=5)
        self.assertEqual(result['steady_rpm'], 5000)
        self.assertAlmostEqual(result['tau_s'], baseline['tau_s'], places=10)
        self.assertEqual(result['validation_count'], 1)
        self.assertGreater(result['validation_metrics']['rmse_rpm'], 1000)
        self.assertEqual([t['split'] for t in result['traces']], ['train', 'validation'])

    def test_singleton_has_no_validation(self):
        result = fit_startup([pulse(1)])
        self.assertEqual(result['validation_count'], 0)
        self.assertIsNone(result['validation_metrics'])
        self.assertIn('单次脉冲，无独立验证', result['flags'])

    def test_precheck_and_flagged_are_excluded_without_loading_samples(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with (folder/'pulses.csv').open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['pulse_id', 'phase', 'quality', 'reasons'])
                writer.writeheader()
                writer.writerows([dict(pulse_id='trial', phase='precheck', quality='pass', reasons=''),
                                  dict(pulse_id='bad', phase='experiment', quality='flagged', reasons='not_steady')])
            valid, excluded = load_pulses(folder)
            self.assertEqual(valid, [])
            self.assertEqual([e['pulse_id'] for e in excluded], ['trial', 'bad'])


if __name__ == '__main__':
    unittest.main()
