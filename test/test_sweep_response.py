"""The sweep diagnostic must preserve phase/sign and identify only parameter ratios."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('sweep_response',Path(__file__).with_name('analyze_yaw_sweep.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class SweepResponse(unittest.TestCase):
    def test_sinusoidal_phasor_ignores_bias_and_slow_drift(self):
        t=np.arange(0,8,.004)
        signal=2*np.cos(2*np.pi*.5*t)-.5*np.sin(2*np.pi*.5*t)+.3+.02*t
        value,error=module.sinusoid(t,signal,.5)
        self.assertAlmostEqual(value.real,2.)
        self.assertAlmostEqual(value.imag,.5)
        self.assertLess(error,1e-10)

    def test_frequency_model_recovers_ratios_and_delay(self):
        frequencies=np.array([.5,1.,2.,3.])
        response=module.response_model(frequencies,8.,.3,.02)
        rows=[dict(frequency_hz=f,real=g.real,imag=g.imag) for f,g in zip(frequencies,response)]
        fit=module.fit_ratio(rows)
        self.assertTrue(fit['converged'])
        self.assertAlmostEqual(fit['km_over_izz'],8.,places=4)
        self.assertAlmostEqual(fit['damping_over_izz'],.3,places=4)
        self.assertAlmostEqual(fit['effective_delay_s'],.02,places=5)
        self.assertNotIn('Izz',fit)


if __name__=='__main__': unittest.main()
