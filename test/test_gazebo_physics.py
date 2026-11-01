import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'gazebo_physics_process.py'
SPEC = importlib.util.spec_from_file_location('gazebo_physics_process', str(SCRIPT))
PROCESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROCESS)


class GazeboPhysicsTest(unittest.TestCase):
    config = dict(
        mass_kg=1.2,
        rotor_positions_flu_m=[[.1, -.11, 0], [-.1, .11, 0],
                               [.1, .11, 0], [-.1, -.11, 0]],
        yaw_torque_signs=[-1, -1, 1, 1],
        gazebo_input_scaling_rad_s=1000.,
        gazebo_zero_position_armed_rad_s=0.,
        gazebo_max_rot_velocity_rad_s=1000.,
        motor_tau_min_s=.01,
        motor_tau_max_s=.1,
        max_angular_accel_rmse_rad_s2=3.)

    def data(self, seed):
        rng = np.random.RandomState(seed)
        t = np.arange(0., 45., .01)
        levels = rng.uniform(.22, .72, (len(t)//25+1, 4))
        commands = levels[np.arange(len(t))//25]
        state = PROCESS.motor_state(t, commands, .04)
        motor_constant = 3.1e-6
        speed = PROCESS.rotor_speed(state, 1000., 0., 1000.)
        forces = motor_constant*speed**2
        positions = np.asarray(self.config['rotor_positions_flu_m'])
        signs = np.asarray(self.config['yaw_torque_signs'])
        torque = np.column_stack([forces@positions[:, 1],
                                  -forces@positions[:, 0],
                                  .055*(forces@signs)])
        inertia = np.array([.018, .021, .032])
        damping = np.array([.003, .0035, .004])
        rates = np.zeros((len(t), 3))
        for index in range(1, len(t)):
            cross = np.cross(rates[index-1], inertia*rates[index-1])
            derivative = (torque[index-1]-cross-damping*rates[index-1])/inertia
            rates[index] = rates[index-1] + .01*derivative
        acceleration = np.zeros((len(t), 3))
        acceleration[:, 2] = forces.sum(axis=1)/self.config['mass_kg']
        acceleration[:, 2] += rng.normal(0., .01, len(t))
        return dict(t=t, commands=commands, rates=rates, acceleration=acceleration)

    def test_recovers_synthetic_physics(self):
        report = PROCESS.identify(self.data(1), self.data(2), self.config)
        self.assertTrue(report['passed'])
        self.assertAlmostEqual(report['propulsion']['common_motor_time_constant_s'], .04,
                               delta=.006)
        self.assertAlmostEqual(
            report['propulsion']['gazebo_motor_constant_n_per_rad_s_squared'], 3.1e-6,
            delta=.05e-6)
        np.testing.assert_allclose(
            report['rotation']['aggregate_inertia_diagonal_kg_m2'],
            [.018, .021, .032], rtol=.15)
        self.assertAlmostEqual(report['rotation']['gazebo_moment_constant_m'], .055,
                               delta=.01)


if __name__ == '__main__':
    unittest.main()
