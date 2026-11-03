"""生成合成电机与刚体响应，验证物理参数恢复；不启动 ROS、Gazebo 或硬件。"""
import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'gazebo_physics_process.py'
SPEC = importlib.util.spec_from_file_location('gazebo_physics_process', str(SCRIPT))
PROCESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROCESS)


class GazeboPhysicsTest(unittest.TestCase):
    """提供已知惯量、阻尼和电机映射的合成飞行数据。"""
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

    def test_px4_thrust_curve_is_inverted_before_motor_dynamics(self):
        speed = np.array([[.1, .4, .6, .9]])
        thrust = .59*speed**2 + .41*speed
        np.testing.assert_allclose(PROCESS.thrust_to_speed_command(thrust, .59), speed)
        np.testing.assert_allclose(PROCESS.thrust_to_speed_command(speed, 0), speed)

    def test_nonphysical_inertia_is_rejected(self):
        matrix = np.eye(7)
        with self.assertRaisesRegex(ValueError, 'triangle'):
            PROCESS.fit_rotation(matrix, np.array([.001, .002, .02, .06, .01, .01, .01]))

    def test_rpm_alignment_rejects_clock_resets_and_gaps(self):
        t = np.arange(0,1,.004)
        speed = np.ones((len(t),4))*700
        aligned = PROCESS.align_rotor_speed(t,speed,np.arange(.1,.8,.008))
        np.testing.assert_allclose(aligned,700)
        with self.assertRaisesRegex(ValueError,'strictly increasing'):
            PROCESS.align_rotor_speed(np.r_[t,t],np.vstack([speed,speed]),t)
        with self.assertRaisesRegex(ValueError,'gap'):
            PROCESS.align_rotor_speed(np.array([0.,.1,.2]),np.ones((3,4))*700,np.array([.05,.15]))
        with self.assertRaisesRegex(ValueError,'cover'):
            PROCESS.align_rotor_speed(t,speed,np.array([-.1,.2]))

    def test_measured_rpm_bypasses_motor_delay(self):
        training,validation = self.data(5),self.data(6)
        for data in (training,validation):
            data['rotor_speed']=PROCESS.rotor_speed(PROCESS.motor_state(data['t'],data['commands'],.04),1000,0,1000)
        report=PROCESS.identify(training,validation,self.config)
        self.assertTrue(report['passed'])
        self.assertEqual(report['rotor_input'],'measured')
        self.assertIsNone(report['propulsion']['common_motor_time_constant_s'])
        self.assertAlmostEqual(report['propulsion']['gazebo_motor_constant_n_per_rad_s_squared'],3.1e-6,delta=.05e-6)

    def test_partial_yaw_prior_and_acceleration_fit(self):
        training, validation = self.data(7), self.data(8)
        for data in (training,validation):
            data['rotor_speed']=PROCESS.rotor_speed(PROCESS.motor_state(data['t'],data['commands'],.04),1000,0,1000)
        config = dict(self.config,rotation_fit_method='angular_acceleration',
                      inertia_prior_axes=[2],inertia_prior_kg_m2=[.032],
                      inertia_prior_std_kg_m2=[.002],inertia_prior_source='synthetic known fixture')
        report = PROCESS.identify(training,validation,config)
        self.assertTrue(report['passed'])
        self.assertTrue(report['rotation_fit']['inertia_prior_used'])
        self.assertEqual(report['rotation_fit']['inertia_prior_axes'],[2])
        np.testing.assert_allclose(report['rotation']['aggregate_inertia_diagonal_kg_m2'],[.018,.021,.032],rtol=.2)

    def test_inertia_prior_requires_uncertainty_and_source(self):
        with self.assertRaisesRegex(ValueError,'standard deviation'):
            PROCESS.fit_angular_acceleration(np.eye(7),np.ones(7),dict(inertia_prior_kg_m2=[.01,.01,.01]))

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
