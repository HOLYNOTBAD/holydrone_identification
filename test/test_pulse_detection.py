"""回中闭环与高度补偿不能污染辨识的双脉冲边界。"""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
spec=importlib.util.spec_from_file_location('brct_process',Path(__file__).resolve().parents[1]/'scripts/brct1order_process.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class PulseDetection(unittest.TestCase):
    def test_altitude_thrust_variation_and_feedback_rejection(self):
        t=np.arange(0,5,.005);u=np.zeros((len(t),4));u[:,3]=.34
        u[(t>=1)&(t<1.35),0]=.2;u[(t>=1.35)&(t<1.7),0]=-.2
        active=(t>=1)&(t<1.7);u[active,3]+=.02*np.sin(5*t[active])
        recovery=(t>=1.7)&(t<3);u[recovery,0]=.25*np.sin(6*(t[recovery]-1.7))
        groups=module.pulse_groups(t,u)
        self.assertEqual(len(groups),1);self.assertEqual(groups[0][0],0)
        self.assertAlmostEqual(groups[0][1],1);self.assertAlmostEqual(groups[0][2],1.695)
    def test_thrust_and_incomplete_pair(self):
        t=np.arange(0,4,.005);u=np.zeros((len(t),4));u[:,3]=.34
        u[(t>=1)&(t<1.35),3]+=.015;u[(t>=1.35)&(t<1.7),3]-=.015
        u[(t>=2)&(t<2.35),1]=.2
        groups=module.pulse_groups(t,u);self.assertEqual(len(groups),1);self.assertEqual(groups[0][0],3)

    def test_position_braking_between_pulses(self):
        t=np.arange(0,7,.005);u=np.zeros((len(t),4));u[:,3]=.34
        mode=np.zeros(len(t),bool)
        mode[(t>=.7)&(t<1.5)]=True;mode[(t>=4.7)&(t<5.5)]=True
        u[(t>=1)&(t<1.35),1]=.2;u[(t>=5)&(t<5.35),1]=-.2
        # A constant position-controller output must not be treated as excitation.
        u[(t>=2)&(t<3),1]=.2
        groups=module.pulse_groups(t,u,mode)
        self.assertEqual(len(groups),2)
        self.assertEqual([g[0] for g in groups],[1,1])
        self.assertAlmostEqual(groups[0][1],1.)
        self.assertAlmostEqual(groups[0][2],1.345)
        self.assertAlmostEqual(groups[1][1],5.)
