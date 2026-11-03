"""用独立合成 CSV 验证一阶响应拟合、完整脉冲筛选与验证集复用拒绝。
只使用临时目录，不需要 ROS Master、仿真或硬件。"""
import csv
import tempfile
import unittest
from pathlib import Path
import numpy as np
from response import predict, identify_response, segments, read_csv


class IdentificationTest(unittest.TestCase):
    """使用临时目录保存训练与验证 CSV，测试结束后清理。"""
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.root=Path(self.temporary.name)
    def tearDown(self):self.temporary.cleanup()

    def response_file(self,name,seed,corrupt=False):
        rng=np.random.RandomState(seed); rows=[]; duration=3.
        truth=[]
        for axis,channel in enumerate(('roll','pitch','yaw','thrust')):
            params=(.04+.01*axis,.025,1.1 if axis<3 else 32.,.02 if axis<3 else -.2)
            truth.append(params)
            t=np.arange(0,duration,.02)
            levels=rng.choice([-1.,1.],6)
            u=levels[np.minimum((t/.5).astype(int),5)]*(.35 if axis<3 else .04)+(0 if axis<3 else .3)
            y=predict(t,u,params[2]*u[0]+params[3],*params)+rng.normal(0,.001,len(t))
            if corrupt:y+=2*np.sin(10*t)
            for i,stamp in enumerate(t):
                row=dict(t=stamp+axis*4.,segment=axis,channel=channel,valid=1,
                         cmd_wx=0.,cmd_wy=0.,cmd_wz=0.,cmd_thrust=.3,wx=0.,wy=0.,wz=0.,fz=9.81)
                row[('cmd_wx','cmd_wy','cmd_wz','cmd_thrust')[axis]]=u[i]
                row[('wx','wy','wz','fz')[axis]]=y[i];rows.append(row)
        path=self.root/name
        with path.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        return path,truth

    def test_response_recovery_and_holdout(self):
        train,truth=self.response_file('train.csv',1);test,_=self.response_file('test.csv',2)
        report=identify_response(train,test)
        self.assertTrue(report['passed'])
        for row,expected in zip(report['channels'],truth):
            np.testing.assert_allclose([row['tau_s'],row['delay_s']],expected[:2],atol=.004)
            self.assertAlmostEqual(row['gain'],expected[2],delta=.05)
        bad,_=self.response_file('bad.csv',3,True)
        self.assertFalse(identify_response(train,bad)['passed'])

    def test_aborted_pulse_rejected(self):
        path,_=self.response_file('aborted.csv',1)
        data=read_csv(path)
        augmented=np.empty(len(data),dtype=data.dtype.descr+[('phase','U16')])
        for name in data.dtype.names:augmented[name]=data[name]
        augmented['phase']='pulse'
        augmented['phase'][np.flatnonzero(data['segment']==0)[-1]]='fault'
        with self.assertRaises(ValueError):segments(augmented,0)

    def test_completed_final_pulse_is_accepted(self):
        path,_=self.response_file('complete.csv',1)
        data=read_csv(path)
        augmented=np.empty(len(data),dtype=data.dtype.descr+[('phase','U16'),
                                                              ('sequence_complete','i4')])
        for name in data.dtype.names:augmented[name]=data[name]
        augmented['phase']='pulse';augmented['sequence_complete']=0
        augmented['sequence_complete'][-1]=1
        self.assertEqual(len(segments(augmented,3)),1)

    def test_reused_validation_rejected(self):
        train,_=self.response_file('train.csv',1)
        with self.assertRaises(ValueError):identify_response(train,train)


if __name__=='__main__':unittest.main()
