#!/usr/bin/env python3
"""独立 master 的辨识 FSM 回归；只模拟 ROS 服务/反馈，不连接飞控。"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import yaml


class IdentificationMock(unittest.TestCase):
    def test_both_plans_and_protection(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
        os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); log=(root/'ros.log').open('w'); children=[]
            def spawn(cmd):
                p=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True);children.append(p);return p
            def close(p):
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGINT)
                    try:p.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
                children.remove(p)
            try:
                spawn(['roscore','-p',str(port)])
                import rospy,rosgraph
                from mavros_msgs.msg import State,AttitudeTarget
                from mavros_msgs.srv import SetMode,SetModeResponse
                from geometry_msgs.msg import PoseStamped,TwistStamped
                from std_msgs.msg import String,Bool
                from sensor_msgs.msg import Imu
                from std_srvs.srv import Trigger
                def wait(fn,timeout=10):
                    end=time.monotonic()+timeout
                    while not fn():
                        if time.monotonic()>end:self.fail('timeout; latest status='+str(statuses[-1:]));
                        time.sleep(.01)
                statuses=[];wait(rosgraph.is_master_online)
                rospy.init_node('identification_mock',disable_signals=True)
                state=State(connected=True,armed=False,mode='POSCTL')
                motion=dict(z=3.,x=0.,vz=0.,tilt=0.,enabled=True,old=False)
                thrust=dict(value=.34,enabled=True)
                imu_pub=rospy.Publisher('/mavros/imu/data',Imu,queue_size=1)
                commands=[];mode_calls=[];finished=[]
                subscribers=[rospy.Subscriber('/mavros/setpoint_raw/attitude',AttitudeTarget,lambda m:commands.append((time.monotonic(),m))),
                    rospy.Subscriber('/identification/status',String,lambda m:statuses.append(json.loads(m.data))),
                    rospy.Subscriber('/identification_fsm/finished',Bool,lambda m:finished.append(m.data))]
                pubs=[rospy.Publisher('/mavros/state',State,queue_size=1),rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1),
                    rospy.Publisher('/mavros/local_position/velocity_local',TwistStamped,queue_size=1),rospy.Publisher('/mavros/setpoint_raw/target_attitude',AttitudeTarget,queue_size=1)]
                def feedback(_):
                    pubs[0].publish(state)
                    if motion['enabled']:
                        p=PoseStamped();p.header.stamp=rospy.Time.now()-rospy.Duration(2 if motion['old'] else 0)
                        p.pose.position.z=motion['z'];p.pose.position.x=motion['x'];p.pose.orientation.w=math.cos(math.radians(motion['tilt'])/2);p.pose.orientation.x=math.sin(math.radians(motion['tilt'])/2);pubs[1].publish(p)
                        imu=Imu();imu.header=p.header;imu.angular_velocity.z=motion.get("rate",0.);imu_pub.publish(imu)
                        v=TwistStamped();v.header=p.header;v.twist.linear.z=motion['vz'];pubs[2].publish(v)
                    if thrust['enabled']:
                        m=AttitudeTarget();m.header.stamp=rospy.Time.now();m.thrust=thrust['value'];pubs[3].publish(m)
                timer=rospy.Timer(rospy.Duration(.02),feedback)
                def mode(req):
                    mode_calls.append(req.custom_mode);time.sleep(.15)
                    if len(mode_calls)>=2:state.mode=req.custom_mode
                    return SetModeResponse(True)
                server=rospy.Service('/mavros/set_mode',SetMode,mode)
                package=Path(__file__).resolve().parents[1]
                node=None
                def launch(kind='brct1order',**extra):
                    statuses.clear();commands.clear();mode_calls.clear();finished.clear()
                    state.mode='POSCTL';state.armed=False
                    motion.update(z=3.,x=0.,vz=0.,tilt=0.,enabled=True,old=False,rate=0.);thrust.update(value=.34,enabled=True)
                    cfg=yaml.safe_load((package/'config'/ (kind+'.yaml')).read_text());cfg.update(repetitions=1,pulse_hold=.15)
                    cfg['recovery'].update(settle_s=.05,timeout_s=1.)
                    cfg.pop('pulse_durations_s',None)
                    cfg.update(extra);path=root/'config.yaml';path.write_text(yaml.safe_dump(cfg))
                    p=spawn(['roslaunch','holydrone_identification','identification_fsm.launch','config:='+str(path),'exciter:='+kind+'_excite'])
                    wait(lambda:statuses);rospy.wait_for_service('/identification_fsm/stop',timeout=10)
                    return p
                def enter():
                    state.armed=True;wait(lambda:statuses[-1]['thrust_ready']);state.mode='OFFBOARD';wait(lambda:statuses[-1]['thrust_locked'])
                for kind,count in [('brct1order',4),('gazebo_physics',12)]:
                    node=launch(kind)
                    time.sleep(.1);self.assertFalse(commands)
                    enter();wait(lambda:statuses[-1]['sequence_complete'])
                    groups={s['segment'] for s in statuses if s['phase']=='pulse'}
                    self.assertEqual(groups,set(range(count)))
                    self.assertTrue(finished[-1]);self.assertGreaterEqual(len(mode_calls),2)
                    self.assertLess(max(b[0]-a[0] for a,b in zip(commands,commands[1:])),.15)
                    # Frozen thrust pair retains the commanded increment; autopilot feedback cannot overwrite capture.
                    values=[m.thrust for _,m in commands]
                    amp=.015 if kind=='brct1order' else .025
                    self.assertTrue(any(abs(v-(.34+amp))<1e-5 for v in values))
                    self.assertTrue(any(abs(v-(.34-amp))<1e-5 for v in values))
                    close(node);node=None
                # Sine segments keep the same measured-motion recovery gate.
                node=launch('gazebo_physics',yaw_sweep=dict(enabled=True,frequencies_hz=[2.,3.],cycles=2,amplitude_rad_s=.25))
                enter();wait(lambda:statuses[-1]['phase']=='pulse' and statuses[-1].get('sine_hz')==2.)
                motion['rate']=.3
                wait(lambda:statuses[-1]['phase']=='recover')
                held=statuses[-1]['segment'];time.sleep(.15)
                self.assertEqual(statuses[-1]['segment'],held)
                self.assertEqual(statuses[-1]['phase'],'recover')
                motion['rate']=0.
                wait(lambda:statuses[-1]['sequence_complete'])
                self.assertTrue(any(.05<abs(m.body_rate.z)<.22 for _,m in commands))
                self.assertTrue(any(s.get('sine_hz')==3. and s['phase']=='pulse' for s in statuses))
                close(node);node=None
                # ATTITUDE_TARGET stops before the 1 Hz HEARTBEAT announces Offboard.
                for delay, accepted in [(.8, True), (1.7, False)]:
                    node=launch();state.armed=True
                    wait(lambda:statuses[-1]['thrust_ready'])
                    thrust['enabled']=False;time.sleep(delay);state.mode='OFFBOARD'
                    if accepted:
                        wait(lambda:statuses[-1]['thrust_locked'])
                    else:
                        wait(lambda:statuses[-1]['phase']=='aborted')
                        self.assertFalse(any(s['phase']=='pulse' for s in statuses))
                    close(node);node=None
                node=launch();enter();wait(lambda:statuses[-1]['phase']=='pulse')
                thrust['value']=.7;motion.update(z=2.8,vz=-.2,tilt=30.)
                wait(lambda:commands[-1][1].thrust>.39)
                wait(lambda:statuses[-1]['phase']=='recover' and statuses[-1]['segment']==1)
                time.sleep(.2);self.assertEqual(statuses[-1]['segment'],1)
                self.assertLess(commands[-1][1].body_rate.x,-.1)
                motion.update(z=3.,vz=0.,tilt=0.,rate=.3)
                time.sleep(.15);self.assertEqual(statuses[-1]['phase'],'recover')
                motion['rate']=0.;wait(lambda:statuses[-1]['phase']=='pulse')
                # Old-stamped messages continue arriving but must not satisfy freshness.
                motion['old']=True;wait(lambda:statuses[-1]['phase']=='aborted')
                self.assertIn('stale',statuses[-1]['reason']);self.assertNotIn(True,finished)
                close(node);node=None
                for label,change in [('height',dict(z=1.8)),('tilt',dict(tilt=60.)),('timeout',dict(tilt=15.))]:
                    node=launch();enter();wait(lambda:statuses[-1]['phase']=='pulse');motion.update(change)
                    wait(lambda:statuses[-1]['phase']=='aborted');self.assertIn(label if label!='height' else 'height',statuses[-1]['reason'])
                    self.assertNotIn(True,finished);close(node);node=None
                node=launch();state.armed=True;state.mode='OFFBOARD'
                wait(lambda:statuses[-1]['phase']=='aborted');self.assertFalse(any(s['phase']=='pulse' for s in statuses));close(node);node=None
                node=launch();enter();wait(lambda:statuses[-1]['phase']=='pulse')
                self.assertTrue(rospy.ServiceProxy('/identification_fsm/stop',Trigger)().success)
                wait(lambda:state.mode=='POSCTL');self.assertNotIn(True,finished);close(node);node=None
                timer.shutdown()
            finally:
                for p in list(reversed(children)):close(p)
                log.close()

if __name__=='__main__':unittest.main()
