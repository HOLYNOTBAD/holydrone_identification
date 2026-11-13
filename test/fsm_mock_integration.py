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
                from mavros_msgs.msg import State,AttitudeTarget,PositionTarget
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
                motion=dict(z=3.,x=0.,vx=0.,vy=0.,vz=0.,tilt=0.,enabled=True,old=False)
                thrust=dict(value=.34,enabled=True)
                imu_pub=rospy.Publisher('/mavros/imu/data',Imu,queue_size=1)
                commands=[];positions=[];mode_calls=[];finished=[]
                subscribers=[rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,lambda m:positions.append((time.monotonic(),m))),
                    rospy.Subscriber('/mavros/setpoint_raw/attitude',AttitudeTarget,lambda m:commands.append((time.monotonic(),m))),
                    rospy.Subscriber('/identification/status',String,lambda m:statuses.append(json.loads(m.data))),
                    rospy.Subscriber('/identification_fsm/finished',Bool,lambda m:finished.append(m.data))]
                pubs=[rospy.Publisher('/mavros/state',State,queue_size=1),rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1),
                    rospy.Publisher('/mavros/local_position/velocity_local',TwistStamped,queue_size=1),rospy.Publisher('/mavros/setpoint_raw/target_attitude',AttitudeTarget,queue_size=1)]
                state_feedback = dict(interval=.02, last=0.)
                imu_sequence = 0
                def feedback(_):
                    nonlocal imu_sequence
                    now = time.monotonic()
                    if now-state_feedback['last'] >= state_feedback['interval']:
                        pubs[0].publish(state); state_feedback['last'] = now
                    if motion['enabled']:
                        p=PoseStamped();p.header.stamp=rospy.Time.now()-rospy.Duration(2 if motion['old'] else 0)
                        p.pose.position.z=motion['z'];p.pose.position.x=motion['x']
                        r=math.radians(motion['tilt'])/2;y=math.radians(motion.get('yaw',0))/2
                        p.pose.orientation.w=math.cos(r)*math.cos(y);p.pose.orientation.x=math.sin(r)*math.cos(y)
                        p.pose.orientation.y=math.sin(r)*math.sin(y);p.pose.orientation.z=math.cos(r)*math.sin(y)
                        pubs[1].publish(p)
                        imu=Imu();imu.header.stamp=p.header.stamp+rospy.Duration(motion.get('imu_shift',0.))
                        imu_sequence += 1
                        imu.angular_velocity.z=motion.get("rate",0.) + motion.get('vibration',0.)*(-1 if imu_sequence%2 else 1)
                        if motion.get('imu_enabled',True): imu_pub.publish(imu)
                        v=TwistStamped();v.header=p.header;v.twist.linear.x=motion['vx'];v.twist.linear.y=motion['vy'];v.twist.linear.z=motion['vz'];pubs[2].publish(v)
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
                def launch(kind='flight',hover_mode='POSCTL',**extra):
                    statuses.clear();commands.clear();positions.clear();mode_calls.clear();finished.clear()
                    state.mode=hover_mode;state.armed=False
                    motion.update(z=3.,x=0.,vx=0.,vy=0.,vz=0.,tilt=0.,enabled=True,old=False,rate=0.,vibration=0.,yaw=0.,imu_shift=0.,imu_enabled=True);thrust.update(value=.34,enabled=True)
                    cfg=yaml.safe_load((package/'config'/ ('gazebo_physics/gazebo_physics.yaml' if kind=='gazebo_physics' else 'excitation/pulse_medium.yaml')).read_text());cfg.update(repetitions=1,pulse_hold=.15)
                    cfg['recovery'].update(settle_s=.1)
                    cfg.pop('pulse_durations_s',None)
                    cfg.update(extra);path=root/'config.yaml';path.write_text(yaml.safe_dump(cfg))
                    p=spawn(['roslaunch','holydrone_identification','identification_fsm.launch','config:='+str(path),'exciter:='+kind+'_excite'])
                    wait(lambda:statuses);rospy.wait_for_service('/identification_fsm/stop',timeout=10)
                    return p
                def enter():
                    state.armed=True;wait(lambda:len(positions)>5)
                    state.mode='OFFBOARD';wait(lambda:statuses[-1]['origin_locked'])
                # Real FCU state is about 1 Hz: pre-stream must not wait for OFFBOARD or thrust.
                node=launch()
                state_feedback['interval']=1.
                thrust['enabled']=False
                state.armed=True
                wait(lambda:statuses[-1]['command_gate']=='streaming')
                wait(lambda:statuses[-1]['command_hz']>100)
                self.assertEqual(statuses[-1]['state'],'IDLE')
                self.assertFalse(statuses[-1]['thrust_ready'])
                self.assertFalse(commands)
                n=len(positions);time.sleep(1.2)
                self.assertGreater(len(positions)-n,100)
                stamps=[t for t,_ in positions[n:]]
                self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.2)
                motion['old']=True
                wait(lambda:'stale' in statuses[-1]['command_gate'])
                time.sleep(.2);n=len(positions);time.sleep(.2)
                self.assertEqual(len(positions),n)
                self.assertGreater(statuses[-1]['imu_rejected'],0)
                motion['old']=False
                wait(lambda:statuses[-1]['command_gate']=='streaming')
                # Position and velocity stay valid: isolate IMU timestamp/receipt failures.
                for shift in [.2, -.2]:
                    rejected=statuses[-1]['imu_rejected']
                    motion['imu_shift']=shift
                    wait(lambda:'IMU timestamp' in statuses[-1]['command_gate'])
                    time.sleep(.2);n=len(positions);time.sleep(.2)
                    self.assertEqual(len(positions),n)
                    self.assertGreater(statuses[-1]['imu_rejected'],rejected)
                    motion['imu_shift']=0.
                    wait(lambda:statuses[-1]['command_gate']=='streaming')
                    n=len(positions);wait(lambda:len(positions)>n+20)
                motion['imu_enabled']=False
                wait(lambda:'IMU timestamp' in statuses[-1]['command_gate'])
                motion['imu_enabled']=True
                wait(lambda:statuses[-1]['command_gate']=='streaming')
                # No new heartbeat for >2 s must stop pre-stream, then recover without restart.
                state_feedback['interval']=10.;state_feedback['last']=time.monotonic()
                wait(lambda:'FCU state missing or stale'==statuses[-1]['command_gate'])
                state_feedback['interval']=1.;state_feedback['last']=0.
                wait(lambda:statuses[-1]['command_gate']=='streaming')
                state.mode='STABILIZED'
                wait(lambda:'requires POSCTL' in statuses[-1]['command_gate'])
                self.assertFalse(mode_calls)
                close(node);node=None;state_feedback['interval']=.02
                # High-frequency zero-mean rates must not starve the stable/thrust windows.
                node=launch()
                motion['vibration']=.2
                enter();wait(lambda:statuses[-1]['phase']=='pulse')
                self.assertGreater(statuses[-1]['body_rate_rad_s'],math.radians(5))
                self.assertLess(statuses[-1]['settled_body_rate_rad_s'],math.radians(5))
                self.assertTrue(statuses[-1]['thrust_locked'])
                self.assertTrue(any(s['excitation_gate']=='settling window' for s in statuses))
                close(node);node=None
                # A real persistent rotation still blocks excitation and names the condition.
                node=launch();motion['rate']=.2;enter()
                time.sleep(8.5)
                self.assertEqual(statuses[-1]['phase'],'recover')
                self.assertIn('filtered angular speed',statuses[-1]['excitation_gate'])
                self.assertFalse(commands)
                self.assertTrue(positions)
                motion['rate']=0.
                wait(lambda:statuses[-1]['phase']=='pulse')
                self.assertEqual(statuses[-1]['segment'],0)
                self.assertEqual(statuses[-1]['pulse'],0)
                close(node);node=None
                for kind,count in [('flight',4),('brct1order',4),('brct2order',4),('brct3order',4),('gazebo_physics',12)]:
                    node=launch(kind, hover_mode='AUTO.LOITER' if kind=='brct1order' else 'POSCTL')
                    time.sleep(.1);self.assertFalse(commands);self.assertFalse(positions)
                    enter()
                    wait(lambda:statuses[-1]['phase']=='pulse')
                    # Fresh hover thrust is reacquired before each pulse, never during a pulse.
                    self.assertAlmostEqual(statuses[-1]['hover_thrust'],.34,places=5)
                    thrust['value']=.36
                    wait(lambda:statuses[-1]['sequence_complete'],timeout=180)
                    groups={s['segment'] for s in statuses if s['phase']=='pulse'}
                    self.assertEqual(groups,set(range(count)))
                    self.assertTrue(finished[-1]);self.assertFalse(mode_calls)
                    self.assertEqual(state.mode,'OFFBOARD')
                    self.assertAlmostEqual(statuses[-1]['hover_thrust'],.36,places=5)
                    values=[m.thrust for _,m in commands]
                    amp=.025 if kind=='gazebo_physics' else .015
                    base=.34 if kind=='gazebo_physics' else .36
                    self.assertTrue(any(abs(v-(base+amp))<1e-5 for v in values))
                    self.assertTrue(any(abs(v-(.36-amp))<1e-5 for v in values))
                    # Both publishers together form a continuous stream through handovers.
                    stamps=sorted(t for t,_ in commands+positions)
                    self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.2)
                    n=len(positions);time.sleep(.1);self.assertGreater(len(positions),n)
                    close(node);node=None
                node=launch('gazebo_physics',yaw_sweep=dict(enabled=True,frequencies_hz=[2.,3.],cycles=2,amplitude_rad_s=.25))
                enter();wait(lambda:statuses[-1]['sequence_complete'],timeout=180)
                self.assertTrue(any(.05<abs(m.body_rate.z)<.22 for _,m in commands))
                self.assertTrue(any(s.get('sine_hz')==3. and s['phase']=='pulse' for s in statuses))
                close(node);node=None
                # Zero angular rate alone must not release a drifting aircraft.
                node=launch();enter();motion.update(vx=1.35,tilt=33.)
                time.sleep(.5)
                self.assertFalse(any(s['phase']=='pulse' for s in statuses))
                motion.update(vx=0.,tilt=0.)
                wait(lambda:statuses[-1]['phase']=='pulse')
                motion['vx']=1.
                wait(lambda:statuses[-1]['phase']=='recover' and statuses[-1]['pulse']==1)
                n=len(positions);time.sleep(.4)
                self.assertGreater(len(positions),n)
                self.assertFalse(any(s['phase']=='pulse' and s['pulse']==1 for s in statuses))
                motion['vx']=0.
                wait(lambda:statuses[-1]['phase']=='pulse' and statuses[-1]['pulse']==1)
                motion['old']=True;wait(lambda:statuses[-1]['phase']=='aborted')
                self.assertIn('stale',statuses[-1]['reason']);self.assertNotIn(True,finished)
                time.sleep(.6);n=len(positions)+len(commands);time.sleep(.15)
                self.assertEqual(len(positions)+len(commands),n)
                close(node);node=None
                # Position recovery switches directly to a nonzero excitation command.
                node=launch();enter();wait(lambda:bool(commands))
                self.assertAlmostEqual(commands[0][1].body_rate.x,.2,places=5)
                self.assertFalse(any(s['phase']=='handover' for s in statuses))
                wait(lambda:statuses[-1]['phase']=='recover' and statuses[-1]['pulse']==1)
                n=len(commands);time.sleep(.15)
                self.assertEqual(len(commands),n)
                self.assertEqual(statuses[-1]['control_mode'],'px4_position')
                close(node);node=None
                # Invalid recovery feedback keeps position control until it settles.
                for changed in [dict(z=3.3),dict(yaw=15.),dict(vy=1.),dict(tilt=8.),dict(rate=.4)]:
                    node=launch();enter();motion.update(changed)
                    time.sleep(1.)
                    self.assertEqual(statuses[-1]['phase'],'recover')
                    self.assertFalse(commands)
                    motion.update(z=3.,yaw=0.,vy=0.,tilt=0.,rate=0.)
                    wait(lambda:statuses[-1]['phase']=='pulse')
                    close(node);node=None
                # No valid position-control thrust => no rate handover or pulse.
                node=launch();thrust['enabled']=False;enter()
                time.sleep(8.5)
                self.assertEqual(statuses[-1]['phase'],'recover')
                wait(lambda:statuses[-1]['excitation_gate']=='waiting for valid FCU thrust')
                self.assertFalse(commands)
                self.assertTrue(positions);self.assertFalse(mode_calls)
                close(node);node=None
                # Stop/finish keeps position setpoints and never changes flight mode.
                node=launch();enter();wait(lambda:statuses[-1]['phase']=='pulse')
                self.assertTrue(rospy.ServiceProxy('/identification_fsm/stop',Trigger)().success)
                wait(lambda:statuses[-1]['phase']=='aborted');time.sleep(.2)
                n=len(positions);time.sleep(.1);self.assertGreater(len(positions),n)
                self.assertFalse(mode_calls);self.assertNotIn(True,finished)
                state.mode='POSCTL';wait(lambda:not statuses[-1]['origin_locked'])
                n=len(commands);time.sleep(.2);self.assertEqual(len(commands),n)
                close(node);node=None
                timer.shutdown()
            finally:
                for p in list(reversed(children)):close(p)
                log.close()

if __name__=='__main__':unittest.main()
