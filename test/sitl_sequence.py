#!/usr/bin/env python3
"""隔离 ROS/Gazebo master 的闭环 SITL 回归（会解锁、起飞、进入 Offboard、降落仿真机）。
先 source ROS 和工作空间，再运行：
  python3 sitl_sequence.py --kind brct1order --px4 /path/to/PX4-Autopilot --output /tmp/new-run
只允许本机仿真；若固定 MAVLink 端口已占用则拒绝启动。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=['brct1order', 'gazebo_physics'], required=True)
    parser.add_argument('--px4', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, help='explicit excitation config; default is selected by kind')
    parser.add_argument('--quick', action='store_true', help='one repetition')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    for port in (14540, 14580, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', port))
    # PX4 startup override: default + high-rate topics, before logger initializes.
    os.environ['PX4_PARAM_SDLOG_PROFILE'] = '17'
    os.environ['GAZEBO_PLUGIN_PATH'] = str(Path(__file__).resolve().parents[3]/'devel/lib')+':'+os.environ.get('GAZEBO_PLUGIN_PATH','')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11331', GAZEBO_MASTER_URI='http://127.0.0.1:11351', ROS_LOG_DIR=str(root/'roslogs'))
    import rospy
    import rosgraph
    import rospkg
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, ParamValue
    from mavros_msgs.srv import SetMode, CommandBool, ParamSet, ParamPull
    from std_msgs.msg import String
    rp = rospkg.RosPack()
    package = 'holydrone_identification'
    source = Path(rp.get_path(package))/'config'/(args.kind+'.yaml')
    if args.config is not None: source=args.config
    cfg = yaml.safe_load(source.read_text())
    if args.quick:
        cfg['repetitions'] = 1
    (root/'config.yaml').write_text(yaml.safe_dump(cfg))
    processes, samples, events = [], [], []
    state, pose = [None], [None]
    log = (root/'console.log').open('w')
    stream = (root/'events.jsonl').open('w')

    def spawn(command):
        p = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append(p)
        return p

    def wait(condition, timeout, label):
        until = time.monotonic()+timeout
        while not condition():
            if time.monotonic() > until:
                raise RuntimeError('timeout: '+label)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('test child exited; see console.log')
            time.sleep(.05)

    def onpose(msg):
        pose[0] = msg
        p, q = msg.pose.position, msg.pose.orientation
        tilt = math.degrees(math.acos(max(-1, min(1, 1-2*(q.x*q.x+q.y*q.y)))))
        samples.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, tilt])

    def onevent(msg):
        entry = dict(t=pose[0].header.stamp.to_sec() if pose[0] else 0, **json.loads(msg.data))
        events.append(entry)
        stream.write(json.dumps(entry)+'\n'); stream.flush()

    def finished():
        return any(e.get('sequence_complete') or e.get('phase') == 'aborted' for e in events)

    try:
        if rosgraph.is_master_online():
            raise RuntimeError('isolated test master already exists')
        spawn(['roslaunch', '-p', '11331', 'holydrone', 'sitl.launch', 'aircraft:=holydrone_lite', 'gui:=false', 'start_mavros:=true',
               'px4_dir:='+str(args.px4.resolve()), 'px4_runtime_dir:='+str(root/'px4'),
               'world:='+rp.get_path('holydrone')+'/gazebo/worlds/empty.world', 'spawn_z:=0.055'])
        wait(rosgraph.is_master_online, 20, 'master')
        rospy.init_node('excitation_sitl_check', disable_signals=True)
        subscriptions = [rospy.Subscriber('/mavros/state', State, lambda m: state.__setitem__(0, m)),
                         rospy.Subscriber('/mavros/local_position/pose', PoseStamped, onpose),
                         rospy.Subscriber('/identification/status', String, onevent)]
        wait(lambda: state[0] and state[0].connected and pose[0], 45, 'FCU feedback')
        # 从正在运行的Gazebo读取真实link惯量，包含相机、GPS和旋翼，供离线交叉验证。
        from gazebo_msgs.srv import GetModelProperties, GetLinkProperties, GetLinkState
        def pose_dict(p):
            return dict(position=[p.position.x,p.position.y,p.position.z],
                        orientation=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        model = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)('holydrone')
        truth = {}
        for name in model.body_names:
            full = 'holydrone::'+name
            prop = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)(full)
            link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)(full, 'holydrone::base_link')
            assert prop.success and link.success
            truth[name] = dict(mass=prop.mass,com=pose_dict(prop.com),
                               inertia=[prop.ixx,prop.iyy,prop.izz,prop.ixy,prop.ixz,prop.iyz],
                               pose=pose_dict(link.link_state.pose))
        (root/'gazebo_links.json').write_text(json.dumps(truth,indent=2)+'\n')
        spawn(['rosrun', package, 'record_motor_speed.py', '_output:='+str(root/'motor_speed.csv')])
        wait(lambda: (root/'motor_speed.csv').exists() and (root/'motor_speed.csv').stat().st_size > 200, 15, 'actual rotor telemetry')
        command = ['roslaunch', '--wait', package, 'identification_fsm.launch', 'config:='+str(root/'config.yaml')]
        command += ['exciter:='+args.kind+'_excite']
        spawn(command)
        wait(lambda: bool(events), 15, 'FSM status')
        # ROS manual_control is transmitted by MAVROS; keep Position mode available without an external QGC.
        from mavros_msgs.msg import ManualControl
        joystick = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=1)
        def manual(_):
            msg = ManualControl(); msg.header.stamp = rospy.Time.now(); msg.z = 500.
            joystick.publish(msg)
        joystick_timer = rospy.Timer(rospy.Duration(.05), manual)
        mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        assert rospy.ServiceProxy('/mavros/param/pull', ParamPull)(True).success
        for name, val in [('COM_RC_IN_MODE', 1), ('COM_RCL_EXCEPT', 4), ('MIS_TAKEOFF_ALT', 4.0)]:
            response = param(name, ParamValue(integer=val) if isinstance(val, int) else ParamValue(real=val))
            assert response.success, name
        wait(lambda: rospy.Time.now().to_sec() > 12, 30, 'estimator initialization')
        assert mode(0, 'AUTO.TAKEOFF').mode_sent
        wait(lambda: arm(True).success or time.sleep(1), 30, 'normal arming checks')
        wait(lambda: pose[0].pose.position.z > 3.5, 40, 'takeoff')
        assert mode(0, 'POSCTL').mode_sent
        wait(lambda: state[0].mode == 'POSCTL', 10, 'Position')
        time.sleep(6)
        start = pose[0].header.stamp.to_sec()
        origin = pose[0].pose.position
        assert mode(0, 'OFFBOARD').mode_sent
        wait(finished, 300, 'excitation sequence')
        end = pose[0].header.stamp.to_sec()
        flight = [s for s in samples if start <= s[0] <= end]
        complete = any(e.get('sequence_complete') for e in events)
        result = dict(kind=args.kind, complete=complete, duration_s=end-start,
                      origin=[origin.x, origin.y, origin.z], altitude_min_m=min(s[3] for s in flight),
                      altitude_max_m=max(s[3] for s in flight), max_drop_m=origin.z-min(s[3] for s in flight),
                      max_tilt_deg=max(s[4] for s in flight), max_xy_distance_m=max(math.hypot(s[1]-origin.x,s[2]-origin.y) for s in flight))

        (root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        (root/'pose_samples.json').write_text(json.dumps(flight))
        print(json.dumps(result, indent=2), flush=True)
        assert complete, 'sequence aborted; see events.jsonl'
        assert result['max_drop_m'] < .3, 'excessive drop'
        assert result['max_tilt_deg'] < 45, 'excessive tilt'
        assert result['max_xy_distance_m'] < 2, 'excessive drift'
    finally:
        if 'mode' in locals():
            try:
                def land():
                    if state[0] and not state[0].armed: return True
                    mode(0, 'AUTO.LAND'); time.sleep(1); return False
                wait(land, 45, 'land/disarm')
            except Exception as error:
                print('cleanup:', error, flush=True)
        (root/'final_state.json').write_text(json.dumps(dict(armed=state[0].armed if state[0] else None,mode=state[0].mode if state[0] else None)))
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL); p.wait()
        stream.close(); log.close()


if __name__ == '__main__':
    main()
