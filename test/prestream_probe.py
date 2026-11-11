import os, sys, socket, subprocess, time, signal, json
with socket.socket() as s:
    s.bind(('127.0.0.1', 0)); port = s.getsockname()[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
children = []
log = open(sys.argv[2] + '.log', 'w')
try:
    children.append(subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log, start_new_session=True))
    import rospy, rosgraph
    from mavros_msgs.msg import State, PositionTarget
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import Imu
    end = time.monotonic()+15
    while not rosgraph.is_master_online():
        if time.monotonic()>end: raise RuntimeError('isolated master unavailable')
        time.sleep(.1)
    rospy.init_node('prestream_probe', disable_signals=True)
    pubs = [rospy.Publisher('/mavros/state', State, queue_size=1),
            rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1),
            rospy.Publisher('/mavros/local_position/velocity_local', TwistStamped, queue_size=1),
            rospy.Publisher('/mavros/imu/data', Imu, queue_size=1)]
    received = []
    sub = rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, lambda m: received.append(time.monotonic()))
    children.append(subprocess.Popen([sys.argv[1]], stdout=log, stderr=log, start_new_session=True))
    state = State(connected=True, armed=True, mode='POSCTL')
    last_state = 0
    result = {}
    for name, shift in [('normal', 0), ('future_imu', .2), ('recovered', 0)]:
        start = time.monotonic()
        while time.monotonic()-start < 5:
            now = time.monotonic()
            if now-last_state >= 1: pubs[0].publish(state); last_state = now
            p=PoseStamped(); p.header.stamp=rospy.Time.now(); p.pose.position.z=3; p.pose.orientation.w=1
            v=TwistStamped(); v.header.stamp=p.header.stamp
            i=Imu(); i.header.stamp=p.header.stamp+rospy.Duration(shift)
            pubs[1].publish(p); pubs[2].publish(v); pubs[3].publish(i)
            time.sleep(.02)
        samples=[t for t in received if start+2 <= t <= start+5]
        result[name]={'hz':len(samples)/3, 'count':len(samples)}
    assert result['normal']['hz'] > 100, result
    assert result['future_imu']['count'] == 0, result
    assert result['recovered']['hz'] > 100, result
    result['isolated_master']=os.environ['ROS_MASTER_URI']
    result['binary']=sys.argv[1]
    result['passed']=True
    open(sys.argv[2], 'w').write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait()
    log.close()
