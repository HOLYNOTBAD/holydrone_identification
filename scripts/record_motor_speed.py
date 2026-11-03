#!/usr/bin/env python3
"""记录 /identification/motor_speed 的实际机械角速度；一份 CSV 只容许一个时钟周期。"""
import csv
from pathlib import Path
import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('record_motor_speed')
    path = Path(rospy.get_param('~output')).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_ros_s'] + ['omega_'+str(i)+'_rad_s' for i in range(4)])
        previous = [None]
        def record(msg):
            t = msg.header.stamp.to_sec()
            if previous[0] is not None and t <= previous[0]:
                rospy.logerr('Motor telemetry clock reset/duplicate; start a new recording')
                rospy.signal_shutdown('non-monotonic telemetry'); return
            try:
                values = [msg.velocity[msg.name.index('motor_'+str(i))] for i in range(4)]
            except (ValueError, IndexError):
                rospy.logerr('Missing motor telemetry channel'); return
            writer.writerow([t]+values); stream.flush(); previous[0] = t
        subscription = rospy.Subscriber('/identification/motor_speed', JointState, record, queue_size=1000)
        rospy.spin()


if __name__ == '__main__':
    main()
