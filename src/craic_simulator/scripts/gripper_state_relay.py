#!/usr/bin/env python3
"""Relay /gripper/state to /joint_states so robot_state_publisher can drive gripper joints in RViz."""
import rospy
from sensor_msgs.msg import JointState

pub = None
last_stamp = None

def cb(msg):
    global last_stamp
    # Avoid publishing with the same timestamp (causes TF_REPEATED_DATA warnings)
    now = rospy.Time.now()
    if last_stamp is not None and now == last_stamp:
        return
    last_stamp = now
    msg.header.stamp = now
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gripper_state_relay', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.Subscriber('/gripper/state', JointState, cb)
    rospy.spin()
