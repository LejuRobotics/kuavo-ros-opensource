#!/usr/bin/env python3
"""Topic relay node for CRAIC simulator: syncs cmd_pose_world and kuavo_arm_traj with timestamps."""
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped


class TopicRelay:
    def __init__(self):
        rospy.init_node('topic_relay_node')

        self.publish_rate = rospy.get_param('~publish_rate', 50)

        self.last_cmd_twist = None
        self.cmd_twist_sub = rospy.Subscriber(
            '/cmd_pose_world', Twist, self.cmd_twist_callback
        )
        self.cmd_twist_pub = rospy.Publisher(
            '/cmd_pose_world_synced', TwistStamped, queue_size=10
        )

        self.last_joint_state = None
        self.joint_state_sub = rospy.Subscriber(
            '/kuavo_arm_traj', JointState, self.joint_state_callback
        )
        self.joint_state_pub = rospy.Publisher(
            '/kuavo_arm_traj_synced', JointState, queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.timer_callback
        )

        rospy.loginfo("Topic relay node started at {} Hz".format(self.publish_rate))

    def cmd_twist_callback(self, msg):
        self.last_cmd_twist = msg

    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def timer_callback(self, event):
        current_time = rospy.Time.now()
        if self.last_cmd_twist is not None:
            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = current_time
            stamped_msg.header.frame_id = "world"
            stamped_msg.twist = self.last_cmd_twist
            self.cmd_twist_pub.publish(stamped_msg)
        if self.last_joint_state is not None:
            joint_msg = JointState()
            joint_msg.header = self.last_joint_state.header
            joint_msg.header.stamp = current_time
            joint_msg.name = self.last_joint_state.name
            joint_msg.position = self.last_joint_state.position
            joint_msg.velocity = self.last_joint_state.velocity
            joint_msg.effort = self.last_joint_state.effort
            self.joint_state_pub.publish(joint_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        relay = TopicRelay()
        relay.run()
    except rospy.ROSInterruptException:
        pass
