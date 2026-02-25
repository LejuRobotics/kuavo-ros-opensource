#!/usr/bin/env python3
# filepath: /root/gsh/studio_debug_interface/src/studio_debug_interface/src/status_pub.py

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_matrix
import math
import time


class RobotStatusPublisher:
    def __init__(self):
        self.count = 0
        rospy.init_node("dispatch_robot_status")
        # Subscribers
        self.odom_info_pub = rospy.Publisher('/dispatch/robot_odom_info', Float32MultiArray, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Internal state
        self.odom_pose = None  # To store the latest odom pose
        self.robot_pose = [0, 0, 0] + [0, 0, 0]
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def timer_callback(self, event):
        robot_pose_msg = Float32MultiArray()
        robot_pose_msg.data = self.robot_pose
        self.odom_info_pub.publish(robot_pose_msg)

    def odom_callback(self, msg):
        if self.count % 300 != 0:
            return
        """Callback for /odom topic to update and publish robot pose."""
        try:
            self.odom_pose = msg.pose.pose

            # Extract position and orientation
            position = [self.odom_pose.position.x, self.odom_pose.position.y, self.odom_pose.position.z]
            orientation = [self.odom_pose.orientation.x, self.odom_pose.orientation.y,
                           self.odom_pose.orientation.z, self.odom_pose.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(orientation)

            # Publish robot pose as Float32MultiArray
            robot_pose_msg = Float32MultiArray()
            self.robot_pose = position + [roll, pitch, yaw]
        except Exception as e:
            rospy.logerr(f"Error in odom_callback: {e}")


    def run(self):
        """Keep the node running."""
        rospy.loginfo("Studio Debug Interface Status Publisher is running...")
        rospy.spin()


if __name__ == "__main__":
    try:
        publisher = RobotStatusPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Status Publisher shutting down.")

