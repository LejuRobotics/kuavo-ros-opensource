#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
外部 odom->base_link TF 发布脚本

订阅 odom_topic（默认 /odom），将 nav_msgs/Odometry 中的位姿发布为 odom->base_link 的 TF 变换。
当此脚本运行时，MobileManipulatorDummyVisualization 会检测到外部 odom 并停止发布内部的 odom->base_link，
避免重复发布。

/odom 数据来源（取决于运行场景）：
  - Gazebo 仿真：gazebo_shm_interface.cpp 发布，数据来自 model_->WorldPose()（Gazebo 世界坐标系下的机器人位姿）
  - MuJoCo 仿真：mujoco_node.cc 发布
  - 实机：wheel_bridge / auto_wheel_bridge 或 humanoid_estimation 等发布

用法：
    rosrun humanoid_wheel_interface_ros external_odom_tf_publisher.py

或指定 odom 话题（默认 /odom）：
    rosrun humanoid_wheel_interface_ros external_odom_tf_publisher.py _odom_topic:=/your_odom_topic
"""

import sys

try:
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from std_msgs.msg import Bool
except ImportError as e:
    print("请确保在 ROS 环境下运行:", e)
    sys.exit(1)


def main():
    rospy.init_node("external_odom_tf_publisher", anonymous=False)
    odom_topic = rospy.get_param("~odom_topic", "/odom")
    base_frame = rospy.get_param("~base_frame", "base_link")
    odom_frame = rospy.get_param("~odom_frame", "odom")

    tf_broadcaster = TransformBroadcaster()
    # 发布 /external_odom/active，通知 C++ 有外部 odom，停止内部发布
    active_pub = rospy.Publisher("/external_odom/active", Bool, queue_size=1, latch=True)
    active_pub.publish(Bool(data=True))

    def odom_cb(msg):
        # odom->base_link 变换全为 0（平移 0，旋转单位四元数）
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = -5.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

    rospy.Subscriber(odom_topic, Odometry, odom_cb, queue_size=10)
    rospy.loginfo(
        "[external_odom_tf] 订阅 %s，发布 odom->base_link TF，并通知内部停止发布", odom_topic
    )
    rospy.spin()


if __name__ == "__main__":
    main()
