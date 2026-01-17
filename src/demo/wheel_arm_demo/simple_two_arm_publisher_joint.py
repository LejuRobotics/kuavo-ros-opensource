#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose

def main():
    rospy.init_node("two_arm_hand_pose_cmd_sweeper")

    pub = rospy.Publisher("/mm/two_arm_hand_pose_cmd", twoArmHandPoseCmd, queue_size=10)
    rate_hz = rospy.get_param("~rate_hz", 20)
    rate = rospy.Rate(rate_hz)

    # 关节数量（每臂），按你的机器人配置设置；常见7
    num_joints = rospy.get_param("~num_joints", 7)

    # 目标关节索引（第4个关节 -> 索引3）
    target_idx = rospy.get_param("~target_idx", 3)

    # 扫描范围（度）与周期（秒）
    start_deg = rospy.get_param("~start_deg", 0.0)
    end_deg = rospy.get_param("~end_deg", 90.0)
    period_sec = rospy.get_param("~period_sec", 4.0)  # 一个完整往返周期，默认4秒
    half_period = max(1e-3, period_sec / 2.0)

    rospy.loginfo("Publishing to /mm/two_arm_hand_pose_cmd with Frame=joint Space")
    rospy.loginfo("Sweeping joint %d from %.1f deg to %.1f deg", target_idx, start_deg, end_deg)

    t0 = rospy.Time.now().to_sec()
    amplitude = (end_deg - start_deg)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        phase = t % max(1e-3, period_sec)
        # 三角波：0->half_period 线性上升；half_period->period 线性下降（等速）
        if phase < half_period:
            val_deg = start_deg + amplitude * (phase / half_period)
        else:
            val_deg = end_deg - amplitude * ((phase - half_period) / half_period)

        # 构造消息
        msg = twoArmHandPoseCmd()
        msg.frame = 5  # 关节空间
        msg.use_custom_ik_param = False
        msg.joint_angles_as_q0 = False

        # 填充两臂末端位姿（可置零，关节角为主要控制输入）
        msg.hand_poses.left_pose.pos_xyz = [0.0, 0.0, 0.0]
        msg.hand_poses.left_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        msg.hand_poses.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]

        msg.hand_poses.right_pose.pos_xyz = [0.0, 0.0, 0.0]
        msg.hand_poses.right_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        msg.hand_poses.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]

        # 关节角（度）。注意：订阅端会转换为弧度
        left_j = [0.0] * num_joints
        right_j = [0.0] * num_joints
        # 设置第4个关节（索引3）
        if 0 <= target_idx < num_joints:
            left_j[target_idx] = float(-val_deg)
            right_j[target_idx] = float(-val_deg)

        msg.hand_poses.left_pose.joint_angles = left_j
        msg.hand_poses.right_pose.joint_angles = right_j

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass