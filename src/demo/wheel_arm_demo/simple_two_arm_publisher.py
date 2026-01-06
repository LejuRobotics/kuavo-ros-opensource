#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的双臂手部位姿命令发布脚本
Simple publisher for /mm/two_arm_hand_pose_cmd topic
"""

import rospy
import numpy as np
from std_msgs.msg import Header
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose, ikSolveParam
import tf.transformations as tf_trans

def create_simple_pose(x, y, z, roll=0, pitch=0, yaw=0):
    """创建简单的手部位姿"""
    pose = armHandPose()
    
    # 位置
    pose.pos_xyz = [x, y, z]
    
    # 姿态（欧拉角转四元数）
    quat = tf_trans.quaternion_from_euler(
        np.radians(roll), 
        np.radians(pitch), 
        np.radians(yaw)
    )
    # 手动归一化四元数
    quat_norm = np.linalg.norm(quat)
    if quat_norm > 1e-8:  # 避免除零
        quat_normalized = quat / quat_norm
    else:
        quat_normalized = np.array([0.0, 0.0, 0.0, 1.0])  # 默认单位四元数
    pose.quat_xyzw = [quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]]
    
    # 默认值
    pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    pose.joint_angles = [0.0] * 7
    
    return pose

def create_default_ik_param():
    """创建默认IK参数"""
    param = ikSolveParam()
    param.major_optimality_tol = 1e-6
    param.major_feasibility_tol = 1e-6
    param.minor_feasibility_tol = 1e-6
    param.major_iterations_limit = 1000
    param.oritation_constraint_tol = 0.01
    param.pos_constraint_tol = 0.01
    param.pos_cost_weight = 1.0
    return param

def slerp_quaternion(q1, q2, t):
    """
    四元数球面线性插值 (SLERP)
    q1, q2: 四元数 [x, y, z, w]
    t: 插值参数，范围 [0, 1]
    """
    # 计算点积
    dot = np.dot(q1, q2)
    
    # 如果点积为负，取反一个四元数以获得最短路径
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    
    # 如果四元数非常接近，使用线性插值避免数值问题
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)
    
    # 计算夹角
    theta_0 = np.arccos(np.abs(dot))
    sin_theta_0 = np.sin(theta_0)
    
    # 计算插值权重
    theta = theta_0 * t
    s1 = np.sin(theta) / sin_theta_0
    s0 = np.cos(theta) - dot * s1
    
    return s0 * q1 + s1 * q2

def interpolate_pose(start_pose, target_pose, t):
    """
    在起始位姿和目标位姿之间进行插值
    start_pose, target_pose: [x, y, z, roll, pitch, yaw] (角度制)
    t: 插值参数，范围 [0, 1]，0表示起始位姿，1表示目标位姿
    返回: [x, y, z, roll, pitch, yaw]
    """
    # 位置插值
    pos = [
        start_pose[0] + (target_pose[0] - start_pose[0]) * t,
        start_pose[1] + (target_pose[1] - start_pose[1]) * t,
        start_pose[2] + (target_pose[2] - start_pose[2]) * t
    ]
    
    # 姿态插值：转换为四元数进行SLERP插值
    start_quat = tf_trans.quaternion_from_euler(
        np.radians(start_pose[3]), np.radians(start_pose[4]), np.radians(start_pose[5])
    )
    target_quat = tf_trans.quaternion_from_euler(
        np.radians(target_pose[3]), np.radians(target_pose[4]), np.radians(target_pose[5])
    )
    
    # 球面线性插值
    interpolated_quat = slerp_quaternion(start_quat, target_quat, t)
    
    # 转换回欧拉角
    roll, pitch, yaw = tf_trans.euler_from_quaternion(interpolated_quat)
    
    return pos + [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]

def main():
    rospy.init_node('simple_two_arm_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    
    # 等待连接
    rospy.sleep(1.0)
    
    # 定义目标位置（去掉起始位置，直接发送目标）
    # 格式: [x, y, z, roll, pitch, yaw] (角度制)
    left_target = [0.4, 0.150, 0.65, 0.0, -90.0, 0.0]   # 左手目标位置
    right_target = [0.4, -0.150, 0.65, 0.0, -90.0, 0.0] # 右手目标位置
    
    rospy.loginfo("发送双臂目标位姿...")
    rospy.loginfo(f"左手目标: {left_target[:3]}, RPY=[{left_target[3]:.1f}°, {left_target[4]:.1f}°, {left_target[5]:.1f}°]")
    rospy.loginfo(f"右手目标: {right_target[:3]}, RPY=[{right_target[3]:.1f}°, {right_target[4]:.1f}°, {right_target[5]:.1f}°]")
    
    # 创建消息
    msg = twoArmHandPoseCmd()
    
    # 设置消息头
    msg.hand_poses.header = Header()
    msg.hand_poses.header.stamp = rospy.Time.now()
    msg.hand_poses.header.frame_id = "base_link"
    
    # 直接设置目标位姿
    msg.hand_poses.left_pose = create_simple_pose(
        x=left_target[0], y=left_target[1], z=left_target[2],
        roll=left_target[3], pitch=left_target[4], yaw=left_target[5]
    )
    
    msg.hand_poses.right_pose = create_simple_pose(
        x=right_target[0], y=right_target[1], z=right_target[2],
        roll=right_target[3], pitch=right_target[4], yaw=right_target[5]
    )
    
    # 设置IK参数
    msg.use_custom_ik_param = True
    msg.joint_angles_as_q0 = False
    msg.ik_param = create_default_ik_param()
    
    # 坐标系: 1 = world frame
    msg.frame = 1
    
    # 发布消息（发送几次确保接收到）
    pub.publish(msg)
    
    rospy.loginfo("目标位姿发送完成！")
    rospy.loginfo("机器人双臂应该开始移动到目标位置")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
