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

def interpolate_pose(start_pose, target_pose, t):
    """
    在起始位姿和目标位姿之间进行线性插值
    t: 插值参数，范围 [0, 1]，0表示起始位姿，1表示目标位姿
    """
    # 位置插值
    pos = [
        start_pose[0] + (target_pose[0] - start_pose[0]) * t,
        start_pose[1] + (target_pose[1] - start_pose[1]) * t,
        start_pose[2] + (target_pose[2] - start_pose[2]) * t
    ]
    
    # 姿态插值（简单线性插值，对于小角度变化是合理的）
    yaw = start_pose[3] + (target_pose[3] - start_pose[3]) * t
    
    return pos + [yaw]

def main():
    rospy.init_node('simple_two_arm_publisher', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    
    # 等待连接
    rospy.sleep(1.0)
    
    # 设置发布频率: 10Hz
    rate = rospy.Rate(10)
    
    # 定义起始位置和目标位置
    # 格式: [x, y, z, yaw]
    left_start = [0.0, 0.4, 0.0, 0.0]    # 起始位置
    left_target = [0.4, 0.4, 0.0, 0.0]  # 目标位置
    
    right_start = [0.0, -0.4, 0.0, 0.0]   # 起始位置  
    right_target = [0.4, -0.4, 0.0, 0.0] # 目标位置
    
    # 插值步数（总时间约5秒）
    total_steps = 50
    step_duration = 0.1  # 每步100ms
    
    rospy.loginfo("开始双臂位姿插值发布...")
    rospy.loginfo(f"插值步数: {total_steps} 步")
    rospy.loginfo(f"每步时长: {step_duration} 秒")
    rospy.loginfo(f"总时长: {total_steps * step_duration} 秒")
    rospy.loginfo("左手目标: [1.44, 2.0, 0.8], yaw=30°")
    rospy.loginfo("右手目标: [0.56, 2.0, 0.8], yaw=30°")
    
    # 执行插值发布
    for step in range(total_steps + 1):
        if rospy.is_shutdown():
            break
            
        # 计算插值参数 (0 到 1)
        t = step / total_steps
        
        # 插值计算当前位姿
        left_current = interpolate_pose(left_start, left_target, t)
        right_current = interpolate_pose(right_start, right_target, t)
        
        # 创建消息
        msg = twoArmHandPoseCmd()
        
        # 设置消息头
        msg.hand_poses.header = Header()
        msg.hand_poses.header.stamp = rospy.Time.now()
        msg.hand_poses.header.frame_id = "base_link"
        
        # 设置插值后的位姿
        msg.hand_poses.left_pose = create_simple_pose(
            x=left_current[0], y=left_current[1], z=left_current[2],
            roll=0, pitch=0, yaw=left_current[3]
        )
        
        msg.hand_poses.right_pose = create_simple_pose(
            x=right_current[0], y=right_current[1], z=right_current[2],
            roll=0, pitch=0, yaw=right_current[3]
        )
        
        # 设置IK参数
        msg.use_custom_ik_param = True
        msg.joint_angles_as_q0 = False
        msg.ik_param = create_default_ik_param()
        
        # 坐标系: 2 = local frame
        msg.frame = 2
        
        # 发布消息
        pub.publish(msg)
        
        # 显示进度
        if step % 10 == 0 or step == total_steps:
            rospy.loginfo(f"步骤 {step}/{total_steps} (t={t:.2f})")
            rospy.loginfo(f"  左手: [{left_current[0]:.2f}, {left_current[1]:.2f}, {left_current[2]:.2f}], yaw={left_current[3]:.1f}°")
            rospy.loginfo(f"  右手: [{right_current[0]:.2f}, {right_current[1]:.2f}, {right_current[2]:.2f}], yaw={right_current[3]:.1f}°")
        
        # 按设定频率等待
        rate.sleep()
    
    rospy.loginfo("插值发布完成！")
    rospy.loginfo("机器人应该已经到达目标位置")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
