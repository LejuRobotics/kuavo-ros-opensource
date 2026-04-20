#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂手部位姿命令测试脚本（无插值版）
测试列表：x, y, z, yaw, pitch, roll
"""

import rospy
import numpy as np
from std_msgs.msg import Header, Float32
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose, ikSolveParam
import tf.transformations as tf_trans
import lb_ctrl_api as ct
import argparse

# -------------- 全局变量 --------------
reach_time = 0.0

# -------------- 回调函数 --------------
def time_callback(msg):
    global reach_time
    reach_time = msg.data
    rospy.loginfo(f"reach_time is {reach_time:.3f} s")

# -------------- 业务函数 --------------
def build_hand_pose(x, y, z, yaw, pitch, roll):
    """根据 x,y,z,yaw,pitch,roll 构造 armHandPose"""
    pose = armHandPose()
    pose.pos_xyz = [x, y, z]
    quat = tf_trans.quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
    quat_norm = quat / np.linalg.norm(quat) if np.linalg.norm(quat) > 1e-8 else [0, 0, 0, 1]
    pose.quat_xyzw = quat_norm.tolist()
    pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    pose.joint_angles = [0.0] * 7
    return pose

def build_ik_param():
    """构造默认 IK 参数"""
    p = ikSolveParam()
    p.major_optimality_tol = 1e-6
    p.major_feasibility_tol = 1e-6
    p.minor_feasibility_tol = 1e-6
    p.major_iterations_limit = 1000
    p.oritation_constraint_tol = 0.01
    p.pos_constraint_tol = 0.01
    p.pos_cost_weight = 1.0
    return p

def build_two_arm_pose_cmd(left_xyz_ypr, right_xyz_ypr):
    """根据左右手 [x,y,z,yaw,pitch,roll] 快速构造 twoArmHandPoseCmd"""
    msg = twoArmHandPoseCmd()
    msg.hand_poses = twoArmHandPose()
    msg.hand_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
    msg.hand_poses.left_pose  = build_hand_pose(*left_xyz_ypr)
    msg.hand_poses.right_pose = build_hand_pose(*right_xyz_ypr)
    msg.use_custom_ik_param = True
    msg.joint_angles_as_q0  = False
    msg.ik_param = build_ik_param()
    msg.frame = 2  # world frame
    return msg

def execute_two_arm_tests():
    """依次发布若干组双臂位姿，并等待每次运动结束"""
    global reach_time

    rospy.init_node('test_two_arm_pose_publisher', anonymous=True)

    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    rospy.Subscriber('/lb_arm_ee_reach_time/left', Float32, time_callback)

    ct.set_arm_control_mode(1)  # 重置手臂, 避免奇异点问题
    rospy.sleep(1.0)
    ct.set_arm_control_mode(2)

    # 创建解析器
    parser = argparse.ArgumentParser(description='设置笛卡尔跟踪焦点')
    
    # 添加参数
    parser.add_argument('--focus', '-f', type=str, default='ee',
                       choices=['torso', 'ee'],
                       help='跟踪焦点: torso(躯干) 或 ee(末端)')
    
    # 解析参数
    args = parser.parse_args()

    # 使用参数
    focus_ee = (args.focus == 'ee')  # 简洁写法
    ct.set_focus_ee(focus_ee)

    # 测试用例列表： (名称, 左手[x,y,z,yaw,pitch,roll], 右手[...])
    test_cases = [
        ("左右展开",       [0.1, 0.4, 0.7, 0.0, 0.0, 0.0],   [0.1, -0.4, 0.7, 0.0, 0.0, 0.0]),
        ("前摆臂",         [0.3, 0.4, 0.7, 0.0, -90, 0.0],   [0.3, -0.4, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",         [0.3, 0.2, 0.7, 0.0, -90, 0.0],   [0.3, -0.2, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",         [0.5, 0.2, 0.7, 0.0, -90, 0.0],   [0.5, -0.2, 0.7, 0.0, -90, 0.0]),
        ("前摆臂",         [0.5, 0.2, 0.85, 0.0, -90, 0.0],   [0.5, -0.2, 0.85, 0.0, -90, 0.0]),
        ("前摆臂",         [1.2, 0.2, 0.85, 0.0, -90, 0.0],   [1.2, -0.2, 0.85, 0.0, -90, 0.0]),
        ("前摆臂",         [0.5, 0.2, 0.85, 0.0, -90, 0.0],   [0.5, -0.2, 0.85, 0.0, -90, 0.0]),
    ]

    rospy.loginfo("开始发布双臂位姿测试数据...")

    for idx, (name, left, right) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  左手: {left}, 右手: {right}")

        reach_time = 0.0
        pub.publish(build_two_arm_pose_cmd(left, right))

        while reach_time == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.sleep(reach_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有双臂位姿测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_two_arm_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()