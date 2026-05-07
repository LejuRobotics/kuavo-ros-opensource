#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂关节角度命令测试脚本（使用twoArmHandPoseCmd话题，关节空间模式）
测试列表：14个关节角度（左臂7个 + 右臂7个）
"""

import rospy
import numpy as np
from std_msgs.msg import Header, Float32
from kuavo_msgs.msg import twoArmHandPoseCmd, twoArmHandPose, armHandPose, ikSolveParam
import tf.transformations as tf_trans
import lb_ctrl_api as ct

# -------------- 全局变量 --------------
reach_time = 0.0

# -------------- 回调函数 --------------
def time_callback(msg):
    global reach_time
    reach_time = msg.data
    rospy.loginfo(f"reach_time is {reach_time:.3f} s")

# -------------- 业务函数 --------------
def build_hand_pose_with_joints(joint_angles):
    """根据7个关节角度构造 armHandPose（关节空间模式）"""
    pose = armHandPose()
    # 在关节空间模式下，位置和姿态信息可能被忽略，但为了安全都设置为0
    pose.pos_xyz = [0.0, 0.0, 0.0]
    pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
    pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
    pose.joint_angles = [float(angle) for angle in joint_angles]  # 7个关节角度
    return pose

def build_ik_param():
    """构造默认 IK 参数（在关节空间模式下可能不需要，但为了完整性保留）"""
    p = ikSolveParam()
    p.major_optimality_tol = 1e-6
    p.major_feasibility_tol = 1e-6
    p.minor_feasibility_tol = 1e-6
    p.major_iterations_limit = 1000
    p.oritation_constraint_tol = 0.01
    p.pos_constraint_tol = 0.01
    p.pos_cost_weight = 1.0
    return p

def build_two_arm_joint_cmd(left_joints, right_joints):
    """根据左右手7个关节角度快速构造 twoArmHandPoseCmd（关节空间模式）"""
    msg = twoArmHandPoseCmd()
    msg.hand_poses = twoArmHandPose()
    msg.hand_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
    msg.hand_poses.left_pose = build_hand_pose_with_joints(left_joints)
    msg.hand_poses.right_pose = build_hand_pose_with_joints(right_joints)
    msg.use_custom_ik_param = False  # 关节空间模式下不需要IK求解
    msg.joint_angles_as_q0 = True    # 使用提供的关节角度作为初始值
    msg.ik_param = build_ik_param()  # 保留但可能不会被使用
    msg.frame = 5  # 关键：5表示关节空间数据
    return msg

def execute_two_arm_joint_tests():
    """依次发布若干组双臂关节角度，并等待每次运动结束"""
    global reach_time

    rospy.init_node('test_two_arm_joint_publisher', anonymous=True)

    # 发布器保持不变，使用twoArmHandPoseCmd话题
    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    # 订阅到达时间话题（使用关节控制的话题）
    rospy.Subscriber('/lb_arm_joint_reach_time', Float32, time_callback)

    rospy.sleep(1.0)

    # 测试用例列表： (名称, 左臂7关节角度, 右臂7关节角度)
    test_cases = [ 
        ("左右展开", 
         [-30.0, 20.0, 15.0, -45.0, 25.0, 10.0, -35.0],   # 左臂
         [-30.0, -20.0, -15.0, -45.0, -25.0, -10.0, -35.0]),  # 右臂
         
        ("前伸姿势", 
         [-20.0, 30.0, -25.0, -20.0, 40.0, -15.0, 25.0],   # 左臂
         [-20.0, -30.0, 25.0, -20.0, -40.0, 15.0, 25.0]),  # 右臂
         
        ("零位姿势", 
         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 左臂 joints
         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),   # 右臂 joints
    ]

    rospy.loginfo("开始发布双臂关节角度测试数据（关节空间模式）...")
    rospy.loginfo("使用frame=5 (关节空间数据)")

    for idx, (name, left_joints, right_joints) in enumerate(test_cases, 1):
        rospy.loginfo(f"\n=== 第{idx}组测试: {name} ===")
        rospy.loginfo(f"  左臂关节: {left_joints}")
        rospy.loginfo(f"  右臂关节: {right_joints}")

        reach_time = 0.0
        pub.publish(build_two_arm_joint_cmd(left_joints, right_joints))

        # 等待直到收到reach_time
        while reach_time == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # 等待运动完成
        rospy.sleep(reach_time + 0.5)
        rospy.loginfo(f"  {name} 完成!")

    rospy.loginfo("\n所有双臂关节角度测试数据发布完成！")

# -------------- 主入口 --------------
def main():
    try:
        execute_two_arm_joint_tests()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS 中断异常")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")

if __name__ == '__main__':
    main()