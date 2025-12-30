#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam, twoArmHandPose, robotHandPosition
from kuavo_msgs.srv import changeArmCtrlMode
import numpy as np
from key_listener import KeyListener  # Import KeyListener
import argparse  # 导入 argparse 模块
import tf2_ros
import tf
import geometry_msgs.msg
import sys

# 全局变量用于存储当前位置
left_current_pos = [0.0, 0.0, 0.0]
right_current_pos = [0.0, 0.0, 0.0]
# decide use custom ik param or not
use_custom_ik_param = True
# joint angles as initial guess for ik
joint_angles_as_q0 = False # True for custom initial guess, False for default initial guess
# ik solver param
ik_solve_param = ikSolveParam()
# snopt params
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 100
# constraint and cost params
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 # work when pos_cost_weight==0.0
ik_solve_param.pos_cost_weight = 10.0 # If U need high accuracy, set this to 0.0 !!!

close_hand = [100, 100, 80, 75, 75, 75]    # catch pose
open_hand = [0, 100, 0, 0, 0, 0]          # open pose

def get_key():
    """
    获取键盘输入
    """
    return input("Enter command (w/s/a/d/q/e), or 'quit' to exit: ")

def round_position(pos):
    """
    将位置数组中的所有值四舍五入到小数点后一位
    """
    return [round(x, 2) for x in pos]

def get_tf_position(tf_buffer, target_frame, timeout=1.0):
    """
    从tf获取目标frame相对于base_link的位置
    """
    try:
        trans = tf_buffer.lookup_transform('base_link', target_frame, rospy.Time(0), rospy.Duration(timeout))
        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"无法获取 {target_frame} 的tf信息: {e}")
        return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Control robot arm via keyboard.')
    parser.add_argument('--hand', choices=['left', 'right'], default='right', help='Choose which hand to control (left or right)')
    parser.add_argument('--step', type=float, default=0.05, help='Step size for movement (default: 0.05)')
    args = parser.parse_args()
    rospy.init_node("keyboard_control_robot_arm_demo", anonymous=True)
    
    # 等待手臂控制模式切换服务
    rospy.loginfo("等待手臂控制模式切换服务...")
    rospy.wait_for_service('/change_arm_ctrl_mode')
    try:
        change_mode = rospy.ServiceProxy('/change_arm_ctrl_mode', changeArmCtrlMode)
        resp = change_mode(2)  # 切换到外部控制模式
        if resp.result:
            rospy.loginfo("成功切换到外部控制模式")
        else:
            rospy.logerr(f"切换控制模式失败: {resp.message}")
            sys.exit(1)
    except rospy.ServiceException as e:
        rospy.logerr(f"调用服务失败: {e}")
        sys.exit(1)

    pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)

    # 设置tf监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)  # 等待tf缓存数据

    # 尝试从tf获取初始位置
    left_init_pose_xyz = get_tf_position(tf_buffer, 'zarm_l7_end_effector')
    right_init_pose_xyz = get_tf_position(tf_buffer, 'zarm_r7_end_effector')

    # 如果无法获取tf信息，使用默认值
    if left_init_pose_xyz is None:
        left_init_pose_xyz = [-0.05, 0.25, 0.04]
        rospy.loginfo("使用左手默认初始位置")
    else:
        rospy.loginfo("使用左手tf位置作为初始位置")

    if right_init_pose_xyz is None:
        right_init_pose_xyz = [-0.05, -0.25, 0.04]
        rospy.loginfo("使用右手默认初始位置")
    else:
        rospy.loginfo("使用右手tf位置作为初始位置")

    init_pose_quat = [0.0, -0.706825181105366, 0.0, 0.7073882691671997]
    rate = rospy.Rate(10)  # 降低频率以便于输入

    # 添加位置变量和步长
    current_pos = left_init_pose_xyz.copy() if args.hand == 'left' else right_init_pose_xyz.copy()
    step = args.step

    kl = KeyListener()  # Initialize KeyListener

    # Define key control for position
    key_to_position = {
        'w': (0, step), 
        's': (0, -step),
        'a': (1, -step),
        'd': (1, step),
        'q': (2, step),
        'e': (2, -step)
    }

    key_to_hand_pose = {
        'h': close_hand,
        'k': open_hand
    }

    def arm_pose_callback(key):
        if key in key_to_position:
            axis, delta = key_to_position[key]
            global current_pos
            current_pos[axis] += delta
            current_pos = round_position(current_pos)  # Round position
            print(f"Updated position: {current_pos}")
            eef_pose_msg = twoArmHandPoseCmd()
            eef_pose_msg.ik_param = ik_solve_param
            eef_pose_msg.use_custom_ik_param = use_custom_ik_param
            eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0

            eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7) # rads
            eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

            eef_pose_msg.hand_poses.left_pose.quat_xyzw = init_pose_quat
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

            eef_pose_msg.hand_poses.right_pose.quat_xyzw = init_pose_quat
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
            if args.hand == 'left':
                eef_pose_msg.hand_poses.left_pose.pos_xyz = current_pos
                eef_pose_msg.hand_poses.right_pose.pos_xyz = right_init_pose_xyz
            else:
                eef_pose_msg.hand_poses.left_pose.pos_xyz = left_init_pose_xyz
                eef_pose_msg.hand_poses.right_pose.pos_xyz = current_pos
            pub.publish(eef_pose_msg)

    def control_hand_pose(key):
        if key in key_to_hand_pose:
            hand_pose = key_to_hand_pose[key]
            hand_pose_msg = robotHandPosition()
            hand_pose_msg.left_hand_position = hand_pose
            hand_pose_msg.right_hand_position = hand_pose
            control_hand_pub.publish(hand_pose_msg)
            
    # Register key callbacks
    for key in key_to_position.keys():
        kl.register_callback(key, arm_pose_callback)
    
    for key in key_to_hand_pose.keys():
        kl.register_callback(key, control_hand_pose)

    try:
        print("\033[96m---------------------------------------------\n"
            "-位置控制：\n"
            "  前后:[按键<w>, 按键<s>]\n"
            "  左右:[按键<a>, 按键<d>]\n"
            "  上下:[按键<q>, 按键<e>]\n"
            "-手部控制：\n"
            "  抓取:[按键<h>]\n"
            "  松开:[按键<k>]\n"
            "----------------------------------------------\033[0m")
        # 按键监听
        kl.loop_control()
    except KeyboardInterrupt:
        kl.stop()
