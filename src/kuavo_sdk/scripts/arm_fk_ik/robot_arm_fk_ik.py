#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# from ....manipulation_nodes.kuavo_msgs.srv import fkSrv
from kuavo_msgs.srv import fkSrv
# from kuavo_msgs.srv import fkSrv
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
import math
import argparse
import numpy as np
import time


# 自定义ik参数
use_custom_ik_param = True
# 使用默认的关节角度作为ik的初始预测
joint_angles_as_q0 = False 
# 创建ikSolverParam对象
ik_solve_param = ikSolveParam()
# 设置ikSolveParam对应参数
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 100
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 
ik_solve_param.pos_cost_weight = 0.0 

# 获取机器人版本
def get_parameter(param_name):
    try:
        # 获取参数值
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！程序退出。")
        rospy.signal_shutdown("参数获取失败") 
        return None

# FK正解服务
def fk_srv_client(joint_angles):
    # 确保要调用的服务可用
    rospy.wait_for_service('/ik/fk_srv')

    # 初始化服务代理
    fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
    # 调取服务并获得响应
    fk_result = fk_srv(joint_angles)
    # 打印是否求解成功
    print("FK result:", fk_result.success)
    # 返回正解结果（手臂位置姿态等信息）
    return fk_result.hand_poses


# IK 逆解服务
def call_ik_srv(eef_pose_msg):
    # 确保要调用的服务可用
    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    try:
        # 初始化服务代理
        ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
        # 调取服务并获得响应
        res = ik_srv(eef_pose_msg)
        # 返回逆解结果
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False, []


# 设置手臂运动模式
def set_arm_control_mode(mode):
    # 创建服务代理，用于与服务通信
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # 创建请求对象
    request = changeArmCtrlModeRequest()
    request.control_mode = mode  # 设置请求的控制模式

    # 发送请求并接收响应
    response = arm_traj_change_mode_client(request)

    if response.result:
        # 如果响应结果为真，表示成功更改控制模式
        rospy.loginfo(f"Successfully changed arm control mode to {mode}: {response.message}")
    else:
        # 如果响应结果为假，表示更改控制模式失败
        rospy.logwarn(f"Failed to change arm control mode to {mode}: {response.message}")


# 发布手臂目标姿态
def publish_arm_target_poses(times, values):
    # 创建Publisher对象
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建消息对象并设置传入的times和values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("正在将手臂目标姿态发布到话题 'kuavo_arm_target_poses'")

    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    pub.publish(msg)
    rospy.loginfo("消息已发布。")


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("robot_arm_fk_ik_node", anonymous=True)

    # 获取机器人版本
    robot_version = get_parameter('robot_version')
    # 设置手臂运动模式为外部控制
    set_arm_control_mode(2)

    # 创建请求对象（单位：弧度）
    def start_with_version(version_number:int, series:int):
        """判断版本号是否属于某系列"""
        # PPPPMMMMN
        MMMMN_MASK = 100000
        return (version_number % MMMMN_MASK) == series
    # 42
    if start_with_version(robot_version, 42):
        joint_angles_optiops = {
        1: [-1.0, 0.3, -1.0, 0.19, 1.0, -0.5, -0.3,
        -1.38, -1.39, -0.29, -0.43, -0.5, -0.17, 0.75],
        2: [-1.7, 0.8, -1.57, -1.4, 0.0, -0.8, 0.0,
         -1.6, -0.8, 1.57, -1.4, 0.0, -0.8, 0.0],
        3: [-1.1, -0.25, -1.0, -0.12, 0.59, 1.2, -0.5,
         -1.25, 0.3, 1.0, -0.15, 0.37, -1.0, -1.0],
        }
    
    # 45 or 49
    if start_with_version(robot_version, 45) or start_with_version(robot_version, 49):
        joint_angles_optiops = {
        1: [-1.0, 1.0, -0.3, -1.2, 0.0, -0.5, -0.2,
            -1.9, -0.5, -0.0, -1.0, -0.0, 0.5, 0.65],
        2: [-1.8, 1.0, -1.5, -1.8, 0.0, -0.0, -0.8,
            -1.8, -1.0, 1.5, -1.8, 0.0, -0.0, -0.8],
        3: [0.4, 1.0, 1.2, -1.5, 1.0, 0.4, -0.75,
            0.4, -0.5, -1.3, -1.5, -1.0, -0.7, -0.5],
        }
#0.6, 1.0, 1.2, -1.0, 1.0, 0.4, -0.75,
#0.6, -0.5, -1.3, -1.0, -1.0, -0.7, -0.5
    # 解析命令行参数  
    parser = argparse.ArgumentParser(description="选择不同的 joint_angles")
    parser.add_argument("--joint_angles_id", type=int, choices=[1, 2, 3], required=True, help="选择 joint_angles 的 ID (1-3)")
    args = parser.parse_args()

    # 根据传入的 joint_angles_id 选择对应的 joint_angles
    joint_angles = joint_angles_optiops[args.joint_angles_id]

    # 调用 FK 正解服务
    fk_hand_poses = fk_srv_client(joint_angles)

      # 打印部分正解结果
    if fk_hand_poses is not None:
        print("left eef position:", fk_hand_poses.left_pose.pos_xyz)
        print("left eef position quat:", fk_hand_poses.left_pose.quat_xyzw)
        print("right eef position: ", fk_hand_poses.right_pose.pos_xyz)
        print("right eef position quat:", fk_hand_poses.right_pose.quat_xyzw)
    else:
        print("No hand poses returned")


    degrees_list = [math.degrees(rad) for rad in joint_angles]
    # 调用函数并传入times和values
    publish_arm_target_poses([3], degrees_list)
    print("已到达正解给定位置，正解结束")
    
    time.sleep(10)

    # 回到初始位置
    publish_arm_target_poses([3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print("回到初始位置，开始逆解")

    time.sleep(10)

    # 创建请求对象
    eef_pose_msg = twoArmHandPoseCmd()
    # 设置请求参数
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)


    # 设置左手末端执行器的位置和姿态
    eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array(fk_hand_poses.left_pose.pos_xyz)
    eef_pose_msg.hand_poses.left_pose.quat_xyzw = fk_hand_poses.left_pose.quat_xyzw
    eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
    # 设置右手末端执行器的位置和姿态
    eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array(fk_hand_poses.right_pose.pos_xyz)
    eef_pose_msg.hand_poses.right_pose.quat_xyzw = fk_hand_poses.right_pose.quat_xyzw
    eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)


    # 调用 IK 逆解服务
    res = call_ik_srv(eef_pose_msg)

    # 逆解成功
    if(res.success):
        l_pos = res.hand_poses.left_pose.pos_xyz
        l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos = res.hand_poses.right_pose.pos_xyz
        r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)
        
        # 打印部分逆解结果
        print(f"time_cost: {res.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
        print(f"left_joint_angles: {res.hand_poses.left_pose.joint_angles}")
        print(f"right_joint_angles: {res.hand_poses.right_pose.joint_angles}")


        joint_fk_angles = res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
        degrees_list = [math.degrees(rad) for rad in joint_fk_angles]
        # 调用函数并传入times和values
        publish_arm_target_poses([3], degrees_list)
        print("完成逆解并根据逆解结果到达定位置")
        time.sleep(15)
        # 回到初始位置
        publish_arm_target_poses([3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(8)
        set_arm_control_mode(1)
        print("测试程序结束")

