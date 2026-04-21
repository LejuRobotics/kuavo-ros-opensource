#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam


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


if __name__ == "__main__":
    # 确保要调用的服务可用
    rospy.init_node("robot_arm_ik_srv_node", anonymous=True)
    time.sleep(1.5)

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
    eef_pose_msg.hand_poses.left_pose.pos_xyz =  np.array([0.45,0.25,0.11988012])
    eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0,-0.70682518,0.0,0.70738827] # 四元数
    eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

    # 设置右手末端执行器的位置和姿态
    eef_pose_msg.hand_poses.right_pose.pos_xyz =  np.array([0.45,-0.25,0.11988012])
    eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,-0.70682518,0.0,0.70738827] # 四元数
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
    # 逆解失败
    else:
        print(f"ik success: {res.success}")