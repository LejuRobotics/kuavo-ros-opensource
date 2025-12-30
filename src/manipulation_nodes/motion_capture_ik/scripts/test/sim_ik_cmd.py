#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from sensor_msgs.msg import JointState

import numpy as np

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
ik_solve_param.pos_constraint_tol = 1e-3 # 0.001m, work when pos_cost_weight==0.0
ik_solve_param.pos_cost_weight = 0.0 # If U need high accuracy, set this to 0.0 !!!

def call_ik_srv(eef_pose_msg):
    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    try:
        ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
        res = ik_srv(eef_pose_msg)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False, []

def get_joint_states_msg(q_now):
    msg = JointState()
    msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
    msg.header.stamp = rospy.Time.now()
    msg.position = 180.0 / np.pi * np.array(q_now)
    return msg

if __name__ == "__main__":
    rospy.init_node("sim_ik_cmd", anonymous=True)
    pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    pub_result = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    record_data = []
    r = 0.15
    w = 0.05
    bias = 0.15
    for i in range(int(2*np.pi/w)):
        # xyz = [0.2+bias, 0.26 + r*np.sin(w*i), 0.2 + r*np.cos(w*i)]
        xyz = [0.3+bias, 0.25, 0.18+r*np.cos(w*i)]
        quat = [0.0, -0.706825181105366, 0.0, 0.7073882691671997]
        record_data.append(np.concatenate([xyz, quat]))
    record_data = np.array(record_data)
    print(f"data size: {len(record_data)}")
    rate = rospy.Rate(100) # 1/5=0.2s maximum value
    idx = 0
    forward_direction = True
    # 循环读取数据并发布
    while not rospy.is_shutdown():# and idx <= 10:
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.ik_param = ik_solve_param
        eef_pose_msg.use_custom_ik_param = use_custom_ik_param
        eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0

        eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7) # rads
        eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

        eef_pose_msg.hand_poses.left_pose.pos_xyz = record_data[idx, :3]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = record_data[idx, -4:]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array(record_data[idx, :3])
        eef_pose_msg.hand_poses.right_pose.pos_xyz[1] = -record_data[idx, 1]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = record_data[idx, -4:]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
        
        # pub.publish(eef_pose_msg) # 话题
        res = call_ik_srv(eef_pose_msg) # 服务
        if(res.success):
            l_pos = res.hand_poses.left_pose.pos_xyz
            l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
            r_pos = res.hand_poses.right_pose.pos_xyz
            r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)
            print(f"time_cost: {res.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
            pub_result.publish(get_joint_states_msg(res.q_arm))
        else:
            print(f"success: {res.success}")
        rate.sleep()
        idx = idx + 1 if forward_direction else idx - 1
        if idx == len(record_data) - 1:
            forward_direction = False
        elif idx == 0:
            forward_direction = True
        
        # print(f"eef_pose_msg[{idx}]:\n {eef_pose_msg}")
