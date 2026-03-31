#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from motion_capture_ik.msg import twoArmHandPoseCmd, ikSolveParam
from motion_capture_ik.srv import twoArmHandPoseCmdSrv
from sensor_msgs.msg import JointState

import numpy as np
import tf.transformations as tf_trans
import copy
# from arm_kinematics import quat_to_rot, rot_to_quat

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


def quat_to_rot(q):
    w, x, y, z = q
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]], dtype=np.float64)
    return R

def rot_to_quat(R):
    w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return np.array([w, x, y, z])

def get_joint_states_msg(q_now):
    msg = JointState()
    msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
    msg.header.stamp = rospy.Time.now()
    msg.position = 180.0 / np.pi * np.array(q_now)
    return msg

def get_eef_pose_msg(l_pos, l_quat, r_pos, r_quat):
    eef_pose_msg = twoArmHandPoseCmd()
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0

    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7) # rads
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

    eef_pose_msg.hand_poses.left_pose.pos_xyz = l_pos
    eef_pose_msg.hand_poses.left_pose.quat_xyzw = l_quat
    eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

    eef_pose_msg.hand_poses.right_pose.pos_xyz = r_pos
    eef_pose_msg.hand_poses.right_pose.quat_xyzw = r_quat
    eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    return eef_pose_msg

# def inter
def get_next_pose(xyz, quat, delta_pos, delta_euler):
    # pos
    xyz = xyz + np.array(delta_pos)
    # quat
    euler_delta = np.array(delta_euler)
    # print(f"euler_delta: {euler_delta}")
    quat_delta_xyzw = tf_trans.quaternion_from_euler(euler_delta[0], euler_delta[1], euler_delta[2], 'sxyz')
    quat_delta_wxyz = np.array([quat_delta_xyzw[3], *quat_delta_xyzw[:3]])
    R_delta = quat_to_rot(quat_delta_wxyz)
    quat_wxyz = [quat[3], quat[0], quat[1], quat[2]]
    R_wxyz = quat_to_rot(quat_wxyz)
    R_res = R_wxyz @ R_delta
    quat_wxyz = rot_to_quat(R_res)
    quat = np.array([*quat_wxyz[1:], quat_wxyz[0]])
    return xyz, quat

if __name__ == "__main__":
    rospy.init_node("sim_ik_cmd", anonymous=True)
    pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    pub_result = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    xyz_init = np.array([0.4, 0.3, 0.1])
    quat_init = np.array([0.0, -0.70710678, 0.0, 0.70710678])
    record_data = []
    delta_poses =[[[0.00, 0.0, 0.00], [0.0, 0.0, 0.0]],
                  [[0.00, 0.0, 0.00], [-np.pi/2, 0.0, 0.0]],
                  [[0.00, 0.0, 0.10], [-np.pi/2, 0.0, 0.0]],
                  [[0.00, -0.05, 0.20], [-np.pi/2, 0.0, 0.0]],
                  [[0.00, -0.10, 0.30], [-np.pi/2, 0.0, 0.0]],
                  [[0.00, -0.15, 0.30], [-np.pi/2, 0.0, 0.0]],
                  ]
    # delta_poses =[[[0.0, 0.0, 0.0],  [0.0, 0.0, 0.0]],
    #               [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    #               [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    #               ]
    xyz, quat = xyz_init, quat_init
    for i in range(len(delta_poses)):
        xyz, quat = get_next_pose(xyz_init, quat_init, delta_poses[i][0], delta_poses[i][1])
        record_data.append(np.concatenate([xyz, quat]))


    record_data = np.array(record_data)
    print(f"data size: {len(record_data)}")
    rate = rospy.Rate(1) # 1/5=0.2s maximum value
    idx = 0
    forward_direction = True
    # 循环读取数据并发布
    while not rospy.is_shutdown() and idx < len(record_data):
        right_xyz = copy.deepcopy(record_data[idx, :3])
        right_xyz[1] *= -1
        right_quat_xyzw = copy.deepcopy(record_data[idx, -4:])
        right_quat_wxyz = np.array([right_quat_xyzw[3], *right_quat_xyzw[:3]])
        quat_delta_xyzw = tf_trans.quaternion_from_euler(np.pi, 0, 0, 'sxyz')
        quat_delta_wxyz = np.array([quat_delta_xyzw[3], *quat_delta_xyzw[:3]])
        R_right = quat_to_rot(right_quat_wxyz) @ quat_to_rot(quat_delta_wxyz)
        right_quat_wxyz = rot_to_quat(R_right)
        right_quat = np.array([*right_quat_wxyz[1:], right_quat_wxyz[0]])
        
        
        print(f"record_data[{idx}]:\n {record_data[idx]}")
        eef_pose_msg = get_eef_pose_msg(record_data[idx, :3], record_data[idx, -4:], right_xyz, right_quat)
        
        pub.publish(eef_pose_msg) # 话题
        rate.sleep()
        idx = idx + 1
        # idx = idx + 1 if forward_direction else idx - 1
        # if idx == len(record_data) - 1:
        #     forward_direction = False
        # elif idx == 0:
        #     forward_direction = True
        
        # print(f"eef_pose_msg[{idx}]:\n {eef_pose_msg}")
