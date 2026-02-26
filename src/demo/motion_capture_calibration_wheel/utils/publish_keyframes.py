#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
发布躯干位置和手臂关节角度
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolRequest
from kuavo_msgs.srv import changeTorsoCtrlMode
import numpy as np


def set_mpc_control_mode(control_mode):
    rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
    service = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
    resp = service(control_mode=control_mode)
    if resp.result:
        rospy.loginfo(f"MPC控制模式: 已切换到模式 {control_mode}")
        return True
    else:
        rospy.logerr(f"MPC控制模式切换失败: {resp.message}")
        return False
   


def set_arm_quick_mode(enable):
    """设置手臂快速模式"""
    rospy.wait_for_service('/enable_lb_arm_quick_mode')
    try:
        service = rospy.ServiceProxy('/enable_lb_arm_quick_mode', SetBool)
        req = SetBoolRequest()
        req.data = enable
        resp = service(req)
        if resp.success:
            rospy.loginfo(f"手臂快速模式: {'启用' if enable else '禁用'}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"设置快速模式失败: {e}")
        return False


def publish_torso_pose(pub, torso_pose):
    """
    发布躯干位置
    
    Args:
        pub: 发布器
        torso_pose: 躯干位姿 [x, y, z, roll, pitch, yaw]
    """
    msg = Twist()
    msg.linear.x = float(torso_pose[0])
    msg.linear.y = float(torso_pose[1])
    msg.linear.z = float(torso_pose[2])
    msg.angular.x = float(torso_pose[3])
    msg.angular.y = float(torso_pose[4])
    msg.angular.z = float(torso_pose[5])
    pub.publish(msg)


def publish_arm_joints(pub, joint_angles):
    """
    发布手臂关节角度
    
    Args:
        pub: 发布器
        joint_angles: 14个关节角度（单位：度）
    """
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 
                   'joint8', 'joint9', 'joint10', 'joint11', 'joint12', 'joint13', 'joint14']
    
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = joint_names
    msg.position = [float(x) for x in joint_angles]  # 单位：度
    pub.publish(msg)


def publish_single_frame(torso_pub, arm_pub, frame):
    """
    发布单个关键帧的躯干位置和手臂关节角度
    
    Args:
        torso_pub: 躯干位置发布器
        arm_pub: 手臂关节角度发布器
        frame: 关键帧，包含 torso_pose 和 joint_angles
    """
    publish_torso_pose(torso_pub, frame['torso_pose'])
    publish_arm_joints(arm_pub, frame['joint_angles'])

