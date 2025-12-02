#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 控制相关工具：服务调用与模式切换
- call_ik_srv: 调用 /ik/two_arm_hand_pose_cmd_srv
- call_change_arm_ctrl_mode_service: humanoid_change_arm_ctrl_mode 模式切换
- call_mm_mpc_control_service: /mobile_manipulator_mpc_control 模式切换
- enable_mm_wbc_arm_trajectory_control: 调用 /enable_mm_wbc_arm_trajectory_control 使能
- enable_wbc_arm_trajectory_control: 调用 /enable_wbc_arm_trajectory_control 使能
"""
import rospy
from kuavo_msgs.srv import twoArmHandPoseCmdSrv, changeArmCtrlMode, fkSrv


def call_ik_srv(eef_pose_msg):
    """调用双臂 IK 服务，成功返回响应，否则返回 None。"""
    service_name = '/ik/two_arm_hand_pose_cmd_srv'
    rospy.wait_for_service(service_name)
    try:
        ik_srv = rospy.ServiceProxy(service_name, twoArmHandPoseCmdSrv)
        res = ik_srv(eef_pose_msg)
        return res
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None


def call_change_arm_ctrl_mode_service(arm_ctrl_mode: int) -> bool:
    """切换手臂控制模式。返回是否成功。"""
    result = True
    service_name = 'humanoid_change_arm_ctrl_mode'
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        change_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo('Service call successful')
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f'Service {service_name} not available')
        result = False
    finally:
        return result


essential_doc = """
常用控制模式：
- humanoid_change_arm_ctrl_mode: 切换上半身/手臂控制模式
- /mobile_manipulator_mpc_control: 切换移动操作器MPC控制模式
"""


def call_mm_mpc_control_service(control_mode: int) -> bool:
    """切换移动操作器MPC控制模式。常用: 0(None/停止), 1(ArmOnly), 2(BaseArm)"""
    result = True
    service_name = '/mobile_manipulator_mpc_control'
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
        change_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        change_mode(control_mode=control_mode)
        rospy.loginfo('Set %s = %d OK', service_name, control_mode)
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed (%s): %s', service_name, e)
        result = False
    except rospy.ROSException:
        rospy.logwarn('Service %s not available', service_name)
        result = False
    finally:
        return result


def fk_srv_client(joints_for_fk):
    """
    调用正运动学服务，输入关节角度（14维，单位弧度），返回 hand_poses（包含 left_pose 和 right_pose）。
    """
    rospy.wait_for_service('/ik/fk_srv')
    fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
    response = fk_srv(joints_for_fk)
    return response.hand_poses


def enable_mm_wbc_arm_trajectory_control(timeout: float = 1.0) -> bool:
    """调用 /enable_mm_wbc_arm_trajectory_control 服务（设置 control_mode=1）。"""
    service_name = '/enable_mm_wbc_arm_trajectory_control'
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        srv(control_mode=1)
        rospy.loginfo('Set %s control_mode=1 OK', service_name)
        return True
    except rospy.ROSException as e:
        rospy.logwarn('Service %s not available: %s', service_name, e)
        return False
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed (%s): %s', service_name, e)
        return False


def enable_wbc_arm_trajectory_control(timeout: float = 1.0) -> bool:
    """调用 /enable_wbc_arm_trajectory_control 服务（设置 control_mode=1）。"""
    service_name = '/enable_wbc_arm_trajectory_control'
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        srv(control_mode=1)
        rospy.loginfo('Set %s control_mode=1 OK', service_name)
        return True
    except rospy.ROSException as e:
        rospy.logwarn('Service %s not available: %s', service_name, e)
        return False
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed (%s): %s', service_name, e)
        return False
