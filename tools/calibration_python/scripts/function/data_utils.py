#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据加载和ROS服务调用工具函数：
- 加载关节角、位置、工具矩阵等数据文件
- 调用ROS FK/IK服务
- 关节角格式转换（单臂<->双臂）
- 误差计算和统计信息
- ROS服务调用（手臂控制模式切换等）
- 数据保存和SVD配准
"""

import numpy as np
from pathlib import Path
import os
import rospy
from kuavo_msgs.srv import fkSrv, changeArmCtrlMode, twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam


def load_joints_deg(path: Path) -> np.ndarray:
    """
    加载关节角文件（度），自动处理列/行存储格式
    
    Args:
        path: 文件路径，每行7个关节角（度）
    
    Returns:
        np.ndarray: (N, 7) 关节角数组（度）
    """
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    # 支持列存储：如果第一维是7，则转置
    if data.shape[0] == 7:
        data = data.T
    return data  # (N,7) 度


def load_positions(path: Path) -> np.ndarray:
    """
    加载位置文件（米），自动处理列/行存储格式
    
    Args:
        path: 文件路径，每行3个坐标（x, y, z）
    
    Returns:
        np.ndarray: (N, 3) 位置数组（米）
    """
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    # 支持列存储：如果第一维是3，则转置
    if data.shape[0] == 3:
        data = data.T
    return data  # (N,3) 米


def load_pe0(pe0_path: Path) -> np.ndarray:
    """
    加载工具坐标系平移 pe0（从4×4矩阵文件中提取第4列）
    
    Args:
        pe0_path: 工具矩阵文件路径（4×4齐次变换矩阵）
    
    Returns:
        np.ndarray: (4, 1) 齐次坐标（米）
    """
    mat = np.loadtxt(pe0_path)
    return mat[0:4, 3:4]


def call_fk_service(joints_rad_14, arm_side: str = "left", timeout: float = 2.0):
    """
    调用 ROS /ik/fk_srv 服务计算末端位置
    
    Args:
        joints_rad_14: 14个关节角（弧度），可以是列表或numpy数组
        arm_side: 手臂侧，"left" 或 "right"
        timeout: 服务等待超时时间（秒）
    
    Returns:
        np.ndarray: 末端位置 (3,) 米，如果失败返回 None
    """
    service_name = "/ik/fk_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        fk_client = rospy.ServiceProxy(service_name, fkSrv)
        
        # 确保是列表格式
        if isinstance(joints_rad_14, np.ndarray):
            joints_list = joints_rad_14.tolist()
        else:
            joints_list = joints_rad_14
        
        res = fk_client(joints_list)
        if not res.success:
            return None
        
        if arm_side == "left":
            return np.array(res.hand_poses.left_pose.pos_xyz)
        elif arm_side == "right":
            return np.array(res.hand_poses.right_pose.pos_xyz)
        else:
            rospy.logwarn(f"未知的手臂侧: {arm_side}，默认返回右手")
            return np.array(res.hand_poses.right_pose.pos_xyz)
    except rospy.ROSException as e:
        rospy.logwarn(f"FK服务调用失败: {e}")
        return None
    except Exception as e:
        rospy.logwarn(f"FK服务调用异常: {e}")
        return None


def build_dual_arm(theta7: np.ndarray, arm_side: str, init_pos_deg: np.ndarray = None) -> np.ndarray:
    """
    将单臂7关节角扩展为双臂14关节角格式
    
    Args:
        theta7: 单臂关节角（7,）或（N, 7），度或弧度
        arm_side: 手臂侧，"left" 或 "right"
        init_pos_deg: 可选，另一臂的初始位置（7,）度，如果不提供则使用零
    
    Returns:
        np.ndarray: 双臂关节角（14,）或（N, 14）
    """
    if theta7.ndim == 1:
        full = np.zeros(14)
        if arm_side == "left":
            full[:7] = theta7
            if init_pos_deg is not None:
                full[7:] = init_pos_deg
        else:  # right
            full[7:] = theta7
            if init_pos_deg is not None:
                full[:7] = init_pos_deg
        return full
    else:  # (N, 7)
        N = theta7.shape[0]
        full = np.zeros((N, 14))
        if arm_side == "left":
            full[:, :7] = theta7
            if init_pos_deg is not None:
                full[:, 7:] = init_pos_deg
        else:  # right
            full[:, 7:] = theta7
            if init_pos_deg is not None:
                full[:, :7] = init_pos_deg
        return full


def norm_err(a, b):
    """
    计算两个位置数组之间的欧氏距离误差
    
    Args:
        a: 位置数组 (N, 3) 或 (N,)
        b: 位置数组 (N, 3) 或 (N,)
    
    Returns:
        np.ndarray: 每个样本的误差 (N,)
    """
    return np.linalg.norm(a - b, axis=1)


def stats(name, arr, unit="mm"):
    """
    打印统计信息（均值、最大值、最小值、标准差）
    
    Args:
        name: 统计项名称
        arr: 数据数组
        unit: 单位，默认 "mm"（会自动将米转换为毫米）
    """
    if unit == "mm":
        arr_display = arr * 1000.0
        unit_str = "mm"
    else:
        arr_display = arr
        unit_str = unit
    
    print(f"{name}: mean={np.nanmean(arr_display):.3f}{unit_str}, "
          f"max={np.nanmax(arr_display):.3f}{unit_str}, "
          f"min={np.nanmin(arr_display):.3f}{unit_str}, "
          f"std={np.nanstd(arr_display):.3f}{unit_str}")


def call_fk_service_both_arms(joint_angles_rad, timeout: float = 2.0):
    """
    调用 ROS /ik/fk_srv 服务计算左右手末端位置
    
    Args:
        joint_angles_rad: 14个关节角（弧度），可以是列表或numpy数组
        timeout: 服务等待超时时间（秒）
    
    Returns:
        tuple: (left_pos, right_pos) 或 (None, None) 如果失败
            - left_pos: 左手末端位置 (3,) 米
            - right_pos: 右手末端位置 (3,) 米
    """
    if joint_angles_rad is None or len(joint_angles_rad) < 14:
        return None, None
    
    service_name = "/ik/fk_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        fk_client = rospy.ServiceProxy(service_name, fkSrv)
        
        # 确保是列表格式
        if isinstance(joint_angles_rad, np.ndarray):
            joints_list = joint_angles_rad.tolist()
        else:
            joints_list = joint_angles_rad
        
        res = fk_client(joints_list)
        if res.success:
            left_pos = np.array(res.hand_poses.left_pose.pos_xyz)
            right_pos = np.array(res.hand_poses.right_pose.pos_xyz)
            return left_pos, right_pos
    except Exception as exc:
        rospy.logwarn(f"FK服务调用失败: {exc}")
    return None, None


def call_fk_service_with_pose(joint_angles_rad, arm_side="right", timeout=2.0):
    """
    调用正运动学服务，返回位置和四元数
    
    Args:
        joint_angles_rad: 14个关节角（弧度）
        arm_side: 手臂侧，"left" 或 "right"
        timeout: 服务等待超时时间（秒）
    
    Returns:
        tuple: (pos, quat) 或 (None, None) 如果失败
            - pos: 末端位置 (3,) 米
            - quat: 四元数 (4,) [x, y, z, w]
    """
    service_name = "/ik/fk_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        fk_client = rospy.ServiceProxy(service_name, fkSrv)
        
        if isinstance(joint_angles_rad, np.ndarray):
            joints_list = joint_angles_rad.tolist()
        else:
            joints_list = joint_angles_rad
        
        res = fk_client(joints_list)
        if not res.success:
            return None, None
        
        if arm_side == "left":
            pos = np.array(res.hand_poses.left_pose.pos_xyz)
            quat = np.array(res.hand_poses.left_pose.quat_xyzw)  # [x, y, z, w]
        elif arm_side == "right":
            pos = np.array(res.hand_poses.right_pose.pos_xyz)
            quat = np.array(res.hand_poses.right_pose.quat_xyzw)  # [x, y, z, w]
        else:
            rospy.logwarn(f"未知的手臂侧: {arm_side}")
            return None, None
        
        return pos, quat
    except Exception as e:
        rospy.logwarn(f"FK服务调用失败: {e}")
        return None, None


def call_change_arm_ctrl_mode_service(arm_ctrl_mode, timeout: float = 1.0):
    """
    调用服务改变手臂控制模式
    
    Args:
        arm_ctrl_mode: 控制模式 (2为双臂控制模式)
        timeout: 服务等待超时时间（秒）
    
    Returns:
        bool: 是否成功
    """
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        change_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        change_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo(f"手臂控制模式切换成功: {arm_ctrl_mode}")
        return True
    except rospy.ServiceException as e:
        rospy.logwarn(f"服务调用失败: {e}")
        return False
    except rospy.ROSException:
        rospy.logwarn(f"服务 {service_name} 不可用")
        return False


def call_ik_service(pos, quat, arm_side="right", ik_param_dict=None, timeout=2.0,
                    other_arm_pos=None, other_arm_quat=None, q0_joints=None, service_name=None):
    """
    调用逆运动学服务，返回关节角
    
    Args:
        pos: 末端位置 (3,) 米，可以是列表或numpy数组
        quat: 四元数 (4,) [x, y, z, w]，可以是列表或numpy数组
        arm_side: 手臂侧，"left" 或 "right"
        ik_param_dict: IK参数字典，如果为None则使用默认参数
            格式: {
                'major_optimality_tol': float,
                'major_feasibility_tol': float,
                'minor_feasibility_tol': float,
                'major_iterations_limit': int,
                'oritation_constraint_tol': float,
                'pos_constraint_tol': float,
                'pos_cost_weight': float,
            }
        other_arm_pos: 另一只手的末端位置 (3,) 米，用于全身IK约束；None则使用默认值
        other_arm_quat: 另一只手的末端四元数 (4,) [x, y, z, w]；None则使用单位四元数
        q0_joints: 可选，当前求解手臂的关节初值 (7,) 弧度。
            如果提供，则会将 joint_angles_as_q0 设为 True，并使用该关节作为 q0
        timeout: 服务等待超时时间（秒）
        service_name: 服务名称，如果为None则使用默认服务 "/ik/two_arm_hand_pose_cmd_srv"
    
    Returns:
        np.ndarray: 关节角 (7,) 弧度，如果失败返回 None
    """
    if service_name is None:
        service_name = "/ik/two_arm_hand_pose_cmd_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        ik_client = rospy.ServiceProxy(service_name, twoArmHandPoseCmdSrv)
        
        # 创建请求消息
        eef_pose_msg = twoArmHandPoseCmd()
        
        # 设置IK参数
        if ik_param_dict is None:
            # 默认参数
            ik_param_dict = {
                'major_optimality_tol': 1e-3,
                'major_feasibility_tol': 1e-3,
                'minor_feasibility_tol': 1e-3,
                'major_iterations_limit': 100,
                'oritation_constraint_tol': 1e-3,
                'pos_constraint_tol': 1e-3,
                'pos_cost_weight': 0.0,
                # default: pos soft + ori hard (01 -> 1)
                'constraint_mode': 1,
            }
        
        ik_solve_param = ikSolveParam()
        ik_solve_param.major_optimality_tol = ik_param_dict['major_optimality_tol']
        ik_solve_param.major_feasibility_tol = ik_param_dict['major_feasibility_tol']
        ik_solve_param.minor_feasibility_tol = ik_param_dict['minor_feasibility_tol']
        ik_solve_param.major_iterations_limit = ik_param_dict['major_iterations_limit']
        ik_solve_param.oritation_constraint_tol = ik_param_dict['oritation_constraint_tol']
        ik_solve_param.pos_constraint_tol = ik_param_dict['pos_constraint_tol']
        ik_solve_param.pos_cost_weight = ik_param_dict['pos_cost_weight']
        ik_solve_param.constraint_mode = int(ik_param_dict.get('constraint_mode', 1))
        
        eef_pose_msg.ik_param = ik_solve_param
        eef_pose_msg.use_custom_ik_param = True
        
        # 判断是否提供了关节初值
        if q0_joints is not None:
            # 如果提供了关节初值，使用它并设置 joint_angles_as_q0 = True
            eef_pose_msg.joint_angles_as_q0 = True
            if not isinstance(q0_joints, np.ndarray):
                q0_joints = np.array(q0_joints)
            if q0_joints.shape[0] != 7:
                rospy.logwarn(
                    f"q0_joints 长度不是7（实际为 {q0_joints.shape[0]}），将忽略并使用默认值"
                )
                eef_pose_msg.joint_angles_as_q0 = False
                eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
                eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)
            else:
                # 根据 arm_side 设置对应手臂的关节初值
                if arm_side == "left":
                    eef_pose_msg.hand_poses.left_pose.joint_angles = q0_joints.tolist()
                    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)
                elif arm_side == "right":
                    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
                    eef_pose_msg.hand_poses.right_pose.joint_angles = q0_joints.tolist()
                else:
                    rospy.logwarn(f"未知的手臂侧: {arm_side}，将忽略 q0_joints")
                    eef_pose_msg.joint_angles_as_q0 = False
                    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
                    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)
        else:
            # 如果没有提供关节初值，设置为 False，关节角保持为 0
            eef_pose_msg.joint_angles_as_q0 = False
            eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
            eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)
        
        # 确保输入是numpy数组
        if not isinstance(pos, np.ndarray):
            pos = np.array(pos)
        if not isinstance(quat, np.ndarray):
            quat = np.array(quat)
        if other_arm_pos is not None and not isinstance(other_arm_pos, np.ndarray):
            other_arm_pos = np.array(other_arm_pos)
        if other_arm_quat is not None and not isinstance(other_arm_quat, np.ndarray):
            other_arm_quat = np.array(other_arm_quat)

        # 设置目标姿态
        # 注意：IK服务要求两只手的四元数都不能为全0
        unit_quat = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数（无旋转）
        default_pos = np.array([0.0, 0.0, 0.0])

        if arm_side == "left":
            # 左手：使用目标姿态
            eef_pose_msg.hand_poses.left_pose.pos_xyz = pos.tolist()
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = quat.tolist()
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3).tolist()

            # 右手：如果提供了另一只手的FK结果，则使用；否则退回默认
            right_pos = other_arm_pos if other_arm_pos is not None else default_pos
            right_quat = other_arm_quat if other_arm_quat is not None else unit_quat
            eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pos.tolist()
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_quat.tolist()
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3).tolist()
        elif arm_side == "right":
            # 右手：使用目标姿态
            eef_pose_msg.hand_poses.right_pose.pos_xyz = pos.tolist()
            eef_pose_msg.hand_poses.right_pose.quat_xyzw = quat.tolist()
            eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3).tolist()

            # 左手：如果提供了另一只手的FK结果，则使用；否则退回默认
            left_pos = other_arm_pos if other_arm_pos is not None else default_pos
            left_quat = other_arm_quat if other_arm_quat is not None else unit_quat
            eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pos.tolist()
            eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_quat.tolist()
            eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3).tolist()
        else:
            rospy.logwarn(f"未知的手臂侧: {arm_side}")
            return None
        
        # 调用IK服务
        res = ik_client(eef_pose_msg)
        
        # 提取error_reason（如果存在）
        error_reason = ""
        if hasattr(res, 'error_reason'):
            error_reason = res.error_reason if res.error_reason else ""
        
        if not res.success:
            rospy.logwarn(f"IK服务求解失败: {error_reason}")
            return None, None, error_reason
        
        # 提取关节角
        if arm_side == "left":
            joint_angles = np.array(res.hand_poses.left_pose.joint_angles)
        elif arm_side == "right":
            joint_angles = np.array(res.hand_poses.right_pose.joint_angles)
        else:
            return None, None, error_reason
        
        # 提取求解时间（毫秒）
        time_cost_ms = res.time_cost
        
        return joint_angles, time_cost_ms, error_reason
    except rospy.ROSException as e:
        rospy.logwarn(f"IK服务调用失败: {e}")
        return None, None, str(e)
    except Exception as e:
        rospy.logwarn(f"IK服务调用异常: {e}")
        return None, None, str(e)


def call_ik_service_both_arms(
    left_pos,
    left_quat,
    right_pos,
    right_quat,
    ik_param_dict=None,
    timeout: float = 2.0,
    q0_joints_14=None,
    service_name=None,
):
    """
    调用逆运动学服务，同时求解双臂，返回左右臂关节角（7+7）。
    
    Args:
        left_pos/right_pos: 末端位置 (3,) 米
        left_quat/right_quat: 四元数 (4,) [x, y, z, w]
        ik_param_dict: IK参数字典（同 call_ik_service）
        timeout: 服务等待超时时间（秒）
        q0_joints_14: 可选，14维关节初值（弧度）。若提供则 joint_angles_as_q0=True
        service_name: 服务名称，默认 "/ik/two_arm_hand_pose_cmd_srv"
    
    Returns:
        (left_joints, right_joints, time_cost_ms, error_reason)
        - left_joints/right_joints: np.ndarray (7,) 弧度，失败时为 None
        - time_cost_ms: float 或 None
        - error_reason: str
    """
    if service_name is None:
        service_name = "/ik/two_arm_hand_pose_cmd_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        ik_client = rospy.ServiceProxy(service_name, twoArmHandPoseCmdSrv)
        
        eef_pose_msg = twoArmHandPoseCmd()
        
        # 默认参数
        if ik_param_dict is None:
            ik_param_dict = {
                'major_optimality_tol': 1e-3,
                'major_feasibility_tol': 1e-3,
                'minor_feasibility_tol': 1e-3,
                'major_iterations_limit': 100,
                'oritation_constraint_tol': 1e-3,
                'pos_constraint_tol': 1e-3,
                'pos_cost_weight': 0.0,
                # default: pos soft + ori hard (01 -> 1)
                'constraint_mode': 1,
            }
        
        ik_solve_param = ikSolveParam()
        ik_solve_param.major_optimality_tol = ik_param_dict['major_optimality_tol']
        ik_solve_param.major_feasibility_tol = ik_param_dict['major_feasibility_tol']
        ik_solve_param.minor_feasibility_tol = ik_param_dict['minor_feasibility_tol']
        ik_solve_param.major_iterations_limit = ik_param_dict['major_iterations_limit']
        ik_solve_param.oritation_constraint_tol = ik_param_dict['oritation_constraint_tol']
        ik_solve_param.pos_constraint_tol = ik_param_dict['pos_constraint_tol']
        ik_solve_param.pos_cost_weight = ik_param_dict['pos_cost_weight']
        ik_solve_param.constraint_mode = int(ik_param_dict.get('constraint_mode', 1))
        
        eef_pose_msg.ik_param = ik_solve_param
        eef_pose_msg.use_custom_ik_param = True
        
        # q0（可选）
        if q0_joints_14 is not None:
            if not isinstance(q0_joints_14, np.ndarray):
                q0_joints_14 = np.array(q0_joints_14)
            if q0_joints_14.shape[0] == 14:
                eef_pose_msg.joint_angles_as_q0 = True
                eef_pose_msg.hand_poses.left_pose.joint_angles = q0_joints_14[:7].tolist()
                eef_pose_msg.hand_poses.right_pose.joint_angles = q0_joints_14[7:].tolist()
            else:
                rospy.logwarn(
                    f"q0_joints_14 长度不是14（实际为 {q0_joints_14.shape[0]}），将忽略并使用默认值"
                )
                eef_pose_msg.joint_angles_as_q0 = False
                eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7).tolist()
                eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7).tolist()
        else:
            eef_pose_msg.joint_angles_as_q0 = False
            eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7).tolist()
            eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7).tolist()
        
        # 确保输入为 numpy
        left_pos = np.array(left_pos) if not isinstance(left_pos, np.ndarray) else left_pos
        right_pos = np.array(right_pos) if not isinstance(right_pos, np.ndarray) else right_pos
        left_quat = np.array(left_quat) if not isinstance(left_quat, np.ndarray) else left_quat
        right_quat = np.array(right_quat) if not isinstance(right_quat, np.ndarray) else right_quat
        
        # 设置双臂目标姿态
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pos.tolist()
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_quat.tolist()
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3).tolist()
        
        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pos.tolist()
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_quat.tolist()
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3).tolist()
        
        res = ik_client(eef_pose_msg)
        
        error_reason = ""
        if hasattr(res, 'error_reason'):
            error_reason = res.error_reason if res.error_reason else ""
        
        if not res.success:
            rospy.logwarn(f"IK服务求解失败: {error_reason}")
            return None, None, None, error_reason
        
        left_joints = np.array(res.hand_poses.left_pose.joint_angles)
        right_joints = np.array(res.hand_poses.right_pose.joint_angles)
        time_cost_ms = res.time_cost
        
        return left_joints, right_joints, time_cost_ms, error_reason
    except rospy.ROSException as e:
        rospy.logwarn(f"IK服务调用失败: {e}")
        return None, None, None, str(e)
    except Exception as e:
        rospy.logwarn(f"IK服务调用异常: {e}")
        return None, None, None, str(e)


def save_position_to_file(pos, filepath, append=True, precision=4):
    """
    将位置数据保存到文件
    
    Args:
        pos: 位置数据，可以是 (3,) 或 (N, 3) 数组，或列表
        filepath: 文件路径（字符串或Path对象）
        append: 是否追加模式，默认True
        precision: 小数位数，默认4
    """
    filepath = Path(filepath) if not isinstance(filepath, str) else Path(filepath)
    
    # 确保是numpy数组
    if not isinstance(pos, np.ndarray):
        pos = np.array(pos)
    
    # 如果是1维，转换为2维
    if pos.ndim == 1:
        pos = pos.reshape(1, -1)
    
    mode = "a" if append else "w"
    with open(filepath, mode) as f:
        for row in pos:
            f.write(",".join([f"{v:.{precision}f}" for v in row]) + "\n")


def svd_alignment(pos_measured, pos_theoretical):
    """
    使用SVD方法计算两个点云之间的最优旋转和平移（配准）
    
    用于消除坐标系偏差，计算最优的旋转矩阵R和平移向量t，使得：
    pos_measured ≈ R @ pos_theoretical + t
    
    Args:
        pos_measured: 实测位置 (3, N) 或 (N, 3)，米
        pos_theoretical: 理论位置 (3, N) 或 (N, 3)，米
    
    Returns:
        dict: 包含配准结果的字典
            - R: 最优旋转矩阵 (3, 3)
            - t: 最优平移向量 (3, 1)
            - aligned_pos: 配准后的理论位置 (3, N)
            - errors: 配准误差 (N,)，米
    """
    # 转换为 (3, N) 格式
    if pos_measured.ndim == 2 and pos_measured.shape[0] != 3:
        pos_measured = pos_measured.T
    if pos_theoretical.ndim == 2 and pos_theoretical.shape[0] != 3:
        pos_theoretical = pos_theoretical.T
    
    # 计算中心
    pos_meas_mean = np.mean(pos_measured, axis=1, keepdims=True)
    pos_theo_mean = np.mean(pos_theoretical, axis=1, keepdims=True)
    
    # 去中心化
    pos_meas_centered = pos_measured - pos_meas_mean
    pos_theo_centered = pos_theoretical - pos_theo_mean
    
    # SVD配准
    H = pos_meas_centered @ pos_theo_centered.T
    U, S, Vt = np.linalg.svd(H)
    R = U @ Vt
    
    # 计算平移
    t = pos_meas_mean - R @ pos_theo_mean
    
    # 配准后的位置
    aligned_pos = R @ pos_theoretical + t
    
    # 计算误差
    errors = np.linalg.norm(pos_measured - aligned_pos, axis=0)
    
    return {
        'R': R,
        't': t,
        'aligned_pos': aligned_pos,
        'errors': errors
    }

