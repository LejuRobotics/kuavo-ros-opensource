# -*- coding: utf-8 -*-
"""
解算模块：根据阵列配置和刚体位姿，求 joint 在 mocap 系下的位姿 T_joint_to_mocap。
"""
import numpy as np

from .config_loader import ArrayConfig
from .transform_utils import pose_to_transform_matrix


def _compute_joint_to_mocap(
    array_config: ArrayConfig,
    rigid_body_pos: np.ndarray,
    rigid_body_quat: np.ndarray,
    enable_motion_control: bool = False,
) -> np.ndarray:
    """
    计算 joint 在 mocap 系下的位姿 T_joint_to_mocap。

    计算流程：
    1. 构建 T_rigid_body_to_mocap（从刚体位姿）
    2. 计算 T_mocap_to_joint = T_rigid_body_to_joint @ inv(T_rigid_body_to_mocap)
    3. 返回 T_joint_to_mocap = inv(T_mocap_to_joint)

    参数:
        array_config: 阵列配置
        rigid_body_pos: 刚体在mocap系下的位置 [x, y, z]，单位 mm
        rigid_body_quat: 刚体在mocap系下的四元数 [x, y, z, w]
        enable_motion_control: 如果为True，使用单位矩阵而不是配置文件中的T_rigid_body_to_joint

    返回:
        T_joint_to_mocap: joint在mocap系下的4x4位姿矩阵
    """
    # 构建 T_rigid_body_to_mocap（刚体在 mocap 下的位姿）
    T_rigid_body_to_mocap = pose_to_transform_matrix(rigid_body_pos, rigid_body_quat)
    
    # 根据 enable_motion_control 选择使用配置文件转换还是单位矩阵
    if enable_motion_control:
        # 使用单位矩阵，相当于直接使用刚体在动捕系下的位姿
        T_rigid_body_to_joint = np.eye(4)
    else:
        # 使用配置文件中的转换矩阵
        T_rigid_body_to_joint = array_config.T_rigid_body_to_joint
    
    # T_mocap_to_joint = T_rigid_body_to_joint @ inv(T_rigid_body_to_mocap)
    T_mocap_to_joint = T_rigid_body_to_joint @ np.linalg.inv(T_rigid_body_to_mocap)
    
    # 返回 T_joint_to_mocap = inv(T_mocap_to_joint)
    return np.linalg.inv(T_mocap_to_joint)
