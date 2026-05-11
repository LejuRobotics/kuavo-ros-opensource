# -*- coding: utf-8 -*-
"""
批量解算：根据刚体位姿数据，将指定 joint 解算到指定参考系下。
统一流程：先求各关节在 mocap 下的位姿，再按参考系转到目标系。
"""
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config_loader import AllArraysConfig
from .frame_solver import _compute_joint_to_mocap


def solve_joints_in_frame(
    all_config: AllArraysConfig,
    rigid_body_poses: Dict[str, Dict[str, np.ndarray]],
    reference_joint: str,
    target_joints: List[str],
    enable_motion_control: bool = False,
) -> Tuple[Dict[str, Optional[np.ndarray]], Optional[np.ndarray]]:
    """
    将配置中指定的 joint 转到指定基坐标系下表示。

    函数内部先求各阵列对应 joint 在 mocap 下的位姿 T_joint_to_mocap，
    再求参考系在 mocap 下的位姿 T_ref_to_mocap，然后统一：
    T_joint_to_ref = inv(T_ref_to_mocap) @ T_joint_to_mocap。

    参数:
        all_config: 加载的配置
        rigid_body_poses: {array_name: {"pos": [x,y,z], "quat": [x,y,z,w]}}，单位 mm
        reference_joint: 作为基坐标系的 joint 名，如 "base"、"joint_1"、"elbow"
        target_joints: 要转到该基系下的 joint 名列表，如 ["joint_1", "elbow", "tool"]
        enable_motion_control: 如果为True，所有关节都不乘配置文件中的 T_rigid_body_to_joint，直接使用刚体在动捕系下的位姿（如 joint_0 标定）

    返回:
        (result_dict, T_ref_to_mocap)
        - result_dict: {joint_name: T_joint_to_ref}，缺失或失败为 None
        - T_ref_to_mocap: 参考系在 mocap 下的位姿，便于调用方补全打印（如 result["base"]=T_base_to_mocap）
    """
    # 尝试导入 rospy（用于调试日志，如果不在 ROS 环境中则忽略）。
    # 注意：在未 init_node 的纯离线脚本中不能调用基于时间的日志接口（logwarn_throttle 等），
    # 所以下面所有日志调用都需同时判断 rospy.is_initialized()。
    try:
        import rospy
    except ImportError:
        rospy = None
    
    # 1) 求每个阵列对应 joint 在 mocap 下的位姿
    T_joint_to_mocap: Dict[str, np.ndarray] = {}
    for name in all_config.names():
        cfg = all_config.get(name)
        if cfg is None or name not in rigid_body_poses:
            # 调试：如果参考关节或目标关节缺失数据，输出警告
            if cfg is not None and (cfg.joint == reference_joint or cfg.joint in target_joints):
                if rospy is not None and getattr(rospy, "is_initialized", lambda: False)():
                    try:
                        rospy.logwarn_throttle(
                            1.0,
                            f"[batch_solver] 阵列 '{name}' (关节 '{cfg.joint}') 在 rigid_body_poses 中不存在，"
                            f"当前可用键: {list(rigid_body_poses.keys())}, "
                            f"reference_joint='{reference_joint}', target_joints={target_joints}"
                        )
                    except Exception:
                        # 离线环境下忽略日志异常
                        pass
            continue
        rb = rigid_body_poses[name]
        # enable_motion_control 为 True 时，所有关节都不乘配置偏移（如 joint_0）；否则使用配置文件
        T_joint_to_mocap[cfg.joint] = _compute_joint_to_mocap(
            cfg, rb["pos"], rb["quat"], enable_motion_control=enable_motion_control
        )

    T_ref_to_mocap = T_joint_to_mocap.get(reference_joint)
    if T_ref_to_mocap is None:
        # 调试：参考关节缺失时输出警告
        if rospy is not None and getattr(rospy, "is_initialized", lambda: False)():
            try:
                rospy.logwarn_throttle(
                    1.0,
                    f"[batch_solver] 参考关节 '{reference_joint}' 在 T_joint_to_mocap 中不存在，"
                    f"已计算的关节: {list(T_joint_to_mocap.keys())}, "
                    f"rigid_body_poses 中的键: {list(rigid_body_poses.keys())}"
                )
            except Exception:
                pass
        return {j: None for j in target_joints}, None

    T_ref_to_mocap_inv = np.linalg.inv(T_ref_to_mocap)
    result = {}
    for j in target_joints:
        T_j = T_joint_to_mocap.get(j)
        if T_j is not None:
            result[j] = T_ref_to_mocap_inv @ T_j
        else:
            # 调试：目标关节缺失时输出警告
            if rospy is not None and getattr(rospy, "is_initialized", lambda: False)():
                try:
                    rospy.logwarn_throttle(
                        1.0,
                        f"[batch_solver] 目标关节 '{j}' 在 T_joint_to_mocap 中不存在，"
                        f"已计算的关节: {list(T_joint_to_mocap.keys())}, "
                        f"rigid_body_poses 中的键: {list(rigid_body_poses.keys())}"
                    )
                except Exception:
                    pass
            result[j] = None

    return result, T_ref_to_mocap
