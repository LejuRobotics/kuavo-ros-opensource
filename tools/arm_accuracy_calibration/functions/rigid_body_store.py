# -*- coding: utf-8 -*-
"""
刚体位姿数据存储与回调
"""
from typing import Dict
import numpy as np

# 数据结构：存储 joint_1~7、joint_end、belly、tag 的位姿
# {name: {"pos": [x, y, z], "quat": [x, y, z, w]}}
_rigid_body_poses: Dict[str, Dict[str, np.ndarray]] = {}
# 上一帧四元数，用于符号一致性检查
_prev_quats: Dict[str, np.ndarray] = {}


def _ensure_quaternion_continuity(quat_new: np.ndarray, name: str) -> np.ndarray:
    """
    保证四元数符号连续，避免 q 与 -q 互换导致的跳变。
    q 和 -q 表示同一旋转，但数值跳变会影响滤波和后续计算。
    """
    quat_new = np.array(quat_new, dtype=float)
    quat_new = quat_new / np.linalg.norm(quat_new)  # 归一化
    if name in _prev_quats:
        if np.dot(quat_new, _prev_quats[name]) < 0:
            quat_new = -quat_new
    _prev_quats[name] = quat_new.copy()
    return quat_new


def update_rigid_body_pose(name: str, pos: np.ndarray, quat: np.ndarray) -> None:
    """
    更新刚体位姿

    参数:
        name: 刚体名称，如 'joint_1', 'joint_2'
        pos: 位置 [x, y, z]，单位 mm
        quat: 四元数 [x, y, z, w]
    """
    quat = _ensure_quaternion_continuity(quat, name)
    _rigid_body_poses[name] = {
        "pos": np.array(pos, dtype=float),
        "quat": quat,
    }


def get_all_rigid_body_poses() -> Dict[str, Dict[str, np.ndarray]]:
    """获取所有刚体位姿"""
    return dict(_rigid_body_poses)


def _pose_callback(msg, name: str):
    """通用的位姿回调函数，订阅 PoseStamped，使用其中的 pose 字段"""
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    update_rigid_body_pose(name, pos, quat)


def joint_1_callback(msg):
    """joint_1 话题回调"""
    _pose_callback(msg, "joint_1")


def joint_2_callback(msg):
    """joint_2 话题回调"""
    _pose_callback(msg, "joint_2")


def belly_pose_callback(msg):
    """belly_pose 话题回调"""
    _pose_callback(msg, "belly")


def tag_pose_callback(msg):
    """tag_pose 话题回调（动捕刚体 tag 阵列）"""
    _pose_callback(msg, "tag")


def joint_3_callback(msg):
    """joint_3 话题回调"""
    _pose_callback(msg, "joint_3")


def joint_4_callback(msg):
    """joint_4 话题回调"""
    _pose_callback(msg, "joint_4")


def joint_5_callback(msg):
    """joint_5 话题回调"""
    _pose_callback(msg, "joint_5")


def joint_6_callback(msg):
    """joint_6 话题回调"""
    _pose_callback(msg, "joint_6")


def joint_7_callback(msg):
    """joint_7 话题回调"""
    _pose_callback(msg, "joint_7")


def joint_end_callback(msg):
    """joint_end 话题回调"""
    _pose_callback(msg, "joint_end")