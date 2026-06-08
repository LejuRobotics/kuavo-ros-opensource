# -*- coding: utf-8 -*-
"""
坐标变换工具：旋转矩阵、四元数、欧拉角之间的转换
"""
import numpy as np


def rotation_matrix_to_euler(R: np.ndarray) -> tuple:
    """
    旋转矩阵转欧拉角（ZYX顺序，即 yaw-pitch-roll）

    参数:
        R: 3x3 旋转矩阵

    返回:
        (yaw, pitch, roll): 欧拉角，单位度
    """
    yaw = np.arctan2(R[1, 0], R[0, 0]) * 180 / np.pi
    pitch = np.arcsin(-R[2, 0]) * 180 / np.pi
    roll = np.arctan2(R[2, 1], R[2, 2]) * 180 / np.pi
    return yaw, pitch, roll


def quaternion_array_to_euler(quat: np.ndarray) -> tuple:
    """
    四元数数组转欧拉角（ZYX顺序）

    参数:
        quat: 四元数数组 [x, y, z, w] 或 [qx, qy, qz, qw]

    返回:
        (yaw, pitch, roll): 欧拉角，单位度
    """
    R = quaternion_to_rotation_matrix(quat)
    return rotation_matrix_to_euler(R)


def quaternion_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    """
    四元数转旋转矩阵

    参数:
        quat: 四元数数组 [x, y, z, w]

    返回:
        3x3 旋转矩阵
    """
    qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]
    R = np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
    ])
    return R


def pose_to_transform_matrix(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """
    位置和四元数转换为4x4齐次变换矩阵

    参数:
        pos: 位置 [x, y, z]，单位 mm
        quat: 四元数 [x, y, z, w]

    返回:
        4x4 齐次变换矩阵
    """
    T = np.eye(4)
    T[:3, :3] = quaternion_to_rotation_matrix(quat)
    T[:3, 3] = np.array(pos, dtype=float)
    return T


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    将3x3旋转矩阵转换为四元数 [x, y, z, w]

    参数:
        R: 3x3 旋转矩阵

    返回:
        四元数数组 [x, y, z, w]
    """
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


