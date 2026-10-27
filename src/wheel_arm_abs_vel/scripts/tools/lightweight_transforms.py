"""Small rigid-transform helpers for the Quest mapper.

This module intentionally has no Drake dependency.  Velocity-IK takeover uses
the Python process only for Quest packet validation, retargeting and lifecycle
management; loading a second multibody/optimization stack before registering
the raw ROS subscriptions adds startup latency without contributing to the
21-D solve.
"""

import math
from functools import lru_cache

import numpy as np
import tf


def quaternion_to_RPY(quat):
    return np.asarray(tf.transformations.euler_from_quaternion(quat), dtype=float)


def RPY_to_quaternion(rpy):
    return np.asarray(
        tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
        dtype=float,
    )


def quaternion_to_matrix(quat):
    x, y, z, w = (float(v) for v in quat)
    norm2 = x*x + y*y + z*z + w*w
    if not math.isfinite(norm2) or norm2 <= 1.0e-24:
        raise ValueError("quaternion must be finite and non-zero")
    scale = 2.0 / norm2
    xx, yy, zz = scale*x*x, scale*y*y, scale*z*z
    xy, xz, yz = scale*x*y, scale*x*z, scale*y*z
    wx, wy, wz = scale*w*x, scale*w*y, scale*w*z
    return np.array([[1.0-yy-zz, xy-wz, xz+wy],
                     [xy+wz, 1.0-xx-zz, yz-wx],
                     [xz-wy, yz+wx, 1.0-xx-yy]])


def matrix_to_quaternion(mat):
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(mat, dtype=float)
    quat = np.asarray(tf.transformations.quaternion_from_matrix(transform), dtype=float)
    if quat[3] < 0.0:
        quat = -quat
    return quat


def matrix_to_rpy(mat):
    return np.asarray(tf.transformations.euler_from_matrix(mat), dtype=float)


def rpy_to_matrix(rpy):
    return np.asarray(
        tf.transformations.euler_matrix(rpy[0], rpy[1], rpy[2])[:3, :3],
        dtype=float,
    )


def pos_rpy_to_transform(pos, rpy):
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = np.asarray(pos, dtype=float)
    return transform


def transform_to_pos_rpy(transform):
    transform = np.asarray(transform, dtype=float)
    return transform[:3, 3].copy(), matrix_to_rpy(transform[:3, :3])


def transform_inverse(transform):
    transform = np.asarray(transform, dtype=float)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def LHS_to_RHS(transform):
    reflection = np.diag([1.0, 1.0, -1.0])
    result = np.eye(4)
    result[:3, :3] = reflection @ transform[:3, :3] @ reflection
    result[:3, 3] = reflection @ transform[:3, 3]
    return result


def head_to_robot_transform(transform):
    robot_to_world = np.eye(4)
    robot_to_world[:3, :3] = rpy_to_matrix([-math.pi / 2.0, math.pi / 2.0, 0.0])
    return transform_inverse(robot_to_world) @ transform @ robot_to_world


def pos_rpy_LHS_to_RHS(pos, rpy, is_reverse=False):
    result_rpy = np.array([rpy[0], rpy[1], -rpy[2]], dtype=float)
    result_pos = np.array([pos[0], pos[1], -pos[2]], dtype=float)
    if is_reverse:
        result_rpy[1] += math.pi
    return result_pos, result_rpy


@lru_cache(maxsize=16)
def _vr_fixed_rotation(side, bias_agl):
    sign = 1.0 if side == "Right" else -1.0
    bias = rpy_to_matrix([sign * bias_agl, 0.0, 0.0])
    fixed = rpy_to_matrix([-math.pi / 2.0, 0.0, 0.0 if side == "Right" else -math.pi])
    fixed.setflags(write=False)
    bias.setflags(write=False)
    return fixed, bias


def vr_quat2robot_quat(vr_quat, side, bias_agl=20.0 * math.pi / 180.0):
    source = quaternion_to_matrix(vr_quat)
    fixed, bias = _vr_fixed_rotation(side, float(bias_agl))
    return matrix_to_quaternion(source @ fixed @ bias)


def rpy_to_quaternion(roll, pitch, yaw):
    return np.asarray(
        tf.transformations.quaternion_from_euler(roll, pitch, yaw), dtype=float
    )


def original_LHS_pos_rpy_to_RHS_in_robot_frame(
        pose_lhs, head_pose_lhs, is_reverse=False):
    position, rpy = pos_rpy_LHS_to_RHS(*pose_lhs, is_reverse=is_reverse)
    target = head_to_robot_transform(pos_rpy_to_transform(position, rpy))
    head_position, head_rpy = pos_rpy_LHS_to_RHS(*head_pose_lhs)
    head = head_to_robot_transform(pos_rpy_to_transform(head_position, head_rpy))
    relative = np.eye(4)
    relative[:3, :3] = target[:3, :3] @ head[:3, :3].T
    relative[:3, 3] = target[:3, 3] - head[:3, 3]
    return transform_to_pos_rpy(relative)


def compute_rpy_error(rpy1, rpy2):
    relative = rpy_to_matrix(rpy1).T @ rpy_to_matrix(rpy2)
    return matrix_to_rpy(relative)


def matrix_to_axis_angle(matrix):
    matrix = np.asarray(matrix, dtype=float)
    cosine = np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(cosine)
    if angle < 1.0e-9:
        return np.zeros(3), 0.0
    if math.pi - angle < 1.0e-6:
        diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        axis[0] = math.copysign(axis[0], matrix[2, 1] - matrix[1, 2])
        axis[1] = math.copysign(axis[1], matrix[0, 2] - matrix[2, 0])
        axis[2] = math.copysign(axis[2], matrix[1, 0] - matrix[0, 1])
        norm = np.linalg.norm(axis)
        return angle * axis / max(norm, 1.0e-12), angle
    axis = np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ]) / (2.0 * math.sin(angle))
    return angle * axis, angle


def axis_angle_to_matrix(rotvec):
    rotvec = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rotvec)
    if angle < 1.0e-12:
        return np.eye(3)
    axis = rotvec / angle
    cross = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)
