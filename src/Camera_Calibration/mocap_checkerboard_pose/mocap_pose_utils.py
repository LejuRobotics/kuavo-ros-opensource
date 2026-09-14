#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动捕位姿工具：四元数/矩阵转换、有效性检查、3σ 滤波（轻量实现，不依赖 matplotlib）。"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

POS_INVALID_THRESHOLD = 9999.0  # mm，与 run_batch_process 一致


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX 欧拉角 → 旋转矩阵（弧度）。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def load_mocap_frame_align_matrix(cfg: Optional[Dict[str, Any]]) -> np.ndarray:
    """
    从配置读取 Motive parent 系 → URDF parent 系的旋转矩阵。

    用于 hand_in_torso 与 TF hand_in_waist 对比前，将动捕相对平移变到 URDF 约定。
    配置示例见 hand_accuracy.yaml 的 mocap_frame_align。
    """
    if not cfg or not cfg.get("enabled", False):
        return np.eye(3, dtype=float)
    flip = cfg.get("axis_flip")
    if flip is not None:
        if len(flip) != 3:
            raise ValueError("mocap_frame_align.axis_flip 须为 3 元组，如 [1, -1, 1]")
        return np.diag([float(flip[0]), float(flip[1]), float(flip[2])])
    rpy = cfg.get("rpy_rad", [0.0, 0.0, 0.0])
    if len(rpy) != 3:
        raise ValueError("mocap_frame_align.rpy_rad 须为 3 元组 (roll, pitch, yaw)")
    return rpy_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))


def align_relative_translation(pos_m: np.ndarray, R_align: np.ndarray) -> np.ndarray:
    """将 child-in-parent 平移从 Motive parent 系变到 URDF parent 系（米）。"""
    p = np.asarray(pos_m, dtype=float).reshape(3)
    return (np.asarray(R_align, dtype=float).reshape(3, 3) @ p).reshape(3)


def quaternion_mean(quats: np.ndarray) -> np.ndarray:
    """四元数特征值法平均（与 fit_center_by_pose 一致，无 matplotlib 依赖）。"""
    if len(quats) == 0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    Q = np.asarray(quats, dtype=float)
    norms = np.linalg.norm(Q, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    Q = Q / norms
    A = Q.T @ Q
    eigvals, eigvecs = np.linalg.eigh(A)
    mean_q = eigvecs[:, np.argmax(eigvals)]
    if mean_q[3] < 0:
        mean_q = -mean_q
    return mean_q / np.linalg.norm(mean_q)


def filter_poses_by_sigma(
    points: np.ndarray,
    quats: np.ndarray,
    sigma_multiplier: float = 3.0,
) -> Tuple[np.ndarray, ...]:
    """
    3σ 规则：先按位置筛，再按旋转角距筛（逻辑同 fit_center_by_pose，避免 import 其 matplotlib）。
    """
    center = np.mean(points, axis=0)
    pos_distances = np.linalg.norm(points - center, axis=1)
    pos_mean_dist = float(np.mean(pos_distances))
    pos_std_dist = float(np.std(pos_distances))
    pos_threshold = pos_mean_dist + sigma_multiplier * pos_std_dist
    pos_mask = pos_distances <= pos_threshold
    pos_filtered_points = points[pos_mask]
    pos_filtered_quats = quats[pos_mask]

    if len(pos_filtered_points) > 0:
        mean_quat = quaternion_mean(pos_filtered_quats)
        Q = pos_filtered_quats / np.linalg.norm(
            pos_filtered_quats, axis=1, keepdims=True
        ).clip(min=1e-12)
        mean_q_norm = mean_quat / np.linalg.norm(mean_quat)
        dot_products = np.abs(np.sum(Q * mean_q_norm, axis=1))
        dot_products = np.clip(dot_products, 0.0, 1.0)
        rot_distances = 2 * np.arccos(dot_products)
        rot_mean_dist = float(np.mean(rot_distances))
        rot_std_dist = float(np.std(rot_distances))
        rot_threshold = rot_mean_dist + sigma_multiplier * rot_std_dist
        rot_mask = rot_distances <= rot_threshold
        filtered_points = pos_filtered_points[rot_mask]
        filtered_quats = pos_filtered_quats[rot_mask]
    else:
        mean_quat = quaternion_mean(quats)
        filtered_points = np.empty((0, 3))
        filtered_quats = np.empty((0, 4))
        rot_mean_dist = rot_std_dist = 0.0
        rot_mask = np.array([], dtype=bool)

    final_mask = np.zeros(len(points), dtype=bool)
    if len(pos_filtered_points) > 0:
        final_mask[pos_mask] = rot_mask

    return (
        filtered_points,
        filtered_quats,
        points[~pos_mask],
        quats[~pos_mask],
        pos_filtered_points[~rot_mask] if len(pos_filtered_points) else np.empty((0, 3)),
        pos_filtered_quats[~rot_mask] if len(pos_filtered_points) else np.empty((0, 4)),
        center,
        mean_quat if len(filtered_points) else quaternion_mean(quats),
        pos_mean_dist,
        pos_std_dist,
        rot_mean_dist,
        rot_std_dist,
        final_mask,
    )


def load_bodies_config(path: str) -> Dict[str, Any]:
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_link_offsets_mm(cfg: Dict[str, Any]) -> Dict[str, np.ndarray]:
    """从 bodies.yaml 读取各刚体工装→link 平移偏移 (mm)，未配置则为 0。"""
    offsets: Dict[str, np.ndarray] = {}
    for body in cfg.get("bodies", []):
        name = body["name"]
        off = body.get("link_offset_mm", [0.0, 0.0, 0.0])
        offsets[name] = np.asarray(off, dtype=float).reshape(3)
    return offsets


def tooling_pose_to_link_pose(
    pos_mm: np.ndarray,
    quat_xyzw: np.ndarray,
    link_offset_mm: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    动捕得到工装刚体位姿，反推 link 在 mocap 系下的位姿。

    link_offset_mm：工装原点在 link 系下的位置 (mm)。
    关系：p_tooling = p_link + R_link @ offset  =>  p_link = p_tooling - R @ offset
    姿态：工装与 link 固连，四元数不变。
    """
    offset = np.asarray(link_offset_mm, dtype=float).reshape(3)
    pos = np.asarray(pos_mm, dtype=float).reshape(3)
    quat = np.asarray(quat_xyzw, dtype=float).reshape(4)
    if np.allclose(offset, 0.0):
        return pos.copy(), quat.copy()
    R = quat_xyzw_to_R(quat)
    pos_link = pos - R @ offset
    return pos_link, quat.copy()


def apply_link_offsets_to_poses(
    poses: Dict[str, Dict[str, np.ndarray]],
    link_offsets: Dict[str, np.ndarray],
) -> Dict[str, Dict[str, np.ndarray]]:
    """对一帧内各刚体位姿应用工装→link 偏移修正。"""
    corrected: Dict[str, Dict[str, np.ndarray]] = {}
    for name, pose in poses.items():
        off = link_offsets.get(name, np.zeros(3))
        pos, quat = tooling_pose_to_link_pose(pose["pos"], pose["quat"], off)
        corrected[name] = {"pos": pos, "quat": quat}
    return corrected


def normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n <= 0:
        raise ValueError("invalid quaternion norm")
    return q / n


def quat_xyzw_to_R(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(q_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


def R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
    return normalize_quat_xyzw(np.array([x, y, z, w], dtype=float))


def pose_to_T(pos_mm: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """刚体在 mocap 系下的 4×4 齐次变换（位置 mm）。"""
    p = np.asarray(pos_mm, dtype=float).reshape(3)
    R = quat_xyzw_to_R(quat_xyzw)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def R_to_rpy(R: np.ndarray) -> np.ndarray:
    """ZYX 欧拉角 roll/pitch/yaw（弧度）。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = float(np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
    if sy > 1e-12:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        pitch = float(np.arctan2(-R[2, 0], sy))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll = float(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = float(np.arctan2(-R[2, 0], sy))
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


def is_valid_pose(pos: np.ndarray, quat: np.ndarray) -> bool:
    """检查动捕位姿是否有效（非有限、pos 过大视为无效）。"""
    pos = np.asarray(pos, dtype=float).reshape(3)
    quat = np.asarray(quat, dtype=float).reshape(4)
    if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(quat)):
        return False
    if np.any(np.abs(pos) > POS_INVALID_THRESHOLD):
        return False
    if np.linalg.norm(quat) < 1e-12:
        return False
    return True


def ensure_quat_continuity(quat_new: np.ndarray, prev: Optional[np.ndarray]) -> np.ndarray:
    """四元数符号连续，避免 q/-q 跳变。"""
    q = normalize_quat_xyzw(quat_new)
    if prev is not None and np.dot(q, prev) < 0:
        q = -q
    return q


def compute_relative_poses(
    child_pos_mm: np.ndarray,
    child_quat: np.ndarray,
    parent_pos_mm: np.ndarray,
    parent_quat: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    计算 child 在 parent 系下的位姿。
    T_child_in_parent = inv(T_parent) @ T_child
    返回 (pos_mm, quat_xyzw)
    """
    T_child = pose_to_T(child_pos_mm, child_quat)
    T_parent = pose_to_T(parent_pos_mm, parent_quat)
    T_rel = inv_T(T_parent) @ T_child
    pos = T_rel[:3, 3]
    quat = R_to_quat_xyzw(T_rel[:3, :3])
    return pos, quat


def filter_and_aggregate_poses(
    points_mm: np.ndarray,
    quats: np.ndarray,
    sigma: float = 3.0,
) -> Dict[str, Any]:
    """
    3σ 滤波后聚合位姿。
    返回均值平移（mm）、四元数、标准差等。
    """
    if len(points_mm) == 0:
        raise ValueError("no frames to process")

    (
        filtered_pts,
        filtered_quats,
        _,
        _,
        _,
        _,
        _,
        _,
        pos_mean_dist,
        pos_std_dist,
        rot_mean_dist,
        rot_std_dist,
        final_mask,
    ) = filter_poses_by_sigma(points_mm, quats, sigma_multiplier=sigma)

    if len(filtered_pts) == 0:
        raise ValueError("all frames removed by 3-sigma filter")

    mean_pos = np.mean(filtered_pts, axis=0)
    mean_quat = quaternion_mean(filtered_quats)

    return {
        "xyz_mm": mean_pos,
        "quaternion_xyzw": mean_quat,
        "raw_frames": len(points_mm),
        "valid_frames": len(filtered_pts),
        "pos_std_mm": float(np.std(filtered_pts, axis=0).max()),
        "rot_std_deg": float(np.degrees(rot_std_dist)),
        "pos_mean_dist_mm": float(pos_mean_dist),
        "pos_std_dist_mm": float(pos_std_dist),
        "rot_mean_dist_deg": float(np.degrees(rot_mean_dist)),
        "rot_std_dist_deg": float(np.degrees(rot_std_dist)),
        "filter_mask": final_mask,
    }
