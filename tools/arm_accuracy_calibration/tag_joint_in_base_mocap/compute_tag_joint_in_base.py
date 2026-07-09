#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计算 tag 关节在参考关节系下的位姿（当前参考为 joint_1；输出里的 base 即该系）。

从 ROS 订阅 tag_pose 与 joint_1 刚体（PoseStamped，mm + xyzw），
用 optitrack_poses.yaml 中 tag_array、joint_1_array 的 T_rigid_body_to_joint；
多帧则平移均值、四元数平均。

离线：tag_array 与 joint_1_array 含 position_mm / quaternion_xyzw 时可用 --offline-from-yaml。
"""

from __future__ import print_function

import argparse
import os
import sys
from typing import Any, Dict, List, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

_script_dir = os.path.dirname(os.path.abspath(__file__))
_arm_accuracy_dir = os.path.dirname(_script_dir)  # tools/arm_accuracy_calibration
_tools_dir = os.path.dirname(_arm_accuracy_dir)  # tools

# 让 `import arm_accuracy_calibration...` 生效：该包位于 tools/arm_accuracy_calibration
# 因此 sys.path 里需要包含 tools/（而不是 repo 根目录）
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)


def _normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n <= 0:
        raise ValueError("invalid quaternion norm: {}".format(n))
    return q / n


def quat_xyzw_to_R(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quat_xyzw(q_xyzw)
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
    q = np.array([x, y, z, w], dtype=float)
    return _normalize_quat_xyzw(q)


def pose_to_T_rigid_body_to_mocap(p_mm: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
    """由 mocap 输出的 (pos_mm, quat_xyzw) 构建 T_rigid_body_to_mocap。"""
    p = np.asarray(p_mm, dtype=float).reshape(3)
    R = quat_xyzw_to_R(q_xyzw)
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
    """
    返回 roll/pitch/yaw（弧度），采用 ZYX（yaw-pitch-roll）分解：
      R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = float(np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
    singular = sy < 1e-12

    if not singular:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        pitch = float(np.arctan2(-R[2, 0], sy))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll = float(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = float(np.arctan2(-R[2, 0], sy))
        yaw = 0.0

    return np.array([roll, pitch, yaw], dtype=float)


def quat_mean_xyzw(quats: List[np.ndarray]) -> np.ndarray:
    """对单位四元数 (x,y,z,w) 做简单欧式平均并归一化；与第一帧同号以避免 q/-q 相消。"""
    if not quats:
        raise ValueError("empty quat list")
    qs = np.array([_normalize_quat_xyzw(q) for q in quats], dtype=float)
    ref = qs[0].copy()
    aligned = []
    for q in qs:
        if np.dot(q, ref) < 0.0:
            q = -q
        aligned.append(q)
    m = np.mean(aligned, axis=0)
    n = float(np.linalg.norm(m))
    if n < 1e-12:
        return qs[0]
    return m / n


def load_optitrack_poses(path: str) -> Dict[str, Any]:
    if yaml is None:
        raise ImportError("需要安装 PyYAML: pip install pyyaml")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    poses = data.get("poses", {})
    if not isinstance(poses, dict):
        raise ValueError("YAML 格式错误：缺少 poses 字段或类型不对")
    return poses


def yaml_has_static_mocap_poses(poses: Dict[str, Any]) -> bool:
    try:
        tag = poses["tag_array"]
        j1 = poses["joint_1_array"]
    except KeyError:
        return False
    return (
        "position_mm" in tag
        and "quaternion_xyzw" in tag
        and "position_mm" in j1
        and "quaternion_xyzw" in j1
    )


def load_extrinsic_T_matrices(poses: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """tag 与参考系（joint_1）的 T_rigid_body_to_joint，均来自 optitrack_poses.yaml。"""
    tag = poses["tag_array"]
    ref = poses["joint_1_array"]
    if "T_rigid_body_to_joint" not in tag or "T_rigid_body_to_joint" not in ref:
        raise KeyError("YAML 需包含 tag_array / joint_1_array 的 T_rigid_body_to_joint")
    T_tag = np.array(tag["T_rigid_body_to_joint"], dtype=float).reshape(4, 4)
    T_ref = np.array(ref["T_rigid_body_to_joint"], dtype=float).reshape(4, 4)
    return T_tag, T_ref


def compute_T_base_to_tag_joint(
    tag_position_mm: np.ndarray,
    tag_quat_xyzw: np.ndarray,
    joint_1_position_mm: np.ndarray,
    joint_1_quat_xyzw: np.ndarray,
    T_tag_rb_to_joint: np.ndarray,
    T_joint1_rb_to_joint1: np.ndarray,
) -> np.ndarray:
    """base 表示参考关节系（此处为 joint_1）。"""
    T_tag_rb_to_mocap = pose_to_T_rigid_body_to_mocap(tag_position_mm, tag_quat_xyzw)
    T_j1_rb_to_mocap = pose_to_T_rigid_body_to_mocap(joint_1_position_mm, joint_1_quat_xyzw)
    T_tag_joint_to_mocap = T_tag_rb_to_mocap @ np.linalg.inv(T_tag_rb_to_joint)
    T_base_to_mocap = T_j1_rb_to_mocap @ np.linalg.inv(T_joint1_rb_to_joint1)
    return inv_T(T_base_to_mocap) @ T_tag_joint_to_mocap


def compute_tag_joint_in_base(poses: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """离线单帧；poses 中含 tag_array / joint_1_array 的动捕位姿与外参。"""
    T_tag, T_j1 = load_extrinsic_T_matrices(poses)
    tag = poses["tag_array"]
    j1 = poses["joint_1_array"]
    T_bt = compute_T_base_to_tag_joint(
        tag["position_mm"],
        tag["quaternion_xyzw"],
        j1["position_mm"],
        j1["quaternion_xyzw"],
        T_tag,
        T_j1,
    )
    return T_bt[:3, 3], R_to_quat_xyzw(T_bt[:3, :3])


def collect_tag_joint_in_base_live(
    poses_path: str,
    tag_topic: str,
    joint_1_topic: str,
    samples: int,
    rate_hz: float,
    wait_timeout: float,
) -> Tuple[np.ndarray, np.ndarray, int]:
    import rospy
    from geometry_msgs.msg import PoseStamped

    from arm_accuracy_calibration.functions import (
        get_all_rigid_body_poses,
        joint_1_callback,
        tag_pose_callback,
    )

    poses_cfg = load_optitrack_poses(poses_path)
    T_tag, T_joint1 = load_extrinsic_T_matrices(poses_cfg)

    rospy.init_node("compute_tag_joint_in_base", anonymous=True)
    rospy.Subscriber(tag_topic, PoseStamped, tag_pose_callback, queue_size=20)
    rospy.Subscriber(joint_1_topic, PoseStamped, joint_1_callback, queue_size=20)

    t_deadline = rospy.Time.now() + rospy.Duration(wait_timeout)
    while not rospy.is_shutdown():
        all_p = get_all_rigid_body_poses()
        if "tag" in all_p and "joint_1" in all_p:
            break
        if rospy.Time.now() > t_deadline:
            raise RuntimeError(
                "等待 tag/joint_1 位姿超时（{} s）。请确认话题 {} / {} 有 PoseStamped（单位 mm）。".format(
                    wait_timeout, tag_topic, joint_1_topic
                )
            )
        rospy.sleep(0.05)

    pos_samples: List[np.ndarray] = []
    quat_samples: List[np.ndarray] = []
    r = rospy.Rate(rate_hz) if rate_hz > 0 else None
    while len(pos_samples) < samples and not rospy.is_shutdown():
        all_p = get_all_rigid_body_poses()
        if "tag" not in all_p or "joint_1" not in all_p:
            (r.sleep() if r else rospy.sleep(0.01))
            continue
        T_bt = compute_T_base_to_tag_joint(
            all_p["tag"]["pos"],
            all_p["tag"]["quat"],
            all_p["joint_1"]["pos"],
            all_p["joint_1"]["quat"],
            T_tag,
            T_joint1,
        )
        pos_samples.append(T_bt[:3, 3].copy())
        quat_samples.append(R_to_quat_xyzw(T_bt[:3, :3]))
        (r.sleep() if r else rospy.sleep(0.001))

    if len(pos_samples) < samples:
        raise RuntimeError("采集中断：仅收集到 {} 帧（目标 {}）".format(len(pos_samples), samples))

    p_mean = np.mean(np.stack(pos_samples, axis=0), axis=0)
    q_mean = quat_mean_xyzw(quat_samples)
    return p_mean, q_mean, len(pos_samples)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="计算 tag 在 joint_1 参考系下的位姿（输出键名仍为 tag_joint_in_base）。默认 ROS 多帧均值。"
    )
    parser.add_argument(
        "--poses",
        default=None,
        help="optitrack_poses.yaml：含 tag_array、joint_1_array 的 T_rigid_body_to_joint（默认见代码内路径）",
    )
    parser.add_argument(
        "--offline-from-yaml",
        action="store_true",
        help="使用 optitrack_poses.yaml 内 tag_array 与 joint_1_array 的 position_mm / quaternion_xyzw 单帧计算",
    )
    parser.add_argument("--tag-topic", default="tag_pose", help="tag 刚体 PoseStamped 话题")
    parser.add_argument(
        "--joint-1-topic",
        default="joint_1",
        help="joint_1 刚体 PoseStamped 话题（与 OptiTrack 桥接一致时常为 joint_1）",
    )
    parser.add_argument("--samples", type=int, default=1000, help="ROS 模式下采集帧数")
    parser.add_argument("--rate", type=float, default=120.0, help="采集节拍 (Hz)，0 表示尽快采样")
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=30.0,
        help="等待首帧 tag+joint_1 的超时时间 (s)",
    )
    args = parser.parse_args()

    if args.poses is None:
        poses_path = os.path.join(_arm_accuracy_dir, "config", "optitrack_poses.yaml")
    else:
        poses_path = args.poses

    if args.offline_from_yaml:
        poses = load_optitrack_poses(poses_path)
        if not yaml_has_static_mocap_poses(poses):
            print(
                "错误：{} 中未包含 tag_array 与 joint_1_array 的 position_mm 与 quaternion_xyzw，"
                "无法使用 --offline-from-yaml。请去掉该选项以使用 ROS 实时采集。".format(poses_path),
                file=sys.stderr,
            )
            sys.exit(1)
        p, q = compute_tag_joint_in_base(poses)
        n_used = 1
    else:
        p, q, n_used = collect_tag_joint_in_base_live(
            poses_path,
            args.tag_topic,
            args.joint_1_topic,
            args.samples,
            args.rate,
            args.wait_timeout,
        )

    R = quat_xyzw_to_R(q)
    rpy = R_to_rpy(R)
    p_m = p / 1000.0

    def _fmt_space(vals):
        # 一行空格分隔，方便直接粘贴到 xyz="x y z" / rpy="r p y"
        return " ".join("{:.15g}".format(float(v)) for v in vals)

    print("tag_joint_in_base:")
    print('  position_m: "{}"'.format(_fmt_space(p_m)))
    print('  rpy_rad: "{}"'.format(_fmt_space(rpy)))
    print("  samples_used: {}".format(int(n_used)))


if __name__ == "__main__":
    main()

