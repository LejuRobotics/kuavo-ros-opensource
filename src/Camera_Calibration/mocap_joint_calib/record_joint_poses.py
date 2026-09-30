#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯动捕关节零位标定——采集脚本（按 kuavo.json 适配机型）。

流程：双臂 + 头按 teach JSON 下发到一组姿态，到位静止 hold 后，
同步采集 /sensors_data_raw 的完整 joint_q（维数由 kuavo.json 决定）与动捕刚体
相对 torso 的 6DOF 位姿，输出 JSON 供 mocap_optimize.sh（Ceres）离线优化。

复用：
- demos/kuavo_both_arms/both_arms_table_publisher.py  双臂下发/切模式/hold
- demos/kuavo_head_demo/head_table_publisher.py        头部下发/hold
- mocap_checkerboard_pose/mocap_pose_utils.py          相对位姿/工装偏移/3σ
- hand_accuracy_test/run_hand_accuracy_test.py         控制切换逻辑参考
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import rospy
from kuavo_msgs.msg import robotHeadMotionData, sensorsData
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float64

_SCRIPT_DIR = Path(__file__).resolve().parent
_CAMERA_CAL_DIR = _SCRIPT_DIR.parent
_KUAVO_CONFIG_DIR = _SCRIPT_DIR.parents[1] / "kuavo_assets" / "config"
_MOCAP_DIR = _CAMERA_CAL_DIR / "mocap_checkerboard_pose"
_BOTH_ARMS_DIR = _CAMERA_CAL_DIR / "demos" / "kuavo_both_arms"
_HEAD_DIR = _CAMERA_CAL_DIR / "demos" / "kuavo_head_demo"
_ROBOT_CALIB_DEFAULTS = {
    "45": ("biped45", "biped_v3_arm_s45.urdf", "base_link"),
    "52": ("biped52", "biped_v3_arm.urdf", "waist_yaw_link"),
    "56": ("biped56", "biped_v3_arm_s56.urdf", "waist_yaw_link"),
    "62": ("wheel62", "biped_v3_arm_s62.urdf", "waist_yaw_link"),
    "63": ("wheel62", "biped_v3_arm_s62.urdf", "waist_yaw_link"),
}
for _p in (str(_MOCAP_DIR), str(_BOTH_ARMS_DIR), str(_HEAD_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mocap_pose_utils import (  # noqa: E402
    apply_link_offsets_to_poses,
    compute_relative_poses,
    ensure_quat_continuity,
    filter_and_aggregate_poses,
    is_valid_pose,
)
from plot_board_error_from_csv import (  # noqa: E402
    fk_root_to_tip_transform,
    load_urdf_joints,
)
from both_arms_table_publisher import (  # noqa: E402
    SensorsArmMonitor,
    _try_call,
    _try_enable_arm_traj_interpolator,
    _try_lb_quick_mode,
    hold_pose as arm_hold_pose,
    load_arm_points_from_teach_json,
    publish_segment as arm_publish_segment,
)
from head_table_publisher import (  # noqa: E402
    HeadJointStateMonitor,
    hold_pose as head_hold_pose,
    load_head_points_from_teach_json,
    publish_segment as head_publish_segment,
)
_deg = lambda r: float(r) * 180.0 / math.pi  # noqa: E731


def _apply_fixture_override(
    cfg: Dict[str, Any], fixture_id: str, mocap_source: str
) -> Dict[str, Any]:
    """从当前 YAML 的 fixtures 选择本次采集的左右手偏移。"""
    descriptions = {"legacy": "旧手臂末端工装", "new": "新手臂末端工装"}
    if fixture_id not in descriptions:
        raise ValueError(f"未知手臂工装: {fixture_id}")
    offsets = (cfg.get("fixtures") or {}).get(fixture_id)
    if not isinstance(offsets, dict) or not {"l_hand", "r_hand"} <= set(offsets):
        raise ValueError(f"配置缺少 fixtures.{fixture_id}.l_hand/r_hand")
    bodies = {body.get("name"): body for body in cfg.get("mocap_bodies", [])}
    for name in ("l_hand", "r_hand"):
        if name not in bodies:
            raise ValueError(f"mocap_bodies 缺少 {name}")
        bodies[name]["link_offset_mm"] = list(offsets[name])
    torso = bodies.get("torso") or {}
    fixture = {
        "id": fixture_id,
        "description": descriptions[fixture_id],
        "primary_source": mocap_source,
        "torso_offset_mm": list(torso.get("link_offset_mm", [])),
    }
    cfg["fixture"] = fixture
    return fixture


def _load_config(path: Path) -> Dict[str, Any]:
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _run_parallel_actions(stage: str, actions: Dict[str, Any]) -> None:
    """同时执行头部和双臂动作；等待所有动作结束后再汇总异常。

    某一路先失败时不会取消另一路，避免头臂其中一个发布器异常导致另一条
    轨迹只执行一半；两路都结束后再由主流程停止本次采集。
    """
    if not actions:
        return
    if len(actions) == 1:
        next(iter(actions.values()))()
        return

    errors = []
    rospy.loginfo("并行执行头臂动作: %s", stage)
    with ThreadPoolExecutor(max_workers=len(actions)) as executor:
        futures = {name: executor.submit(action) for name, action in actions.items()}
        for name, future in futures.items():
            try:
                future.result()
            except Exception as exc:  # 先等待另一动作完成，再统一抛错
                errors.append((name, exc))
                rospy.logerr("并行动作失败: stage=%s part=%s error=%s", stage, name, exc)
    if errors:
        failed = ", ".join(name for name, _ in errors)
        raise RuntimeError(f"并行头臂动作失败: stage={stage}, parts={failed}") from errors[0][1]


def _wait_for_subscriber(pub, topic: str, timeout: float = 10.0) -> None:
    """联合采集前确认触发话题已有订阅者，避免运动完成后才发现头部零样本。"""
    start = time.time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            rospy.loginfo("头部采集订阅已连接: %s", topic)
            return
        if timeout > 0 and (time.time() - start) > timeout:
            raise TimeoutError(
                f"等待头部采集订阅者超时: {topic}；"
                "请先启动 kuavo_head_demo.launch 的 capture_to_csv"
            )
        rate.sleep()


class FullJointQMonitor:
    """订阅 /sensors_data_raw，校验并保存完整 joint_q（供 FK 用实测关节角）。"""

    def __init__(self, topic: str = "/sensors_data_raw", expected_len: int = 29) -> None:
        self._topic = topic
        self._expected_len = int(expected_len)
        self._last_msg = None
        self._sub = rospy.Subscriber(topic, sensorsData, self._cb, queue_size=10)

    def _cb(self, msg: sensorsData) -> None:
        self._last_msg = msg

    def wait_until_valid(self, timeout: float) -> None:
        start = time.time()
        rate = rospy.Rate(50)
        observed_len = None
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                q = list(self._last_msg.joint_data.joint_q)
                observed_len = len(q)
                if len(q) == self._expected_len:
                    return
            if timeout > 0 and (time.time() - start) > timeout:
                observed = "尚未收到消息" if observed_len is None else f"实际={observed_len}"
                raise TimeoutError(
                    f"等待有效 sensorsData（joint_q 长度应为 {self._expected_len}）超时，{observed}；"
                    "请核对 ROBOT_VERSION 与当前机器人 kuavo.json"
                )
            rate.sleep()

    def get_joint_q_full(self) -> List[float]:
        if self._last_msg is None:
            raise RuntimeError("尚未收到 sensorsData 消息")
        return [float(x) for x in self._last_msg.joint_data.joint_q]


class RigidBodySampler:
    """通用动捕刚体采样：任意刚体集合，逐帧扣 link_offset 后按 relative_pairs 算相对位姿。

    relative_pairs: [(child, parent), ...]，输出 child_in_parent。
    reference 保留兼容（旧调用）；relative_pairs 优先。
    """

    def __init__(
        self,
        bodies_cfg: List[Dict[str, Any]],
        reference: str = "torso",
        relative_pairs: Optional[List[tuple]] = None,
    ) -> None:
        self._bodies = bodies_cfg
        self._reference = reference
        if relative_pairs:
            self._pairs = [tuple(p) for p in relative_pairs]
        else:
            # 兼容旧调用：所有非 reference 刚体相对 reference
            self._pairs = [
                (b["name"], reference)
                for b in bodies_cfg
                if b["name"] != reference
            ]
        self._link_offsets = {
            b["name"]: np.asarray(b.get("link_offset_mm", [0, 0, 0]), dtype=float)
            for b in bodies_cfg
        }
        self._cache: Dict[str, Optional[Dict[str, np.ndarray]]] = {
            b["name"]: None for b in bodies_cfg
        }
        self._prev_quats: Dict[str, np.ndarray] = {}
        self._subs = []
        for body in bodies_cfg:
            topic = body["topic"]
            msg_type = body.get("msg_type", "PoseStamped")
            if msg_type == "Pose":
                sub = rospy.Subscriber(
                    topic,
                    Pose,
                    lambda msg, n=body["name"]: self._on_pose(msg, n),
                    queue_size=10,
                )
            else:
                sub = rospy.Subscriber(
                    topic,
                    PoseStamped,
                    lambda msg, n=body["name"]: self._on_pose_stamped(msg, n),
                    queue_size=10,
                )
            self._subs.append(sub)
            rospy.loginfo("[mocap] 订阅 %s (%s) -> %s", topic, msg_type, body["name"])

    def _extract_pose(self, msg, name: str) -> None:
        if hasattr(msg, "pose"):
            p = msg.pose.position
            o = msg.pose.orientation
        else:
            p = msg.position
            o = msg.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = np.array([o.x, o.y, o.z, o.w], dtype=float)
        if not is_valid_pose(pos, quat):
            return
        prev = self._prev_quats.get(name)
        quat = ensure_quat_continuity(quat, prev)
        self._prev_quats[name] = quat.copy()
        self._cache[name] = {"pos": pos, "quat": quat}

    def _on_pose_stamped(self, msg: PoseStamped, name: str) -> None:
        self._extract_pose(msg, name)

    def _on_pose(self, msg: Pose, name: str) -> None:
        self._extract_pose(msg, name)

    def _snapshot_ready(self) -> bool:
        return all(self._cache[n] is not None for n in self._cache)

    def _frame_relative_poses(self) -> Optional[Dict[str, Dict[str, np.ndarray]]]:
        if not self._snapshot_ready():
            return None
        raw = {n: self._cache[n].copy() for n in self._cache}  # type: ignore
        corrected = apply_link_offsets_to_poses(raw, self._link_offsets)
        out: Dict[str, Dict[str, np.ndarray]] = {}
        for child, parent in self._pairs:
            pos_mm, quat = compute_relative_poses(
                corrected[child]["pos"],
                corrected[child]["quat"],
                corrected[parent]["pos"],
                corrected[parent]["quat"],
            )
            out[f"{child}_in_{parent}"] = {"pos": pos_mm, "quat": quat}
        return out

    def collect_window(
        self, duration_sec: float
    ) -> List[Dict[str, Dict[str, np.ndarray]]]:
        frames: List[Dict[str, Dict[str, np.ndarray]]] = []
        t_end = time.time() + max(0.0, duration_sec)
        rate = rospy.Rate(100)
        while time.time() < t_end and not rospy.is_shutdown():
            rel = self._frame_relative_poses()
            if rel is not None:
                frames.append(rel)
            rate.sleep()
        return frames

    def collect_raw_window(
        self, duration_sec: float
    ) -> List[Dict[str, Dict[str, np.ndarray]]]:
        """采集窗口内收集原始世界位姿（未扣 link_offset，话题原始数据）。

        返回: [{刚体名: {"pos": mm, "quat": xyzw}}, ...]（世界系）
        用于保存动捕原始数据，便于优化端恢复完整对齐（含 torso 世界姿态）。
        """
        frames: List[Dict[str, Dict[str, np.ndarray]]] = []
        t_end = time.time() + max(0.0, duration_sec)
        rate = rospy.Rate(100)
        while time.time() < t_end and not rospy.is_shutdown():
            if self._snapshot_ready():
                snapshot = {
                    n: {"pos": self._cache[n]["pos"].copy(),  # type: ignore
                        "quat": self._cache[n]["quat"].copy()}  # type: ignore
                    for n in self._cache
                }
                frames.append(snapshot)
            rate.sleep()
        return frames


def _aggregate_body(frames: List[Dict[str, Dict[str, np.ndarray]]], key: str, sigma: float):
    pts = np.array([f[key]["pos"] for f in frames], dtype=float)
    quats = np.array([f[key]["quat"] for f in frames], dtype=float)
    agg = filter_and_aggregate_poses(pts, quats, sigma=sigma)
    return {
        "xyz_mm": agg["xyz_mm"].tolist(),
        "quaternion_xyzw": agg["quaternion_xyzw"].tolist(),
        "valid_frames": agg["valid_frames"],
        "pos_std_mm": agg["pos_std_mm"],
        "rot_std_deg": agg["rot_std_deg"],
    }


def _aggregate_raw(frames: List[Dict[str, Dict[str, np.ndarray]]], sigma: float):
    """聚合原始世界位姿（未扣 link_offset），输出 {刚体名: {xyz_mm, quaternion}}。

    保留原始世界系数据，供优化端恢复完整对齐（含 torso 世界姿态）。
    """
    if not frames:
        return {}
    body_names = frames[0].keys()
    out = {}
    for name in body_names:
        pts = np.array([f[name]["pos"] for f in frames if name in f], dtype=float)
        quats = np.array([f[name]["quat"] for f in frames if name in f], dtype=float)
        if len(pts) < 3:
            continue
        agg = filter_and_aggregate_poses(pts, quats, sigma=sigma)
        out[name] = {
            "xyz_mm": agg["xyz_mm"].tolist(),
            "quaternion_xyzw": agg["quaternion_xyzw"].tolist(),
            "valid_frames": agg["valid_frames"],
            "pos_std_mm": agg["pos_std_mm"],
            "rot_std_deg": agg["rot_std_deg"],
        }
    return out


def _setup_robot_control(
    robot_cfg: Dict[str, Any],
    monitor: SensorsArmMonitor,
) -> rospy.Publisher:
    """切 wheel62 外控/quick mode/插补，返回 /kuavo_arm_traj publisher。"""
    from sensor_msgs.msg import JointState

    dt = float(robot_cfg.get("dt", 0.01))
    is_wheel62 = robot_cfg.get("robot_layout", "wheel62") == "wheel62"
    pre_mode_hold = float(robot_cfg.get("pre_mode_hold_sec", 0.6))
    post_mode_hold = float(robot_cfg.get("post_mode_hold_sec", 0.6))

    cur = monitor.get_current_left_right_deg()
    current_left = cur["left_deg"]
    current_right = cur["right_deg"]

    arm_pub = rospy.Publisher(
        robot_cfg.get("arm_traj_topic", "/kuavo_arm_traj"),
        JointState,
        queue_size=10,
        tcp_nodelay=True,
    )

    if pre_mode_hold > 0:
        arm_hold_pose(arm_pub, current_left, current_right, pre_mode_hold, dt)

    if robot_cfg.get("set_external_control_mode", True):
        if is_wheel62:
            services = (
                "/wheel_arm_change_arm_ctrl_mode",
                "/change_arm_ctrl_mode",
                "/arm_traj_change_mode",
                "/humanoid_change_arm_ctrl_mode",
            )
        else:
            services = (
                "/arm_traj_change_mode",
                "/humanoid_change_arm_ctrl_mode",
                "/change_arm_ctrl_mode",
            )
        ok = False
        for srv in services:
            ok = _try_call(srv, 2) or ok
        if not ok:
            rospy.logwarn("未能切换到 external_control(2)，继续执行")

    if robot_cfg.get("enable_arm_quick_mode", True):
        if is_wheel62:
            if not _try_lb_quick_mode("/enable_lb_arm_quick_mode", 2):
                rospy.logwarn("未能使能 /enable_lb_arm_quick_mode")
        else:
            if not _try_call("/enable_wbc_arm_trajectory_control", 1):
                rospy.logwarn("未能使能 /enable_wbc_arm_trajectory_control")

    if is_wheel62 and robot_cfg.get("enable_arm_traj_interpolator", True):
        if not _try_enable_arm_traj_interpolator(True):
            rospy.logwarn("未能开启 /enable_arm_traj_interpolator")

    rospy.sleep(0.2)
    if post_mode_hold > 0:
        arm_hold_pose(arm_pub, current_left, current_right, post_mode_hold, dt)

    return arm_pub


def _sanity_check_arm_layout(
    commanded_left_deg: List[float],
    joint_q_full: List[float],
    joint_indices: Dict[str, Any],
    tag: str,
) -> None:
    """比较某点 command(deg) 与 reported(rad) 的首关节，布局疑似错误时 WARN。"""
    lo, hi = joint_indices.get("left_arm", [13, 20])
    if not (0 <= lo < len(joint_q_full) and hi <= len(joint_q_full)):
        rospy.logwarn("[%s] joint_q 长度不足，跳过布局 sanity check", tag)
        return
    reported0_deg = _deg(float(joint_q_full[lo]))
    commanded0_deg = float(commanded_left_deg[0]) if commanded_left_deg else 0.0
    diff = abs(reported0_deg - commanded0_deg)
    if diff > 0.2 * 180.0 / math.pi:  # ~11.5°，明显不匹配
        rospy.logwarn(
            "[%s] ⚠️ 疑似 joint_q 索引布局错误：left_arm[%d]=%.2f° vs 下发=%.2f°，"
            "偏差 %.2f°。请核对 calib.yaml joint_indices 与 62 机实际布局。",
            tag, lo, reported0_deg, commanded0_deg, diff,
        )


def _record_one_waypoint(
    sampler: RigidBodySampler,
    q_monitor: FullJointQMonitor,
    collect_sec: float,
    sigma: float,
) -> Dict[str, Any]:
    """静止后：读一次完整 joint_q 快照 + 采集动捕窗口，聚合输出。

    同时采集原始世界位姿（raw_world）和相对位姿（bodies），
    以便优化端恢复完整对齐（含 torso 世界姿态）。
    """
    joint_q_full = q_monitor.get_joint_q_full()
    frames = sampler.collect_window(collect_sec)
    raw_frames = sampler.collect_raw_window(collect_sec)
    bodies_out: Dict[str, Any] = {}
    for child, parent in sampler._pairs:
        key = f"{child}_in_{parent}"
        valid = [f for f in frames if key in f]
        if len(valid) < 3:
            rospy.logwarn("刚体 %s 有效帧过少 (%d)，该样本此刚体置空", key, len(valid))
            bodies_out[key] = None
            continue
        bodies_out[key] = _aggregate_body(valid, key, sigma)
    raw_world = _aggregate_raw(raw_frames, sigma)
    return {
        "joint_q_full": joint_q_full,
        "bodies": bodies_out,
        "raw_world": raw_world,
    }


def _print_waypoint(
    tag: str,
    rec: Dict[str, Any],
    joint_indices: Dict[str, Any],
    urdf_path: Path = None,
    fk_root: str = "base_link",
    body_links: Dict[str, str] = None,
    R_align: np.ndarray = None,
) -> None:
    """终端打印当前样本的关节角 + 动捕 6DOF + FK 预测对比，便于现场核对。"""
    print("\n" + "=" * 64)
    print(f"  样本 {tag} 采集明细")
    print("=" * 64)

    q = rec["joint_q_full"]
    ji = joint_indices
    # 提取关节角
    arm_names = [
        "waist_yaw_joint",
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
        "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
        "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
    ]
    def _val(idx):
        if idx >= 0:
            return float(q[idx]) if idx < len(q) else float("nan")
        return float(q[len(q) + idx]) if (len(q) + idx) >= 0 else float("nan")

    print("\n  -- 关节角 (rad) --")
    if "waist" in ji:
        print(f"  {'waist_yaw':<12} {_val(int(ji.get('waist', 12))):+.6f}")
    lo, _hi = ji.get("left_arm", [13, 20])
    for k, name in enumerate(arm_names[1:8]):
        print(f"  {name:<12} {_val(int(lo) + k):+.6f}")
    lo, _hi = ji.get("right_arm", [20, 27])
    for k, name in enumerate(arm_names[8:15]):
        print(f"  {name:<12} {_val(int(lo) + k):+.6f}")

    # 动捕 6DOF
    print("\n  -- 动捕 6DOF (hand_in_torso) --")
    for key, obs in rec["bodies"].items():
        if obs is None:
            print(f"  {key:<18} 无效（帧不足）")
            continue
        xyz = obs["xyz_mm"]
        quat = obs["quaternion_xyzw"]
        print(f"  {key:<18} pos(mm)=[{xyz[0]:+.1f}, {xyz[1]:+.1f}, {xyz[2]:+.1f}]  "
              f"quat(xyzw)=[{quat[0]:+.4f}, {quat[1]:+.4f}, {quat[2]:+.4f}, {quat[3]:+.4f}]")
        print(f"  {'':<18} valid_frames={obs['valid_frames']}  "
              f"pos_std={obs['pos_std_mm']:.2f}mm  rot_std={obs['rot_std_deg']:.3f}°")

    # FK 预测对比（若提供 URDF）
    if urdf_path is not None and urdf_path.exists():
        try:
            joints = load_urdf_joints(urdf_path)
        except Exception as e:
            print(f"\n  -- FK 预测: 加载 URDF 失败 ({e}) --")
            print("=" * 64 + "\n")
            return
        print(f"\n  -- FK 预测 vs 动捕 (相对 {fk_root}) --")
        FREES = arm_names[1:]
        for body_key, obs in rec["bodies"].items():
            if obs is None or body_links is None:
                continue
            # 只验证相对 torso（腰部）的观测；相对左肩(l_shoulder)的观测
            # 因动捕肩刚体未对齐 URDF 参考系，无 FK 验证意义，跳过。
            if "_in_" in body_key and not body_key.endswith("_in_torso"):
                continue
            body = body_key.split("_in_")[0]
            tip = body_links.get(body)
            if tip is None:
                continue
            fk_root_this = fk_root
            q_used = {}
            # 填 FK 链上所有关节
            from plot_board_error_from_csv import find_chain_joint_names
            try:
                chain = find_chain_joint_names(joints, fk_root_this, tip)
            except RuntimeError as e:
                print(f"  {body:<8} (无法 FK: {fk_root_this}->{tip} 链不通)")
                continue
            for jn in chain:
                if jn in arm_names:
                    if "waist" in ji and jn == "waist_yaw_joint":
                        q_used[jn] = _val(int(ji["waist"]))
                    elif jn.startswith("zarm_l"):
                        k = int(jn.split("_")[1][1]) - 1  # l1->0
                        q_used[jn] = _val(int(ji.get("left_arm",[13,20])[0]) + k)
                    elif jn.startswith("zarm_r"):
                        k = int(jn.split("_")[1][1]) - 1
                        q_used[jn] = _val(int(ji.get("right_arm",[20,27])[0]) + k)
            try:
                T = fk_root_to_tip_transform(joints, fk_root_this, tip, q_used)
            except RuntimeError as e:
                print(f"  {body:<8} (无法 FK: {e})")
                continue
            fk_p = T[:3, 3] * 1000.0
            # 动捕（flip 后）
            m = R_align @ np.array(obs["xyz_mm"]) if R_align is not None else np.array(obs["xyz_mm"])
            diff = fk_p - m
            print(f"  {body:<8} FK(mm)=[{fk_p[0]:+.1f}, {fk_p[1]:+.1f}, {fk_p[2]:+.1f}]  "
                  f"动捕(align后)=[{m[0]:+.1f}, {m[1]:+.1f}, {m[2]:+.1f}]  "
                  f"差=[{diff[0]:+.1f}, {diff[1]:+.1f}, {diff[2]:+.1f}]")

    print("=" * 64 + "\n")



def _resolve_robot_version(arg: str) -> str:
    """解析机型；auto 读取 ROBOT_VERSION，未设置时返回空串以兼容旧配置。"""
    if arg and arg != "auto":
        return str(arg).strip()
    return os.environ.get("ROBOT_VERSION", "").strip()


def _find_kuavo_json(robot_version: str, override: str = "") -> Optional[Path]:
    """定位当前机型的 kuavo.json；显式路径优先。"""
    if override:
        path = Path(override).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"kuavo.json 不存在: {path}")
        return path
    if not robot_version:
        return None
    path = _KUAVO_CONFIG_DIR / f"kuavo_v{robot_version}" / "kuavo.json"
    if not path.is_file():
        raise FileNotFoundError(
            f"找不到 ROBOT_VERSION={robot_version} 对应的 kuavo.json: {path}"
        )
    return path


def _apply_kuavo_joint_layout(
    robot: Dict[str, Any],
    robot_version: str,
    kuavo_json_override: str = "",
) -> Optional[Dict[str, Any]]:
    """由 kuavo.json 推导 joint_q 中腰、双臂和头部的连续索引。"""
    if not bool(robot.get("auto_joint_layout", True)):
        return None

    path = _find_kuavo_json(robot_version, kuavo_json_override)
    if path is None:
        print(
            "[WARN] 未设置 ROBOT_VERSION，无法自动读取 kuavo.json；沿用 YAML 关节索引",
            file=sys.stderr,
        )
        return None

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    try:
        num_joint = int(data["NUM_JOINT"])
        num_arm = int(data["NUM_ARM_JOINT"])
        num_head = int(data.get("NUM_HEAD_JOINT", 0))
        num_waist = int(data.get("NUM_WAIST_JOINT", 0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path} 的关节数量字段无效: {exc}") from exc

    motors = data.get("MOTORS_TYPE")
    if not isinstance(motors, list) or len(motors) != num_joint:
        actual = len(motors) if isinstance(motors, list) else "非数组"
        raise ValueError(
            f"{path} 的 MOTORS_TYPE 长度({actual})与 NUM_JOINT({num_joint})不一致"
        )
    if num_arm != 14:
        raise ValueError(
            f"当前采集轨迹只支持双臂各 7 轴，但 {path} 的 NUM_ARM_JOINT={num_arm}"
        )
    if num_head < 0 or num_waist < 0:
        raise ValueError(f"{path} 的 NUM_HEAD_JOINT/NUM_WAIST_JOINT 不能为负数")

    prefix_count = num_joint - num_arm - num_head
    if prefix_count < num_waist:
        raise ValueError(
            f"{path} 无法推导布局: 前缀关节数={prefix_count}, 腰部关节数={num_waist}"
        )

    left_start = prefix_count
    right_start = left_start + num_arm // 2
    head_start = right_start + num_arm // 2
    indices: Dict[str, Any] = {
        "left_arm": [left_start, right_start],
        "right_arm": [right_start, head_start],
        "head": [head_start, head_start + num_head],
    }
    if num_waist > 0:
        # kuavo.json 的排列约定为下肢、腰部、双臂、头部；首个腰关节为 yaw。
        indices["waist"] = prefix_count - num_waist

    robot["joint_q_length"] = num_joint
    robot["joint_indices"] = indices
    robot["left_start_index"] = left_start
    robot["right_start_index"] = right_start
    return {
        "robot_version": robot_version or "custom",
        "kuavo_json": str(path),
        "num_joint": num_joint,
        "joint_indices": indices,
    }


def _apply_robot_calibration_defaults(
    robot: Dict[str, Any],
    calibration: Dict[str, Any],
    robot_version: str,
) -> Optional[Dict[str, str]]:
    """按实机版本选择控制布局、标定 URDF 和 FK 根；显式 CLI 参数可再覆盖。"""
    if not robot_version:
        return None
    defaults = _ROBOT_CALIB_DEFAULTS.get(robot_version)
    if defaults is None:
        supported = ", ".join(_ROBOT_CALIB_DEFAULTS)
        raise ValueError(
            f"ROBOT_VERSION={robot_version} 尚无标定 URDF 映射；当前支持 {supported}"
        )
    layout, urdf_name, fk_root = defaults
    urdf_path = (_CAMERA_CAL_DIR / urdf_name).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"ROBOT_VERSION={robot_version} 对应 URDF 不存在: {urdf_path}")
    robot["robot_layout"] = layout
    calibration["urdf"] = str(urdf_path)
    calibration["fk_root"] = fk_root
    return {
        "robot_layout": layout,
        "urdf": str(urdf_path),
        "fk_root": fk_root,
    }


def _resolve_config_path(arg: str, robot_version: str = "") -> Path:
    """按 --config 或 ROBOT_VERSION 解析 config 路径。

    auto：读 ROBOT_VERSION 环境变量
      45  → config/calib_s45.yaml（biped45，无腰）
      52/56 → config/calib.yaml（biped52/56，走默认）
      62/63 → config/calib.yaml（wheel62，默认）
    也可显式传 --robot-version 45/52/56/62/63 覆盖。
    """
    if arg:
        return Path(arg)

    rv = robot_version or os.environ.get("ROBOT_VERSION", "").strip()
    if rv == "45":
        return _SCRIPT_DIR / "config" / "calib_s45.yaml"
    if rv in ("52", "56", "62", "63"):
        return _SCRIPT_DIR / "config" / "calib.yaml"
    if rv:
        print(f"[WARN] 未知 ROBOT_VERSION={rv}，回退默认 calib.yaml", file=sys.stderr)
    return _SCRIPT_DIR / "config" / "calib.yaml"


def _resolve_path(base: Path, p: str) -> Path:
    """相对路径以 base（默认脚本目录）为基准解析，不依赖当前工作目录。

    与相机方案（both_arms_table_publisher.py / head_table_publisher.py）一致：
    配置里的相对路径按脚本目录推算，保证从 workspace 根或任意目录运行都能找到。
    """
    path = Path(p)
    return path if path.is_absolute() else (base / path).resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description="纯动捕关节零位标定——采集（按 kuavo.json 适配关节布局）")
    parser.add_argument(
        "--config",
        default="",
        help="calib.yaml 路径（默认按 ROBOT_VERSION 自动选：45→calib_s45.yaml，其余→calib.yaml）",
    )
    parser.add_argument(
        "--robot-version",
        default="auto",
        help="机型版本：auto（读 ROBOT_VERSION）或 kuavo_assets 中的版本号",
    )
    parser.add_argument(
        "--kuavo-json",
        default="",
        help="显式指定 kuavo.json（默认按 ROBOT_VERSION 从 kuavo_assets/config 查找）",
    )
    parser.add_argument(
        "--robot-layout",
        choices=("biped45", "biped52", "biped56", "wheel62"),
        default="",
        help="显式覆盖配置中的控制布局",
    )
    parser.add_argument(
        "--fixture",
        choices=("legacy", "new"),
        default=os.environ.get("MOCAP_FIXTURE", "legacy"),
        help="手臂末端工装（默认读 MOCAP_FIXTURE，否则 legacy）",
    )
    parser.add_argument(
        "--mocap-source",
        choices=("auto", "motive", "qingtong"),
        default="auto",
        help="工装参数对应的动捕源；auto 根据刚体话题判断",
    )
    parser.add_argument("--urdf", default="", help="显式覆盖标定使用的 URDF")
    parser.add_argument("--fk-root", default="", help="显式覆盖 FK 根节点")
    head_mode_group = parser.add_mutually_exclusive_group()
    head_mode_group.add_argument(
        "--no-capture-head",
        action="store_true",
        help="仅采集双臂，不下发头部轨迹（纯双臂零点标定推荐）",
    )
    head_mode_group.add_argument(
        "--capture-head-camera",
        action="store_true",
        help=(
            "联合采集：下发头部 teach 轨迹，并在双臂每个有效静止点发布"
            " /head_keyframe_flag；结束时发布 /head_keyframe_done"
        ),
    )
    parser.add_argument("--output", default=None, help="输出 JSON 路径（默认自动带时间戳）")
    parser.add_argument(
        "--motion-only",
        action="store_true",
        help="只运动模式：仅下发 teach 轨迹逐点运动，不采集动捕、不写 JSON（用于核对运动点位是否正常）",
    )
    args = parser.parse_args()

    robot_version = _resolve_robot_version(args.robot_version)
    cfg = _load_config(_resolve_config_path(args.config, robot_version))
    robot = cfg["robot"]
    layout_info = _apply_kuavo_joint_layout(robot, robot_version, args.kuavo_json)
    calib = cfg["calibration"]
    model_info = _apply_robot_calibration_defaults(robot, calib, robot_version)
    if args.robot_layout:
        robot["robot_layout"] = args.robot_layout
    if args.urdf:
        calib["urdf"] = str(Path(args.urdf).expanduser().resolve())
    if args.fk_root:
        calib["fk_root"] = args.fk_root
    if args.no_capture_head:
        robot["capture_head"] = False
    elif args.capture_head_camera:
        robot["capture_head"] = True
    mocap_source = args.mocap_source
    if mocap_source == "auto":
        topics = [str(body.get("topic", "")) for body in cfg.get("mocap_bodies", [])]
        mocap_source = "qingtong" if any("/vrpn_client_node/" in topic for topic in topics) else "motive"
    fixture_info = _apply_fixture_override(cfg, args.fixture, mocap_source)
    bodies_cfg = cfg["mocap_bodies"]
    joint_indices = robot["joint_indices"]
    if args.capture_head_camera:
        head_range = joint_indices.get("head", [])
        if (
            not isinstance(head_range, (list, tuple))
            or len(head_range) != 2
            or int(head_range[1]) - int(head_range[0]) != 2
        ):
            raise ValueError(
                f"联合采集只支持两轴头部，但当前 kuavo.json 推导 head={head_range}"
            )

    rospy.init_node("record_joint_poses", anonymous=False)
    rospy.loginfo(
        "手臂工装: %s (%s), mocap=%s, torso_offset_mm=%s",
        fixture_info["id"], fixture_info["description"], mocap_source,
        fixture_info["torso_offset_mm"],
    )
    if layout_info is not None:
        rospy.loginfo(
            "按 kuavo.json 适配关节布局: version=%s joint_q=%d left=%s right=%s head=%s waist=%s",
            layout_info["robot_version"],
            layout_info["num_joint"],
            joint_indices["left_arm"],
            joint_indices["right_arm"],
            joint_indices["head"],
            joint_indices.get("waist", "无"),
        )

    # 加载 teach（相对路径以脚本目录为基准解析，不依赖 cwd）
    # capture_head: 是否采集/下发头部（默认 False）。s45 头部无工装、不参与双臂零位标定，
    #               因此默认只采集左右手，头部保持不动。
    capture_head = bool(robot.get("capture_head", False))
    left_points = load_arm_points_from_teach_json(
        _resolve_path(_SCRIPT_DIR, calib["teach_left"]), "left_arm_joints")
    right_points = load_arm_points_from_teach_json(
        _resolve_path(_SCRIPT_DIR, calib["teach_right"]), "right_arm_joints")
    head_points = (
        load_head_points_from_teach_json(_resolve_path(_SCRIPT_DIR, calib["teach_head"]))
        if capture_head else []
    )
    rospy.loginfo(
        "加载 teach: left=%d right=%d head=%d (capture_head=%s, head_camera=%s)",
        len(left_points), len(right_points), len(head_points), capture_head,
        args.capture_head_camera,
    )

    # 双臂轨迹首尾通常是重复零位，实际采集点为中间 9 点；联合模式将头部 9 个
    # teach 姿态严格映射到这 9 个有效点，避免漏掉 head[0] 或重复 head[-1]。
    n_pts = max(len(left_points), len(right_points))
    skip_first_last = bool(calib.get("skip_first_last", True))
    sampled_arm_indices = [
        i for i in range(n_pts)
        if not (skip_first_last and n_pts > 2 and (i == 0 or i == n_pts - 1))
    ]
    head_index_by_arm_index = {
        arm_index: head_index
        for head_index, arm_index in enumerate(sampled_arm_indices)
    }
    if args.capture_head_camera and len(head_points) != len(sampled_arm_indices):
        raise ValueError(
            "联合采集要求头部 teach 点数等于双臂有效采样点数："
            f"head={len(head_points)}, arm_samples={len(sampled_arm_indices)}, "
            f"arm_total={n_pts}, skip_first_last={skip_first_last}"
        )

    # 等待传感器有效
    # 先校验完整 joint_q 长度，机型配置不一致时可直接看到期望值和实际值。
    q_monitor = FullJointQMonitor(
        topic=robot.get("sensor_topic", "/sensors_data_raw"),
        expected_len=int(robot.get("joint_q_length", 29)),
    )
    q_monitor.wait_until_valid(timeout=float(robot.get("wait_sensors_timeout", 30.0)))
    monitor = SensorsArmMonitor(
        topic=robot.get("sensor_topic", "/sensors_data_raw"),
        left_start_index=int(robot.get("left_start_index", 13)),
        right_start_index=int(robot.get("right_start_index", 20)),
    )
    monitor.wait_until_valid_arms(timeout=float(robot.get("wait_sensors_timeout", 30.0)))
    head_monitor = None
    if capture_head:
        head_monitor = HeadJointStateMonitor(topic=robot.get("sensor_topic", "/sensors_data_raw"))
        head_monitor.wait_until_valid_head(timeout=float(robot.get("wait_sensors_timeout", 30.0)))

    # 切控制模式
    arm_pub = _setup_robot_control(robot, monitor)
    head_pub = None
    head_keyframe_pub = None
    head_done_pub = None
    if capture_head:
        head_pub = rospy.Publisher(
            robot.get("head_traj_topic", "/robot_head_motion_data"),
            robotHeadMotionData,
            queue_size=10,
        )
    # motion-only 只复用联合轨迹，不启动/等待相机采集端。
    if args.capture_head_camera and not args.motion_only:
        keyframe_topic = robot.get("head_keyframe_flag_topic", "/head_keyframe_flag")
        done_topic = robot.get("head_keyframe_done_topic", "/head_keyframe_done")
        head_keyframe_pub = rospy.Publisher(
            keyframe_topic,
            Float64,
            queue_size=10,
        )
        head_done_pub = rospy.Publisher(
            done_topic,
            Bool,
            queue_size=1,
        )
        subscriber_timeout = float(robot.get("head_capture_subscriber_timeout", 10.0))
        _wait_for_subscriber(head_keyframe_pub, keyframe_topic, subscriber_timeout)
        _wait_for_subscriber(head_done_pub, done_topic, subscriber_timeout)

    # 动捕采样器（relative_pairs 从 config 读：默认 hand_in_torso + hand_in_l_shoulder）
    sampler = RigidBodySampler(
        bodies_cfg,
        reference="torso",
        relative_pairs=cfg.get("relative_pairs"),
    )

    # 对齐首点
    first_l = left_points[0]["deg"]
    first_r = right_points[0]["deg"]
    first_h = ({"yaw_deg": head_points[0]["yaw_deg"], "pitch_deg": head_points[0]["pitch_deg"]}
               if capture_head else {"yaw_deg": 0.0, "pitch_deg": 0.0})
    cur = monitor.get_current_left_right_deg()
    current_left, current_right = list(cur["left_deg"]), list(cur["right_deg"])
    cur_h = (head_monitor.get_current_yaw_pitch_deg()
             if capture_head and head_monitor is not None
             else {"yaw_deg": 0.0, "pitch_deg": 0.0})

    dt = float(robot.get("dt", 0.01))
    move_duration = float(robot.get("move_duration", 2.0))
    hold_sec = float(robot.get("hold_sec", 5.0))
    collect_sec = float(robot.get("collect_sec", 1.0))
    startup_align_sec = float(robot.get("startup_align_sec", 2.0))
    return_to_zero_sec = float(robot.get("return_to_zero_sec", 2.0))
    sigma = float(calib.get("sigma", 3.0))

    # FK 打印参数（用于 _print_waypoint 现场核对 FK vs 动捕）
    _urdf_path = None
    _fk_root = calib.get("fk_root", "base_link")
    _body_links = {b["name"]: b.get("urdf_link") for b in bodies_cfg}
    _R_align = None
    _urdf_rel = calib.get("urdf")
    if _urdf_rel:
        _p = Path(_urdf_rel)
        if not _p.is_absolute():
            _p = _SCRIPT_DIR / _p
        _urdf_path = _p.resolve()
    from mocap_pose_utils import load_mocap_frame_align_matrix
    _R_align = load_mocap_frame_align_matrix(cfg.get("mocap_frame_align"))

    startup_actions = {
        "arms": lambda: arm_publish_segment(
            arm_pub, current_left, current_right, first_l, first_r,
            startup_align_sec, dt,
        ),
    }
    if capture_head and head_pub is not None:
        startup_actions["head"] = lambda: head_publish_segment(
            head_pub, cur_h, first_h, startup_align_sec, dt,
        )
    _run_parallel_actions("startup_align", startup_actions)
    current_left, current_right = list(first_l), list(first_r)
    cur_h = dict(first_h)

    samples: List[Dict[str, Any]] = []

    for i in range(n_pts):
        if rospy.is_shutdown():
            break

        if i > 0:
            tgt_l = left_points[i]["deg"] if i < len(left_points) else left_points[-1]["deg"]
            tgt_r = right_points[i]["deg"] if i < len(right_points) else right_points[-1]["deg"]
            move_actions = {
                "arms": lambda: arm_publish_segment(
                    arm_pub, current_left, current_right, tgt_l, tgt_r,
                    move_duration, dt,
                ),
            }
            tgt_h = None
            if capture_head and head_pub is not None:
                if args.capture_head_camera:
                    # 跳过首点时 i=1 仍对应 head[0]；尾部跳过点保持 head[-1]。
                    completed_sample_count = sum(1 for arm_i in sampled_arm_indices if arm_i <= i)
                    head_i = max(0, min(completed_sample_count - 1, len(head_points) - 1))
                else:
                    head_i = min(i, len(head_points) - 1)
                tgt_h = {
                    "yaw_deg": head_points[head_i]["yaw_deg"],
                    "pitch_deg": head_points[head_i]["pitch_deg"],
                }
                move_actions["head"] = lambda: head_publish_segment(
                    head_pub, cur_h, tgt_h, move_duration, dt,
                )
            _run_parallel_actions(f"move_T{i}", move_actions)
            if tgt_h is not None:
                cur_h = dict(tgt_h)
            current_left, current_right = list(tgt_l), list(tgt_r)

        name = f"T{i}"
        # 跳过首尾零位重复帧
        if skip_first_last and n_pts > 2 and (i == 0 or i == n_pts - 1):
            rospy.loginfo("跳过首尾点采集: %s", name)
            hold_actions = {
                "arms": lambda: arm_hold_pose(
                    arm_pub, current_left, current_right, hold_sec, dt,
                ),
            }
            if capture_head and head_pub is not None:
                hold_actions["head"] = lambda: head_hold_pose(
                    head_pub, cur_h, hold_sec, dt,
                )
            _run_parallel_actions(f"hold_{name}", hold_actions)
            continue

        rospy.loginfo("关键点 %s: 静止 %.1fs 后%s", name, hold_sec,
                      "运动（只运动模式，不采集）" if args.motion_only else "采集动捕+关节")
        hold_actions = {
            "arms": lambda: arm_hold_pose(
                arm_pub, current_left, current_right, hold_sec, dt,
            ),
        }
        if capture_head and head_pub is not None:
            hold_actions["head"] = lambda: head_hold_pose(
                head_pub, cur_h, hold_sec, dt,
            )
        _run_parallel_actions(f"hold_{name}", hold_actions)

        if args.motion_only:
            # 只运动模式：打印目标点位，不采集动捕、不写 JSON
            rospy.loginfo("[motion-only] %s: 左臂目标(deg)=%s", name, [f"{v:.1f}" for v in current_left])
            rospy.loginfo("[motion-only] %s: 右臂目标(deg)=%s", name, [f"{v:.1f}" for v in current_right])
            continue

        if args.capture_head_camera:
            if head_keyframe_pub is None or i not in head_index_by_arm_index:
                raise RuntimeError(f"联合采集关键帧映射无效: arm_index={i}")
            keyframe_id = head_index_by_arm_index[i] + 1
            head_keyframe_pub.publish(Float64(data=float(keyframe_id)))
            rospy.loginfo(
                "触发头部棋盘采样: arm_point=%s, head_point=%d/%d, keyframe=%d",
                name, keyframe_id, len(head_points), keyframe_id,
            )

        rec = _record_one_waypoint(sampler, q_monitor, collect_sec, sigma)
        _sanity_check_arm_layout(current_left, rec["joint_q_full"], joint_indices, name)
        _print_waypoint(name, rec, joint_indices,
                        urdf_path=_urdf_path, fk_root=_fk_root,
                        body_links=_body_links, R_align=_R_align)
        rec["index"] = i
        rec["commanded"] = {
            "left_deg": current_left,
            "right_deg": current_right,
            "head_deg": [cur_h["yaw_deg"], cur_h["pitch_deg"]],
        }
        samples.append(rec)
        n_body_ok = sum(1 for v in rec["bodies"].values() if v is not None)
        rospy.loginfo("点 %s 记录完成，动捕有效刚体 %d/%d", name, n_body_ok, len(rec["bodies"]))

    # 回零（双臂；capture_head=True 时头部也回零）
    zero = [0.0] * 7
    return_actions = {
        "arms": lambda: arm_publish_segment(
            arm_pub, current_left, current_right, zero, zero,
            return_to_zero_sec, dt,
        ),
    }
    if capture_head and head_pub is not None:
        zero_h = {"yaw_deg": 0.0, "pitch_deg": 0.0}
        return_actions["head"] = lambda: head_publish_segment(
            head_pub, cur_h, zero_h, return_to_zero_sec, dt,
        )
    _run_parallel_actions("return_to_zero", return_actions)

    if args.capture_head_camera and not args.motion_only:
        if head_done_pub is None:
            raise RuntimeError("联合采集缺少 head_done publisher")
        head_done_pub.publish(Bool(data=True))
        rospy.loginfo("头部棋盘采样完成：已发送 /head_keyframe_done")
        # 允许 TCPROS 将结束消息送达；CSV 的完整落盘由外层一键脚本继续等待。
        rospy.sleep(0.2)

    if args.motion_only:
        rospy.loginfo("只运动模式完成：已逐点下发 teach 轨迹并回零（未采集、未写 JSON）")
        return 0

    if not samples:
        rospy.logfatal("未采到任何样本")
        return 1

    out = {
        "meta": {
            "timestamp": datetime.now().isoformat(),
            "robot_version": robot_version or None,
            "robot_layout": robot.get("robot_layout"),
            "joint_layout_source": layout_info,
            "robot_model_source": model_info,
            "urdf": str(_resolve_path(_SCRIPT_DIR, calib["urdf"])),
            "fk_root": calib["fk_root"],
            "joint_q_indices": joint_indices,
            "mocap_frame_align": cfg.get("mocap_frame_align", {}),
            "fixture": fixture_info,
            "head_camera_capture": bool(args.capture_head_camera),
            "head_camera_keyframes": len(samples) if args.capture_head_camera else 0,
        },
        "bodies": [
            {
                "name": b["name"],
                "tip_link": b["urdf_link"],
                "link_offset_mm": b["link_offset_mm"],
                "record_only": bool(b.get("record_only", False)),
            }
            for b in bodies_cfg
        ],
        "samples": samples,
    }

    output_path = Path(args.output) if args.output else _SCRIPT_DIR / f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    rospy.loginfo("采集完成，共 %d 样本 -> %s", len(samples), output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
