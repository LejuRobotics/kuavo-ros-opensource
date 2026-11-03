#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实机末端 l_hand / r_hand 动捕精度测试。

流程：双臂同步执行 teach JSON → 每点静止 hold_sec → 采集动捕 + TF →
扣 link_offset 后算 hand 相对 torso 位姿，与 TF 末端在 waist_yaw_link 下平移对比。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

# 复用 mocap_pose_utils 与 both_arms 下发逻辑
_SCRIPT_DIR = Path(__file__).resolve().parent
_CAMERA_CAL_DIR = _SCRIPT_DIR.parent
_MOCAP_DIR = _CAMERA_CAL_DIR / "mocap_checkerboard_pose"
_BOTH_ARMS_DIR = _CAMERA_CAL_DIR / "demos" / "kuavo_both_arms"
for _p in (str(_CAMERA_CAL_DIR), str(_MOCAP_DIR), str(_BOTH_ARMS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mocap_pose_utils import (  # noqa: E402
    align_relative_translation,
    apply_link_offsets_to_poses,
    compute_relative_poses,
    ensure_quat_continuity,
    filter_and_aggregate_poses,
    is_valid_pose,
    load_mocap_frame_align_matrix,
)
from both_arms_table_publisher import (  # noqa: E402
    SensorsArmMonitor,
    _try_call,
    _try_enable_arm_traj_interpolator,
    _try_lb_quick_mode,
    hold_pose,
    load_arm_points_from_teach_json,
    publish_segment,
)
def _default_config_path() -> str:
    return str(_SCRIPT_DIR / "config" / "hand_accuracy.yaml")


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    # teach 路径相对本模块目录
    teach = cfg.get("teach", {})
    for key in ("left_json", "right_json"):
        if key in teach and teach[key]:
            p = Path(teach[key])
            if not p.is_absolute():
                teach[key] = str((_SCRIPT_DIR / p).resolve())

    # 自动从 ROBOT_VERSION 解析布局，提供机器人默认值（YAML 显式值优先）
    _apply_robot_defaults(cfg)

    return cfg


def _resolve_robot_layout_from_env() -> str:
    """ROBOT_VERSION → robot_layout；未设置则用 config 值。"""
    rv = os.environ.get("ROBOT_VERSION", "").strip()
    if rv == "45":
        return "biped45"
    if rv == "52":
        return "biped52"
    if rv == "56":
        return "biped56"
    if rv in ("62", "63"):
        return "wheel62"
    return ""


def _apply_fixture_override(cfg: Dict[str, Any], fixture_id: str, primary_source: str) -> Dict[str, Any]:
    """从当前 YAML 的 fixtures 选择本次测试的左右手偏移。"""
    descriptions = {"legacy": "旧手臂末端工装", "new": "新手臂末端工装"}
    if fixture_id not in descriptions:
        raise ValueError(f"未知手臂工装: {fixture_id}")
    offsets = (cfg.get("fixtures") or {}).get(fixture_id)
    if not isinstance(offsets, dict) or not {"l_hand", "r_hand"} <= set(offsets):
        raise ValueError(f"配置缺少 fixtures.{fixture_id}.l_hand/r_hand")
    for key in ("mocap_bodies", "mocap_bodies_alt"):
        bodies = {body.get("name"): body for body in cfg.get(key, [])}
        if not bodies:
            continue
        for name in ("l_hand", "r_hand"):
            if name not in bodies:
                raise ValueError(f"{key} 缺少 {name}")
            bodies[name]["link_offset_mm"] = list(offsets[name])
    torso_bodies = {body.get("name"): body for body in cfg.get("mocap_bodies", [])}
    torso = torso_bodies.get("torso") or {}
    fixture = {
        "id": fixture_id,
        "description": descriptions[fixture_id],
        "primary_source": primary_source,
        "torso_offset_mm": list(torso.get("link_offset_mm", [])),
    }
    cfg["fixture"] = fixture
    return fixture

ROBOT_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "wheel62": {
        "robot_layout": "wheel62",
        "robot.left_start_index": 4,
        "robot.right_start_index": 11,
        "tf.parent_frame": "waist_yaw_link",
        "robot.set_external_control_mode": True,
        "robot.enable_arm_quick_mode": True,
        "robot.enable_arm_traj_interpolator": True,
    },
    "biped56": {
        "robot_layout": "biped56",
        "robot.left_start_index": 13,
        "robot.right_start_index": 20,
        "tf.parent_frame": "waist_yaw_link",
        "robot.set_external_control_mode": True,
        "robot.enable_arm_quick_mode": True,
        "robot.enable_arm_traj_interpolator": False,
    },
    "biped52": {
        "robot_layout": "biped52",
        "robot.left_start_index": 13,
        "robot.right_start_index": 20,
        "tf.parent_frame": "waist_yaw_link",
        "robot.set_external_control_mode": True,
        "robot.enable_arm_quick_mode": True,
        "robot.enable_arm_traj_interpolator": False,
    },
    "biped45": {
        "robot_layout": "biped45",
        "robot.left_start_index": 12,
        "robot.right_start_index": 19,
        "tf.parent_frame": "base_link",
        "robot.set_external_control_mode": True,
        "robot.enable_arm_quick_mode": True,
        "robot.enable_arm_traj_interpolator": False,
    },
}


def _apply_robot_defaults(cfg: Dict[str, Any]) -> None:
    """按 ROBOT_VERSION 适配机器人；环境变量存在时作为当前实机事实优先。"""
    env_layout = _resolve_robot_layout_from_env()
    yaml_layout = str(cfg.get("robot", {}).get("robot_layout", ""))
    force_robot_defaults = bool(env_layout)
    layout = env_layout
    if not layout and yaml_layout and yaml_layout not in ("auto", ""):
        layout = yaml_layout
    if not layout or layout not in ROBOT_DEFAULTS:
        return

    if env_layout and yaml_layout not in ("", "auto", env_layout):
        rospy.logwarn(
            "[config] YAML robot_layout=%s 与 ROBOT_VERSION 对应布局 %s 不一致，以 ROBOT_VERSION 为准",
            yaml_layout,
            env_layout,
        )

    defaults = ROBOT_DEFAULTS[layout]
    cfg.setdefault("robot", {})
    tf = cfg.setdefault("tf", {})
    if force_robot_defaults or str(cfg["robot"].get("robot_layout", "")).lower() in ("auto", ""):
        cfg["robot"]["robot_layout"] = defaults["robot_layout"]
    for key in (
        "left_start_index",
        "right_start_index",
        "set_external_control_mode",
        "enable_arm_quick_mode",
        "enable_arm_traj_interpolator",
    ):
        if force_robot_defaults or key not in cfg["robot"]:
            cfg["robot"][key] = defaults[f"robot.{key}"]
    for tf_key in ("parent_frame",):
        if force_robot_defaults or tf_key not in tf or str(tf[tf_key]).lower() in ("auto", ""):
            tf[tf_key] = defaults[f"tf.{tf_key}"]

    layout_name = cfg["robot"]["robot_layout"]
    rospy.loginfo(
        "[config] ROBOT_VERSION=%s => robot_layout=%s, parent_frame=%s, indices=[%d,%d]",
        os.environ.get("ROBOT_VERSION", ""),
        layout_name,
        tf.get("parent_frame", ""),
        cfg["robot"]["left_start_index"],
        cfg["robot"]["right_start_index"],
    )


def _resolve_output_paths(cfg: Dict[str, Any]) -> Tuple[str, str]:
    """返回 (json_path, csv_base) — csv_base 不含 .csv 后缀，用于派生多个 CSV。"""
    out = cfg.get("output", {})
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = out.get("json_path") or ""
    csv_path = out.get("csv_path") or ""
    if not json_path:
        json_path = str(_SCRIPT_DIR / f"hand_accuracy_report_{ts}.json")
    elif not Path(json_path).is_absolute():
        json_path = str((_SCRIPT_DIR / json_path).resolve())
    if not csv_path:
        csv_base = str(_SCRIPT_DIR / f"hand_accuracy_report_{ts}")
    elif csv_path.endswith(".csv"):
        csv_base = str(Path(csv_path).with_suffix(""))
    else:
        csv_base = csv_path
    if not Path(csv_base).is_absolute():
        csv_base = str((_SCRIPT_DIR / csv_base).resolve())
    return json_path, csv_base


class MocapTripleSampler:
    """订阅 l_hand / r_hand / torso / l_shoulder，采集同步帧，算相对位姿。"""

    def __init__(self, bodies_cfg: List[Dict[str, Any]]):
        self.body_names = [b["name"] for b in bodies_cfg]
        self._link_offsets = {
            b["name"]: np.asarray(b.get("link_offset_mm", [0, 0, 0]), dtype=float)
            for b in bodies_cfg
        }
        self._has_l_shoulder = "l_shoulder" in self.body_names
        self._cache: Dict[str, Optional[Dict[str, np.ndarray]]] = {
            n: None for n in self.body_names
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
        return all(self._cache[n] is not None for n in self.body_names)

    def _frame_relative_poses(self) -> Optional[Dict[str, Dict[str, np.ndarray]]]:
        """单帧：扣工装偏移后算相对位姿，同时保留原始工装刚体位姿（mm）。"""
        if not self._snapshot_ready():
            return None
        raw = {n: self._cache[n].copy() for n in self.body_names}  # type: ignore
        corrected = apply_link_offsets_to_poses(raw, self._link_offsets)
        torso = corrected["torso"]
        result: Dict[str, Dict[str, np.ndarray]] = {}
        # 原始工装刚体位姿（Motive 系下 mm，未扣 link_offset）
        result["raw_tooling"] = {
            n: {"pos": raw[n]["pos"].copy(), "quat": raw[n]["quat"].copy()}
            for n in self.body_names
        }
        # torso-relative (always)
        l_pos, l_quat = compute_relative_poses(
            corrected["l_hand"]["pos"], corrected["l_hand"]["quat"],
            torso["pos"], torso["quat"],
        )
        r_pos, r_quat = compute_relative_poses(
            corrected["r_hand"]["pos"], corrected["r_hand"]["quat"],
            torso["pos"], torso["quat"],
        )
        result["l_hand_in_torso"] = {"pos": l_pos, "quat": l_quat}
        result["r_hand_in_torso"] = {"pos": r_pos, "quat": r_quat}
        # l_shoulder-relative (if available)
        if self._has_l_shoulder:
            lsh = corrected["l_shoulder"]
            ls_pos, ls_quat = compute_relative_poses(
                corrected["l_hand"]["pos"], corrected["l_hand"]["quat"],
                lsh["pos"], lsh["quat"],
            )
            rs_pos, rs_quat = compute_relative_poses(
                corrected["r_hand"]["pos"], corrected["r_hand"]["quat"],
                lsh["pos"], lsh["quat"],
            )
            result["l_hand_in_l_shoulder"] = {"pos": ls_pos, "quat": ls_quat}
            result["r_hand_in_l_shoulder"] = {"pos": rs_pos, "quat": rs_quat}
        return result

    def collect_window(self, duration_sec: float) -> List[Dict[str, Dict[str, np.ndarray]]]:
        """在 duration_sec 内以 ~100Hz 采样有效相对位姿帧。"""
        frames: List[Dict[str, Dict[str, np.ndarray]]] = []
        t_end = time.time() + max(0.0, duration_sec)
        rate = rospy.Rate(100)
        while time.time() < t_end and not rospy.is_shutdown():
            rel = self._frame_relative_poses()
            if rel is not None:
                frames.append(rel)
            rate.sleep()
        return frames


def aggregate_relative_side(
    frames: List[Dict[str, Dict[str, np.ndarray]]],
    side_key: str,
    sigma: float,
) -> Dict[str, Any]:
    """对一侧 hand_in_torso 多帧做 3σ 聚合。"""
    if not frames:
        raise ValueError(f"无有效动捕帧: {side_key}")
    pts = np.array([f[side_key]["pos"] for f in frames], dtype=float)
    quats = np.array([f[side_key]["quat"] for f in frames], dtype=float)
    return filter_and_aggregate_poses(pts, quats, sigma=sigma)


def lookup_tf_translation_m(
    buffer: tf2_ros.Buffer,
    parent: str,
    child: str,
    timeout_sec: float,
) -> np.ndarray:
    """查询 parent→child 平移（米）。"""
    trans = buffer.lookup_transform(
        parent,
        child,
        rospy.Time(0),
        rospy.Duration(timeout_sec),
    )
    t = trans.transform.translation
    return np.array([t.x, t.y, t.z], dtype=float)


def compute_error_mm(p_mocap_m: np.ndarray, p_tf_m: np.ndarray) -> Dict[str, float]:
    """平移误差：分轴与范数（mm）。"""
    diff_m = p_mocap_m - p_tf_m
    diff_mm = diff_m * 1000.0
    return {
        "dx_mm": float(diff_mm[0]),
        "dy_mm": float(diff_mm[1]),
        "dz_mm": float(diff_mm[2]),
        "norm_mm": float(np.linalg.norm(diff_mm)),
    }


def _aggregate_raw_tooling_positions(
    frames: List[Dict[str, Any]],
    body_name: str,
) -> Optional[Dict[str, Any]]:
    """聚合某个刚体的原始工装位置（Motive 系 mm），返回均值+std。"""
    pts = []
    for f in frames:
        raw = f.get("raw_tooling", {})
        if body_name in raw:
            pts.append(raw[body_name]["pos"])
    if not pts:
        return None
    arr = np.array(pts, dtype=float)
    mean_xyz = np.mean(arr, axis=0)
    std_xyz = np.std(arr, axis=0)
    return {
        "x_mm": float(mean_xyz[0]),
        "y_mm": float(mean_xyz[1]),
        "z_mm": float(mean_xyz[2]),
        "std_x_mm": float(std_xyz[0]),
        "std_y_mm": float(std_xyz[1]),
        "std_z_mm": float(std_xyz[2]),
        "frames": len(pts),
    }


def _stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"mean": 0.0, "max": 0.0, "std": 0.0}
    arr = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(arr)),
        "max": float(np.max(arr)),
        "std": float(np.std(arr)),
    }


def setup_robot_control(cfg: Dict[str, Any], monitor: SensorsArmMonitor) -> Tuple:
    """切换外控 / quick mode / 插补，返回 (arm_pub, current_left, current_right, dt)。"""
    robot = cfg["robot"]
    dt = float(robot.get("dt", 0.01))
    is_wheel62 = str(robot.get("robot_layout", "wheel62")) == "wheel62"
    pre_mode_hold = float(robot.get("pre_mode_hold_sec", 0.6))
    post_mode_hold = float(robot.get("post_mode_hold_sec", 0.6))

    cur = monitor.get_current_left_right_deg()
    current_left = cur["left_deg"]
    current_right = cur["right_deg"]

    arm_pub = rospy.Publisher(
        robot.get("arm_traj_topic", "/kuavo_arm_traj"),
        JointState,
        queue_size=10,
        tcp_nodelay=True,
    )

    if pre_mode_hold > 0:
        hold_pose(arm_pub, current_left, current_right, pre_mode_hold, dt)

    if robot.get("set_external_control_mode", True):
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

    if robot.get("enable_arm_quick_mode", True):
        if is_wheel62:
            if not _try_lb_quick_mode("/enable_lb_arm_quick_mode", 2):
                rospy.logwarn("未能使能 /enable_lb_arm_quick_mode")
        else:
            if not _try_call("/enable_wbc_arm_trajectory_control", 1):
                rospy.logwarn("未能使能 /enable_wbc_arm_trajectory_control")

    if is_wheel62 and robot.get("enable_arm_traj_interpolator", True):
        if not _try_enable_arm_traj_interpolator(True):
            rospy.logwarn("未能开启 /enable_arm_traj_interpolator")

    rospy.sleep(0.2)
    if post_mode_hold > 0:
        hold_pose(arm_pub, current_left, current_right, post_mode_hold, dt)

    return arm_pub, current_left, current_right, dt


def print_summary(report: Dict[str, Any], warn_mm: float) -> None:
    """终端摘要表。"""
    header = f"{'点':<6} {'waist左':>10} {'waist右':>10}"
    sep = "-" * (6 + 20)
    header += f" {'状态':>8}"
    sep = "-" * (6 + 20)

    print("\n" + "=" * 72)
    print("  末端动捕精度测试摘要")
    print("=" * 72)
    print(header)
    print(sep)
    for pt in report["waypoints"]:
        ew_l = pt["error_left"]["norm_mm"]
        ew_r = pt["error_right"]["norm_mm"]
        worst = ew_l if ew_l > ew_r else ew_r
        status = "WARN" if worst > warn_mm else "OK"
        print(f"{pt['name']:<6} {ew_l:>10.2f} {ew_r:>10.2f} {status:>8}")
    print(sep)
    s = report["summary"]
    print(
        f"左臂(waist): mean={s['left_norm_mm']['mean']:.3f} max={s['left_norm_mm']['max']:.3f} "
        f"std={s['left_norm_mm']['std']:.3f} mm"
    )
    print(
        f"右臂(waist): mean={s['right_norm_mm']['mean']:.3f} max={s['right_norm_mm']['max']:.3f} "
        f"std={s['right_norm_mm']['std']:.3f} mm"
    )
    if s.get("alt_diff_left_mm"):
        adl = s["alt_diff_left_mm"]
        adr = s["alt_diff_right_mm"]
        print(f"两套动捕差(左): mean={adl['mean']:.3f} max={adl['max']:.3f} mm")
        print(f"两套动捕差(右): mean={adr['mean']:.3f} max={adr['max']:.3f} mm")
    print(f"告警阈值: > {warn_mm:.1f} mm")
    print("=" * 72 + "\n")


def write_csv_reports(csv_base: str, report: Dict[str, Any]) -> List[str]:
    """生成 2 个 CSV：raw_tooling（原始工装位姿）、waist 系对比。"""
    paths: List[str] = []
    wpts = report["waypoints"]
    if not wpts:
        return paths

    # 收集原始 tooling 刚体名
    raw_bodies = sorted(wpts[0].get("raw_tooling_mm", {}).keys())

    # ---- 1) raw_tooling.csv ----
    raw_path = csv_base + "_raw_tooling.csv"
    raw_fields = ["waypoint", "index"]
    for bn in raw_bodies:
        raw_fields.extend([f"{bn}_x_mm", f"{bn}_y_mm", f"{bn}_z_mm"])
    raw_fields.append("mocap_frames")
    with open(raw_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=raw_fields)
        w.writeheader()
        for pt in wpts:
            row: Dict[str, Any] = {"waypoint": pt["name"], "index": pt["index"],
                                   "mocap_frames": pt.get("mocap_raw_frames", 0)}
            rt = pt.get("raw_tooling_mm", {})
            for bn in raw_bodies:
                d = rt.get(bn) or {}
                row[f"{bn}_x_mm"] = d.get("x_mm")
                row[f"{bn}_y_mm"] = d.get("y_mm")
                row[f"{bn}_z_mm"] = d.get("z_mm")
            w.writerow(row)
    paths.append(raw_path)

    # ---- 2) _waist.csv (waist系: 动捕 vs FK vs 误差) ----
    waist_path = csv_base + "_waist.csv"
    waist_fields = [
        "waypoint", "index",
        "mocap_l_x_m", "mocap_l_y_m", "mocap_l_z_m",
        "fk_waist_l_x_m", "fk_waist_l_y_m", "fk_waist_l_z_m",
        "err_l_dx_mm", "err_l_dy_mm", "err_l_dz_mm", "err_l_norm_mm",
        "mocap_r_x_m", "mocap_r_y_m", "mocap_r_z_m",
        "fk_waist_r_x_m", "fk_waist_r_y_m", "fk_waist_r_z_m",
        "err_r_dx_mm", "err_r_dy_mm", "err_r_dz_mm", "err_r_norm_mm",
        "alt_mocap_l_x_m", "alt_mocap_l_y_m", "alt_mocap_l_z_m",
        "alt_err_l_norm_mm", "alt_diff_l_mm",
        "alt_mocap_r_x_m", "alt_mocap_r_y_m", "alt_mocap_r_z_m",
        "alt_err_r_norm_mm", "alt_diff_r_mm",
        "mocap_frames",
    ]
    with open(waist_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=waist_fields)
        w.writeheader()
        for pt in wpts:
            ml = pt["mocap_left_in_torso_m"]
            mr = pt["mocap_right_in_torso_m"]
            flw = pt["tf_left_in_waist_m"]
            frw = pt["tf_right_in_waist_m"]
            ew_l = pt["error_left"]
            ew_r = pt["error_right"]
            # 第二套动捕（可选）
            alt_l = pt.get("alt_left_in_torso_m") or []
            alt_r = pt.get("alt_right_in_torso_m") or []
            ael = pt.get("alt_err_left") or {}
            aer = pt.get("alt_err_right") or {}
            w.writerow({
                "waypoint": pt["name"], "index": pt["index"],
                "mocap_l_x_m": ml[0], "mocap_l_y_m": ml[1], "mocap_l_z_m": ml[2],
                "fk_waist_l_x_m": flw[0], "fk_waist_l_y_m": flw[1], "fk_waist_l_z_m": flw[2],
                "err_l_dx_mm": ew_l["dx_mm"], "err_l_dy_mm": ew_l["dy_mm"],
                "err_l_dz_mm": ew_l["dz_mm"], "err_l_norm_mm": ew_l["norm_mm"],
                "mocap_r_x_m": mr[0], "mocap_r_y_m": mr[1], "mocap_r_z_m": mr[2],
                "fk_waist_r_x_m": frw[0], "fk_waist_r_y_m": frw[1], "fk_waist_r_z_m": frw[2],
                "err_r_dx_mm": ew_r["dx_mm"], "err_r_dy_mm": ew_r["dy_mm"],
                "err_r_dz_mm": ew_r["dz_mm"], "err_r_norm_mm": ew_r["norm_mm"],
                "alt_mocap_l_x_m": alt_l[0] if alt_l else "", "alt_mocap_l_y_m": alt_l[1] if alt_l else "", "alt_mocap_l_z_m": alt_l[2] if alt_l else "",
                "alt_err_l_norm_mm": ael.get("norm_mm", ""), "alt_diff_l_mm": pt.get("alt_diff_left_mm", ""),
                "alt_mocap_r_x_m": alt_r[0] if alt_r else "", "alt_mocap_r_y_m": alt_r[1] if alt_r else "", "alt_mocap_r_z_m": alt_r[2] if alt_r else "",
                "alt_err_r_norm_mm": aer.get("norm_mm", ""), "alt_diff_r_mm": pt.get("alt_diff_right_mm", ""),
                "mocap_frames": pt.get("mocap_raw_frames", 0),
            })
    paths.append(waist_path)

    return paths


def run_test(cfg: Dict[str, Any], mocap_mode: str = "both") -> int:
    robot = cfg["robot"]
    teach = cfg["teach"]
    tf_cfg = cfg["tf"]
    proc = cfg.get("processing", {})
    sigma = float(proc.get("sigma", 3.0))
    warn_mm = float(proc.get("warn_threshold_mm", 5.0))
    skip_first_last = bool(teach.get("skip_first_last", False))
    debug_cfg = cfg.get("debug", {})
    print_fk = bool(debug_cfg.get("print_fk_positions", True))

    # ---- 按动捕模式解析主/备数据源 ----
    # both: Motive(主) + 青瞳(备, mocap_bodies_alt) 交叉验证
    # motive: 仅 Motive；qingtong: 仅青瞳（以 mocap_bodies_alt 为主源）
    alt_bodies_cfg = cfg.get("mocap_bodies_alt")
    primary_bodies = cfg["mocap_bodies"]
    alt_bodies: Optional[List[Dict[str, Any]]] = None
    if mocap_mode == "qingtong":
        primary_bodies = alt_bodies_cfg or cfg["mocap_bodies"]
        R_mocap_align = load_mocap_frame_align_matrix(cfg.get("mocap_bodies_alt_align"))
        rospy.loginfo("[mocap-mode] 仅青瞳：主源用 mocap_bodies_alt（align=mocap_bodies_alt_align）")
    else:
        R_mocap_align = load_mocap_frame_align_matrix(cfg.get("mocap_frame_align"))
        if mocap_mode == "motive":
            rospy.loginfo("[mocap-mode] 仅 Motive")
        else:
            alt_bodies = alt_bodies_cfg
            rospy.loginfo("[mocap-mode] 全部：Motive(主) + 青瞳(备) 交叉验证")
    if not np.allclose(R_mocap_align, np.eye(3)):
        rospy.loginfo(
            "[align] 已启用主源 mocap_frame_align，hand_in_torso 平移将变到 URDF parent 系后再对比 TF"
        )

    teach_left = Path(teach["left_json"])
    teach_right = Path(teach["right_json"])
    if not teach_left.is_file():
        raise FileNotFoundError(f"找不到 teach left: {teach_left}")
    if not teach_right.is_file():
        raise FileNotFoundError(f"找不到 teach right: {teach_right}")

    left_points = load_arm_points_from_teach_json(teach_left, "left_arm_joints")
    right_points = load_arm_points_from_teach_json(teach_right, "right_arm_joints")
    n_pts = min(len(left_points), len(right_points))
    left_points = left_points[:n_pts]
    right_points = right_points[:n_pts]
    rospy.loginfo(
        "加载 teach: left=%d right=%d 配对=%d",
        len(left_points),
        len(right_points),
        n_pts,
    )

    monitor = SensorsArmMonitor(
        topic=robot.get("sensor_topic", "/sensors_data_raw"),
        left_start_index=int(robot.get("left_start_index", 4)),
        right_start_index=int(robot.get("right_start_index", 11)),
    )
    monitor.wait_until_valid_arms(timeout=float(robot.get("wait_sensors_timeout", 30.0)))

    arm_pub, current_left, current_right, dt = setup_robot_control(cfg, monitor)

    mocap_sampler = MocapTripleSampler(primary_bodies)
    # 第二套动捕（可选，交叉验证）：mocap_bodies_alt + mocap_bodies_alt_align
    alt_sampler: Optional[MocapTripleSampler] = None
    R_alt_align: Optional[np.ndarray] = None
    if alt_bodies:
        alt_sampler = MocapTripleSampler(alt_bodies)
        R_alt_align = load_mocap_frame_align_matrix(cfg.get("mocap_bodies_alt_align"))
        rospy.loginfo("[mocap-alt] 已启用第二套动捕交叉验证（%d 刚体）", len(alt_bodies))
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    move_duration = float(robot.get("move_duration", 2.0))
    hold_sec = float(robot.get("hold_sec", 5.0))
    mocap_collect_sec = float(robot.get("mocap_collect_sec", 1.0))
    startup_align = float(robot.get("startup_align_sec", 2.0))
    return_zero = float(robot.get("return_to_zero_sec", 2.0))
    tf_parent = tf_cfg["parent_frame"]
    tf_left = tf_cfg["left_child_frame"]
    tf_right = tf_cfg["right_child_frame"]
    tf_timeout = float(tf_cfg.get("lookup_timeout_sec", 2.0))

    # 对齐首点
    first_l = left_points[0]["deg"]
    first_r = right_points[0]["deg"]
    publish_segment(arm_pub, current_left, current_right, first_l, first_r, startup_align, dt)
    current_left, current_right = list(first_l), list(first_r)

    waypoint_results: List[Dict[str, Any]] = []

    for i in range(n_pts):
        if rospy.is_shutdown():
            break

        # 非首点：从上一姿态插值到当前点
        if i > 0:
            tgt_l = left_points[i]["deg"]
            tgt_r = right_points[i]["deg"]
            publish_segment(
                arm_pub, current_left, current_right, tgt_l, tgt_r, move_duration, dt
            )
            current_left, current_right = list(tgt_l), list(tgt_r)

        name = left_points[i]["name"]
        k = i + 1
        n_total = n_pts
        if skip_first_last and n_total > 2 and (k == 1 or k == n_total):
            rospy.loginfo("跳过首尾点采集: %s (index=%d)", name, i)
            hold_pose(arm_pub, current_left, current_right, hold_sec, dt)
            continue

        rospy.loginfo("关键点 %s: 静止 %.1fs 后采集动捕+TF", name, hold_sec)
        # 静止保持姿态
        hold_pose(arm_pub, current_left, current_right, hold_sec, dt)

        # 动捕窗口采集
        frames = mocap_sampler.collect_window(mocap_collect_sec)
        if len(frames) < 3:
            rospy.logwarn("点 %s 动捕有效帧过少 (%d)，跳过误差计算", name, len(frames))
            continue

        agg_l = aggregate_relative_side(frames, "l_hand_in_torso", sigma)
        agg_r = aggregate_relative_side(frames, "r_hand_in_torso", sigma)
        p_mocap_l_m = align_relative_translation(agg_l["xyz_mm"] / 1000.0, R_mocap_align)
        p_mocap_r_m = align_relative_translation(agg_r["xyz_mm"] / 1000.0, R_mocap_align)

        # 原始工装刚体位姿（Motive 系 mm，未扣 link_offset）
        raw_tooling: Dict[str, Any] = {}
        for body_name in mocap_sampler.body_names:
            agg_raw = _aggregate_raw_tooling_positions(frames, body_name)
            if agg_raw is not None:
                raw_tooling[body_name] = agg_raw

        # TF 末端在 waist_yaw_link 下平移
        try:
            p_tf_l_m = lookup_tf_translation_m(tf_buffer, tf_parent, tf_left, tf_timeout)
            p_tf_r_m = lookup_tf_translation_m(tf_buffer, tf_parent, tf_right, tf_timeout)
            # 不再查询左肩参考系 TF（s45 无 l_shoulder 刚体 / 左肩参考系未对齐 URDF）
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("点 %s TF 查询失败: %s", name, e)
            continue

        err_l = compute_error_mm(p_mocap_l_m, p_tf_l_m)
        err_r = compute_error_mm(p_mocap_r_m, p_tf_r_m)

        # ---- 第二套动捕交叉验证（可选）----
        alt_data: Dict[str, Any] = {}
        if alt_sampler is not None and R_alt_align is not None:
            alt_frames = alt_sampler.collect_window(mocap_collect_sec)
            if len(alt_frames) >= 3:
                a_l = aggregate_relative_side(alt_frames, "l_hand_in_torso", sigma)
                a_r = aggregate_relative_side(alt_frames, "r_hand_in_torso", sigma)
                p_alt_l_m = align_relative_translation(a_l["xyz_mm"] / 1000.0, R_alt_align)
                p_alt_r_m = align_relative_translation(a_r["xyz_mm"] / 1000.0, R_alt_align)
                alt_err_l = compute_error_mm(p_alt_l_m, p_tf_l_m)
                alt_err_r = compute_error_mm(p_alt_r_m, p_tf_r_m)
                diff_l_mm = (p_mocap_l_m - p_alt_l_m) * 1000.0
                diff_r_mm = (p_mocap_r_m - p_alt_r_m) * 1000.0
                alt_data = {
                    "alt_left_in_torso_m": p_alt_l_m.tolist(),
                    "alt_right_in_torso_m": p_alt_r_m.tolist(),
                    "alt_err_left": alt_err_l,
                    "alt_err_right": alt_err_r,
                    "alt_diff_left_mm": float(np.linalg.norm(diff_l_mm)),
                    "alt_diff_right_mm": float(np.linalg.norm(diff_r_mm)),
                    "alt_valid_frames_left": a_l["valid_frames"],
                    "alt_valid_frames_right": a_r["valid_frames"],
                }
            else:
                rospy.logwarn("点 %s 第二套动捕有效帧过少 (%d)，跳过交叉验证", name, len(alt_frames))

        waypoint_results.append(
            {
                "name": name,
                "index": i,
                "mocap_left_in_torso_m": p_mocap_l_m.tolist(),
                "mocap_right_in_torso_m": p_mocap_r_m.tolist(),
                "tf_left_in_waist_m": p_tf_l_m.tolist(),   # 旧 key，保持兼容
                "tf_right_in_waist_m": p_tf_r_m.tolist(),
                "error_left": err_l,
                "error_right": err_r,
                "raw_tooling_mm": raw_tooling,   # 原始工装刚体位姿（Motive系mm，未扣offset）
                "mocap_raw_frames": len(frames),
                "mocap_valid_frames_left": agg_l["valid_frames"],
                "mocap_valid_frames_right": agg_r["valid_frames"],
                **alt_data,
            }
        )
        if print_fk:
            # ---- waist 系：动捕 → FK → 误差 ----
            rospy.loginfo(
                "  点 %s 动捕(waist_yaw系):    左=[%+.4f %+.4f %+.4f] m  右=[%+.4f %+.4f %+.4f] m",
                name,
                p_mocap_l_m[0], p_mocap_l_m[1], p_mocap_l_m[2],
                p_mocap_r_m[0], p_mocap_r_m[1], p_mocap_r_m[2],
            )
            rospy.loginfo(
                "  点 %s FK  (waist_yaw系):    左=[%+.4f %+.4f %+.4f] m  右=[%+.4f %+.4f %+.4f] m",
                name,
                p_tf_l_m[0], p_tf_l_m[1], p_tf_l_m[2],
                p_tf_r_m[0], p_tf_r_m[1], p_tf_r_m[2],
            )
            rospy.loginfo(
                "  点 %s 误差(waist_yaw系):    左=%.2f mm  右=%.2f mm",
                name, err_l["norm_mm"], err_r["norm_mm"],
            )
            if alt_data:
                rospy.loginfo(
                    "  点 %s 两套动捕差:       左=%.2f mm  右=%.2f mm  | 第二套误差: 左=%.2f 右=%.2f mm",
                    name,
                    alt_data["alt_diff_left_mm"], alt_data["alt_diff_right_mm"],
                    alt_data["alt_err_left"]["norm_mm"], alt_data["alt_err_right"]["norm_mm"],
                )

    # 回零
    zero = [0.0] * 7
    publish_segment(arm_pub, current_left, current_right, zero, zero, return_zero, dt)

    left_waist_norms = [p["error_left"]["norm_mm"] for p in waypoint_results]
    right_waist_norms = [p["error_right"]["norm_mm"] for p in waypoint_results]

    json_path, csv_base = _resolve_output_paths(cfg)
    summary: Dict[str, Any] = {
        "left_norm_mm": _stats(left_waist_norms),     # 旧 key 保持兼容
        "right_norm_mm": _stats(right_waist_norms),
    }
    # 两套动捕交叉验证差值统计（可选）
    alt_diff_l = [p["alt_diff_left_mm"] for p in waypoint_results if "alt_diff_left_mm" in p]
    alt_diff_r = [p["alt_diff_right_mm"] for p in waypoint_results if "alt_diff_right_mm" in p]
    summary["alt_diff_left_mm"] = _stats(alt_diff_l) if alt_diff_l else None
    summary["alt_diff_right_mm"] = _stats(alt_diff_r) if alt_diff_r else None
    report: Dict[str, Any] = {
        "meta": {
            "timestamp": datetime.now().isoformat(),
            "config": cfg,
            "teach_left": str(teach_left),
            "teach_right": str(teach_right),
            "waypoint_count": n_pts,
            "measured_count": len(waypoint_results),
            "tf_parent": tf_parent,
            "tf_left_child": tf_left,
            "tf_right_child": tf_right,
            "warn_threshold_mm": warn_mm,
        },
        "waypoints": waypoint_results,
        "summary": summary,
        "output": {"json": json_path, "csvs": csv_base},
    }

    os.makedirs(os.path.dirname(json_path) or ".", exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    csv_paths = write_csv_reports(csv_base, report)
    print_summary(report, warn_mm)
    rospy.loginfo("报告已写入:\n  JSON: %s", json_path)
    for p in csv_paths:
        rospy.loginfo("  CSV : %s", p)
    return 0 if waypoint_results else 1


def _prompt_mocap_mode() -> str:
    """交互式选择动捕模式：默认全部（Motive+青瞳），可单选。"""
    print("\n=== 动捕模式选择 ===")
    print("  [1] 全部（Motive + 青瞳交叉验证）   <-- 默认")
    print("  [2] 仅 Motive")
    print("  [3] 仅 青瞳")
    try:
        choice = input("选择 [1/2/3，回车=1]: ").strip().lower()
    except EOFError:  # 非交互环境
        choice = ""
    if choice in ("2", "m", "motive"):
        return "motive"
    if choice in ("3", "q", "qingtong"):
        return "qingtong"
    return "both"


def main() -> int:
    parser = argparse.ArgumentParser(description="实机末端 l_hand/r_hand 动捕精度测试")
    parser.add_argument("--config", default=_default_config_path(), help="hand_accuracy.yaml")
    parser.add_argument(
        "--mocap-mode",
        choices=("", "both", "motive", "qingtong"),
        default="",
        help="动捕模式：both(默认,交叉验证)|motive|qingtong；留空则启动时交互选择",
    )
    parser.add_argument(
        "--fixture",
        choices=("legacy", "new"),
        default=os.environ.get("MOCAP_FIXTURE", "legacy"),
        help="手臂末端工装（默认读 MOCAP_FIXTURE，否则 legacy）",
    )
    args = parser.parse_args()

    rospy.init_node("hand_accuracy_test", anonymous=False)
    cfg = load_config(args.config)
    mocap_mode = args.mocap_mode or _prompt_mocap_mode()
    primary_source = "qingtong" if mocap_mode == "qingtong" else "motive"
    fixture = _apply_fixture_override(cfg, args.fixture, primary_source)
    fixture["selected_mocap_mode"] = mocap_mode
    rospy.loginfo(
        "手臂工装: %s (%s), mocap=%s, torso_offset_mm=%s",
        fixture["id"], fixture["description"], mocap_mode,
        fixture["torso_offset_mm"],
    )
    try:
        return run_test(cfg, mocap_mode)
    except Exception as e:
        rospy.logfatal("测试失败: %s", e)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
