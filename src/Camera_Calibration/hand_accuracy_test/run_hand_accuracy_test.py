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
for _p in (str(_MOCAP_DIR), str(_BOTH_ARMS_DIR)):
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
    return cfg


def _resolve_output_paths(cfg: Dict[str, Any]) -> Tuple[str, str]:
    out = cfg.get("output", {})
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = out.get("json_path") or ""
    csv_path = out.get("csv_path") or ""
    if not json_path:
        json_path = str(_SCRIPT_DIR / f"hand_accuracy_report_{ts}.json")
    elif not Path(json_path).is_absolute():
        json_path = str((_SCRIPT_DIR / json_path).resolve())
    if not csv_path:
        csv_path = str(_SCRIPT_DIR / f"hand_accuracy_report_{ts}.csv")
    elif not Path(csv_path).is_absolute():
        csv_path = str((_SCRIPT_DIR / csv_path).resolve())
    return json_path, csv_path


class MocapTripleSampler:
    """订阅 l_hand / r_hand / torso，在指定时长内采集同步帧。"""

    def __init__(self, bodies_cfg: List[Dict[str, Any]]):
        self.body_names = [b["name"] for b in bodies_cfg]
        self._link_offsets = {
            b["name"]: np.asarray(b.get("link_offset_mm", [0, 0, 0]), dtype=float)
            for b in bodies_cfg
        }
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
        """单帧：扣工装偏移后算 hand 相对 torso 位姿（mm）。"""
        if not self._snapshot_ready():
            return None
        raw = {n: self._cache[n].copy() for n in self.body_names}  # type: ignore
        corrected = apply_link_offsets_to_poses(raw, self._link_offsets)
        torso = corrected["torso"]
        l_pos, l_quat = compute_relative_poses(
            corrected["l_hand"]["pos"],
            corrected["l_hand"]["quat"],
            torso["pos"],
            torso["quat"],
        )
        r_pos, r_quat = compute_relative_poses(
            corrected["r_hand"]["pos"],
            corrected["r_hand"]["quat"],
            torso["pos"],
            torso["quat"],
        )
        return {
            "l_hand_in_torso": {"pos": l_pos, "quat": l_quat},
            "r_hand_in_torso": {"pos": r_pos, "quat": r_quat},
        }

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
    print("\n" + "=" * 72)
    print("  末端动捕精度测试摘要")
    print("=" * 72)
    print(f"{'点':<6} {'左范数(mm)':>12} {'右范数(mm)':>12} {'状态':>8}")
    print("-" * 72)
    for pt in report["waypoints"]:
        el = pt["error_left"]["norm_mm"]
        er = pt["error_right"]["norm_mm"]
        status = "WARN" if (el > warn_mm or er > warn_mm) else "OK"
        print(f"{pt['name']:<6} {el:>12.3f} {er:>12.3f} {status:>8}")
    print("-" * 72)
    s = report["summary"]
    print(
        f"左臂: mean={s['left_norm_mm']['mean']:.3f} max={s['left_norm_mm']['max']:.3f} "
        f"std={s['left_norm_mm']['std']:.3f} mm"
    )
    print(
        f"右臂: mean={s['right_norm_mm']['mean']:.3f} max={s['right_norm_mm']['max']:.3f} "
        f"std={s['right_norm_mm']['std']:.3f} mm"
    )
    print(f"告警阈值: > {warn_mm:.1f} mm")
    print("=" * 72 + "\n")


def write_csv_report(path: str, report: Dict[str, Any]) -> None:
    """每关键点一行 CSV。"""
    fields = [
        "waypoint",
        "index",
        "mocap_l_x_m",
        "mocap_l_y_m",
        "mocap_l_z_m",
        "tf_l_x_m",
        "tf_l_y_m",
        "tf_l_z_m",
        "err_l_dx_mm",
        "err_l_dy_mm",
        "err_l_dz_mm",
        "err_l_norm_mm",
        "mocap_r_x_m",
        "mocap_r_y_m",
        "mocap_r_z_m",
        "tf_r_x_m",
        "tf_r_y_m",
        "tf_r_z_m",
        "err_r_dx_mm",
        "err_r_dy_mm",
        "err_r_dz_mm",
        "err_r_norm_mm",
        "mocap_frames",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for pt in report["waypoints"]:
            ml = pt["mocap_left_in_torso_m"]
            mr = pt["mocap_right_in_torso_m"]
            tl = pt["tf_left_in_waist_m"]
            tr = pt["tf_right_in_waist_m"]
            el = pt["error_left"]
            er = pt["error_right"]
            w.writerow(
                {
                    "waypoint": pt["name"],
                    "index": pt["index"],
                    "mocap_l_x_m": ml[0],
                    "mocap_l_y_m": ml[1],
                    "mocap_l_z_m": ml[2],
                    "tf_l_x_m": tl[0],
                    "tf_l_y_m": tl[1],
                    "tf_l_z_m": tl[2],
                    "err_l_dx_mm": el["dx_mm"],
                    "err_l_dy_mm": el["dy_mm"],
                    "err_l_dz_mm": el["dz_mm"],
                    "err_l_norm_mm": el["norm_mm"],
                    "mocap_r_x_m": mr[0],
                    "mocap_r_y_m": mr[1],
                    "mocap_r_z_m": mr[2],
                    "tf_r_x_m": tr[0],
                    "tf_r_y_m": tr[1],
                    "tf_r_z_m": tr[2],
                    "err_r_dx_mm": er["dx_mm"],
                    "err_r_dy_mm": er["dy_mm"],
                    "err_r_dz_mm": er["dz_mm"],
                    "err_r_norm_mm": er["norm_mm"],
                    "mocap_frames": pt.get("mocap_raw_frames", 0),
                }
            )


def run_test(cfg: Dict[str, Any]) -> int:
    robot = cfg["robot"]
    teach = cfg["teach"]
    tf_cfg = cfg["tf"]
    proc = cfg.get("processing", {})
    sigma = float(proc.get("sigma", 3.0))
    warn_mm = float(proc.get("warn_threshold_mm", 5.0))
    skip_first_last = bool(teach.get("skip_first_last", False))
    R_mocap_align = load_mocap_frame_align_matrix(cfg.get("mocap_frame_align"))
    if not np.allclose(R_mocap_align, np.eye(3)):
        rospy.loginfo(
            "[align] 已启用 mocap_frame_align，hand_in_torso 平移将变到 URDF parent 系后再对比 TF"
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

    mocap_sampler = MocapTripleSampler(cfg["mocap_bodies"])
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

        # TF 末端在 waist_yaw_link 下平移
        try:
            p_tf_l_m = lookup_tf_translation_m(tf_buffer, tf_parent, tf_left, tf_timeout)
            p_tf_r_m = lookup_tf_translation_m(tf_buffer, tf_parent, tf_right, tf_timeout)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("点 %s TF 查询失败: %s", name, e)
            continue

        err_l = compute_error_mm(p_mocap_l_m, p_tf_l_m)
        err_r = compute_error_mm(p_mocap_r_m, p_tf_r_m)

        waypoint_results.append(
            {
                "name": name,
                "index": i,
                "mocap_left_in_torso_m": p_mocap_l_m.tolist(),
                "mocap_right_in_torso_m": p_mocap_r_m.tolist(),
                "tf_left_in_waist_m": p_tf_l_m.tolist(),
                "tf_right_in_waist_m": p_tf_r_m.tolist(),
                "error_left": err_l,
                "error_right": err_r,
                "mocap_raw_frames": len(frames),
                "mocap_valid_frames_left": agg_l["valid_frames"],
                "mocap_valid_frames_right": agg_r["valid_frames"],
            }
        )
        rospy.loginfo(
            "点 %s 误差: 左=%.2f mm 右=%.2f mm",
            name,
            err_l["norm_mm"],
            err_r["norm_mm"],
        )

    # 回零
    zero = [0.0] * 7
    publish_segment(arm_pub, current_left, current_right, zero, zero, return_zero, dt)

    left_norms = [p["error_left"]["norm_mm"] for p in waypoint_results]
    right_norms = [p["error_right"]["norm_mm"] for p in waypoint_results]

    json_path, csv_path = _resolve_output_paths(cfg)
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
        "summary": {
            "left_norm_mm": _stats(left_norms),
            "right_norm_mm": _stats(right_norms),
        },
        "output": {"json": json_path, "csv": csv_path},
    }

    os.makedirs(os.path.dirname(json_path) or ".", exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    write_csv_report(csv_path, report)
    print_summary(report, warn_mm)
    rospy.loginfo("报告已写入:\n  JSON: %s\n  CSV:  %s", json_path, csv_path)
    return 0 if waypoint_results else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="实机末端 l_hand/r_hand 动捕精度测试")
    parser.add_argument("--config", default=_default_config_path(), help="hand_accuracy.yaml")
    args = parser.parse_args()

    rospy.init_node("hand_accuracy_test", anonymous=False)
    cfg = load_config(args.config)
    try:
        return run_test(cfg)
    except Exception as e:
        rospy.logfatal("测试失败: %s", e)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
