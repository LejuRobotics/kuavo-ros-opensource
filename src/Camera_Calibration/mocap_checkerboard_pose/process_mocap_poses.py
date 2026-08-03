#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线处理 Motive 动捕 CSV：计算 checkerboard 相对 l_shoulder / torso 位姿，3σ 滤波后输出 JSON。
"""

from __future__ import print_function

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from typing import Any, Dict, List, Tuple

import numpy as np

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from mocap_pose_utils import (  # noqa: E402
    R_to_rpy,
    apply_link_offsets_to_poses,
    compute_relative_poses,
    filter_and_aggregate_poses,
    is_valid_pose,
    load_bodies_config,
    load_link_offsets_mm,
    quat_xyzw_to_R,
)


def _default_config_path() -> str:
    return os.path.join(_script_dir, "config", "bodies.yaml")


def _parse_row(
    row: Dict[str, str],
    body_names: List[str],
) -> Tuple[float, Dict[str, Dict[str, np.ndarray]]]:
    """解析 CSV 一行，返回 timestamp 与各刚体位姿。"""
    ts = float(row["timestamp"])
    poses: Dict[str, Dict[str, np.ndarray]] = {}
    for name in body_names:
        px = float(row[f"{name}_px"])
        py = float(row[f"{name}_py"])
        pz = float(row[f"{name}_pz"])
        qx = float(row[f"{name}_qx"])
        qy = float(row[f"{name}_qy"])
        qz = float(row[f"{name}_qz"])
        qw = float(row[f"{name}_qw"])
        pos = np.array([px, py, pz], dtype=float)
        quat = np.array([qx, qy, qz, qw], dtype=float)
        if not is_valid_pose(pos, quat):
            raise ValueError(f"invalid pose for {name} at t={ts}")
        poses[name] = {"pos": pos, "quat": quat}
    return ts, poses


def load_csv_poses(csv_path: str, body_names: List[str]) -> List[Tuple[float, Dict[str, Dict[str, np.ndarray]]]]:
    """读取 CSV，跳过无效行。"""
    rows_out: List[Tuple[float, Dict[str, Dict[str, np.ndarray]]]] = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            try:
                ts, poses = _parse_row(row, body_names)
                rows_out.append((ts, poses))
            except (ValueError, KeyError) as e:
                print(f"[process] 跳过第 {i + 1} 行: {e}")
    return rows_out


def process_relative_pair(
    frames: List[Tuple[float, Dict[str, Dict[str, np.ndarray]]]],
    child: str,
    parent: str,
    sigma: float,
) -> Dict[str, Any]:
    """逐帧计算 child 在 parent 系下位姿，3σ 滤波并聚合。"""
    rel_positions: List[np.ndarray] = []
    rel_quats: List[np.ndarray] = []

    for _ts, poses in frames:
        c = poses[child]
        p = poses[parent]
        pos_mm, quat = compute_relative_poses(
            c["pos"], c["quat"], p["pos"], p["quat"]
        )
        rel_positions.append(pos_mm)
        rel_quats.append(quat)

    pts = np.array(rel_positions, dtype=float)
    quats = np.array(rel_quats, dtype=float)
    agg = filter_and_aggregate_poses(pts, quats, sigma=sigma)

    # 转为米 + rpy
    xyz_m = (agg["xyz_mm"] / 1000.0).tolist()
    R_mean = quat_xyzw_to_R(agg["quaternion_xyzw"])
    rpy_rad = R_to_rpy(R_mean).tolist()

    key = f"{child}_in_{parent}"
    return {
        key: {
            "xyz": xyz_m,
            "rpy": rpy_rad,
            "quaternion_xyzw": agg["quaternion_xyzw"].tolist(),
            "xyz_mm": agg["xyz_mm"].tolist(),
        },
        "statistics": {
            parent: {
                "pos_std_mm": float(agg["pos_std_mm"]),
                "rot_std_deg": float(agg["rot_std_deg"]),
                "raw_frames": agg["raw_frames"],
                "valid_frames": agg["valid_frames"],
            }
        },
        "filter": {
            "sigma": sigma,
            "raw_frames": agg["raw_frames"],
            "valid_frames": agg["valid_frames"],
        },
    }


def build_output_json(
    csv_path: str,
    config_path: str,
    sigma: float,
) -> Dict[str, Any]:
    cfg = load_bodies_config(config_path)
    body_names = [b["name"] for b in cfg["bodies"]]
    relative_pairs = cfg.get("relative_pairs", [])
    link_offsets = load_link_offsets_mm(cfg)

    raw_frames = load_csv_poses(csv_path, body_names)
    if not raw_frames:
        raise RuntimeError(f"CSV 无有效数据: {csv_path}")

    # 工装位姿 → link 位姿（减去 link 系下的工装偏移）
    frames = [
        (ts, apply_link_offsets_to_poses(poses, link_offsets))
        for ts, poses in raw_frames
    ]

    result: Dict[str, Any] = {
        "source_csv": os.path.abspath(csv_path),
        "processed_at": datetime.now().isoformat(),
        "frame_id": "mocap_frame",
        "units": {"translation": "m", "rotation_rpy": "rad"},
        "link_offset_mm": {
            name: off.tolist() for name, off in link_offsets.items()
        },
        "note": "相对位姿基于 link（已扣除 bodies.yaml 中 link_offset_mm 工装偏移）",
        "filter": {"sigma": sigma, "raw_frames": len(frames)},
        "statistics": {},
    }

    for pair in relative_pairs:
        child = pair["child"]
        parent = pair["parent"]
        part = process_relative_pair(frames, child, parent, sigma)
        key = f"{child}_in_{parent}"
        result[key] = part[key]
        result["statistics"][parent] = part["statistics"][parent]
        # 更新全局 filter 有效帧数（取最后一对的值，各对应相同 raw）
        result["filter"]["valid_frames"] = part["filter"]["valid_frames"]

    # 静止性 WARN
    for parent, stats in result["statistics"].items():
        if stats["pos_std_mm"] > 1.0:
            print(
                f"[process] WARN: {parent} 平移 std={stats['pos_std_mm']:.3f} mm > 1 mm，"
                "建议重新采集"
            )
        if stats["rot_std_deg"] > 0.1:
            print(
                f"[process] WARN: {parent} 旋转 std={stats['rot_std_deg']:.4f}° > 0.1°，"
                "建议重新采集"
            )

    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="处理动捕 CSV，输出 checkerboard 相对位姿 JSON"
    )
    parser.add_argument("--input", required=True, help="输入 CSV")
    parser.add_argument("--output", required=True, help="输出 JSON")
    parser.add_argument(
        "--config",
        default=_default_config_path(),
        help="bodies.yaml",
    )
    parser.add_argument(
        "--sigma",
        type=float,
        default=3.0,
        help="3σ 滤波倍数",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"输入文件不存在: {args.input}", file=sys.stderr)
        return 1

    try:
        out = build_output_json(args.input, args.config, args.sigma)
    except Exception as e:
        print(f"[process] 失败: {e}", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)

    print(f"[process] 已写入 {args.output}")
    for pair_key in ["checkerboard_in_l_shoulder", "checkerboard_in_torso"]:
        if pair_key in out:
            xyz = out[pair_key]["xyz"]
            rpy = out[pair_key]["rpy"]
            print(
                f"  {pair_key}: xyz(m)={[round(x, 4) for x in xyz]}, "
                f"rpy(rad)={[round(r, 4) for r in rpy]}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
