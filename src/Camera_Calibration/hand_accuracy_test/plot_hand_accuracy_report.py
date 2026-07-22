#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据 run_hand_accuracy_test 输出的 JSON/CSV 绘制柱形图：
左右末端动捕相对腰部（waist_yaw_link / torso）的绝对位置精度 ||p_mocap - p_tf|| (mm)。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent


def _load_from_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _load_from_csv(path: Path) -> Dict[str, Any]:
    """从 CSV 重建最小 report 结构（无 meta.summary 时现场计算）。"""
    waypoints: List[Dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            waypoints.append(
                {
                    "name": row["waypoint"],
                    "index": int(row["index"]),
                    "error_left": {
                        "dx_mm": float(row["err_l_dx_mm"]),
                        "dy_mm": float(row["err_l_dy_mm"]),
                        "dz_mm": float(row["err_l_dz_mm"]),
                        "norm_mm": float(row["err_l_norm_mm"]),
                    },
                    "error_right": {
                        "dx_mm": float(row["err_r_dx_mm"]),
                        "dy_mm": float(row["err_r_dy_mm"]),
                        "dz_mm": float(row["err_r_dz_mm"]),
                        "norm_mm": float(row["err_r_norm_mm"]),
                    },
                }
            )
    left_norms = [w["error_left"]["norm_mm"] for w in waypoints]
    right_norms = [w["error_right"]["norm_mm"] for w in waypoints]
    return {
        "meta": {"warn_threshold_mm": 5.0, "tf_parent": "waist_yaw_link"},
        "waypoints": waypoints,
        "summary": {
            "left_norm_mm": _stats(left_norms),
            "right_norm_mm": _stats(right_norms),
        },
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


def _waypoint_labels(waypoints: List[Dict[str, Any]]) -> List[str]:
    """路点标签：T0、T1… 若重名则加 index。"""
    names = [str(w.get("name", f"#{w.get('index', i)}")) for i, w in enumerate(waypoints)]
    if len(set(names)) < len(names):
        return [f"{n}({w.get('index', i)})" for i, (n, w) in enumerate(zip(names, waypoints))]
    return names


def plot_hand_accuracy_bars(
    report: Dict[str, Any],
    out_path: Path,
) -> Path:
    waypoints = report.get("waypoints") or []
    if not waypoints:
        raise ValueError("报告中无 waypoints 数据")

    meta = report.get("meta") or {}
    summary = report.get("summary") or {}
    tf_parent = meta.get("tf_parent", "waist_yaw_link")

    labels = _waypoint_labels(waypoints)
    n = len(labels)
    left_norm = np.array([w["error_left"]["norm_mm"] for w in waypoints], dtype=float)
    right_norm = np.array([w["error_right"]["norm_mm"] for w in waypoints], dtype=float)
    left_axis = np.array(
        [[w["error_left"]["dx_mm"], w["error_left"]["dy_mm"], w["error_left"]["dz_mm"]] for w in waypoints],
        dtype=float,
    )
    right_axis = np.array(
        [[w["error_right"]["dx_mm"], w["error_right"]["dy_mm"], w["error_right"]["dz_mm"]] for w in waypoints],
        dtype=float,
    )

    sl = summary.get("left_norm_mm") or _stats(left_norm.tolist())
    sr = summary.get("right_norm_mm") or _stats(right_norm.tolist())

    color_left = "#2E86AB"
    color_right = "#A23B72"

    fig, axes = plt.subplots(3, 1, figsize=(max(8.0, n * 1.15), 10.0), constrained_layout=True)
    ts = meta.get("timestamp", "")
    fig.suptitle(
        f"Hand mocap absolute position accuracy vs {tf_parent}\n"
        r"$\|p_\mathrm{mocap} - p_\mathrm{TF}\|$ (mm)"
        + (f"  ·  {ts[:19]}" if ts else ""),
        fontsize=12,
        fontweight="600",
    )

    # --- 子图1：各路点绝对精度（范数）---
    ax0 = axes[0]
    x = np.arange(n)
    w = 0.36
    bars_l = ax0.bar(x - w / 2, left_norm, width=w, label="Left hand", color=color_left, edgecolor="0.15")
    bars_r = ax0.bar(x + w / 2, right_norm, width=w, label="Right hand", color=color_right, edgecolor="0.15")
    ax0.set_xticks(x)
    ax0.set_xticklabels(labels)
    ax0.set_ylabel("absolute error (mm)")
    ax0.set_xlabel("waypoint")
    ax0.set_title("Per-waypoint absolute position error")
    ax0.legend(loc="upper right")
    ax0.grid(axis="y", alpha=0.35)
    ax0.set_ylim(bottom=0.0)

    y_top = max(float(left_norm.max()), float(right_norm.max()), 1e-6)
    for bar in list(bars_l) + list(bars_r):
        h = bar.get_height()
        ax0.text(
            bar.get_x() + bar.get_width() / 2,
            h + y_top * 0.02,
            f"{h:.1f}",
            ha="center",
            va="bottom",
            fontsize=7,
        )

    # --- 子图2：左右手平均绝对误差 ---
    ax1 = axes[1]
    mean_labels = ["Left hand", "Right hand"]
    mean_vals = [sl["mean"], sr["mean"]]
    mean_colors = [color_left, color_right]
    x_mean = np.arange(len(mean_labels))
    bars_mean = ax1.bar(x_mean, mean_vals, width=0.5, color=mean_colors, edgecolor="0.15")
    ax1.set_xticks(x_mean)
    ax1.set_xticklabels(mean_labels)
    ax1.set_ylabel("mean absolute error (mm)")
    ax1.set_title("Mean absolute position error (all waypoints)")
    ax1.grid(axis="y", alpha=0.35)
    ax1.set_ylim(bottom=0.0)
    for bar, side_stats in zip(bars_mean, (sl, sr)):
        h = bar.get_height()
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            h + max(mean_vals) * 0.03,
            f"{h:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
            fontweight="600",
        )
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            h * 0.5,
            f"max={side_stats['max']:.1f}\nstd={side_stats['std']:.1f}",
            ha="center",
            va="center",
            fontsize=8,
            color="white",
            fontweight="600",
        )

    # --- 子图3：分轴绝对误差 |dx|,|dy|,|dz| 均值 ---
    ax2 = axes[2]
    axis_names = ["|dx|", "|dy|", "|dz|"]
    x_ax = np.arange(len(axis_names))
    w_ax = 0.35
    left_abs_mean = np.mean(np.abs(left_axis), axis=0)
    right_abs_mean = np.mean(np.abs(right_axis), axis=0)
    ax2.bar(x_ax - w_ax / 2, left_abs_mean, width=w_ax, label="Left (mean |axis|)", color=color_left, edgecolor="0.15")
    ax2.bar(x_ax + w_ax / 2, right_abs_mean, width=w_ax, label="Right (mean |axis|)", color=color_right, edgecolor="0.15")
    ax2.set_xticks(x_ax)
    ax2.set_xticklabels(axis_names)
    ax2.set_ylabel("mean |axis error| (mm)")
    ax2.set_title("Mean absolute axis error (all waypoints)")
    ax2.legend(loc="upper right")
    ax2.grid(axis="y", alpha=0.35)
    ax2.set_ylim(bottom=0.0)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return out_path


def _resolve_input_report(path: Optional[str]) -> Tuple[Path, Dict[str, Any]]:
    if path:
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(p)
    else:
        candidates = sorted(_SCRIPT_DIR.glob("hand_accuracy_report_*.json"), key=lambda x: x.stat().st_mtime, reverse=True)
        if not candidates:
            raise FileNotFoundError("未找到 hand_accuracy_report_*.json，请用 --json 指定")
        p = candidates[0]

    if p.suffix.lower() == ".json":
        return p, _load_from_json(p)
    if p.suffix.lower() == ".csv":
        return p, _load_from_csv(p)
    raise ValueError(f"不支持的输入格式: {p}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="绘制 hand_accuracy_test 报告：左右末端相对腰部的绝对位置精度柱形图"
    )
    parser.add_argument(
        "--json",
        default=None,
        help="hand_accuracy_report_*.json 或 .csv（默认取本目录最新 JSON）",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="输出 PNG 路径（默认与输入同目录、同名 _accuracy_bars.png）",
    )
    args = parser.parse_args()

    try:
        in_path, report = _resolve_input_report(args.json)
    except (FileNotFoundError, ValueError) as e:
        print(f"[plot] 失败: {e}", file=sys.stderr)
        return 1

    if args.output:
        out_path = Path(args.output)
    else:
        stem = in_path.stem
        out_path = in_path.with_name(f"{stem}_accuracy_bars.png")

    try:
        plot_hand_accuracy_bars(report, out_path)
    except Exception as e:
        print(f"[plot] 失败: {e}", file=sys.stderr)
        return 1

    s = report.get("summary") or {}
    sl = s.get("left_norm_mm", {})
    sr = s.get("right_norm_mm", {})
    print(f"[plot] 输入: {in_path}")
    print(f"[plot] 输出: {out_path}")
    print(
        f"[plot] 左臂 mean={sl.get('mean', float('nan')):.2f} mm, "
        f"右臂 mean={sr.get('mean', float('nan')):.2f} mm"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
