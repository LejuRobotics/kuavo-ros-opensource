#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块二：转到 joint_1 下描述。

读取四点循环每轮的 `data.csv`（默认：`output/cycles/cycle_*/data.csv`），
将每帧的动捕刚体位姿乘配置文件中的刚体→关节变换后，按 `run_batch_process.py` 的方式
全部转到 `joint_1` 坐标系下，并为每个 cycle 输出 4 个独立 CSV（每点一个）及图。

依赖配置：tools/arm_accuracy_calibration/config/config.yaml
  - 使用其中 arrays 下各 joint（joint_1～joint_7、joint_end）的 T_rigid_body_to_joint，
    将动捕系下的刚体 pos/quat 转为关节在 mocap 下的位姿，再统一变换到 joint_1 系。

输出：
- 根目录默认：`output/joint1/`
- 每轮：`output/joint1/cycle_N/`
  - `point_0_in_joint1.csv` .. `point_3_in_joint1.csv`
  - `point_0_in_joint1_Tpos.png` 等
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
_pkg_parent = os.path.dirname(_parent)
if _pkg_parent not in sys.path:
    sys.path.insert(0, _pkg_parent)

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

from arm_accuracy_calibration.functions import load_config, solve_joints_in_frame

# 与 four_points_cycle.py 记录顺序一致：8 个刚体
RECORD_BODIES = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7", "joint_end",
]
COLS_PER_BODY = 7
# CSV 前 4 列为 cycle_idx, point_idx, frame_idx, timestamp，随后 8*7 列为动捕，再 7 列为 left_arm
META_COLS = 4
POS_INVALID_THRESHOLD = 9999.0
POINT_IDXS = (0, 1, 2, 3)


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _row_to_rigid_body_poses(row):
    """从四点 CSV 一行构建 rigid_body_poses，单位 mm。"""
    out = {}
    for i, name in enumerate(RECORD_BODIES):
        start = META_COLS + i * COLS_PER_BODY
        if len(row) <= start + 6:
            continue
        px = _parse_float(row[start])
        py = _parse_float(row[start + 1])
        pz = _parse_float(row[start + 2])
        qx = _parse_float(row[start + 3])
        qy = _parse_float(row[start + 4])
        qz = _parse_float(row[start + 5])
        qw = _parse_float(row[start + 6])
        if math.isnan(px) and math.isnan(py) and math.isnan(pz):
            continue
        out[name] = {
            "pos": np.array([px, py, pz], dtype=float),
            "quat": np.array([qx, qy, qz, qw], dtype=float),
        }
    return out


def _is_row_valid(row, min_bodies=7):
    """前 min_bodies 个刚体 pos 有效（非 nan、不超过阈值）才保留。"""
    for i in range(min(min_bodies, len(RECORD_BODIES))):
        start = META_COLS + i * COLS_PER_BODY
        if len(row) <= start + 2:
            return False
        px = _parse_float(row[start])
        py = _parse_float(row[start + 1])
        pz = _parse_float(row[start + 2])
        if math.isnan(px) or math.isnan(py) or math.isnan(pz):
            return False
        if abs(px) > POS_INVALID_THRESHOLD or abs(py) > POS_INVALID_THRESHOLD or abs(pz) > POS_INVALID_THRESHOLD:
            return False
    return True


def _read_and_group_by_point(csv_path):
    """
    读取 CSV，跳过分隔行，按 point_idx 分组。
    返回: {point_idx: [(cycle_idx, frame_idx, timestamp, rigid_body_poses), ...]}
    """
    if not os.path.isfile(csv_path):
        return {}
    by_point = {p: [] for p in POINT_IDXS}
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return by_point
        for row in reader:
            if len(row) <= META_COLS:
                continue
            if str(row[0]).strip().startswith("==="):
                continue
            try:
                cycle_idx = int(row[0])
                point_idx = int(row[1])
                frame_idx = int(row[2])
                ts = _parse_float(row[3])
            except (ValueError, IndexError):
                continue
            if point_idx not in by_point:
                continue
            if not _is_row_valid(row):
                continue
            poses = _row_to_rigid_body_poses(row)
            if len(poses) < 7:
                continue
            by_point[point_idx].append((cycle_idx, frame_idx, ts, poses))
    return by_point


def _out_header():
    cols = ["cycle_idx", "frame_idx", "timestamp"]
    for j in range(2, 8):
        name = "T_joint%d_in_joint1" % j
        cols.extend(["%s_m%d%d" % (name, r, c) for r in range(4) for c in range(4)])
    return cols


def _plot_point_translations(point_csv_path: str, point_idx: int) -> None:
    """从 point_<k>_in_joint1.csv 绘制：每个 joint(2..7) 单独一张图，tx/ty/tz 随时间变化（时间戳不保证连续）。"""
    if not os.path.isfile(point_csv_path):
        return
    rows = []
    with open(point_csv_path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return
        for row in r:
            if len(row) < 3 + 6 * 16:
                continue
            rows.append(row)
    if not rows:
        return

    # 每个 joint 的矩阵占 16 列，行优先展平时 m03/m13/m23 分别位于下标 3/7/11
    t_vals = [_parse_float(x[2]) for x in rows]  # timestamp
    base = 3  # cycle_idx, frame_idx, timestamp

    for j in range(2, 8):
        tx = []
        ty = []
        tz = []
        for row in rows:
            j_offset = (j - 2) * 16
            tx.append(_parse_float(row[base + j_offset + 3]))
            ty.append(_parse_float(row[base + j_offset + 7]))
            tz.append(_parse_float(row[base + j_offset + 11]))

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
        fig.suptitle("Point %d - joint_%d in joint_1 translation (mm)" % (point_idx, j), fontsize=12)
        for ax in axes:
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.ticklabel_format(style="plain", axis="y")
            ax.grid(True, alpha=0.3)
        axes[0].plot(t_vals, tx, "r-", label="tx")
        axes[0].set_ylabel("tx (mm)")
        axes[0].legend(loc="upper right")
        axes[1].plot(t_vals, ty, "g-", label="ty")
        axes[1].set_ylabel("ty (mm)")
        axes[1].legend(loc="upper right")
        axes[2].plot(t_vals, tz, "b-", label="tz")
        axes[2].set_ylabel("tz (mm)")
        axes[2].set_xlabel("time (s)")
        axes[2].legend(loc="upper right")
        plt.tight_layout()
        out_png = os.path.join(os.path.dirname(point_csv_path), "point_%d_joint_%d_in_joint1_Tpos.png" % (point_idx, j))
        fig.savefig(out_png, dpi=150)
        plt.close(fig)


def _process_one_cycle(all_config, input_csv: str, out_cycle_dir: str) -> bool:
    """处理单个 cycle 的 data.csv -> out_cycle_dir 下 4 个 CSV + 4 张图。"""
    data_by_point = _read_and_group_by_point(input_csv)
    if not any(data_by_point[p] for p in POINT_IDXS):
        print("[four_points_to_joint1] 无有效数据或文件不存在: %s" % input_csv)
        return False

    target_names = ["joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
    os.makedirs(out_cycle_dir, exist_ok=True)

    ok_any = False
    for point_idx in POINT_IDXS:
        rows_data = data_by_point.get(point_idx, [])
        if not rows_data:
            continue

        out_path = os.path.join(out_cycle_dir, "point_%d_in_joint1.csv" % point_idx)
        out_rows = []
        for cycle_idx, frame_idx, ts, poses in rows_data:
            all_T = []
            for j in range(6):
                r, _ = solve_joints_in_frame(
                    all_config, poses,
                    reference_joint="joint_1",
                    target_joints=[target_names[j]],
                    enable_motion_control=False,
                )
                all_T.append(r.get(target_names[j]))
            if all(T is not None for T in all_T):
                row = [cycle_idx, frame_idx, "%.6f" % ts]
                for T in all_T:
                    row.extend(["%.8g" % x for x in np.array(T).flatten().tolist()])
                out_rows.append(row)

        if not out_rows:
            continue

        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(_out_header())
            w.writerows(out_rows)
        _plot_point_translations(out_path, point_idx)
        print("[four_points_to_joint1] point_%d: 已写 %s (%d 行)" % (point_idx, out_path, len(out_rows)))
        ok_any = True

    return ok_any


def main():
    parser = argparse.ArgumentParser(description="四点循环：逐 cycle 转 joint_1 系并输出到新目录（依赖 config/config.yaml）")
    parser.add_argument(
        "--input", "-i", type=str, default=None,
        help="输入：可为单个 cycle_N/data.csv，或为 cycles 根目录（默认本目录 output/cycles）",
    )
    parser.add_argument(
        "--output-dir", "-o", type=str, default=None,
        help="输出根目录（默认本目录 output/joint1）。注意：不会写回 input 目录。",
    )
    parser.add_argument(
        "--config", "-c", type=str, default=None,
        help="刚体阵列配置文件路径，默认使用 tools/arm_accuracy_calibration/config/config.yaml",
    )
    args = parser.parse_args()

    if args.input is None:
        args.input = os.path.join(_script_dir, "output", "cycles")
    if args.output_dir is None:
        args.output_dir = os.path.join(_script_dir, "output", "joint1")

    all_config = load_config(config_path=args.config)
    print("[four_points_to_joint1] 已加载配置 %s，阵列: %s" % (
        args.config or "config/config.yaml", all_config.names()))

    input_path = os.path.abspath(args.input)
    out_root = os.path.abspath(args.output_dir)
    os.makedirs(out_root, exist_ok=True)

    # 情况 A：输入是单个 CSV
    if os.path.isfile(input_path):
        out_cycle_dir = os.path.join(out_root, "single_cycle")
        ok = _process_one_cycle(all_config, input_path, out_cycle_dir)
        print("[four_points_to_joint1] 完成，输出：%s" % out_cycle_dir)
        return 0 if ok else 1

    # 情况 B：输入是 cycles 根目录
    if not os.path.isdir(input_path):
        print("[four_points_to_joint1] 输入不存在: %s" % input_path)
        return 1

    cycle_dirs = []
    for name in sorted(os.listdir(input_path)):
        if not name.startswith("cycle_"):
            continue
        d = os.path.join(input_path, name)
        if os.path.isdir(d):
            cycle_dirs.append((name, d))
    if not cycle_dirs:
        print("[four_points_to_joint1] 未找到 cycle_* 子目录: %s" % input_path)
        return 1

    ok_all = True
    for name, d in cycle_dirs:
        csv_path = os.path.join(d, "data.csv")
        out_cycle_dir = os.path.join(out_root, name)
        print("[four_points_to_joint1] 处理: %s -> %s" % (csv_path, out_cycle_dir))
        ok = _process_one_cycle(all_config, csv_path, out_cycle_dir)
        ok_all = ok_all and ok

    print("[four_points_to_joint1] 完成，输出根目录：%s" % out_root)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
