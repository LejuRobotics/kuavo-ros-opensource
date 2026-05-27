#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 0302data 下的 Motive/F3 导出 CSV 转为 raw_poses 格式（与 calibration_output_0227/raw_data_motion 一致），
输出到 calibration_output/raw_data_motion/<csv 文件名>/raw_poses.csv，并绘制各刚体位置曲线图。
- 输入：tools/arm_accuracy_calibration/0302data/*.csv（Motive 多行表头格式）
- 提取：joint_1..joint_7、belly 刚体位姿（px,py,pz,qx,qy,qz,qw）
- 输出：calibration_output/raw_data_motion/<不含扩展名的 csv 名>/raw_poses.csv
- 无时间戳时用默认值 123456
- 转换后在同目录下画位置图 pose_<body>.png
"""
from __future__ import print_function

import csv
import math
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

# 与 raw_poses 列顺序一致：joint_1..7 + joint_end + belly，每刚体 7 列 (px,py,pz,qx,qy,qz,qw)
BODIES_ORDER = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "joint_end",
    "belly",
]
COLS_PER_BODY = 7
DEFAULT_TIMESTAMP = 123456.0
INPUT_DIR = "0304"
OUTPUT_BASE = os.path.join("calibration_output", "raw_data_motion")
RAW_POSES_FILENAME = "raw_poses.csv"

# Motive CSV：通常前几行为元数据，第 2、3 行为 Type / Name，第 6、7 行为 Rotation|Position 与 X,Y,Z,W
MOTIVE_HEADER_ROWS = 8  # 数据从第 9 行（0-based 第 8 行）开始
TIME_COL_ROW7 = 1      # 第 8 行（row index 7）中 "Time (Seconds)" 的列索引


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _parse_motive_header(reader):
    """
    读取前 MOTIVE_HEADER_ROWS 行，解析出每个刚体（joint_1..7, belly）的 7 列起始索引。
    返回 (time_col, body_start_cols, header_rows_list)。
    body_start_cols: dict name -> start_col，该起点的 7 列为 [qx,qy,qz,qw, px,py,pz]（Motive 顺序）。
    """
    rows = []
    for _ in range(MOTIVE_HEADER_ROWS):
        row = next(reader, None)
        if row is None:
            return None, {}, []
        rows.append(row)
    if len(rows) < 8:
        return None, {}, rows
    row2 = rows[2]  # Type
    row3 = rows[3]  # Name
    row7 = rows[7]  # 子列名 Frame, Time, X,Y,Z,W, ...
    time_col = None
    for i, c in enumerate(row7):
        if c and "time" in c.lower() and "second" in c.lower():
            time_col = i
            break
    if time_col is None:
        time_col = 1  # 常见为第 2 列

    body_start_cols = {}
    for i in range(min(len(row2), len(row3))):
        if row2[i] == "Rigid Body" and row3[i] in BODIES_ORDER and row3[i] not in body_start_cols:
            body_start_cols[row3[i]] = i
    return time_col, body_start_cols, rows


def _read_motive_csv(csv_path):
    """
    读取 Motive CSV，返回 (time_col, body_start_cols, data_rows)。
    data_rows: list of list，每行第一个元素为时间，其余为按 BODIES_ORDER 排列的 7*8 个数（缺失用 nan）。
    """
    time_col = None
    body_start_cols = None
    data_rows = []
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        time_col, body_start_cols, header_rows = _parse_motive_header(reader)
        if not body_start_cols:
            return None, {}, []
        for row in reader:
            if not row or len(row) < 2:
                continue
            # 时间
            t = _parse_float(row[time_col]) if time_col < len(row) else DEFAULT_TIMESTAMP
            if math.isnan(t):
                t = DEFAULT_TIMESTAMP
            # 按 BODIES_ORDER 输出每刚体 7 列：px,py,pz,qx,qy,qz,qw（Motive 存的是 qx,qy,qz,qw,px,py,pz）
            out_row = [t]
            for name in BODIES_ORDER:
                start = body_start_cols.get(name)
                if start is None or start + 6 >= len(row):
                    out_row.extend([math.nan] * COLS_PER_BODY)
                    continue
                qx = _parse_float(row[start])
                qy = _parse_float(row[start + 1])
                qz = _parse_float(row[start + 2])
                qw = _parse_float(row[start + 3])
                px = _parse_float(row[start + 4])
                py = _parse_float(row[start + 5])
                pz = _parse_float(row[start + 6])
                out_row.extend([px, py, pz, qx, qy, qz, qw])
            data_rows.append(out_row)
    return time_col, body_start_cols, data_rows


def _raw_poses_header():
    parts = ["timestamp"]
    for name in BODIES_ORDER:
        parts.extend(["%s_px" % name, "%s_py" % name, "%s_pz" % name,
                      "%s_qx" % name, "%s_qy" % name, "%s_qz" % name, "%s_qw" % name])
    return parts


def _write_raw_poses_csv(out_path, data_rows):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(_raw_poses_header())
        for row in data_rows:
            w.writerow(row)
    print("[F3_csv_to_F10] 已写 %s (%d 行)" % (out_path, len(data_rows)))


def _plot_rigid_body_poses(csv_path):
    """按时间为横轴为每个刚体画 X/Y/Z 位置，一刚体一图，保存到 CSV 同目录 pose_<name>.png。"""
    if not os.path.isfile(csv_path):
        print("[F3_csv_to_F10] 未找到 CSV，跳过绘图: %s" % csv_path)
        return
    out_dir = os.path.dirname(csv_path)
    time_col = 0
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return
        for row in reader:
            if len(row) <= 1:
                continue
            rows.append(row)
    if not rows:
        print("[F3_csv_to_F10] CSV 无数据行，跳过绘图")
        return
    for i, name in enumerate(BODIES_ORDER):
        start_col = 1 + i * COLS_PER_BODY
        t_vals = []
        x_vals = []
        y_vals = []
        z_vals = []
        for row in rows:
            if len(row) <= start_col + 2:
                continue
            t_vals.append(_parse_float(row[time_col]))
            x_vals.append(_parse_float(row[start_col]))
            y_vals.append(_parse_float(row[start_col + 1]))
            z_vals.append(_parse_float(row[start_col + 2]))
        if not t_vals or (all(math.isnan(x) for x in x_vals) and all(math.isnan(y) for y in y_vals) and all(math.isnan(z) for z in z_vals)):
            continue
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
        fig.suptitle("Rigid body %s - Position vs time" % name, fontsize=12)
        for ax in axes:
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.ticklabel_format(style="plain", axis="y")
        axes[0].plot(t_vals, x_vals, "r-", label="X")
        axes[0].set_ylabel("X (mm)")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t_vals, y_vals, "g-", label="Y")
        axes[1].set_ylabel("Y (mm)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t_vals, z_vals, "b-", label="Z")
        axes[2].set_ylabel("Z (mm)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        png_path = os.path.join(out_dir, "pose_%s.png" % name)
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        print("[F3_csv_to_F10] 已保存 %s" % png_path)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="0302data 下 Motive CSV 转 raw_poses 并绘图")
    parser.add_argument("--input-dir", "-i", type=str, default=os.path.join(_script_dir, INPUT_DIR),
                        help="输入目录（默认: 脚本同目录下的 %s）" % INPUT_DIR)
    parser.add_argument("--output-base", "-o", type=str, default=os.path.join(_script_dir, OUTPUT_BASE),
                        help="输出根目录（默认: 脚本同目录下的 %s）" % OUTPUT_BASE)
    args = parser.parse_args()

    input_dir = args.input_dir
    output_base = args.output_base
    if not os.path.isdir(input_dir):
        print("[F3_csv_to_F10] 输入目录不存在: %s" % input_dir)
        return 1

    csv_files = [f for f in os.listdir(input_dir) if f.endswith(".csv")]
    if not csv_files:
        print("[F3_csv_to_F10] 未找到 CSV 文件: %s" % input_dir)
        return 1

    for fn in sorted(csv_files):
        csv_path = os.path.join(input_dir, fn)
        basename = os.path.splitext(fn)[0]
        time_col, body_start_cols, data_rows = _read_motive_csv(csv_path)
        if not body_start_cols:
            print("[F3_csv_to_F10] 跳过（无法解析表头）: %s" % csv_path)
            continue
        if not data_rows:
            print("[F3_csv_to_F10] 跳过（无数据行）: %s" % csv_path)
            continue
        out_dir = os.path.join(output_base, basename)
        out_csv = os.path.join(out_dir, RAW_POSES_FILENAME)
        _write_raw_poses_csv(out_csv, data_rows)
        _plot_rigid_body_poses(out_csv)

    print("[F3_csv_to_F10] 完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
