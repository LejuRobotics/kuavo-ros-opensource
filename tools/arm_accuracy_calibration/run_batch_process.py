#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块二：离线批量解算。读取模块一生成的 raw_data_motion CSV，筛除异常点后解算 T 矩阵并保存到 matrix_data_motion。
- joint_0：解算 j2-in-j1、j3-in-j2、j4-in-j3、j5-in-j4、j6-in-j5、j7-in-j6，以及 j1-in-belly（配置中 belly 阵列为 base 系），保存到 matrix_data_motion/joint_0/
- joint_-1 / joint_-1_test1 / joint_-1_test2 / joint_-1_test3 / joint_-1_test4：读取对应 raw 数据，全部转到 joint_1 系下（j2..j7 in j1），分别输出到 matrix_data_motion/joint_-1/、joint_-1_test1/、joint_-1_test2/、joint_-1_test3/、joint_-1_test4/
- joint_1..7：每个文件夹解算对应的一对 T，保存到 matrix_data_motion/joint_<k>/
支持 joint_1..6（及 joint_7）的 CSV 列，与 run_data_collection 列顺序一致。
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

from arm_accuracy_calibration.functions import load_config, solve_joints_in_frame

# 与 run_data_collection 中 CSV 列顺序一致（joint_1..7 + joint_end + belly）
BODIES_NAMES = [
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
COLS_PER_BODY = 7  # px, py, pz, qx, qy, qz, qw
POS_INVALID_THRESHOLD = 9999.0  # 动捕 pos 任一分量 > 该值则视为无效，筛除该行

WORKSPACE_BASE = "calibration_output"  # 工作区根目录，其下为 raw_data_motion、matrix_data_motion、fitted_results
RAW_BASE = "raw_data_motion"
MATRIX_BASE = "matrix_data_motion"
RAW_POSES_FILENAME = "raw_poses.csv"

# joint_0 输出的矩阵文件名（j2-in-j1 ... j7-in-j6，以及 j1-in-belly）；joint_1..7 各一个文件
MATRIX_FILES_JOINT0 = [
    "T_joint2_in_joint1", "T_joint3_in_joint2", "T_joint4_in_joint3",
    "T_joint5_in_joint4", "T_joint6_in_joint5", "T_joint7_in_joint6",
    "T_joint1_in_belly",
]
# joint_k 对应的 (reference_joint, target_joints), k=1..7（仅 1..4 有独立解算，5..7 复用 j4-in-j3）
JOINT_K_SOLVE = [
    ("joint_1", ["joint_2"]),   # joint_1 -> T j2 in j1
    ("joint_2", ["joint_3"]),   # joint_2 -> T j3 in j2
    ("joint_3", ["joint_4"]),   # joint_3 -> T j4 in j3
    ("joint_4", ["joint_5"]),   # joint_4,5,6,7 均输出 j4 in j3
    ("joint_5", ["joint_6"]),
    ("joint_6", ["joint_7"]),
    ("joint_7", ["joint_end"]),
]


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _row_to_rigid_body_poses(row, time_col=0):
    """从 CSV 一行构建 rigid_body_poses：{name: {"pos": [x,y,z], "quat": [x,y,z,w]}}，单位 mm。"""
    out = {}
    for i, name in enumerate(BODIES_NAMES):
        start = 1 + i * COLS_PER_BODY
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


def _is_row_valid_for_solve(row, min_bodies=6):
    """筛除：前 min_bodies 个刚体任一动捕 pos 分量 > POS_INVALID_THRESHOLD 或为 nan 则整行丢弃。"""
    for i in range(min(min_bodies, len(BODIES_NAMES))):
        start = 1 + i * COLS_PER_BODY
        if len(row) <= start + 2:
            return False
        px = _parse_float(row[start])
        py = _parse_float(row[start + 1])
        pz = _parse_float(row[start + 2])
        if math.isnan(px) or math.isnan(py) or math.isnan(pz):
            return False
        if px > POS_INVALID_THRESHOLD or py > POS_INVALID_THRESHOLD or pz > POS_INVALID_THRESHOLD:
            return False
    return True


def _read_and_filter_csv(csv_path):
    """读取 CSV，筛除无效行，返回 [(timestamp, rigid_body_poses), ...]。"""
    if not os.path.isfile(csv_path):
        return []
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return []
        for row in reader:
            if len(row) <= 1:
                continue
            if not _is_row_valid_for_solve(row):
                continue
            t = _parse_float(row[0])
            poses = _row_to_rigid_body_poses(row)
            if len(poses) < 6:  # 至少需要 joint_1..6
                continue
            rows.append((t, poses))
    return rows


def _matrix_csv_header():
    return ["timestamp"] + ["m%d%d" % (r, c) for r in range(4) for c in range(4)]


def _write_matrix_csv(path, rows_with_ts_and_flat_matrix):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(_matrix_csv_header())
        for row in rows_with_ts_and_flat_matrix:
            w.writerow(row)


# 4x4 矩阵 CSV 行优先：m03,m13,m23 为平移 tx,ty,tz（单位 mm）
def _get_T_position_from_matrix_row(row):
    """row 为 [timestamp, m00, m01, ..., m33]，返回 (t, tx, ty, tz)。"""
    if len(row) < 14:
        return None
    t = _parse_float(row[0])
    tx = _parse_float(row[4])   # m03
    ty = _parse_float(row[8])   # m13
    tz = _parse_float(row[12])  # m23
    return (t, tx, ty, tz)


def _plot_matrix_positions(matrix_base):
    """扫描 matrix_data_motion 下所有 */*.csv，对每个文件画 T 的位移 tx,ty,tz 随时间曲线，保存到该 CSV 同目录下。"""
    if not os.path.isdir(matrix_base):
        return
    csv_files = []
    for sub in sorted(os.listdir(matrix_base)):
        sub_dir = os.path.join(matrix_base, sub)
        if not os.path.isdir(sub_dir):
            continue
        for f in sorted(os.listdir(sub_dir)):
            if f.endswith(".csv"):
                csv_files.append((sub, os.path.join(sub_dir, f)))
    if not csv_files:
        print("[run_batch_process] 未找到矩阵 CSV，跳过绘图")
        return
    for sub, path in csv_files:
        name = os.path.splitext(os.path.basename(path))[0]
        rows = []
        with open(path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if len(row) < 14:
                    continue
                pt = _get_T_position_from_matrix_row(row)
                if pt is None or (math.isnan(pt[1]) and math.isnan(pt[2]) and math.isnan(pt[3])):
                    continue
                rows.append(pt)
        if not rows:
            continue
        t_vals = [r[0] for r in rows]
        tx = [r[1] for r in rows]
        ty = [r[2] for r in rows]
        tz = [r[3] for r in rows]
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
        fig.suptitle("%s / %s - T translation (mm)" % (sub, name), fontsize=12)
        for ax in axes:
            ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            ax.ticklabel_format(style="plain", axis="y")
        axes[0].plot(t_vals, tx, "r-", label="tx")
        axes[0].set_ylabel("tx (mm)")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t_vals, ty, "g-", label="ty")
        axes[1].set_ylabel("ty (mm)")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t_vals, tz, "b-", label="tz")
        axes[2].set_ylabel("tz (mm)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        out_dir = os.path.dirname(path)
        png_path = os.path.join(out_dir, "%s_T_position.png" % name)
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        print("[run_batch_process] 已保存 %s" % png_path)


def _process_joint0(all_config, raw_dir, matrix_dir):
    """读取 raw_dir 下 raw_poses.csv，解算 j2-in-j1、j3-in-j2、...、j7-in-j6、j1-in-belly，保存到 matrix_dir 下 CSV。"""
    csv_path = os.path.join(raw_dir, RAW_POSES_FILENAME)
    data = _read_and_filter_csv(csv_path)
    if not data:
        print("[run_batch_process] joint_0: 无有效数据或文件不存在: %s" % csv_path)
        return
    # 解算：(ref, target)；belly 刚体在配置中 joint 名为 base，故 j1-in-belly 为 base <- joint_1
    tasks = [
        ("joint_1", ["joint_2"]),
        ("joint_2", ["joint_3"]),
        ("joint_3", ["joint_4"]),
        ("joint_4", ["joint_5"]),
        ("joint_5", ["joint_6"]),
        ("joint_6", ["joint_7"]),
        ("base", ["joint_1"]),
    ]
    results = [[] for _ in range(7)]  # 每个任务多行 (timestamp, 16 个数)
    for t, poses in data:
        # joint_0：乘配置文件偏移（enable_motion_control=False）
        all_T = []
        for ref, targets in tasks:
            r, _ = solve_joints_in_frame(
                all_config, poses, reference_joint=ref, target_joints=targets,
                enable_motion_control=False,
            )
            all_T.append(r.get(targets[0]))
        for i, T in enumerate(all_T):
            if T is not None:
                results[i].append([t] + list(T.flatten()))
    for i, name in enumerate(MATRIX_FILES_JOINT0):
        out_path = os.path.join(matrix_dir, name + ".csv")
        _write_matrix_csv(out_path, results[i])
        print("[run_batch_process] joint_0: 已写 %s (%d 行)" % (out_path, len(results[i])))


def _process_joint_k(all_config, k, raw_dir, matrix_dir):
    """k 为 1..7。读取 raw_dir 的 CSV，解算 JOINT_K_SOLVE[k-1]，保存一个 CSV 到 matrix_dir。"""
    csv_path = os.path.join(raw_dir, RAW_POSES_FILENAME)
    data = _read_and_filter_csv(csv_path)
    if not data:
        print("[run_batch_process] joint_%d: 无有效数据或文件不存在: %s" % (k, csv_path))
        return
    ref, targets = JOINT_K_SOLVE[k - 1]
    file_name = "T_%s_in_%s" % (targets[0], ref)
    rows = []
    # joint_1..7：不乘配置文件偏移，直接用刚体在动捕系下的位姿
    for t, poses in data:
        result, _ = solve_joints_in_frame(
            all_config, poses, reference_joint=ref, target_joints=targets,
            enable_motion_control=True,
        )
        T = result.get(targets[0])
        if T is not None:
            rows.append([t] + list(T.flatten()))
    if not rows:
        print("[run_batch_process] joint_%d: 无有效解算结果" % k)
        return
    out_path = os.path.join(matrix_dir, file_name + ".csv")
    _write_matrix_csv(out_path, rows)
    print("[run_batch_process] joint_%d: 已写 %s (%d 行)" % (k, out_path, len(rows)))


def _process_joint_minus1(all_config, raw_dir, matrix_dir, out_subdir="joint_-1"):
    """读取 raw_dir 的 raw_poses.csv，解算 T 并全部转到 joint_1 系下，输出到 matrix_dir/<out_subdir>/（joint_2..7 in joint_1）。
    out_subdir 可为 joint_-1、joint_-1_test1、joint_-1_test2、joint_-1_test3、joint_-1_test4 等。"""
    csv_path = os.path.join(raw_dir, RAW_POSES_FILENAME)
    data = _read_and_filter_csv(csv_path)
    if not data:
        print("[run_batch_process] %s: 无有效数据或文件不存在: %s" % (out_subdir, csv_path))
        return
    out_dir = os.path.join(matrix_dir, out_subdir)
    # joint_2..7 在 joint_1 系下的结果
    result_lists = [[] for _ in range(6)]  # j2,j3,j4,j5,j6,j7 in j1
    target_names = ["joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
    for t, poses in data:
        all_T = []
        for j in range(6):
            r, _ = solve_joints_in_frame(
                all_config, poses, reference_joint="joint_1", target_joints=[target_names[j]],
                enable_motion_control=False,
            )
            all_T.append(r.get(target_names[j]))
        for i, T in enumerate(all_T):
            if T is not None:
                result_lists[i].append([t] + list(np.array(T).flatten()))
    if not result_lists[0]:
        print("[run_batch_process] %s: 无有效解算结果" % out_subdir)
        return
    os.makedirs(out_dir, exist_ok=True)
    file_names = ["T_joint2_in_joint1", "T_joint3_in_joint1", "T_joint4_in_joint1",
                  "T_joint5_in_joint1", "T_joint6_in_joint1", "T_joint7_in_joint1"]
    for i, name in enumerate(file_names):
        if result_lists[i]:
            _write_matrix_csv(os.path.join(out_dir, name + ".csv"), result_lists[i])
    print("[run_batch_process] %s: 已转 joint_1 系并写 %s (%d 行)" % (out_subdir, out_dir, len(result_lists[0])))


def main():
    import argparse
    parser = argparse.ArgumentParser(description="模块二：读取 raw_data_motion，解算 T 矩阵写入 matrix_data_motion")
    parser.add_argument("--workspace", "-w", type=str, default=WORKSPACE_BASE,
                        help="工作区根目录（默认: %s）" % WORKSPACE_BASE)
    args = parser.parse_args()
    base = _script_dir
    raw_base = os.path.join(base, args.workspace, RAW_BASE)
    matrix_base = os.path.join(base, args.workspace, MATRIX_BASE)

    all_config = load_config()
    print("[run_batch_process] 已加载配置，阵列: %s" % all_config.names())

    # 1) joint_0
    raw_j0 = os.path.join(raw_base, "joint_0")
    if os.path.isdir(raw_j0):
        _process_joint0(all_config, raw_j0, os.path.join(matrix_base, "joint_0"))
    else:
        print("[run_batch_process] 跳过 joint_0（目录不存在）: %s" % raw_j0)

    # 2) joint_-1 / joint_-1_test1 / joint_-1_test2 / joint_-1_test3 / joint_-1_test4：
    #    读取后全部转到 joint_1 系下，输出到 matrix_data_motion/<对应子目录>/
    for subdir in ("joint_-1", "joint_-1_test1", "joint_-1_test2", "joint_-1_test3", "joint_-1_test4"):
        raw_dir = os.path.join(raw_base, subdir)
        if os.path.isdir(raw_dir):
            _process_joint_minus1(all_config, raw_dir, matrix_base, out_subdir=subdir)
        else:
            print("[run_batch_process] 跳过 %s（目录不存在）: %s" % (subdir, raw_dir))

    # 3) joint_1 .. joint_7
    for k in range(1, 8):
        raw_jk = os.path.join(raw_base, "joint_%d" % k)
        if not os.path.isdir(raw_jk):
            print("[run_batch_process] 跳过 joint_%d（目录不存在）: %s" % (k, raw_jk))
            continue
        _process_joint_k(all_config, k, raw_jk, os.path.join(matrix_base, "joint_%d" % k))

    print("[run_batch_process] 完成，绘制 T 位置曲线...")
    _plot_matrix_positions(matrix_base)
    return 0


if __name__ == "__main__":
    sys.exit(main())
