#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块四：从 joint_0 拟合 T 得到各段 T_dh，并反解 DH 参数。
dh 与 joint 一一对应（dh1<->J1, dh2<->J2, dh3<->J3, dh4<->J4），
读取 fitted_results/joint_0/ 下 *_fitted.csv，将关节系 T 转到 DH 系，再对每段 T_dh 反解 [a, alpha, d, theta]。
若某段（如 T_dh2_to_dh3）反解退化或失败，会跳过该段并提示。
"""
from __future__ import print_function

import argparse
import csv
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import numpy as np

from arm_accuracy_calibration.functions.batch_fit_matrices import (
    load_dh_reference_frames,
    apply_dh_reference_frame_transform,
)
from arm_accuracy_calibration.functions.dh_calibration import initialize_dh_from_transform

WORKSPACE_BASE = "calibration_output"
FITTED_BASE = "fitted_results"
# dh 与 joint 一一对应，3 段：T_dh1_to_dh2, T_dh2_to_dh3, T_dh3_to_dh4
LINK_TASKS = [
    ("T_joint2_in_joint1", "T_dh1_to_dh2"),
    ("T_joint3_in_joint2", "T_dh2_to_dh3"),  # 该段插中间系 vir，仅对 T_vir_to_dh3 反解 DH
    ("T_joint4_in_joint3", "T_dh3_to_dh4"),
]

# 中间系 vir：位置与 dh3 一样，方向与 dh2 一样
# T_dh2_to_vir: 方向单位阵，位置与 T_dh2_to_dh3 一致
# T_vir_to_dh3: 位置为 0，方向与 T_dh2_to_dh3 一致（仅旋转部分做 DH 反解）
def _split_dh2_to_dh3_via_vir(T_dh):
    """从 T_dh2_to_dh3 拆出 T_dh2_to_vir 和 T_vir_to_dh3。"""
    T_dh2_to_vir = np.eye(4)
    T_dh2_to_vir[:3, 3] = T_dh[:3, 3]
    T_vir_to_dh3 = np.eye(4)
    T_vir_to_dh3[:3, :3] = T_dh[:3, :3]
    T_vir_to_dh3[:3, 3] = 0.0
    return T_dh2_to_vir, T_vir_to_dh3


def load_single_matrix_from_csv(csv_path):
    """
    从模块三保存的单矩阵 CSV 加载一个 4x4 矩阵（无 timestamp 列，仅一行 m00..m33）。
    单位与模块三一致：平移为 mm。
    """
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        row = next(reader)
    cols = ["m%d%d" % (r, c) for r in range(4) for c in range(4)]
    vals = [float(row[c]) for c in cols]
    return np.array(vals).reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(
        description="从 joint_0 拟合 T 转到 DH 系，计算 T_dh 并反解 DH 参数（某段解不出则跳过）"
    )
    parser.add_argument(
        "--workspace", "-w", type=str, default=WORKSPACE_BASE,
        help="工作区根目录（默认: %s）" % WORKSPACE_BASE,
    )
    parser.add_argument(
        "--input_dir", "-i", type=str, default=None,
        help="joint_0 拟合结果目录（默认: <workspace>/%s/joint_0）" % FITTED_BASE,
    )
    parser.add_argument("--quiet", "-q", action="store_true", help="少打印")
    args = parser.parse_args()

    base = _script_dir
    workspace = os.path.join(base, args.workspace)
    input_dir = args.input_dir or os.path.join(workspace, FITTED_BASE, "joint_0")
    verbose = not args.quiet

    if verbose:
        print("[run_dh_from_fitted] 输入目录: %s" % input_dir)

    config_path = os.path.join(base, "config", "dh_reference_frames.yaml")
    if not os.path.isfile(config_path):
        print("[run_dh_from_fitted] 错误: 配置文件不存在 %s" % config_path)
        return 1
    try:
        dh_frames = load_dh_reference_frames(config_path)
        if verbose:
            print("[run_dh_from_fitted] 已加载 DH 参考系: %s" % config_path)
    except Exception as e:
        print("[run_dh_from_fitted] 加载 DH 参考系失败: %s" % e)
        return 1

    # 先得到四个 T：T_dh1_to_dh2, T_dh2_to_vir, T_vir_to_dh3, T_dh3_to_dh4（中间段拆成 vir）
    four_T_list = []
    for file_key, out_name in LINK_TASKS:
        csv_path = os.path.join(input_dir, "%s_fitted.csv" % file_key)
        if not os.path.isfile(csv_path):
            if verbose:
                print("[run_dh_from_fitted] 跳过 %s: 文件不存在" % out_name)
            continue
        try:
            T_joint = load_single_matrix_from_csv(csv_path)
            T_dh = apply_dh_reference_frame_transform(T_joint, file_key, dh_frames)
            if out_name == "T_dh2_to_dh3":
                T_dh2_to_vir, T_vir_to_dh3 = _split_dh2_to_dh3_via_vir(T_dh)
                four_T_list.append(("T_dh2_to_vir", T_dh2_to_vir))
                four_T_list.append(("T_vir_to_dh3", T_vir_to_dh3))
            else:
                four_T_list.append((out_name, T_dh))
        except Exception as e:
            print("[run_dh_from_fitted] %s 处理失败: %s" % (out_name, e))
            import traceback
            traceback.print_exc()

    if len(four_T_list) != 4:
        print("[run_dh_from_fitted] 未凑齐四个 T（当前 %d 个）" % len(four_T_list))
        return 1

    # 对四个矩阵统一反解 DH（用 T_dhi+1_to_dhi = inv(T_dhi_to_dhi+1) 解算）
    results = []
    for name, T in four_T_list:
        try:
            dh = initialize_dh_from_transform(T)
            a, alpha, d, theta = float(dh[0]), float(dh[1]), float(dh[2]), float(dh[3])
            results.append({
                "name": name,
                "T_dh": T,
                "a_mm": a, "alpha_rad": alpha, "d_mm": d, "theta_rad": theta,
                "alpha_deg": np.degrees(alpha), "theta_deg": np.degrees(theta),
            })
        except Exception as e:
            print("[run_dh_from_fitted] %s 反解 DH 失败: %s" % (name, e))
            results.append({"name": name, "T_dh": T, "a_mm": None, "alpha_rad": None, "d_mm": None, "theta_rad": None})

    # 打印四个 T_dh（4x4）及 DH
    for r in results:
        print("\n%s:" % r["name"])
        print(np.asarray(r["T_dh"]))
        if r.get("a_mm") is None:
            print("  (DH 反解失败)")
            continue
        print("  a (连杆长度):     %12.6f mm" % r["a_mm"])
        print("  alpha (扭转角):  %12.6f rad = %8.4f deg" % (r["alpha_rad"], r["alpha_deg"]))
        print("  d (连杆偏距):     %12.6f mm" % r["d_mm"])
        print("  theta (关节角):  %12.6f rad = %8.4f deg" % (r["theta_rad"], r["theta_deg"]))

    # 写入 calibration_output/dh_params.csv
    dh_csv_path = os.path.join(workspace, "dh_params.csv")
    with open(dh_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["matrix_name", "a_mm", "alpha_rad", "d_mm", "theta_rad", "alpha_deg", "theta_deg"],
        )
        writer.writeheader()
        for r in results:
            if r.get("a_mm") is None:
                continue
            writer.writerow({
                "matrix_name": r["name"],
                "a_mm": r["a_mm"],
                "alpha_rad": r["alpha_rad"],
                "d_mm": r["d_mm"],
                "theta_rad": r["theta_rad"],
                "alpha_deg": r["alpha_deg"],
                "theta_deg": r["theta_deg"],
            })
    if verbose:
        print("\n[run_dh_from_fitted] DH 结果已写入: %s" % dh_csv_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
