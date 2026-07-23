#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块三：拟合与保存。读取模块二生成的 matrix_data_motion 下 CSV，
- joint_0：对各 T 分别做最小二乘拟合，得到拟合后的 T 保存到 CSV；并拟合模块一 raw_sensor.csv 的传感器角度，输出 CSV；
- joint_-1 / joint_-1_test1 / joint_-1_test2 / joint_-1_test3 / joint_-1_test4：对各自目录下 T 做最小二乘拟合；并拟合对应 raw_sensor.csv 的传感器角度，分别输出到 fitted_results/<对应子目录>/；
- joint_1..7：对 T 的轨迹做圆拟合，得到拟合 T、圆心、半径、轴线，写入 CSV 保存。
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import subprocess
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from arm_accuracy_calibration.functions.fit_matrix_ls import (
    fit_matrix_from_csv,
    save_matrix_to_csv,
)
from arm_accuracy_calibration.functions.fit_rotation_axis_ls import (
    configure_matplotlib_plot_fonts,
    fit_rotation_axis_from_csv,
    visualize_3d_circle,
)

configure_matplotlib_plot_fonts()

WORKSPACE_BASE = "calibration_output"  # 工作区根目录，其下为 raw_data_motion、matrix_data_motion、fitted_results
MATRIX_BASE = "matrix_data_motion"
RAW_BASE = "raw_data_motion"
FITTED_BASE = "fitted_results"
RAW_SENSOR_FILENAME = "raw_sensor.csv"
# 模块二已做筛选，模块三不再滤数据：joint_0 与圆拟合均使用 inf 表示不过滤
NO_FILTER = float("inf")
# joint_1..7 圆拟合时参考轴（与 batch_fit_matrices 中 joint_config 一致）
JOINT_REFERENCE_AXIS = {
    1: "y",
    2: "x",
    3: "z",
    4: "y",
    5: "z",
    6: "x",
    7: "y",
}
AXIS_NAME_TO_VEC = {
    "x": np.array([1.0, 0.0, 0.0], dtype=float),
    "y": np.array([0.0, 1.0, 0.0], dtype=float),
    "z": np.array([0.0, 0.0, 1.0], dtype=float),
}
# 拟合转轴与理想参考轴（+X/+Y/+Z）夹角超过该值（度）时，视为与名义方向相反，对 axis 取反
DEFAULT_AXIS_IDEAL_FLIP_DEG = 150.0
DEFAULT_SAMPLE_COUNT = 500


def _angle_between_unit_vectors_deg(a, b):
    """两向量夹角（度），a、b 可为非单位向量。"""
    a = np.asarray(a, dtype=float).reshape(3)
    b = np.asarray(b, dtype=float).reshape(3)
    la = np.linalg.norm(a)
    lb = np.linalg.norm(b)
    if la < 1e-12 or lb < 1e-12:
        return float("nan")
    a = a / la
    b = b / lb
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _maybe_flip_fitted_axis_to_ideal(axis_vec, ref_axis_name, threshold_deg):
    """
    若拟合轴与理想轴夹角 > threshold_deg，认为正负与名义相反，对 axis 取反。
    threshold_deg <= 0 时不取反。
    返回 (axis_out, flipped, angle_deg)。
    """
    axis_vec = np.asarray(axis_vec, dtype=float).reshape(3)
    ref = AXIS_NAME_TO_VEC.get(ref_axis_name, AXIS_NAME_TO_VEC["z"])
    ref = np.asarray(ref, dtype=float).reshape(3)
    ang = _angle_between_unit_vectors_deg(axis_vec, ref)
    if threshold_deg is not None and float(threshold_deg) > 0.0:
        if not math.isnan(ang) and ang > float(threshold_deg):
            return -axis_vec, True, ang
    return axis_vec.copy(), False, ang


def _build_T_from_center_axis(center_mm, axis_unit):
    """由圆心 (3,) mm 与转轴单位向量 (3,) 构建 4x4 齐次矩阵：原点在圆心，z 轴为转轴。"""
    axis = np.asarray(axis_unit, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    if abs(axis[0]) < 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    else:
        ref = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, ref)
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(axis, u)
    v = v / (np.linalg.norm(v) + 1e-12)
    R = np.eye(3)
    R[:, 0], R[:, 1], R[:, 2] = u, v, axis
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(center_mm, dtype=float)
    return T


def _print_matrix(T, name):
    """在终端打印 4x4 矩阵。"""
    print("\n%s:" % name)
    for r in range(4):
        print("  [ %12.6f  %12.6f  %12.6f  %12.6f ]" % (T[r, 0], T[r, 1], T[r, 2], T[r, 3]))


def _draw_axes(ax, rot, linestyle="-", alpha=1.0, draw_text=True, linewidth=1.8):
    colors = ["r", "g", "b"]
    labels = ["x", "y", "z"]
    for i in range(3):
        axis = rot[:, i]
        ax.plot(
            [0.0, axis[0]],
            [0.0, axis[1]],
            [0.0, axis[2]],
            color=colors[i],
            linestyle=linestyle,
            linewidth=linewidth,
            alpha=alpha,
        )
        if draw_text:
            ax.text(axis[0], axis[1], axis[2], labels[i], color=colors[i], fontsize=8)


def _draw_single_axis(ax, axis, color, linestyle="-", alpha=1.0, label=None, linewidth=2.0):
    ax.plot(
        [0.0, axis[0]],
        [0.0, axis[1]],
        [0.0, axis[2]],
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
        alpha=alpha,
        label=label,
    )


def _select_time_spread_samples(rows_data, sample_count):
    if not rows_data:
        return []
    if sample_count >= len(rows_data):
        return sorted(rows_data, key=lambda x: x["timestamp"])
    data = sorted(rows_data, key=lambda x: x["timestamp"])
    total = len(data)
    samples = []
    for i in range(sample_count):
        start = int(i * total / float(sample_count))
        end = int((i + 1) * total / float(sample_count)) - 1
        end = max(start, end)
        center = (start + end) // 2
        samples.append(data[center])
    return samples


def _load_rotation_samples_from_matrix_csv(csv_path):
    rows_data = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = ["timestamp", "m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"]
        if not reader.fieldnames or any(k not in reader.fieldnames for k in required):
            return rows_data
        for row in reader:
            ts = _parse_float(row.get("timestamp", ""))
            if math.isnan(ts):
                continue
            vals = [_parse_float(row.get(k, "")) for k in required[1:]]
            if any(math.isnan(v) for v in vals):
                continue
            rot = np.array(
                [
                    [vals[0], vals[1], vals[2]],
                    [vals[3], vals[4], vals[5]],
                    [vals[6], vals[7], vals[8]],
                ],
                dtype=float,
            )
            rows_data.append({"timestamp": float(ts), "rot": rot})
    return rows_data


def _visualize_rotation_samples_from_matrix(rows_data, ref_axis, title, output_image_path, sample_count):
    if not rows_data:
        return False
    ref_axis = np.asarray(ref_axis, dtype=float)
    ref_axis = ref_axis / (np.linalg.norm(ref_axis) + 1e-12)
    col_idx = int(np.argmax(np.abs(ref_axis)))
    axis_name = ["x", "y", "z"][col_idx]

    all_angles_deg = []
    all_timestamps = []
    for item in rows_data:
        rot = item["rot"]
        axis_rel = rot[:, col_idx]
        cos_theta = float(np.dot(axis_rel, ref_axis))
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta_deg = math.degrees(math.acos(cos_theta))
        all_angles_deg.append(theta_deg)
        all_timestamps.append(item["timestamp"])

    pairs = sorted(zip(all_timestamps, all_angles_deg), key=lambda x: x[0])
    all_timestamps = [p[0] for p in pairs]
    all_angles_deg = [p[1] for p in pairs]
    mean_angle_deg = float(np.mean(all_angles_deg)) if all_angles_deg else float("nan")

    n = min(sample_count, len(rows_data))
    samples = _select_time_spread_samples(rows_data, n)

    fig = plt.figure(figsize=(20.0, 6.5))
    ax = fig.add_subplot(131, projection="3d")
    ax_axis = fig.add_subplot(132, projection="3d")
    ax_angle = fig.add_subplot(133)
    eye = np.eye(3, dtype=float)
    _draw_axes(ax, eye, linestyle="-", alpha=0.98, draw_text=True)
    _draw_single_axis(ax_axis, eye[:, col_idx], color="b", linestyle="-", alpha=0.98, label="eye %s-axis" % axis_name)

    sample_ts = []
    for item in samples:
        sample_ts.append(item["timestamp"])
        rot = item["rot"]
        _draw_axes(ax, rot, linestyle="-", alpha=0.55, draw_text=False, linewidth=1.8 / 5.0)
        _draw_single_axis(ax_axis, rot[:, col_idx], color="r", linestyle="-", alpha=0.55, linewidth=2.0 / 5.0)

    ref_str = "[%.0f,%.0f,%.0f]" % (ref_axis[0], ref_axis[1], ref_axis[2])
    ax.set_title("I + sampled rotation axes (%s)" % title, fontsize=11)
    ax_axis.set_title("Only %s-axis vs %s" % (axis_name, ref_str), fontsize=11)

    ax_angle.plot(all_timestamps, all_angles_deg, "-", linewidth=0.8, alpha=0.7, label="angle")
    if np.isfinite(mean_angle_deg):
        ax_angle.axhline(mean_angle_deg, color="r", linestyle="-", linewidth=1.0, label="mean = %.3f deg" % mean_angle_deg)
    ax_angle.set_xlabel("time (s)")
    ax_angle.set_ylabel("angle (deg)")
    ax_angle.set_title("Angle between %s-axis and %s" % (axis_name, ref_str))
    ax_angle.grid(True, alpha=0.3)
    ax_angle.legend(loc="best")

    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_zlim(-1.1, 1.1)
    off_lim = (-0.03, 0.03)
    main_lim = (0.0, 1.1)
    ax_axis.set_xlim(main_lim if col_idx == 0 else off_lim)
    ax_axis.set_ylim(main_lim if col_idx == 1 else off_lim)
    ax_axis.set_zlim(main_lim if col_idx == 2 else off_lim)
    for cur_ax in (ax, ax_axis):
        if hasattr(cur_ax, "set_box_aspect"):
            cur_ax.set_box_aspect((1.0, 1.0, 1.0))
        cur_ax.set_xlabel("X")
        cur_ax.set_ylabel("Y")
        cur_ax.set_zlabel("Z")
        cur_ax.grid(True, alpha=0.3)
    ax_axis.legend(loc="upper left")

    sorted_ts = sorted(sample_ts)
    if len(sorted_ts) <= 10:
        ts_preview = ", ".join("{:.3f}".format(t) for t in sorted_ts)
    else:
        head = ", ".join("{:.3f}".format(t) for t in sorted_ts[:3])
        tail = ", ".join("{:.3f}".format(t) for t in sorted_ts[-3:])
        ts_preview = "%s, ..., %s" % (head, tail)
    fig.text(0.02, 0.01, "time-spread sampled timestamps (s): %s (n=%d)" % (ts_preview, len(sorted_ts)), fontsize=8)

    out_dir = os.path.dirname(output_image_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_image_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return True


def _run_matrix_rotation_visualization(matrix_base, fitted_base, sample_count, verbose):
    """兼容 extract_quaternion 的旋转可视化，但数据源改为 matrix_data_motion。"""
    done = 0
    for k in range(1, 8):
        joint_dir = os.path.join(matrix_base, "joint_%d" % k)
        if not os.path.isdir(joint_dir):
            continue
        files = [f for f in sorted(os.listdir(joint_dir)) if f.endswith(".csv")]
        if not files:
            continue
        csv_path = os.path.join(joint_dir, files[0])
        rows_data = _load_rotation_samples_from_matrix_csv(csv_path)
        if not rows_data:
            if verbose:
                print("[run_fit_save] 跳过 joint_%d 旋转可视化：无有效旋转数据 %s" % (k, csv_path))
            continue
        ref_axis_name = JOINT_REFERENCE_AXIS.get(k, "z")
        ref_axis = AXIS_NAME_TO_VEC.get(ref_axis_name, AXIS_NAME_TO_VEC["z"])
        out_img = os.path.join(fitted_base, "joint_%d" % k, "matrix_rotation_axis.png")
        ok = _visualize_rotation_samples_from_matrix(
            rows_data=rows_data,
            ref_axis=ref_axis,
            title="joint_%d / %s" % (k, os.path.basename(csv_path)),
            output_image_path=out_img,
            sample_count=max(1, int(sample_count)),
        )
        if ok:
            done += 1
            if verbose:
                print("[run_fit_save] joint_%d: 已保存旋转轴可视化 %s" % (k, out_img))
    return done


def _process_joint0(matrix_dir, fitted_dir, x_threshold, verbose):
    """对 joint_0 下每个 T CSV 做最小二乘拟合，保存拟合后的 T。返回 [(name, T_fit), ...] 供最后统一打印。"""
    out_list = []
    if not os.path.isdir(matrix_dir):
        if verbose:
            print("[run_fit_save] 跳过 joint_0：目录不存在 %s" % matrix_dir)
        return out_list
    files = [f for f in os.listdir(matrix_dir) if f.endswith(".csv")]
    if not files:
        if verbose:
            print("[run_fit_save] 跳过 joint_0：无 CSV")
        return out_list
    os.makedirs(fitted_dir, exist_ok=True)
    for f in sorted(files):
        csv_path = os.path.join(matrix_dir, f)
        name = os.path.splitext(f)[0]
        try:
            result = fit_matrix_from_csv(
                csv_path, x_threshold=x_threshold, verbose=False
            )
            T_fit = result["T_fit"]
            out_path = os.path.join(fitted_dir, "%s_fitted.csv" % name)
            save_matrix_to_csv(T_fit, out_path, verbose=False)
            out_list.append((name, T_fit))
            if verbose:
                print("[run_fit_save] joint_0: 已保存 %s" % out_path)
        except Exception as e:
            if verbose:
                print("[run_fit_save] joint_0 处理 %s 失败: %s" % (f, e))
    return out_list


def _parse_float(s):
    s = (s or "").strip().lower()
    if s == "nan" or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


# joint_-1 下要拟合的 T 文件名（与模块二 matrix_data_motion/joint_-1/ 输出一致）
JOINT_M1_FIT_FILES = [
    "T_joint2_in_joint1", "T_joint3_in_joint1", "T_joint4_in_joint1",
    "T_joint5_in_joint1", "T_joint6_in_joint1", "T_joint7_in_joint1",
]


def _process_joint_minus1(matrix_base, raw_base, fitted_base, x_threshold, verbose, subdir="joint_-1"):
    """对 matrix_data_motion/<subdir> 下各 T 做最小二乘拟合；并拟合 raw_data_motion/<subdir>/raw_sensor.csv 的传感器角度。
    subdir 可为 joint_-1、joint_-1_test1、joint_-1_test2、joint_-1_test3、joint_-1_test4 等。"""
    matrix_dir = os.path.join(matrix_base, subdir)
    fitted_dir = os.path.join(fitted_base, subdir)
    out_list = []
    if not os.path.isdir(matrix_dir):
        if verbose:
            print("[run_fit_save] 跳过 %s：目录不存在 %s" % (subdir, matrix_dir))
    else:
        os.makedirs(fitted_dir, exist_ok=True)
        for fname in JOINT_M1_FIT_FILES:
            csv_path = os.path.join(matrix_dir, fname + ".csv")
            if not os.path.isfile(csv_path):
                continue
            try:
                result = fit_matrix_from_csv(
                    csv_path, x_threshold=x_threshold, verbose=False
                )
                T_fit = result["T_fit"]
                out_path = os.path.join(fitted_dir, "%s_fitted.csv" % fname)
                save_matrix_to_csv(T_fit, out_path, verbose=False)
                out_list.append((fname, T_fit))
                if verbose:
                    print("[run_fit_save] %s: 已保存 %s" % (subdir, out_path))
            except Exception as e:
                if verbose:
                    print("[run_fit_save] %s 处理 %s 失败: %s" % (subdir, fname, e))

    # 传感器角度拟合：读 raw_data_motion/<subdir>/raw_sensor.csv，对 7 列求均值
    raw_sensor_path = os.path.join(raw_base, subdir, RAW_SENSOR_FILENAME)
    if os.path.isfile(raw_sensor_path):
        angles_fitted = _fit_sensor_angles_from_csv(raw_sensor_path)
        if angles_fitted is not None:
            os.makedirs(fitted_dir, exist_ok=True)
            out_path = os.path.join(fitted_dir, "sensor_angles_fitted.csv")
            with open(out_path, "w", newline="") as fp:
                w = csv.writer(fp)
                w.writerow(["left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6", "left_arm_7"])
                w.writerow(["%.8g" % x for x in angles_fitted])
            if verbose:
                print("[run_fit_save] %s: 已保存传感器角度拟合 %s" % (subdir, out_path))
    elif verbose:
        print("[run_fit_save] 跳过 %s 传感器拟合：文件不存在 %s" % (subdir, raw_sensor_path))
    return out_list


def _fit_sensor_angles_from_csv(csv_path):
    """从 raw_sensor.csv 读取 7 列传感器角度，对每列求均值（忽略 nan）。"""
    if not os.path.isfile(csv_path):
        return None
    cols = [[] for _ in range(7)]
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 8:
                continue
            for i in range(7):
                v = _parse_float(row[i + 1])
                if not math.isnan(v):
                    cols[i].append(v)
    if all(len(c) > 0 for c in cols):
        return [float(np.mean(c)) for c in cols]
    return None


def _process_joint_k(k, matrix_dir, fitted_dir, x_filter_threshold, verbose, axis_ideal_flip_deg=None):
    """对 joint_k 下唯一 T CSV 做圆拟合，得到 T、圆心、半径、轴线，写入 CSV 并保存拟合图。返回 (k, center, radius, axis, image_path) 或 None。"""
    if axis_ideal_flip_deg is None:
        axis_ideal_flip_deg = DEFAULT_AXIS_IDEAL_FLIP_DEG
    if not os.path.isdir(matrix_dir):
        if verbose:
            print("[run_fit_save] 跳过 joint_%d：目录不存在 %s" % (k, matrix_dir))
        return None
    files = [f for f in os.listdir(matrix_dir) if f.endswith(".csv")]
    if not files:
        if verbose:
            print("[run_fit_save] 跳过 joint_%d：无 CSV" % k)
        return None
    csv_path = os.path.join(matrix_dir, files[0])
    ref_axis = JOINT_REFERENCE_AXIS.get(k, "z")
    os.makedirs(fitted_dir, exist_ok=True)
    image_path = os.path.join(fitted_dir, "circle_fit.png")
    try:
        axis_result = fit_rotation_axis_from_csv(
            csv_path,
            reference_axis_name=ref_axis,
            x_filter_threshold=x_filter_threshold,
            verbose=False,
            unit="mm",
            output_image_path=image_path,
        )
    except Exception as e:
        if verbose:
            print("[run_fit_save] joint_%d 圆拟合失败: %s" % (k, e))
        return None
    center = axis_result["center"]
    radius = axis_result["radius"]
    axis_vec = np.asarray(axis_result["axis"], dtype=float)
    axis_vec, did_flip, ang_ideal = _maybe_flip_fitted_axis_to_ideal(
        axis_vec, ref_axis, axis_ideal_flip_deg
    )
    if did_flip:
        if verbose:
            print(
                "[run_fit_save] joint_%d: 拟合轴与理想 %s 轴夹角 %.2f deg > %.1f deg，已对转轴取反"
                % (k, ref_axis.upper(), ang_ideal, float(axis_ideal_flip_deg))
            )
        visualize_3d_circle(
            axis_result["points_filtered"],
            center,
            radius,
            axis_vec,
            axis_result["reference_axis"],
            output_path=image_path,
            reference_axis_name=ref_axis,
        )
    T_fit = _build_T_from_center_axis(center, axis_vec)
    out_path = os.path.join(fitted_dir, "circle_fit_result.csv")
    header = ["m%d%d" % (r, c) for r in range(4) for c in range(4)]
    header += ["center_x_mm", "center_y_mm", "center_z_mm", "radius_mm"]
    header += ["axis_x", "axis_y", "axis_z"]
    row = list(T_fit.flatten()) + [float(center[0]), float(center[1]), float(center[2])]
    row.append(float(radius))
    row.extend([float(axis_vec[0]), float(axis_vec[1]), float(axis_vec[2])])
    with open(out_path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(header)
        w.writerow(row)
    if verbose:
        print("[run_fit_save] joint_%d: 已保存 %s, %s" % (k, out_path, image_path))
    return (k, center, radius, axis_vec, image_path)


def main():
    parser = argparse.ArgumentParser(
        description="模块三：读取 matrix_data_motion，joint_0 做 T 最小二乘拟合，joint_1..7 做圆拟合并保存 T/圆心/半径/轴线"
    )
    parser.add_argument(
        "--workspace", "-w", type=str, default=WORKSPACE_BASE,
        help="工作区根目录（默认: %s）" % WORKSPACE_BASE,
    )
    parser.add_argument(
        "--data_dir", "-d", type=str, default=None,
        help="matrix_data_motion 根目录（默认: <workspace>/matrix_data_motion）",
    )
    parser.add_argument(
        "--output_dir", "-o", type=str, default=None,
        help="拟合结果根目录（默认: <workspace>/fitted_results）",
    )
    parser.add_argument("--quiet", "-q", action="store_true", help="少打印")
    parser.add_argument(
        "--quat-sample-count",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help="matrix_data_motion 旋转可视化采样点数（默认: %d）" % DEFAULT_SAMPLE_COUNT,
    )
    parser.add_argument(
        "--skip-matrix-rotation-plot",
        action="store_true",
        help="跳过基于 matrix_data_motion 的旋转轴可视化出图",
    )
    parser.add_argument(
        "--axis-ideal-flip-deg",
        type=float,
        default=DEFAULT_AXIS_IDEAL_FLIP_DEG,
        help=(
            "圆拟合转轴与理想参考轴夹角大于该值（度）时对拟合轴取反；"
            "默认 %.1f；设为 0 关闭" % DEFAULT_AXIS_IDEAL_FLIP_DEG
        ),
    )
    args = parser.parse_args()

    base = _script_dir
    workspace = os.path.join(base, args.workspace)
    matrix_base = args.data_dir or os.path.join(workspace, MATRIX_BASE)
    raw_base = os.path.join(workspace, RAW_BASE)
    fitted_base = args.output_dir or os.path.join(workspace, FITTED_BASE)
    verbose = not args.quiet

    if verbose:
        print("[run_fit_save] 数据目录: %s" % matrix_base)
        print("[run_fit_save] 输出目录: %s" % fitted_base)

    # joint_0: 三个 T 最小二乘拟合（不再滤数据，模块二已筛选）
    joint0_results = _process_joint0(
        os.path.join(matrix_base, "joint_0"),
        os.path.join(fitted_base, "joint_0"),
        NO_FILTER,
        verbose,
    )

    # joint_0: 传感器角度拟合：读 raw_data_motion/joint_0/raw_sensor.csv，对 7 列求均值
    joint0_fitted_dir = os.path.join(fitted_base, "joint_0")
    joint0_raw_sensor_path = os.path.join(raw_base, "joint_0", RAW_SENSOR_FILENAME)
    if os.path.isfile(joint0_raw_sensor_path):
        angles_fitted = _fit_sensor_angles_from_csv(joint0_raw_sensor_path)
        if angles_fitted is not None:
            os.makedirs(joint0_fitted_dir, exist_ok=True)
            out_path = os.path.join(joint0_fitted_dir, "sensor_angles_fitted.csv")
            with open(out_path, "w", newline="") as fp:
                w = csv.writer(fp)
                w.writerow(["left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6", "left_arm_7"])
                w.writerow(["%.8g" % x for x in angles_fitted])
            if verbose:
                print("[run_fit_save] joint_0: 已保存传感器角度拟合 %s" % out_path)
    elif verbose:
        print("[run_fit_save] 跳过 joint_0 传感器拟合：文件不存在 %s" % joint0_raw_sensor_path)

    # joint_-1 / joint_-1_test1 / joint_-1_test2 / joint_-1_test3 / joint_-1_test4:
    # T 最小二乘拟合 + 传感器角度拟合（流程相同，输出目录不同）
    joint_minus1_results = _process_joint_minus1(
        matrix_base, raw_base, fitted_base, NO_FILTER, verbose, subdir="joint_-1"
    )
    joint_m1_test1_results = _process_joint_minus1(
        matrix_base, raw_base, fitted_base, NO_FILTER, verbose, subdir="joint_-1_test1"
    )
    joint_m1_test2_results = _process_joint_minus1(
        matrix_base, raw_base, fitted_base, NO_FILTER, verbose, subdir="joint_-1_test2"
    )
    joint_m1_test3_results = _process_joint_minus1(
        matrix_base, raw_base, fitted_base, NO_FILTER, verbose, subdir="joint_-1_test3"
    )
    joint_m1_test4_results = _process_joint_minus1(
        matrix_base, raw_base, fitted_base, NO_FILTER, verbose, subdir="joint_-1_test4"
    )

    # joint_1..7: 圆拟合，保存 T、圆心、半径、轴线及拟合图（不再滤数据，模块二已筛选）
    circle_results = []
    for k in range(1, 8):
        res = _process_joint_k(
            k,
            os.path.join(matrix_base, "joint_%d" % k),
            os.path.join(fitted_base, "joint_%d" % k),
            NO_FILTER,
            verbose,
            axis_ideal_flip_deg=args.axis_ideal_flip_deg,
        )
        if res is not None:
            circle_results.append(res)

    # 程序最后统一输出：joint_0 矩阵 + joint_-1 / joint_-1_test1 / joint_-1_test2 / joint_-1_test3 / joint_-1_test4 矩阵 + 圆拟合结果
    if verbose:
        print("\n" + "=" * 60)
        print("joint_0 fitted matrices (4x4)")
        print("=" * 60)
        for name, T in joint0_results:
            _print_matrix(T, name)
        for subdir, results in [
            ("joint_-1", joint_minus1_results),
            ("joint_-1_test1", joint_m1_test1_results),
            ("joint_-1_test2", joint_m1_test2_results),
            ("joint_-1_test3", joint_m1_test3_results),
            ("joint_-1_test4", joint_m1_test4_results),
        ]:
            if results:
                print("\n" + "=" * 60)
                print("%s fitted matrices (4x4)" % subdir)
                print("=" * 60)
                for name, T in results:
                    _print_matrix(T, name)
        print("\n" + "=" * 60)
        print("joint_1..7 circle fit: center (mm), radius (mm), axis")
        print("=" * 60)
        for res in circle_results:
            k, center, radius, axis_vec, image_path = res
            print("\njoint_%d: image %s" % (k, image_path))
            print("  center (mm): (%.6f, %.6f, %.6f)" % (float(center[0]), float(center[1]), float(center[2])))
            print("  radius (mm): %.6f" % float(radius))
            print("  axis: (%.6f, %.6f, %.6f)" % (float(axis_vec[0]), float(axis_vec[1]), float(axis_vec[2])))
    # 末尾自动执行 matrix_data_motion 质量对比（基准: 0318）
    compare_script = os.path.join(_script_dir, "compare_matrix_motion_quality.py")
    if os.path.isfile(compare_script):
        cmd = [
            sys.executable,
            compare_script,
            "--data-dir",
            matrix_base,
            "--label-b",
            "current",
        ]
        if verbose:
            print("\n[run_fit_save] 开始执行: %s" % " ".join(cmd))
        rc = subprocess.call(cmd)
        if rc != 0:
            print("[run_fit_save] 警告: compare_matrix_motion_quality.py 退出码=%d" % rc)
    elif verbose:
        print("[run_fit_save] 跳过对比：脚本不存在 %s" % compare_script)
    if not args.skip_matrix_rotation_plot:
        count = _run_matrix_rotation_visualization(
            matrix_base=matrix_base,
            fitted_base=fitted_base,
            sample_count=args.quat_sample_count,
            verbose=verbose,
        )
        if verbose:
            print("[run_fit_save] matrix_data_motion 旋转可视化完成，共输出 %d 张图" % count)
    if verbose:
        print("\n[run_fit_save] 完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
