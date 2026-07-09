#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""提取 Motive CSV 中任意两个 joint 的四元数（w,x,y,z），并保留时间戳。"""
import argparse
import csv
import math
import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# 采样可视化数量的默认值
DEFAULT_SAMPLE_COUNT = 500

# 各 joint 在自身坐标系下的旋转轴
# 来源：biped_s52.xml 中 zarm_lN_joint 的 axis 属性
# Motive 采集中 joint_N 对应 XML 中 zarm_lN_joint
JOINT_AXES = {
    "joint_1": np.array([0.0, 1.0, 0.0]),  # zarm_l1_joint  axis="0 1 0"
    "joint_2": np.array([1.0, 0.0, 0.0]),  # zarm_l2_joint  axis="1 0 0"
    "joint_3": np.array([0.0, 0.0, 1.0]),  # zarm_l3_joint  axis="0 0 1"
    "joint_4": np.array([0.0, 1.0, 0.0]),  # zarm_l4_joint  axis="0 1 0"
    "joint_5": np.array([0.0, 0.0, 1.0]),  # zarm_l5_joint  axis="0 0 1"
    "joint_6": np.array([1.0, 0.0, 0.0]),  # zarm_l6_joint  axis="1 0 0"
    "joint_7": np.array([0.0, 1.0, 0.0]),  # zarm_l7_joint  axis="0 1 0"
}


def _extract_joint_name_from_filename(filepath):
    """从路径中提取 joint_N 名称，优先文件名，其次父目录名。"""
    basename = os.path.basename(filepath)
    m = re.search(r"(joint_\d+)", basename)
    if m:
        return m.group(1)

    # raw_data_motion 常见路径：.../joint_3/raw_poses.csv
    # 若文件名不含 joint_N，则从完整路径提取最后一个 joint_N
    all_matches = re.findall(r"(joint_\d+)", filepath)
    if all_matches:
        return all_matches[-1]
    return None


def _pad_row(row, length):
    if len(row) >= length:
        return row
    return row + [""] * (length - len(row))


def _find_header_index(rows):
    for i, row in enumerate(rows):
        if len(row) >= 2 and row[0].strip() == "Frame" and row[1].strip() == "Time (Seconds)":
            return i
    raise ValueError("未找到数据表头行（Frame, Time (Seconds)）")


def _find_col_index(header_row, candidates):
    for name in candidates:
        if name in header_row:
            return header_row.index(name)
    raise KeyError(f"未找到列: {candidates}")


def _extract_from_flat_raw_motion_csv(rows, output_csv, joint_a, joint_b):
    """
    解析 raw_data_motion 的 raw_poses.csv 扁平表头格式：
    timestamp,joint_1_px,...,joint_1_qx,joint_1_qy,joint_1_qz,joint_1_qw,...
    """
    if not rows:
        raise ValueError("CSV 为空")

    header_row = [c.strip() for c in rows[0]]
    if not header_row:
        raise ValueError("CSV 表头为空")

    time_col = _find_col_index(header_row, ["timestamp", "time", "Time (Seconds)"])
    qa_x = _find_col_index(header_row, [f"{joint_a}_qx"])
    qa_y = _find_col_index(header_row, [f"{joint_a}_qy"])
    qa_z = _find_col_index(header_row, [f"{joint_a}_qz"])
    qa_w = _find_col_index(header_row, [f"{joint_a}_qw"])
    qb_x = _find_col_index(header_row, [f"{joint_b}_qx"])
    qb_y = _find_col_index(header_row, [f"{joint_b}_qy"])
    qb_z = _find_col_index(header_row, [f"{joint_b}_qz"])
    qb_w = _find_col_index(header_row, [f"{joint_b}_qw"])

    output_header = [
        "timestamp",
        f"{joint_a}_w",
        f"{joint_a}_x",
        f"{joint_a}_y",
        f"{joint_a}_z",
        f"{joint_b}_w",
        f"{joint_b}_x",
        f"{joint_b}_y",
        f"{joint_b}_z",
    ]

    key_a = f"{joint_a}_wxyz"
    key_b = f"{joint_b}_wxyz"
    rows_written = 0
    rows_data = []

    with open(output_csv, "w", encoding="utf-8", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(output_header)

        for row in rows[1:]:
            row = _pad_row(row, len(header_row))
            timestamp = row[time_col].strip()
            vals = [
                row[qa_w].strip(),
                row[qa_x].strip(),
                row[qa_y].strip(),
                row[qa_z].strip(),
                row[qb_w].strip(),
                row[qb_x].strip(),
                row[qb_y].strip(),
                row[qb_z].strip(),
            ]
            if _is_invalid(timestamp) or any(_is_invalid(v) for v in vals):
                continue

            writer.writerow([timestamp] + vals)
            rows_data.append(
                {
                    "timestamp": float(timestamp),
                    key_a: [float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])],
                    key_b: [float(vals[4]), float(vals[5]), float(vals[6]), float(vals[7])],
                }
            )
            rows_written += 1

    return rows_written, rows_data


def _find_quat_cols(name_row, type_row, group_row, axis_row, joint_name):
    mapping = {}
    for idx in range(len(axis_row)):
        if type_row[idx].strip() != "Rigid Body":
            continue
        if name_row[idx].strip() != joint_name:
            continue
        if group_row[idx].strip() != "Rotation":
            continue
        axis = axis_row[idx].strip().upper()
        if axis in {"W", "X", "Y", "Z"}:
            mapping[axis] = idx

    missing = [k for k in ("W", "X", "Y", "Z") if k not in mapping]
    if missing:
        raise KeyError(f"未找到 {joint_name} 的四元数列，缺少轴: {missing}")
    return mapping


def _is_invalid(v):
    s = (v or "").strip().lower()
    return s == "" or s == "nan"


def _quat_wxyz_to_rotmat(quat_wxyz):
    w, x, y, z = quat_wxyz
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        raise ValueError("四元数范数过小，无法转换为旋转矩阵")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


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


def _visualize_random_samples(
    rows_data,
    joint_a,
    joint_b,
    ref_axis,
    sample_count=DEFAULT_SAMPLE_COUNT,
    output_image_path=None,
    show_plot=True,
):
    if not rows_data:
        print("警告: 无有效数据，跳过可视化")
        return

    key_a = f"{joint_a}_wxyz"
    key_b = f"{joint_b}_wxyz"
    rel_label = f"R_{joint_b}^T * R_{joint_a}"

    ref_axis = np.asarray(ref_axis, dtype=float)
    ref_axis = ref_axis / np.linalg.norm(ref_axis)

    # 根据参考轴方向确定 R_rel 中对应的列索引（0=x, 1=y, 2=z）
    col_idx = int(np.argmax(np.abs(ref_axis)))
    axis_name = ["x", "y", "z"][col_idx]

    # 统计全部数据中相对旋转的参考轴与 ref_axis 的夹角
    all_angles_deg = []
    all_timestamps = []
    for item in rows_data:
        qa_full = item[key_a]
        qb_full = item[key_b]
        ra_full = _quat_wxyz_to_rotmat(qa_full)
        rb_full = _quat_wxyz_to_rotmat(qb_full)
        r_rel_full = rb_full.T.dot(ra_full)
        axis_rel_full = r_rel_full[:, col_idx]
        cos_theta = float(np.dot(axis_rel_full, ref_axis))
        # 数值稳定性保护
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta_deg = math.degrees(math.acos(cos_theta))
        all_angles_deg.append(theta_deg)
        all_timestamps.append(item["timestamp"])

    mean_angle_deg = None
    if all_angles_deg:
        # 按时间排序，保证后续可视化是严格基于时间轴顺序
        pairs = sorted(zip(all_timestamps, all_angles_deg), key=lambda x: x[0])
        all_timestamps, all_angles_deg = zip(*pairs)
        all_timestamps = list(all_timestamps)
        all_angles_deg = list(all_angles_deg)

        mean_angle_deg = float(np.mean(all_angles_deg))
        ref_str = "[{:.0f},{:.0f},{:.0f}]".format(*ref_axis)
        print(f"{rel_label} 的 {axis_name} 轴与 {ref_str}^T 的夹角均值: {mean_angle_deg:.6f} deg")

    n = min(sample_count, len(rows_data))
    samples = _select_time_spread_samples(rows_data, n)

    fig = plt.figure(figsize=(20.0, 6.5))
    ax = fig.add_subplot(131, projection="3d")
    ax_z = fig.add_subplot(132, projection="3d")
    ax_angle = fig.add_subplot(133)
    eye = np.eye(3, dtype=float)
    _draw_axes(ax, eye, linestyle="-", alpha=0.98, draw_text=True)
    _draw_single_axis(ax_z, eye[:, col_idx], color="b", linestyle="-", alpha=0.98, label=f"eye {axis_name}-axis")

    timestamps = []
    for item in samples:
        timestamp = item["timestamp"]
        timestamps.append(timestamp)
        qa = item[key_a]
        qb = item[key_b]

        ra = _quat_wxyz_to_rotmat(qa)
        rb = _quat_wxyz_to_rotmat(qb)
        r_rel = rb.T.dot(ra)
        # 采样线条使用实线，线宽为eye的1/5
        _draw_axes(ax, r_rel, linestyle="-", alpha=0.55, draw_text=False, linewidth=1.8 / 5.0)
        _draw_single_axis(ax_z, r_rel[:, col_idx], color="r", linestyle="-", alpha=0.55, linewidth=2.0 / 5.0)

    ax.set_title(f"I (solid RGB) + sampled {rel_label} (solid RGB)", fontsize=11)
    ax_z.set_title(f"Only {axis_name}-axis: eye (blue) + sampled (red) [{rel_label}]", fontsize=11)

    ref_str = "[{:.0f},{:.0f},{:.0f}]".format(*ref_axis)
    # 子图3：全部数据的夹角随时间变化（曲线）
    if all_angles_deg:
        ax_angle.plot(all_timestamps, all_angles_deg, "-", linewidth=0.8, alpha=0.7, label="angle")
        if mean_angle_deg is not None:
            ax_angle.axhline(
                mean_angle_deg,
                color="r",
                linestyle="-",
                linewidth=1.0,
                label=f"mean = {mean_angle_deg:.3f}°",
            )
        ax_angle.set_xlabel("time (s)")
        ax_angle.set_ylabel("angle (deg)")
        ax_angle.set_title(f"Angle between {axis_name} of {rel_label} and {ref_str}")
        ax_angle.grid(True, alpha=0.3)
        ax_angle.legend(loc="best")

    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_zlim(-1.1, 1.1)

    # 图2聚焦在参考轴附近，便于观察随机采样形成的锥状分布
    off_lim = (-0.03, 0.03)
    main_lim = (0.0, 1.1)
    ax_z.set_xlim(main_lim if col_idx == 0 else off_lim)
    ax_z.set_ylim(main_lim if col_idx == 1 else off_lim)
    ax_z.set_zlim(main_lim if col_idx == 2 else off_lim)

    for cur_ax in (ax, ax_z):
        if hasattr(cur_ax, "set_box_aspect"):
            cur_ax.set_box_aspect((1.0, 1.0, 1.0))
        cur_ax.set_xlabel("X")
        cur_ax.set_ylabel("Y")
        cur_ax.set_zlabel("Z")
        cur_ax.grid(True, alpha=0.3)

    ax_z.legend(loc="upper left")

    sorted_ts = sorted(timestamps)
    if len(sorted_ts) <= 10:
        ts_preview = ", ".join("{:.3f}".format(t) for t in sorted_ts)
    else:
        # 避免底部文字过长导致 bbox_inches='tight' 计算出异常超大画布
        head = ", ".join("{:.3f}".format(t) for t in sorted_ts[:3])
        tail = ", ".join("{:.3f}".format(t) for t in sorted_ts[-3:])
        ts_preview = f"{head}, ..., {tail}"
    sample_time_text = f"time-spread sampled timestamps (s): {ts_preview} (n={len(sorted_ts)})"
    fig.text(0.02, 0.01, sample_time_text, fontsize=8)

    plt.tight_layout()
    if output_image_path:
        out_dir = os.path.dirname(output_image_path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(output_image_path, dpi=180, bbox_inches="tight")
        print(f"可视化已保存: {output_image_path}")
    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def extract_quaternion_data(
    input_csv,
    output_csv,
    joint_a="joint_3",
    joint_b="joint_4",
    enable_visualize=False,
    sample_count=DEFAULT_SAMPLE_COUNT,
    output_image_path=None,
    show_plot=True,
):
    if not os.path.isfile(input_csv):
        raise FileNotFoundError(f"找不到输入文件: {input_csv}")

    with open(input_csv, "r", encoding="utf-8-sig", newline="") as f_in:
        rows = list(csv.reader(f_in))
    if not rows:
        raise ValueError("CSV 为空")

    first_non_empty = next((r for r in rows if any(c.strip() for c in r)), [])
    first_cell = first_non_empty[0].strip() if first_non_empty else ""

    # 支持 raw_data_motion 的扁平列名格式
    if first_cell.lower() == "timestamp":
        rows_written, rows_data = _extract_from_flat_raw_motion_csv(rows, output_csv, joint_a, joint_b)
    else:
        # 兼容原始 Motive 分层表头格式
        header_idx = _find_header_index(rows)
        if header_idx < 1:
            raise ValueError("CSV 结构异常：无法找到 Time (Seconds) 之前的分层表头")

        max_cols = max(len(r) for r in rows[: header_idx + 1])
        type_row = _pad_row(rows[header_idx - 5], max_cols) if header_idx >= 5 else [""] * max_cols
        name_row = _pad_row(rows[header_idx - 4], max_cols) if header_idx >= 4 else [""] * max_cols
        group_row = _pad_row(rows[header_idx - 1], max_cols) if header_idx >= 1 else [""] * max_cols
        axis_row = _pad_row(rows[header_idx], max_cols)

        time_col = None
        for i, v in enumerate(axis_row):
            if v.strip() == "Time (Seconds)":
                time_col = i
                break
        if time_col is None:
            raise KeyError("未找到时间戳列 Time (Seconds)")

        ja = _find_quat_cols(name_row, type_row, group_row, axis_row, joint_a)
        jb = _find_quat_cols(name_row, type_row, group_row, axis_row, joint_b)

        output_header = [
            "timestamp",
            f"{joint_a}_w",
            f"{joint_a}_x",
            f"{joint_a}_y",
            f"{joint_a}_z",
            f"{joint_b}_w",
            f"{joint_b}_x",
            f"{joint_b}_y",
            f"{joint_b}_z",
        ]

        key_a = f"{joint_a}_wxyz"
        key_b = f"{joint_b}_wxyz"

        rows_written = 0
        rows_data = []
        with open(output_csv, "w", encoding="utf-8", newline="") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(output_header)

            for row in rows[header_idx + 1 :]:
                row = _pad_row(row, max_cols)
                timestamp = row[time_col].strip()
                vals = [
                    row[ja["W"]].strip(),
                    row[ja["X"]].strip(),
                    row[ja["Y"]].strip(),
                    row[ja["Z"]].strip(),
                    row[jb["W"]].strip(),
                    row[jb["X"]].strip(),
                    row[jb["Y"]].strip(),
                    row[jb["Z"]].strip(),
                ]

                if _is_invalid(timestamp) or any(_is_invalid(v) for v in vals):
                    continue

                writer.writerow([timestamp] + vals)
                rows_data.append(
                    {
                        "timestamp": float(timestamp),
                        key_a: [float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])],
                        key_b: [float(vals[4]), float(vals[5]), float(vals[6]), float(vals[7])],
                    }
                )
                rows_written += 1

    print(f"提取完成：共写入 {rows_written} 行数据（{joint_a} / {joint_b}）")
    print(f"输出文件：{output_csv}")

    # 从输入文件名解析 joint 名称，自动查找旋转轴
    file_joint = _extract_joint_name_from_filename(input_csv)
    if file_joint and file_joint in JOINT_AXES:
        ref_axis = JOINT_AXES[file_joint]
        print(f"参考轴：由文件名 '{os.path.basename(input_csv)}' 识别为 {file_joint}，轴 = {ref_axis.tolist()}")
    elif joint_a in JOINT_AXES:
        ref_axis = JOINT_AXES[joint_a]
        print(f"参考轴：文件名无法识别 joint，改用 joint-a={joint_a} 的轴 = {ref_axis.tolist()}")
    else:
        ref_axis = np.array([0.0, 0.0, 1.0])
        print(f"警告: 无法从文件名或 joint-a 识别 joint，使用默认 z 轴 [0,0,1]")

    if enable_visualize:
        _visualize_random_samples(
            rows_data,
            joint_a=joint_a,
            joint_b=joint_b,
            ref_axis=ref_axis,
            sample_count=sample_count,
            output_image_path=output_image_path,
            show_plot=show_plot,
        )


def _run_batch_all(raw_data_root, output_root, sample_count, show_plots=True):
    """
    批量处理 raw_data_motion/joint_1..joint_6 的 raw_poses.csv，
    分别提取 (joint_k, joint_{k+1}) 并保存可视化图。
    """
    processed = 0
    for k in range(1, 7):
        joint_a = f"joint_{k}"
        joint_b = f"joint_{k + 1}"
        input_csv = os.path.join(raw_data_root, joint_a, "raw_poses.csv")
        if not os.path.isfile(input_csv):
            print(f"[跳过] 输入不存在: {input_csv}")
            continue

        out_csv = os.path.join(output_root, f"{joint_a}_output.csv")
        out_img = os.path.join(output_root, f"{joint_a}_to_{joint_b}_axis.png")
        out_dir = os.path.dirname(out_csv)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        print(f"\n[批处理] {joint_a} / {joint_b}")
        extract_quaternion_data(
            input_csv=input_csv,
            output_csv=out_csv,
            joint_a=joint_a,
            joint_b=joint_b,
            enable_visualize=True,
            sample_count=sample_count,
            output_image_path=out_img,
            show_plot=show_plots,
        )
        processed += 1

    print(f"\n批处理完成：共处理 {processed} 个 joint 对")


def main():
    parser = argparse.ArgumentParser(
        description="提取任意两个 joint 的四元数并可视化（默认无参数时批量输出全部 joint 图）"
    )
    parser.add_argument(
        "input_csv",
        nargs="?",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..",
            "calibration_output",
            "raw_data_motion",
            "joint_3",
            "raw_poses.csv",
        ),
        help="输入 CSV 路径（支持 Motive CSV 或 raw_data_motion/*/raw_poses.csv）",
    )
    parser.add_argument(
        "output_csv",
        nargs="?",
        default=None,
        help=(
            "输出 CSV 路径。若不指定，自动输出到脚本目录下的 output/joint_*_output.csv，"
            "其中 * 由输入文件名中识别的 joint 名称决定。"
        ),
    )
    parser.add_argument(
        "--joint-a",
        default=None,
        help=(
            "第一个 joint 名称（对应 Motive CSV 中的 Rigid Body 名）。"
            "若不指定，自动从输入文件名中解析（如 joint_1.csv → joint_1）。"
        ),
    )
    parser.add_argument(
        "--joint-b",
        default=None,
        help=(
            "第二个 joint 名称（对应 Motive CSV 中的 Rigid Body 名）。"
            "若不指定，自动取 joint-a 的下一个关节（如 joint_1 → joint_2）。"
        ),
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="开启可视化：按时间均匀抽样，绘制 I 与 R_jointB^T * R_jointA",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help=f"可视化采样点数（按时间均匀分布），默认 {DEFAULT_SAMPLE_COUNT}",
    )
    parser.add_argument(
        "--save-fig",
        type=str,
        default=None,
        help="保存当前可视化图片到指定路径（单次模式）",
    )
    parser.add_argument(
        "--batch-all",
        action="store_true",
        help="批量处理 raw_data_motion/joint_1..6，输出全部 CSV 与轴向图",
    )
    parser.add_argument(
        "--raw-data-root",
        type=str,
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..",
            "calibration_output",
            "raw_data_motion",
        ),
        help="批处理输入根目录（默认: ../calibration_output/raw_data_motion）",
    )
    parser.add_argument(
        "--batch-output-root",
        type=str,
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "output",
            "batch_all",
        ),
        help="批处理输出目录（默认: ./output/batch_all）",
    )
    parser.add_argument(
        "--batch-no-show",
        action="store_true",
        help="批处理时只保存图片不弹窗显示",
    )
    args = parser.parse_args()
    # 默认行为：无任何参数时，自动批量输出全部图
    if len(sys.argv) == 1:
        args.batch_all = True

    if args.batch_all:
        try:
            _run_batch_all(
                raw_data_root=args.raw_data_root,
                output_root=args.batch_output_root,
                sample_count=max(1, args.sample_count),
                show_plots=not args.batch_no_show,
            )
        except Exception as e:
            print(f"错误: {e}", file=sys.stderr)
            sys.exit(1)
        return

    # 从文件名自动推导 joint_a / joint_b
    joint_a = args.joint_a
    joint_b = args.joint_b
    if joint_a is None:
        file_joint = _extract_joint_name_from_filename(args.input_csv)
        if file_joint:
            joint_a = file_joint
            print(f"[自动] joint-a 由文件名识别为: {joint_a}")
        else:
            joint_a = "joint_3"
            print(f"[警告] 无法从文件名识别 joint，joint-a 使用默认值: {joint_a}")
    if joint_b is None:
        m = re.search(r"joint_(\d+)$", joint_a)
        if m:
            joint_b = f"joint_{int(m.group(1)) + 1}"
            print(f"[自动] joint-b 由 joint-a 推导为: {joint_b}")
        else:
            joint_b = "joint_4"
            print(f"[警告] 无法推导 joint-b，使用默认值: {joint_b}")

    # 自动生成输出路径：<脚本目录>/output/joint_*_output.csv
    if args.output_csv is not None:
        output_csv = args.output_csv
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_csv = os.path.join(script_dir, "output", f"{joint_a}_output.csv")
        print(f"[自动] 输出文件: {output_csv}")

    output_dir = os.path.dirname(output_csv)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    try:
        extract_quaternion_data(
            args.input_csv,
            output_csv,
            joint_a=joint_a,
            joint_b=joint_b,
            enable_visualize=args.visualize,
            sample_count=max(1, args.sample_count),
            output_image_path=args.save_fig,
            show_plot=True,
        )
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
