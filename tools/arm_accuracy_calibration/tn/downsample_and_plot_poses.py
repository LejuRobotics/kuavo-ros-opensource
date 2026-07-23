#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 pose CSV 每 N 帧拟合成一个点（位置取平均），输出新的 CSV，
并在 3D 图中显示所有下采样后的点。N 可配置，当 N=1 时等价于不做下采样。

假设 CSV 列名为：
timestamp,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w

用法示例（脚本与 CSV 都在 pose_data 目录下）：
    python3 downsample_and_plot_poses.py shoulder_pose.csv effector_pose.csv belly_pose.csv elbow_pose.csv
"""

import os
import sys
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # for 3D projection

DEFAULT_BLOCK_SIZE = 1  # 默认每多少帧合成一个点


def downsample_csv(csv_path, block_size=DEFAULT_BLOCK_SIZE, save=True):
    """读取单个 CSV，每 block_size 帧求平均，返回新的 DataFrame。"""
    df = pd.read_csv(csv_path)

    required_cols = ["timestamp", "pos_x", "pos_y", "pos_z"]
    for c in required_cols:
        if c not in df.columns:
            raise ValueError(f"{csv_path} 缺少列: {c}")

    n = len(df)
    if n == 0:
        print(f"[WARN] {csv_path} 空文件，跳过")
        return None

    # 保证 block_size 至少为 1；为 1 时等价于不做下采样
    block_size = max(1, int(block_size))

    blocks = []
    for start in range(0, n, block_size):
        end = min(start + block_size, n)
        block = df.iloc[start:end]

        # 位置平均
        pos_mean = block[["pos_x", "pos_y", "pos_z"]].mean().values

        # 时间戳取块中间那帧（也可以改成平均）
        mid_idx = (start + end - 1) // 2
        ts = df.iloc[mid_idx]["timestamp"]

        blocks.append({
            "timestamp": float(ts),
            "pos_x": float(pos_mean[0]),
            "pos_y": float(pos_mean[1]),
            "pos_z": float(pos_mean[2]),
        })

    out_df = pd.DataFrame(blocks)

    if save:
        base, ext = os.path.splitext(csv_path)
        out_path = base + "_downsampled" + ext
        out_df.to_csv(out_path, index=False)
        print(f"[INFO] 保存 downsample 结果: {out_path} (共 {len(out_df)} 行)")

    return out_df


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "将 pose CSV 每 N 帧拟合成一个点（位置取平均）并画 3D 图；"
            "通过 --block-size/-b 指定 N，默认为 1（不下采样），设置为更大值可做下采样。"
        )
    )
    parser.add_argument(
        "-b",
        "--block-size",
        type=int,
        default=DEFAULT_BLOCK_SIZE,
        help="每多少帧合成 1 个点，默认为 1（不做下采样）",
    )
    parser.add_argument(
        "csv_paths",
        nargs="+",
        help="一个或多个 pose CSV 文件路径",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    block_size = max(1, args.block_size)
    csv_paths = args.csv_paths

    # 运行时由用户选择是否显示交互 3D 窗口
    resp = input("是否显示交互 3D 窗口用于旋转查看点云？[y/N]: ").strip().lower()
    show_interactive = resp in ("y", "yes", "1")

    # 单独保存图片的目录：相对于脚本位置，向上两级到 arm_accuracy_calibration 目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_dir = os.path.dirname(script_dir)  # 从 functions 目录到 arm_accuracy_calibration 目录
    image_dir = os.path.join(calibration_dir, "image")
    os.makedirs(image_dir, exist_ok=True)

    colors = ["r", "g", "b", "m", "c", "y", "k"]

    for i, csv_path in enumerate(csv_paths):
        if not os.path.isfile(csv_path):
            print(f"[WARN] 文件不存在: {csv_path}，跳过")
            continue

        print(f"[INFO] 处理: {csv_path} (block_size={block_size})")
        df_ds = downsample_csv(csv_path, block_size, save=True)
        if df_ds is None or len(df_ds) == 0:
            continue

        x = df_ds["pos_x"].values
        y = df_ds["pos_y"].values
        z = df_ds["pos_z"].values

        color = colors[i % len(colors)]
        label = os.path.splitext(os.path.basename(csv_path))[0]

        # 日志：确认实际绘制的点数量和坐标范围
        n_points = len(x)
        try:
            print(
                f"[INFO] {label}: 绘制点数 = {n_points}, "
                f"X范围[{x.min():.6f}, {x.max():.6f}], "
                f"Y范围[{y.min():.6f}, {y.max():.6f}], "
                f"Z范围[{z.min():.6f}, {z.max():.6f}]"
            )
        except Exception as e:
            print(f"[WARN] 无法打印 {label} 点云统计信息: {e}")

        # 为当前 CSV 单独生成一张 3D 图并保存
        # 使用更大的 figure 和更高的 DPI 确保所有点都能显示
        fig_single = plt.figure(figsize=(12, 10))
        ax_single = fig_single.add_subplot(111, projection='3d')
        
        # 确保所有点都被绘制：对于大量点，分批绘制但确保所有点都被包含
        # matplotlib 的 scatter3d 在处理大量点时可能会优化，我们分批绘制确保所有点都被绘制
        batch_size = 10000  # 每批绘制 10000 个点
        if n_points > batch_size:
            print(f"[INFO] 点数量较多 ({n_points})，分批绘制确保所有点都被包含...")
            for batch_start in range(0, n_points, batch_size):
                batch_end = min(batch_start + batch_size, n_points)
                batch_x = x[batch_start:batch_end]
                batch_y = y[batch_start:batch_end]
                batch_z = z[batch_start:batch_end]
                # 只在第一批显示 label
                label_text = label if batch_start == 0 else ""
                ax_single.scatter(batch_x, batch_y, batch_z, s=1, c=color, label=label_text, 
                                alpha=0.5, edgecolors='none', rasterized=False)
            print(f"[INFO] 已完成 {n_points} 个点的分批绘制")
        else:
            ax_single.scatter(x, y, z, s=3, c=color, label=label, alpha=0.6, edgecolors='none')
        
        ax_single.set_xlabel("X")
        ax_single.set_ylabel("Y")
        ax_single.set_zlabel("Z")
        ax_single.set_title(f"{label} (every {block_size} frames -> 1 point, total {n_points} points)")
        ax_single.legend()
        single_png = os.path.join(image_dir, f"{label}_downsampled_3d.png")
        fig_single.tight_layout()
        # 提高 DPI 和设置 bbox_inches 确保所有内容都被保存
        fig_single.savefig(single_png, dpi=300, bbox_inches='tight', facecolor='white')
        print(f"[INFO] 单独 3D 图已保存为: {single_png} (包含 {n_points} 个点)")
        if show_interactive:
            # 使用 matplotlib 交互窗口显示，可用鼠标旋转
            plt.show()
        plt.close(fig_single)


if __name__ == "__main__":
    main()

