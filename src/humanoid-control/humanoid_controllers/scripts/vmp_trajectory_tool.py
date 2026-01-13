#!/usr/bin/env python3
"""
VMP轨迹文件工具 - 查看和裁剪VMP二进制轨迹文件

用法:
    # 查看轨迹信息
    python3 vmp_trajectory_tool.py info <file.bin>

    # 裁剪轨迹 (去掉前100帧和后50帧)
    python3 vmp_trajectory_tool.py trim <input.bin> <output.bin> --trim-start 100 --trim-end 50

    # 只保留指定范围的帧
    python3 vmp_trajectory_tool.py trim <input.bin> <output.bin> --start-frame 100 --end-frame 500
"""

import argparse
import numpy as np
import os
import sys

# VMP特征维度 (固定为77)
IN_C = 77

# 特征索引定义
FEATURE_RANGES = {
    "h (高度)": (0, 1),
    "theta (6D方向)": (1, 7),
    "v (速度)": (7, 13),
    "q (关节位置)": (13, 39),
    "q_dot (关节速度)": (39, 65),
    "p (末端位置)": (65, 77),
}

JOINT_NAMES = [
    "leg_l1", "leg_l2", "leg_l3", "leg_l4", "leg_l5", "leg_l6",
    "leg_r1", "leg_r2", "leg_r3", "leg_r4", "leg_r5", "leg_r6",
    "arm_l1", "arm_l2", "arm_l3", "arm_l4", "arm_l5", "arm_l6", "arm_l7",
    "arm_r1", "arm_r2", "arm_r3", "arm_r4", "arm_r5", "arm_r6", "arm_r7"
]

TRACK_LINKS = ["zarm_l5_link", "zarm_r5_link", "leg_l6_link", "leg_r6_link"]


def load_trajectory(file_path: str) -> tuple:
    """加载VMP轨迹文件"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件不存在: {file_path}")

    file_size = os.path.getsize(file_path)
    total_frames = file_size // (IN_C * 4)  # 4 bytes per float32

    if total_frames == 0:
        raise ValueError(f"文件为空或格式错误: {file_path}")

    data = np.fromfile(file_path, dtype=np.float32)
    data = data.reshape(total_frames, IN_C)

    return data, total_frames


def save_trajectory(file_path: str, data: np.ndarray):
    """保存VMP轨迹文件"""
    data.astype(np.float32).tofile(file_path)
    print(f"已保存: {file_path}")
    print(f"  帧数: {data.shape[0]}")
    print(f"  文件大小: {os.path.getsize(file_path)} bytes")


def print_info(file_path: str, verbose: bool = False):
    """打印轨迹文件信息"""
    data, total_frames = load_trajectory(file_path)
    file_size = os.path.getsize(file_path)

    print("=" * 60)
    print(f"VMP轨迹文件: {os.path.basename(file_path)}")
    print("=" * 60)
    print(f"文件路径: {file_path}")
    print(f"文件大小: {file_size:,} bytes ({file_size/1024:.2f} KB)")
    print(f"特征维度: {IN_C}")
    print(f"总帧数: {total_frames}")
    print(f"轨迹时长: {total_frames / 100:.2f} 秒 (100Hz)")
    print()

    # 特征维度分布
    print("特征维度分布:")
    print("-" * 40)
    for name, (start, end) in FEATURE_RANGES.items():
        print(f"  {name}: [{start}:{end}], 维度 {end - start}")
    print()

    # 高度变化
    h_data = data[:, 0]
    print("高度 (h) 统计:")
    print("-" * 40)
    print(f"  初始: {h_data[0]:.4f} m")
    print(f"  最小: {h_data.min():.4f} m")
    print(f"  最大: {h_data.max():.4f} m")
    print(f"  平均: {h_data.mean():.4f} m")
    print(f"  末尾: {h_data[-1]:.4f} m")
    print()

    if verbose:
        # 关节位置
        q_start, q_end = FEATURE_RANGES["q (关节位置)"]
        print("关节位置 (q) - 第1帧 vs 最后帧:")
        print("-" * 40)
        for i, name in enumerate(JOINT_NAMES):
            first = data[0, q_start + i]
            last = data[-1, q_start + i]
            diff = last - first
            print(f"  {name:10s}: {first:8.4f} -> {last:8.4f}  (diff: {diff:+.4f})")
        print()

        # 末端位置
        p_start, p_end = FEATURE_RANGES["p (末端位置)"]
        print("末端位置 (p):")
        print("-" * 40)
        for i, link in enumerate(TRACK_LINKS):
            idx = p_start + i * 3
            print(f"  {link}:")
            print(f"    第1帧:   ({data[0, idx]:.4f}, {data[0, idx+1]:.4f}, {data[0, idx+2]:.4f})")
            print(f"    最后帧: ({data[-1, idx]:.4f}, {data[-1, idx+1]:.4f}, {data[-1, idx+2]:.4f})")


def trim_trajectory(input_path: str, output_path: str,
                    trim_start: int = 0, trim_end: int = 0,
                    start_frame: int = None, end_frame: int = None):
    """裁剪轨迹文件"""
    data, total_frames = load_trajectory(input_path)

    print(f"原始轨迹: {total_frames} 帧")

    # 确定裁剪范围
    if start_frame is not None or end_frame is not None:
        # 使用绝对帧号
        actual_start = start_frame if start_frame is not None else 0
        actual_end = end_frame if end_frame is not None else total_frames
    else:
        # 使用裁剪数量
        actual_start = trim_start
        actual_end = total_frames - trim_end

    # 验证范围
    if actual_start < 0:
        actual_start = 0
    if actual_end > total_frames:
        actual_end = total_frames
    if actual_start >= actual_end:
        raise ValueError(f"无效的裁剪范围: [{actual_start}, {actual_end})")

    # 裁剪
    trimmed_data = data[actual_start:actual_end]
    new_frames = trimmed_data.shape[0]

    print(f"裁剪范围: [{actual_start}, {actual_end})")
    print(f"去掉前: {actual_start} 帧")
    print(f"去掉后: {total_frames - actual_end} 帧")
    print(f"新轨迹: {new_frames} 帧 ({new_frames / 100:.2f} 秒)")

    # 保存
    save_trajectory(output_path, trimmed_data)

    return new_frames


def main():
    parser = argparse.ArgumentParser(
        description="VMP轨迹文件工具 - 查看和裁剪",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 查看轨迹信息
  %(prog)s info task_data_yc_06.bin

  # 详细信息
  %(prog)s info task_data_yc_06.bin -v

  # 去掉前100帧和后50帧
  %(prog)s trim input.bin output.bin --trim-start 100 --trim-end 50

  # 只保留第200-800帧
  %(prog)s trim input.bin output.bin --start-frame 200 --end-frame 800
"""
    )

    subparsers = parser.add_subparsers(dest="command", help="子命令")

    # info 子命令
    info_parser = subparsers.add_parser("info", help="显示轨迹文件信息")
    info_parser.add_argument("file", help="轨迹文件路径 (.bin)")
    info_parser.add_argument("-v", "--verbose", action="store_true",
                             help="显示详细信息（关节位置等）")

    # trim 子命令
    trim_parser = subparsers.add_parser("trim", help="裁剪轨迹文件")
    trim_parser.add_argument("input", help="输入轨迹文件")
    trim_parser.add_argument("output", help="输出轨迹文件")
    trim_parser.add_argument("--trim-start", type=int, default=0,
                             help="去掉开头的帧数 (默认: 0)")
    trim_parser.add_argument("--trim-end", type=int, default=0,
                             help="去掉结尾的帧数 (默认: 0)")
    trim_parser.add_argument("--start-frame", type=int, default=None,
                             help="起始帧号 (与 --trim-start 互斥)")
    trim_parser.add_argument("--end-frame", type=int, default=None,
                             help="结束帧号 (与 --trim-end 互斥)")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    try:
        if args.command == "info":
            print_info(args.file, args.verbose)
        elif args.command == "trim":
            trim_trajectory(
                args.input, args.output,
                trim_start=args.trim_start,
                trim_end=args.trim_end,
                start_frame=args.start_frame,
                end_frame=args.end_frame
            )
    except Exception as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
