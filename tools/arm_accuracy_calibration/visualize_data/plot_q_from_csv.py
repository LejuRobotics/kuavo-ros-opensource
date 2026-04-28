#!/usr/bin/env python3
import argparse
import csv
import os

import matplotlib.pyplot as plt

# 默认 CSV 文件名
DEFAULT_CSV_FILE = "sensors_data_arm_j3_kuavo_90deg.csv"


def parse_args():
    parser = argparse.ArgumentParser(
        description=f"从 {DEFAULT_CSV_FILE} 中绘制 q[12], q[15], q[18] 曲线（单位：deg）"
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=DEFAULT_CSV_FILE,
        help=f"CSV 文件路径（默认：当前目录下的 {DEFAULT_CSV_FILE}）",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    csv_path = args.csv_path

    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"找不到 CSV 文件: {csv_path}")

    times = []
    q12_deg = []
    q15_deg = []
    q18_deg = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)

        # 期望的列名（与 arm_j3_kuavo.py 中写入的表头一致）
        col_time = "sensor_time"
        col_q12 = "q_12_deg"
        col_q15 = "q_15_deg"
        col_q18 = "q_18_deg"

        if col_q12 not in reader.fieldnames or col_q15 not in reader.fieldnames or col_q18 not in reader.fieldnames:
            raise KeyError(
                f"CSV 中未找到列 {col_q12}、{col_q15} 或 {col_q18}，实际列名: {reader.fieldnames}"
            )

        for row in reader:
            try:
                t = float(row[col_time])
                q12 = float(row[col_q12])
                q15 = float(row[col_q15])
                q18 = float(row[col_q18])
            except (ValueError, KeyError):
                # 跳过无法解析的行
                continue

            times.append(t)
            q12_deg.append(q12)
            q15_deg.append(q15)
            q18_deg.append(q18)

    if not times:
        raise RuntimeError("CSV 中没有有效数据可用于绘图")

    # 数据清洗：裁剪掉前10秒和后10秒的数据
    original_count = len(times)
    min_time = min(times)
    max_time = max(times)
    time_start = min_time + 10
    time_end = max_time - 10
    
    if time_end <= time_start:
        raise RuntimeError(
            f"数据总时长不足20秒（总时长: {max_time - min_time:.2f}s），无法裁剪前10秒和后10秒"
        )
    
    # 过滤数据，只保留时间在 [time_start, time_end] 范围内的数据
    filtered_data = [
        (t, q12, q15, q18)
        for t, q12, q15, q18 in zip(times, q12_deg, q15_deg, q18_deg)
        if time_start <= t <= time_end
    ]
    
    if not filtered_data:
        raise RuntimeError(f"数据清洗后没有有效数据（时间范围: {time_start:.2f}s - {time_end:.2f}s）")
    
    # 解包过滤后的数据
    times, q12_deg, q15_deg, q18_deg = zip(*filtered_data)
    times = list(times)
    q12_deg = list(q12_deg)
    q15_deg = list(q15_deg)
    q18_deg = list(q18_deg)
    
    print(f"数据清洗完成：原始数据 {original_count} 个点，清洗后 {len(times)} 个点（时间范围: {time_start:.2f}s - {time_end:.2f}s）")

    # 计算每个数据的均值
    mean_q12 = sum(q12_deg) / len(q12_deg) if q12_deg else 0
    mean_q15 = sum(q15_deg) / len(q15_deg) if q15_deg else 0
    mean_q18 = sum(q18_deg) / len(q18_deg) if q18_deg else 0

    # 创建3个子图，垂直排列
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    
    # 绘制 q[12]
    axes[0].plot(times, q12_deg, label="q[12] (deg)")
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("angle (deg)")
    axes[0].set_title("q[12] vs time")
    axes[0].set_ylim(mean_q12 - 0.05, mean_q12 + 0.05)
    axes[0].legend()
    axes[0].grid(True)
    
    # 绘制 q[15]
    axes[1].plot(times, q15_deg, label="q[15] (deg)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("angle (deg)")
    axes[1].set_title("q[15] vs time")
    axes[1].set_ylim(mean_q15 - 0.05, mean_q15 + 0.05)
    axes[1].legend()
    axes[1].grid(True)
    
    # 绘制 q[18]
    axes[2].plot(times, q18_deg, label="q[18] (deg)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("angle (deg)")
    axes[2].set_title("q[18] vs time")
    axes[2].set_ylim(mean_q18 - 0.05, mean_q18 + 0.05)
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    
    # 自动保存图片，基于CSV文件名生成图片文件名
    csv_basename = os.path.splitext(os.path.basename(csv_path))[0]
    output_path = f"{csv_basename}_plot.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"图片已保存至: {output_path}")
    
    plt.show()


if __name__ == "__main__":
    # print: “输出数据对应手臂pitch的三个电机，与水平仪实测图片匹配使用” in green color
    print("\033[92m输出数据对应手臂pitch的三个电机，与水平仪实测图片匹配使用\033[0m")
    main()

