#!/usr/bin/env python3
"""
PICO Retargeted Pose 时延与抖动分析工具

分析 rosbag 中 /pico/retargeted_pose 话题的:
1. 消息时延 (Message Latency): 消息时间戳与接收时间的差异
2. 消息抖动 (Jitter): 连续消息间隔的波动
3. 频率稳定性: 实际发布频率与期望频率的偏差
4. 数据连续性: 检测丢帧和异常间隔

用法:
    python3 analyze_pico_latency.py <bag_file> [--topic /pico/retargeted_pose] [--output report.txt]
"""

import sys
import os
import argparse
import numpy as np
from datetime import datetime

try:
    import rosbag
except ImportError:
    print("错误: 请安装 rosbag: pip install rosbag")
    print("或者在 ROS 环境中运行此脚本")
    sys.exit(1)


def analyze_latency_and_jitter(bag_file, topic="/pico/retargeted_pose", expected_hz=100.0, output_file=None):
    """
    分析 bag 文件中指定话题的时延和抖动
    
    Args:
        bag_file: rosbag 文件路径
        topic: 要分析的话题名
        expected_hz: 期望的发布频率 (Hz)
        output_file: 输出报告文件路径 (可选)
    
    Returns:
        dict: 包含分析结果的字典
    """
    
    if not os.path.exists(bag_file):
        print(f"错误: 找不到文件 {bag_file}")
        return None
    
    print(f"\n{'='*60}")
    print(f"  PICO 时延与抖动分析")
    print(f"{'='*60}")
    print(f"Bag 文件: {bag_file}")
    print(f"分析话题: {topic}")
    print(f"期望频率: {expected_hz} Hz")
    print(f"{'='*60}\n")
    
    # 数据收集
    msg_timestamps = []      # 消息内部时间戳 (header.stamp)
    recv_timestamps = []     # rosbag 记录时间戳
    latencies = []           # 时延 (recv - msg)
    intervals = []           # 消息间隔
    
    msg_count = 0
    
    try:
        bag = rosbag.Bag(bag_file, 'r')
        
        # 检查话题是否存在
        topics_info = bag.get_type_and_topic_info()
        if topic not in topics_info.topics:
            available_topics = list(topics_info.topics.keys())
            print(f"错误: 话题 '{topic}' 不存在于 bag 文件中")
            print(f"可用话题: {available_topics}")
            bag.close()
            return None
        
        topic_info = topics_info.topics[topic]
        print(f"话题信息:")
        print(f"  消息类型: {topic_info.msg_type}")
        print(f"  消息数量: {topic_info.message_count}")
        print(f"  连接数: {topic_info.connections}")
        print()
        
        prev_msg_time = None
        
        for topic_name, msg, recv_time in bag.read_messages(topics=[topic]):
            msg_count += 1
            
            # 获取消息时间戳 (header.stamp)
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_time = msg.header.stamp.to_sec()
            else:
                # 如果没有 header，使用接收时间
                msg_time = recv_time.to_sec()
            
            recv_time_sec = recv_time.to_sec()
            
            msg_timestamps.append(msg_time)
            recv_timestamps.append(recv_time_sec)
            
            # 计算时延 (接收时间 - 消息时间戳)
            latency = (recv_time_sec - msg_time) * 1000  # 转换为毫秒
            latencies.append(latency)
            
            # 计算消息间隔
            if prev_msg_time is not None:
                interval = (msg_time - prev_msg_time) * 1000  # 转换为毫秒
                intervals.append(interval)
            
            prev_msg_time = msg_time
            
            # 进度显示
            if msg_count % 1000 == 0:
                print(f"\r处理进度: {msg_count} 条消息...", end="", flush=True)
        
        bag.close()
        print(f"\r处理完成: 共 {msg_count} 条消息          ")
        
    except Exception as e:
        print(f"错误: 读取 bag 文件失败: {e}")
        return None
    
    if msg_count < 2:
        print("错误: 消息数量不足，无法进行分析")
        return None
    
    # 转换为 numpy 数组
    latencies = np.array(latencies)
    intervals = np.array(intervals)
    msg_timestamps = np.array(msg_timestamps)
    recv_timestamps = np.array(recv_timestamps)
    
    # 计算统计数据
    expected_interval = 1000.0 / expected_hz  # 期望间隔 (毫秒)
    
    # 时延统计
    latency_stats = {
        'mean': np.mean(latencies),
        'std': np.std(latencies),
        'min': np.min(latencies),
        'max': np.max(latencies),
        'median': np.median(latencies),
        'p95': np.percentile(latencies, 95),
        'p99': np.percentile(latencies, 99),
    }
    
    # 间隔统计
    interval_stats = {
        'mean': np.mean(intervals),
        'std': np.std(intervals),
        'min': np.min(intervals),
        'max': np.max(intervals),
        'median': np.median(intervals),
        'p95': np.percentile(intervals, 95),
        'p99': np.percentile(intervals, 99),
    }
    
    # 抖动 (Jitter): 连续间隔之间的差异
    jitter = np.abs(np.diff(intervals))
    jitter_stats = {
        'mean': np.mean(jitter) if len(jitter) > 0 else 0,
        'std': np.std(jitter) if len(jitter) > 0 else 0,
        'max': np.max(jitter) if len(jitter) > 0 else 0,
        'p95': np.percentile(jitter, 95) if len(jitter) > 0 else 0,
        'p99': np.percentile(jitter, 99) if len(jitter) > 0 else 0,
    }
    
    # 实际频率
    total_duration = msg_timestamps[-1] - msg_timestamps[0]
    actual_hz = (msg_count - 1) / total_duration if total_duration > 0 else 0
    
    # 丢帧检测 (间隔超过期望间隔的 1.5 倍)
    drop_threshold = expected_interval * 1.5
    dropped_frames = np.sum(intervals > drop_threshold)
    drop_rate = dropped_frames / len(intervals) * 100 if len(intervals) > 0 else 0
    
    # 异常间隔检测 (超过 3 倍标准差)
    interval_anomaly_threshold = interval_stats['mean'] + 3 * interval_stats['std']
    anomaly_count = np.sum(intervals > interval_anomaly_threshold)
    
    # 生成报告
    report = []
    report.append("=" * 60)
    report.append("  PICO Retargeted Pose 时延与抖动分析报告")
    report.append("=" * 60)
    report.append(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"Bag 文件: {bag_file}")
    report.append(f"话题: {topic}")
    report.append("")
    
    report.append("-" * 60)
    report.append("【基本信息】")
    report.append("-" * 60)
    report.append(f"  消息总数: {msg_count}")
    report.append(f"  记录时长: {total_duration:.2f} 秒")
    report.append(f"  期望频率: {expected_hz:.1f} Hz")
    report.append(f"  实际频率: {actual_hz:.2f} Hz ({actual_hz/expected_hz*100:.1f}%)")
    report.append("")
    
    report.append("-" * 60)
    report.append("【时延分析 (Latency)】")
    report.append("-" * 60)
    report.append(f"  平均时延: {latency_stats['mean']:.3f} ms")
    report.append(f"  标准差:   {latency_stats['std']:.3f} ms")
    report.append(f"  最小值:   {latency_stats['min']:.3f} ms")
    report.append(f"  最大值:   {latency_stats['max']:.3f} ms")
    report.append(f"  中位数:   {latency_stats['median']:.3f} ms")
    report.append(f"  P95:      {latency_stats['p95']:.3f} ms")
    report.append(f"  P99:      {latency_stats['p99']:.3f} ms")
    report.append("")
    
    report.append("-" * 60)
    report.append("【消息间隔分析 (Interval)】")
    report.append("-" * 60)
    report.append(f"  期望间隔: {expected_interval:.3f} ms")
    report.append(f"  平均间隔: {interval_stats['mean']:.3f} ms")
    report.append(f"  标准差:   {interval_stats['std']:.3f} ms")
    report.append(f"  最小值:   {interval_stats['min']:.3f} ms")
    report.append(f"  最大值:   {interval_stats['max']:.3f} ms")
    report.append(f"  中位数:   {interval_stats['median']:.3f} ms")
    report.append(f"  P95:      {interval_stats['p95']:.3f} ms")
    report.append(f"  P99:      {interval_stats['p99']:.3f} ms")
    report.append("")
    
    report.append("-" * 60)
    report.append("【抖动分析 (Jitter)】")
    report.append("-" * 60)
    report.append(f"  平均抖动: {jitter_stats['mean']:.3f} ms")
    report.append(f"  标准差:   {jitter_stats['std']:.3f} ms")
    report.append(f"  最大抖动: {jitter_stats['max']:.3f} ms")
    report.append(f"  P95:      {jitter_stats['p95']:.3f} ms")
    report.append(f"  P99:      {jitter_stats['p99']:.3f} ms")
    report.append("")
    
    report.append("-" * 60)
    report.append("【数据质量】")
    report.append("-" * 60)
    report.append(f"  疑似丢帧: {dropped_frames} 次 ({drop_rate:.2f}%)")
    report.append(f"  异常间隔: {anomaly_count} 次")
    report.append(f"  丢帧阈值: >{drop_threshold:.2f} ms")
    report.append("")
    
    # 质量评估
    report.append("-" * 60)
    report.append("【质量评估】")
    report.append("-" * 60)
    
    quality_issues = []
    
    # 频率偏差评估
    freq_deviation = abs(actual_hz - expected_hz) / expected_hz * 100
    if freq_deviation > 10:
        quality_issues.append(f"⚠ 频率偏差较大: {freq_deviation:.1f}%")
    elif freq_deviation > 5:
        quality_issues.append(f"△ 频率略有偏差: {freq_deviation:.1f}%")
    else:
        report.append(f"  ✓ 频率稳定: 偏差 {freq_deviation:.1f}%")
    
    # 时延评估
    if latency_stats['p99'] > 50:
        quality_issues.append(f"⚠ P99时延较高: {latency_stats['p99']:.1f} ms")
    elif latency_stats['p99'] > 20:
        quality_issues.append(f"△ P99时延偏高: {latency_stats['p99']:.1f} ms")
    else:
        report.append(f"  ✓ 时延正常: P99 = {latency_stats['p99']:.1f} ms")
    
    # 抖动评估
    if jitter_stats['p99'] > 10:
        quality_issues.append(f"⚠ P99抖动较大: {jitter_stats['p99']:.1f} ms")
    elif jitter_stats['p99'] > 5:
        quality_issues.append(f"△ P99抖动偏大: {jitter_stats['p99']:.1f} ms")
    else:
        report.append(f"  ✓ 抖动正常: P99 = {jitter_stats['p99']:.1f} ms")
    
    # 丢帧评估
    if drop_rate > 5:
        quality_issues.append(f"⚠ 丢帧率较高: {drop_rate:.2f}%")
    elif drop_rate > 1:
        quality_issues.append(f"△ 存在丢帧: {drop_rate:.2f}%")
    else:
        report.append(f"  ✓ 数据连续: 丢帧率 {drop_rate:.2f}%")
    
    for issue in quality_issues:
        report.append(f"  {issue}")
    
    report.append("")
    report.append("=" * 60)
    
    # 输出报告
    report_text = "\n".join(report)
    print(report_text)
    
    # 保存到文件
    if output_file:
        with open(output_file, 'w') as f:
            f.write(report_text)
        print(f"\n报告已保存到: {output_file}")
    
    # 返回结果
    results = {
        'msg_count': msg_count,
        'duration': total_duration,
        'expected_hz': expected_hz,
        'actual_hz': actual_hz,
        'latency': latency_stats,
        'interval': interval_stats,
        'jitter': jitter_stats,
        'dropped_frames': dropped_frames,
        'drop_rate': drop_rate,
        'anomaly_count': anomaly_count,
        'raw_latencies': latencies,
        'raw_intervals': intervals,
        'raw_timestamps': msg_timestamps,
    }
    
    return results


def plot_analysis(results, output_dir=None):
    """
    可视化分析结果
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("警告: matplotlib 未安装，跳过图表生成")
        return
    
    if results is None:
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('PICO Retargeted Pose 时延与抖动分析', fontsize=14, fontweight='bold')
    
    # 1. 时延时序图
    ax1 = axes[0, 0]
    time_axis = results['raw_timestamps'] - results['raw_timestamps'][0]
    ax1.plot(time_axis, results['raw_latencies'], 'b-', alpha=0.7, linewidth=0.5)
    ax1.axhline(y=results['latency']['mean'], color='r', linestyle='--', label=f"Mean: {results['latency']['mean']:.2f} ms")
    ax1.axhline(y=results['latency']['p95'], color='orange', linestyle='--', label=f"P95: {results['latency']['p95']:.2f} ms")
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Latency (ms)')
    ax1.set_title('消息时延 (Latency)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # 2. 时延直方图
    ax2 = axes[0, 1]
    ax2.hist(results['raw_latencies'], bins=50, color='blue', alpha=0.7, edgecolor='black')
    ax2.axvline(x=results['latency']['mean'], color='r', linestyle='--', label=f"Mean: {results['latency']['mean']:.2f} ms")
    ax2.axvline(x=results['latency']['p99'], color='orange', linestyle='--', label=f"P99: {results['latency']['p99']:.2f} ms")
    ax2.set_xlabel('Latency (ms)')
    ax2.set_ylabel('Count')
    ax2.set_title('时延分布')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # 3. 消息间隔时序图
    ax3 = axes[1, 0]
    interval_time = time_axis[1:]  # 间隔比消息少一个
    ax3.plot(interval_time, results['raw_intervals'], 'g-', alpha=0.7, linewidth=0.5)
    expected_interval = 1000.0 / results['expected_hz']
    ax3.axhline(y=expected_interval, color='r', linestyle='--', label=f"Expected: {expected_interval:.2f} ms")
    ax3.axhline(y=results['interval']['mean'], color='orange', linestyle='--', label=f"Mean: {results['interval']['mean']:.2f} ms")
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Interval (ms)')
    ax3.set_title('消息间隔 (Interval)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # 4. 间隔直方图
    ax4 = axes[1, 1]
    ax4.hist(results['raw_intervals'], bins=50, color='green', alpha=0.7, edgecolor='black')
    ax4.axvline(x=expected_interval, color='r', linestyle='--', label=f"Expected: {expected_interval:.2f} ms")
    ax4.axvline(x=results['interval']['mean'], color='orange', linestyle='--', label=f"Mean: {results['interval']['mean']:.2f} ms")
    ax4.set_xlabel('Interval (ms)')
    ax4.set_ylabel('Count')
    ax4.set_title('间隔分布')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_dir:
        plot_file = os.path.join(output_dir, 'latency_analysis.png')
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"图表已保存到: {plot_file}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='PICO Retargeted Pose 时延与抖动分析工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 analyze_pico_latency.py recording.bag
  python3 analyze_pico_latency.py recording.bag --topic /pico/retargeted_pose
  python3 analyze_pico_latency.py recording.bag --hz 90 --output report.txt
  python3 analyze_pico_latency.py recording.bag --plot
        """
    )
    
    parser.add_argument('bag_file', help='ROS bag 文件路径')
    parser.add_argument('--topic', '-t', default='/pico/retargeted_pose',
                        help='要分析的话题 (默认: /pico/retargeted_pose)')
    parser.add_argument('--hz', '-f', type=float, default=100.0,
                        help='期望发布频率 (默认: 100 Hz)')
    parser.add_argument('--output', '-o', help='输出报告文件路径')
    parser.add_argument('--plot', '-p', action='store_true',
                        help='生成可视化图表')
    parser.add_argument('--save-plot', help='保存图表到指定目录')
    
    args = parser.parse_args()
    
    # 分析
    results = analyze_latency_and_jitter(
        args.bag_file,
        topic=args.topic,
        expected_hz=args.hz,
        output_file=args.output
    )
    
    # 可视化
    if results and (args.plot or args.save_plot):
        plot_analysis(results, output_dir=args.save_plot)
    
    return 0 if results else 1


if __name__ == '__main__':
    sys.exit(main())
