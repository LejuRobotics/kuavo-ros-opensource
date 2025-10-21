#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import numpy as np
import os
import sys
import argparse
from collections import defaultdict
import json
from datetime import datetime

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# 定义话题名称
left_arm_topic = '/quest_left_arm_analysis'
right_arm_topic = '/quest_right_arm_analysis'

def extract_data_from_bag(bag_path, topic_name):
    """
    从bag文件中提取指定话题的数据
    返回包含时间戳和数据的列表
    """
    data = []
    try:
        bag = rosbag.Bag(bag_path)
        # 只分析10-15秒的有效数据
        start_time = bag.get_start_time()
        end_time = start_time + 15.0
        valid_start_time = start_time + 10.0
        
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            timestamp = t.to_sec()
            if valid_start_time <= timestamp <= end_time:
                # msg.data 是一个包含6个元素的数组:
                # [z_5th, z_95th, x_95th, hand_to_lower_distance, lower_to_upper_distance, distance_ratio]
                data.append({
                    'timestamp': timestamp,
                    'z_5th': msg.data[0],
                    'z_95th': msg.data[1],
                    'x_95th': msg.data[2],
                    'hand_to_lower_distance': msg.data[3],
                    'lower_to_upper_distance': msg.data[4],
                    'distance_ratio': msg.data[5]
                })
        bag.close()
    except Exception as e:
        print(f"Error reading bag file {bag_path}: {e}")
        return []
    
    return data

def calculate_statistics(data):
    """
    计算数据的统计信息
    """
    if not data:
        return None
    
    # 提取各个字段的值
    z_5th_values = [d['z_5th'] for d in data]
    z_95th_values = [d['z_95th'] for d in data]
    x_95th_values = [d['x_95th'] for d in data]
    hand_to_lower_distances = [d['hand_to_lower_distance'] for d in data]
    lower_to_upper_distances = [d['lower_to_upper_distance'] for d in data]
    distance_ratios = [d['distance_ratio'] for d in data]
    
    # 计算统计信息
    stats = {
        'count': len(data),
        'z_5th': {
            'mean': np.mean(z_5th_values),
            'std': np.std(z_5th_values),
            'min': np.min(z_5th_values),
            'max': np.max(z_5th_values)
        },
        'z_95th': {
            'mean': np.mean(z_95th_values),
            'std': np.std(z_95th_values),
            'min': np.min(z_95th_values),
            'max': np.max(z_95th_values)
        },
        'x_95th': {
            'mean': np.mean(x_95th_values),
            'std': np.std(x_95th_values),
            'min': np.min(x_95th_values),
            'max': np.max(x_95th_values)
        },
        'hand_to_lower_distance': {
            'mean': np.mean(hand_to_lower_distances),
            'std': np.std(hand_to_lower_distances),
            'min': np.min(hand_to_lower_distances),
            'max': np.max(hand_to_lower_distances)
        },
        'lower_to_upper_distance': {
            'mean': np.mean(lower_to_upper_distances),
            'std': np.std(lower_to_upper_distances),
            'min': np.min(lower_to_upper_distances),
            'max': np.max(lower_to_upper_distances)
        },
        'distance_ratio': {
            'mean': np.mean(distance_ratios),
            'std': np.std(distance_ratios),
            'min': np.min(distance_ratios),
            'max': np.max(distance_ratios)
        }
    }
    
    return stats

def compare_devices(device_stats, arm_side):
    """
    比较三个设备的数据一致性，只关注大手和小臂的比例
    """
    comparison = {
        'arm_side': arm_side,
        'differences': {}
    }
    
    # 获取所有设备的键
    devices = list(device_stats.keys())
    
    # 只比较distance_ratio（大手和小臂的比例）
    metric = 'distance_ratio'
    values = []
    for device in devices:
        if device_stats[device] and metric in device_stats[device]:
            values.append(device_stats[device][metric]['mean'])
        else:
            values.append(None)
    
    # 检查是否有None值
    if None in values:
        comparison['differences'][metric] = {
            'values': dict(zip(devices, values)),
            'reason': 'Some devices missing data'
        }
        return comparison
    
    # 计算差异
    max_diff = max(values) - min(values)
    # 如果差异小于平均值的1%，认为是一致的
    avg_value = np.mean(values)
    is_consistent = max_diff < (abs(avg_value) * 0.01) if avg_value != 0 else max_diff < 0.01
    
    comparison['differences'][metric] = {
        'values': dict(zip(devices, [float(round(v, 6)) for v in values])),  # 转换为float以避免JSON序列化问题
        'max_difference': float(round(max_diff, 6))  # 转换为float以避免JSON序列化问题
    }
    
    return comparison

def generate_report(left_arm_stats, right_arm_stats, left_arm_comparison, right_arm_comparison):
    """
    生成分析报告
    """
    report = {
        'timestamp': datetime.now().isoformat(),
        'device_statistics': {
            'left_arm': left_arm_stats,
            'right_arm': right_arm_stats
        },
        'comparison_results': {
            'left_arm': left_arm_comparison,
            'right_arm': right_arm_comparison
        }
    }
    
    return report

def save_report(report, output_path):
    """
    保存报告到文件
    """
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"详细报告已保存到: {output_path}")
    except Exception as e:
        print(f"Error saving report: {e}")

def parse_args():
    """
    解析命令行参数
    """
    parser = argparse.ArgumentParser(description='分析VR设备的手臂数据一致性')
    parser.add_argument('bag_files', nargs='+', help='要分析的bag文件路径')
    parser.add_argument('-o', '--output', default=None, help='输出报告文件路径，默认为脚本目录下的arm_analysis_report.json')
    
    return parser.parse_args()

def main():
    args = parse_args()
    
    # 确定输出路径
    output_path = args.output if args.output else os.path.join(script_dir, 'arm_analysis_report.json')
    
    # 存储所有设备的统计数据
    left_arm_stats = {}
    right_arm_stats = {}
    
    # 分析每个设备的bag文件
    for i, bag_path in enumerate(args.bag_files):
        device_name = f'device{i+1}'
        
        if not os.path.exists(bag_path):
            left_arm_stats[device_name] = None
            right_arm_stats[device_name] = None
            continue
        
        # 提取左臂数据
        left_arm_data = extract_data_from_bag(bag_path, left_arm_topic)
        left_arm_stats[device_name] = calculate_statistics(left_arm_data)
        
        # 提取右臂数据
        right_arm_data = extract_data_from_bag(bag_path, right_arm_topic)
        right_arm_stats[device_name] = calculate_statistics(right_arm_data)
    
    # 比较设备间的一致性
    left_arm_comparison = compare_devices(left_arm_stats, 'left')
    right_arm_comparison = compare_devices(right_arm_stats, 'right')
    
    # 生成报告
    report = generate_report(left_arm_stats, right_arm_stats, left_arm_comparison, right_arm_comparison)
    
    # 保存报告
    save_report(report, output_path)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
