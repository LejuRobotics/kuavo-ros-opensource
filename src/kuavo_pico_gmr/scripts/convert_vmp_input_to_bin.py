#!/usr/bin/env python3
"""
将 VMP 输入数据话题转换为 bin/npz 文件
输入: /vmp/input_data 话题的 bag 文件 (std_msgs/Float32MultiArray)
输出: 
  - 77 维 float32 二进制文件 (.bin) - VMP格式，用于VMP回放
  - npz 文件 (.npz) 包含:
    * vmp_data [N, 77] - VMP输入数据
    * base_pose [N, 3] - base_link世界坐标位置 (x, y, z)
    * timestamps [N] - 时间戳

80 维数据格式 (话题发布格式):
  [0]       : h (机身高度 1D)
  [1-6]     : theta (旋转矩阵前2列 6D)
  [7-12]    : v (机身速度 6D)
  [13-38]   : q (关节位置 26D)
  [39-64]   : q_dot (关节速度 26D)
  [65-76]   : p (末端位置 12D，相对于base_link)
  [77-79]   : base_world_pose (base_link世界坐标位置 x, y, z) - 仅用于录制调试
"""

import rosbag
import numpy as np
import sys
import os
import argparse


def convert_bag_to_bin(bag_file, output_bin_file, topic="/vmp/input_data"):
    """
    从 rosbag 提取 /vmp/input_data 话题数据，转换为 bin 和 npz 文件
    话题数据为80维：77维VMP数据 + 3维base_link世界坐标位置
    bin文件只保存77维VMP数据（用于VMP回放）
    npz文件额外保存base_pose用于调试分析
    """
    if not os.path.exists(bag_file):
        print(f"错误: 找不到文件 {bag_file}")
        return False
    
    print(f"正在读取 bag 文件: {bag_file}")
    print(f"提取话题: {topic}")
    
    all_vmp_frames = []      # 77维VMP数据
    all_base_poses = []      # 3维base_link世界坐标位置
    timestamps = []
    
    try:
        bag = rosbag.Bag(bag_file, 'r')
        
        # 统计话题消息数量
        info = bag.get_type_and_topic_info()
        if topic not in info.topics:
            print(f"错误: bag 文件中未找到话题 {topic}")
            available_topics = list(info.topics.keys())
            print(f"可用话题: {available_topics}")
            bag.close()
            return False
        
        msg_count = info.topics[topic].message_count
        print(f"找到 {msg_count} 条消息")
        
        # 提取数据
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            # Float32MultiArray 消息的数据在 msg.data 中
            frame_data = np.array(msg.data, dtype=np.float32)
            
            # 支持77维（旧格式）和80维（新格式）
            if len(frame_data) == 80:
                # 新格式：77维VMP数据 + 3维base_link世界坐标位置
                vmp_data = frame_data[:77]
                base_pose = frame_data[77:80]
            elif len(frame_data) == 77:
                # 旧格式：只有77维VMP数据，base_pose填0
                vmp_data = frame_data
                base_pose = np.zeros(3, dtype=np.float32)
                print(f"警告: 检测到旧格式数据(77维)，base_pose将填充0")
            else:
                print(f"警告: 帧数据维度异常 (实际: {len(frame_data)}，期望: 77或80)")
                continue
            
            all_vmp_frames.append(vmp_data)
            all_base_poses.append(base_pose)
            timestamps.append(t.to_sec())
        
        bag.close()
        
        if len(all_vmp_frames) == 0:
            print("错误: 未提取到任何数据")
            return False
        
        # 转换为 numpy 数组
        vmp_data_array = np.array(all_vmp_frames, dtype=np.float32)
        base_pose_array = np.array(all_base_poses, dtype=np.float32)
        timestamps_array = np.array(timestamps, dtype=np.float64)
        
        print(f"提取了 {len(all_vmp_frames)} 帧数据")
        print(f"VMP 数据形状: {vmp_data_array.shape} (应为 [N, 77])")
        print(f"Base pose 数据形状: {base_pose_array.shape} (应为 [N, 3])")
        print(f"VMP数据大小: {vmp_data_array.nbytes / 1024:.2f} KB")
        
        # 计算频率
        if len(timestamps) >= 2:
            duration = timestamps[-1] - timestamps[0]
            avg_freq = (len(timestamps) - 1) / duration if duration > 0 else 0
            print(f"录制时长: {duration:.2f} 秒")
            print(f"平均频率: {avg_freq:.1f} Hz")
        
        # 保存为二进制文件
        vmp_data_array.tofile(output_bin_file)
        print(f"✓ 已保存 bin 文件: {output_bin_file}")
        
        # 验证 bin 文件
        file_size = os.path.getsize(output_bin_file)
        expected_size = len(all_vmp_frames) * 77 * 4  # 4 bytes per float32
        if file_size == expected_size:
            print(f"✓ Bin 文件大小验证通过: {file_size} bytes")
        else:
            print(f"⚠ Bin 文件大小异常: {file_size} bytes (期望 {expected_size} bytes)")
        
        # 保存为 npz 文件（包含 base_pose）
        output_npz_file = output_bin_file.replace('.bin', '.npz')
        np.savez(output_npz_file, 
                 vmp_data=vmp_data_array,
                 root=base_pose_array,
                 timestamps=timestamps_array)
        print(f"✓ 已保存 npz 文件: {output_npz_file}")
        
        return True
        
    except Exception as e:
        print(f"错误: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description='将 VMP 输入数据话题转换为 bin/npz 文件'
    )
    parser.add_argument('bag_file', help='输入的 rosbag 文件')
    parser.add_argument('output_bin_file', nargs='?', default=None, 
                        help='输出的 bin 文件 (可选)')
    parser.add_argument('--topic', '-t', default='/vmp/input_data',
                        help='要提取的话题 (默认: /vmp/input_data)')
    
    args = parser.parse_args()
    
    bag_file = args.bag_file
    topic = args.topic
    
    # 自动生成输出文件名
    if args.output_bin_file:
        output_bin_file = args.output_bin_file
    else:
        base_name = os.path.splitext(bag_file)[0]
        output_bin_file = f"{base_name}_vmp_input.bin"
    
    print("=" * 60)
    print("VMP Input Data → Bin 转换工具")
    print("=" * 60)
    print(f"话题: {topic}")
    
    success = convert_bag_to_bin(bag_file, output_bin_file, topic=topic)
    
    if success:
        print("=" * 60)
        print("转换完成！")
        print(f"Bin 文件: {output_bin_file}")
        print(f"Npz 文件: {output_bin_file.replace('.bin', '.npz')}")
        print("")
        print(f"可以在 VMP 配置中使用: vmpTaskDataFile {os.path.basename(output_bin_file)}")
        print("=" * 60)
        sys.exit(0)
    else:
        print("=" * 60)
        print("转换失败")
        print("=" * 60)
        sys.exit(1)


if __name__ == "__main__":
    main()
