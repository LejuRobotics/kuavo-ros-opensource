#!/usr/bin/env python3
"""
将 PICO 录制的 rosbag 转换为 VMP 期望的 79 维 bin 文件格式
输入: /pico/retargeted_pose 话题的 bag 文件
输出: 
  - 79 维 float32 二进制文件 (.bin)
  - npz 文件 (.npz) 包含:
    * vmp_data: 79维VMP数据 [N, 79]
    * base_pose: 机身位姿 [N, 7] (x, y, z, qx, qy, qz, qw in world frame)

79 维数据格式 (参考 vmp_param.info 中的 refTask 配置):
  [0]       : h (机身高度 1D)
  [1-6]     : theta (旋转矩阵前2列 6D，column-major: R00,R10,R20,R01,R11,R21)
  [7-12]    : v (机身速度 6D: linear_xyz, angular_xyz)
  [13-39]   : q (关节位置 27D: Lleg6 + Rleg6 + waist1 + Larm7 + Rarm7)
  [40-66]   : q_dot (关节速度 27D)
  [67-78]   : p (末端位置 12D: 左手3D, 右手3D, 左脚3D, 右脚3D)
"""

import rosbag
import numpy as np
import sys
import os
from scipy.spatial.transform import Rotation


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    四元数转旋转矩阵 (只需要前2列，按列存储 column-major)
    输出格式: [R00, R10, R20, R01, R11, R21] (列0 + 列1)
    """
    # 归一化
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm > 1e-8:
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    # 计算完整的 3x3 旋转矩阵
    r00 = 1.0 - 2.0 * (qy*qy + qz*qz)
    r01 = 2.0 * (qx*qy - qz*qw)
    r02 = 2.0 * (qx*qz + qy*qw)
    r10 = 2.0 * (qx*qy + qz*qw)
    r11 = 1.0 - 2.0 * (qx*qx + qz*qz)
    r12 = 2.0 * (qy*qz - qx*qw)
    r20 = 2.0 * (qx*qz - qy*qw)
    r21 = 2.0 * (qy*qz + qx*qw)
    r22 = 1.0 - 2.0 * (qx*qx + qy*qy)
    
    # 按列存储: 前2列 (column-major)
    # 列0: [R00, R10, R20]
    # 列1: [R01, R11, R21]
    return [r00, r10, r20, r01, r11, r21]


def convert_msg_to_79d(msg):
    """
    将 picoPoseRetarget 消息转换为 79 维数据
    关节布局 (27 DOF): Lleg(6) + Rleg(6) + waist(1) + Larm(7) + Rarm(7)
    """
    vmp_data = np.zeros(79, dtype=np.float32)
    
    # [0] 机身高度
    vmp_data[0] = msg.base_link_pose.position.z
    
    # [1-6] 旋转矩阵前2列 (从四元数转换，column-major 存储)
    # 格式: [R00, R10, R20, R01, R11, R21] (列0 + 列1)
    qx = msg.base_link_pose.orientation.x
    qy = msg.base_link_pose.orientation.y
    qz = msg.base_link_pose.orientation.z
    qw = msg.base_link_pose.orientation.w
    rot_matrix_6d = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    vmp_data[1:7] = rot_matrix_6d
    
    # [7-12] 机身速度 (6D)
    if len(msg.base_velocity) >= 6:
        vmp_data[7:13] = msg.base_velocity[:6]
    
    # [13-39] 关节位置 (27D)
    joint_count = min(len(msg.joint_position), 27)
    if joint_count > 0:
        vmp_data[13:13 + joint_count] = msg.joint_position[:joint_count]
    
    # [40-66] 关节速度 (27D)
    vel_count = min(len(msg.joint_velocity), 27)
    if vel_count > 0:
        vmp_data[40:40 + vel_count] = msg.joint_velocity[:vel_count]
    
    # [67-78] 末端位置 (12D)
    # PICO 输出顺序: [left_foot, right_foot, left_hand, right_hand]
    # VMP 期望顺序: [left_hand, right_hand, left_foot, right_foot]
    # 注意：GMR发布的end_effector_poses是世界坐标系下的绝对位置
    # VMP需要的是世界坐标系下的本体坐标系（相对于base_link的位置）
    # 转换公式：P_relative = P_world_end - P_world_base
    if len(msg.end_effector_poses) >= 4:
        # 获取base_link在世界坐标系的位置
        base_x = msg.base_link_pose.position.x
        base_y = msg.base_link_pose.position.y
        base_z = msg.base_link_pose.position.z
        
        # 左手 (67-69) - 相对于base_link的位置
        vmp_data[67] = msg.end_effector_poses[2].x - base_x
        vmp_data[68] = msg.end_effector_poses[2].y - base_y
        vmp_data[69] = msg.end_effector_poses[2].z - base_z
        
        # 右手 (70-72) - 相对于base_link的位置
        vmp_data[70] = msg.end_effector_poses[3].x - base_x
        vmp_data[71] = msg.end_effector_poses[3].y - base_y
        vmp_data[72] = msg.end_effector_poses[3].z - base_z
        
        # 左脚 (73-75) - 相对于base_link的位置
        vmp_data[73] = msg.end_effector_poses[0].x - base_x
        vmp_data[74] = msg.end_effector_poses[0].y - base_y
        vmp_data[75] = msg.end_effector_poses[0].z - base_z
        
        # 右脚 (76-78) - 相对于base_link的位置
        vmp_data[76] = msg.end_effector_poses[1].x - base_x
        vmp_data[77] = msg.end_effector_poses[1].y - base_y
        vmp_data[78] = msg.end_effector_poses[1].z - base_z
    
    return vmp_data

def convert_bag_to_bin(bag_file, output_bin_file, topic="/pico/retargeted_pose"):
    """
    从 rosbag 提取 /pico/retargeted_pose 话题数据，转换为 79 维 bin 和 npz 文件
    npz 文件包含：
      - 'vmp_data': 79维VMP数据 [N, 79]
      - 'base_pose': 机身位姿 [N, 7] (x, y, z, qx, qy, qz, qw in world frame)
    """
    if not os.path.exists(bag_file):
        print(f"错误: 找不到文件 {bag_file}")
        return False
    
    print(f"正在读取 bag 文件: {bag_file}")
    print(f"提取话题: {topic}")
    
    all_frames = []
    base_poses = []  # 存储 [x, y, z, qx, qy, qz, qw]
    
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
            frame_79d = convert_msg_to_79d(msg)
            all_frames.append(frame_79d)
            
            # 提取机身位姿 (世界坐标系): 位置 + 四元数
            base_pose = [
                msg.base_link_pose.position.x,
                msg.base_link_pose.position.y,
                msg.base_link_pose.position.z,
                msg.base_link_pose.orientation.x,
                msg.base_link_pose.orientation.y,
                msg.base_link_pose.orientation.z,
                msg.base_link_pose.orientation.w
            ]
            base_poses.append(base_pose)
        
        bag.close()
        
        if len(all_frames) == 0:
            print("错误: 未提取到任何数据")
            return False
        
        # 转换为 numpy 数组
        vmp_data_array = np.array(all_frames, dtype=np.float32)
        base_pose_array = np.array(base_poses, dtype=np.float32)
        
        print(f"提取了 {len(all_frames)} 帧数据")
        print(f"VMP 数据形状: {vmp_data_array.shape} (应为 [N, 79])")
        print(f"机身位姿形状: {base_pose_array.shape} (应为 [N, 7])")
        print(f"总数据大小: {(vmp_data_array.nbytes + base_pose_array.nbytes) / 1024:.2f} KB")
        
        # 保存为二进制文件 (仅 VMP 数据)
        vmp_data_array.tofile(output_bin_file)
        print(f"✓ 已保存 bin 文件: {output_bin_file}")
        
        # 验证 bin 文件
        file_size = os.path.getsize(output_bin_file)
        expected_size = len(all_frames) * 79 * 4  # 4 bytes per float32
        if file_size == expected_size:
            print(f"✓ Bin 文件大小验证通过: {file_size} bytes")
        else:
            print(f"⚠ Bin 文件大小异常: {file_size} bytes (期望 {expected_size} bytes)")
        
        # 保存为 npz 文件 (包含 VMP 数据 + 机身位姿)
        output_npz_file = output_bin_file.replace('.bin', '.npz')
        np.savez(output_npz_file, 
                 vmp_data=vmp_data_array,
                 root=base_pose_array)
        print(f"✓ 已保存 npz 文件: {output_npz_file}")
        
        # 验证 npz 文件
        npz_size = os.path.getsize(output_npz_file)
        print(f"✓ Npz 文件大小: {npz_size} bytes")
        print(f"  - vmp_data: {vmp_data_array.shape} ({vmp_data_array.nbytes} bytes)")
        print(f"  - base_pose: {base_pose_array.shape} ({base_pose_array.nbytes} bytes)")
        
        return True
        
    except Exception as e:
        print(f"错误: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='将 PICO 录制的 rosbag 转换为 VMP 期望的 79 维 bin 文件格式'
    )
    parser.add_argument('bag_file', help='输入的 rosbag 文件')
    parser.add_argument('output_bin_file', nargs='?', default=None, 
                        help='输出的 bin 文件 (可选，默认与输入同名)')
    parser.add_argument('--topic', '-t', default='/pico/retargeted_pose',
                        help='要提取的话题 (默认: /pico/retargeted_pose)')
    
    args = parser.parse_args()
    
    bag_file = args.bag_file
    topic = args.topic
    
    # 自动生成输出文件名
    if args.output_bin_file:
        output_bin_file = args.output_bin_file
    else:
        base_name = os.path.splitext(bag_file)[0]
        output_bin_file = f"{base_name}.bin"
    
    print("=" * 60)
    print("PICO Bag → VMP Bin 转换工具")
    print("=" * 60)
    print(f"话题: {topic}")
    
    success = convert_bag_to_bin(bag_file, output_bin_file, topic=topic)
    
    if success:
        print("=" * 60)
        print("转换完成！")
        print(f"Bin 文件: {output_bin_file}")
        print(f"Npz 文件: {output_bin_file.replace('.bin', '.npz')}")
        print("")
        print("Npz 文件包含:")
        print("  - vmp_data: [N, 79] VMP 79维数据")
        print("  - base_pose: [N, 7] 机身位姿 (x, y, z, qx, qy, qz, qw)")
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
