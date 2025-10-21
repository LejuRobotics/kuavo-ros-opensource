#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电流转扭矩转换器
读取 rosbag 中的 /sensors_data_raw 话题，将电流值转换为扭矩值，写入话题 /sensor_data_motor/motor_torque
适用 2025 年 2 月份后批次的机器人的 bag 
"""

import rosbag
import numpy as np
import argparse
import os
from std_msgs.msg import Float64MultiArray

# 电机索引对应的电流到扭矩转换系数 (C2T coefficient)
# 按照关节索引顺序排列的C2T系数
motor_c2t = [2, 1.05, 1.05, 2, 2.1, 2.1, 
             2, 1.05, 1.05, 2, 2.1, 2.1,
             1.05, 5, 2.3, 5, 4.7, 4.7, 4.7,
             1.05, 5, 2.3, 5, 4.7, 4.7, 4.7,
             0.21, 4.7]

class CurrentToTorqueConverter:
    def __init__(self):
        """
        初始化转换器
        
        Args:
            c2t_coefficients: 电机C2T系数列表，如果为None则使用默认系数
            default_c2t: 默认的C2T系数
        """
        self.c2t_coefficients = motor_c2t
    
    def current_to_torque(self, current_data):
        """
        将 sensors_data_raw 中的 joint_torque 电流数据转换为扭矩数据
        
        Args:
            current_data: 电流数据数组
            
        Returns:
            扭矩数据数组
        """
        if len(current_data) != len(self.c2t_coefficients):
            print(f"警告: 电流数据长度({len(current_data)})与C2T系数数量({len(self.c2t_coefficients)})不匹配")
            # 扩展或截断系数数组
            return None
        
        torque_data = []
        # "MOTORS_TYPE":[
        # "PA100_18", "PA100", "PA100", "PA100_18", "CK", "CK",
        # "PA100_18", "PA100", "PA100", "PA100_18", "CK", "CK",
        # "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo",
        # "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo"],
            
            
        for i, current in enumerate(current_data):
            # kuavo-ros-control/src/kuavo_common/include/kuavo_common/common/kuavo_settings.h 
            # 中定义了 ruiwo 电机电流转扭矩系数 CK_C2T = 2.1，所以这里除以 2.1 转化回原始电流
            
            # 13~18 为左臂ruiwo电机数据, 20~25 为右臂ruiwo电机数据
            # 对于这些电机需要先除以2.1转换回原始电流
            if 13 <= i <= 18 or 20 <= i <= 27:
                torque = (current / 2.1) * self.c2t_coefficients[i]
            elif i == 1 or i == 2 or i == 7 or i == 8 or i == 12 or i == 19:
                torque = (current / 1.2) * self.c2t_coefficients[i]
            else:

                # EC 电机 sensors_data_raw 中已经是扭矩值
                torque = current
            torque_data.append(torque)
        
        return np.array(torque_data)

    def torque_to_current(self, torque_data):
        """
        将 sensors_data_raw 中的 joint_torque 电流数据转换为扭矩数据
        
        Args:
            current_data: 电流数据数组
            
        Returns:
            扭矩数据数组
        """
        if len(torque_data) != len(self.c2t_coefficients):
            print(f"警告: 电流数据长度({len(current_data)})与C2T系数数量({len(self.c2t_coefficients)})不匹配")
            # 扩展或截断系数数组
            return None
        
        current_data = []
        # "MOTORS_TYPE":[
        # "PA100_18", "PA100", "PA100", "PA100_18", "CK", "CK",
        # "PA100_18", "PA100", "PA100", "PA100_18", "CK", "CK",
        # "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo",
        # "PA100", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo", "ruiwo"],
            
            
        for i, torque in enumerate(torque_data):
            # kuavo-ros-control/src/kuavo_common/include/kuavo_common/common/kuavo_settings.h 
            # 中定义了 ruiwo 电机电流转扭矩系数 CK_C2T = 2.1，所以这里除以 2.1 转化回原始电流
            
            # 13~18 为左臂ruiwo电机数据, 20~25 为右臂ruiwo电机数据
            # 对于这些电机需要先除以2.1转换回原始电流
            if 13 <= i <= 18 or 20 <= i <= 27:
                current = (torque / 2.1) 
            elif i == 1 or i == 2 or i == 7 or i == 8 or i == 12 or i == 19:
                current = (torque / 1.2) 
            else:

                # EC 电机 sensors_data_raw 中已经是扭矩值
                current = torque / self.c2t_coefficients[i]
            current_data.append(current)
        
        return np.array(current_data)

    def process_rosbag(self, bag_path, output_path=None):
        """
        处理rosbag文件
        
        Args:
            bag_path: rosbag文件路径
            output_path: 输出文件路径
        """
        if not os.path.exists(bag_path):
            print(f"错误: 文件 {bag_path} 不存在")
            return
        
        print(f"正在处理rosbag文件: {bag_path}")
        
        # 存储数据
        timestamps = []
        current_data_list = []
        torque_data_list = []
        
        try:
            
            # 读取原始bag并创建新的bag
            with rosbag.Bag(bag_path, 'r') as bag, rosbag.Bag(output_path, 'w') as new_bag:
                # 读取 /sensor_data_motor/motor_cur 话题
                topic_name = "/sensors_data_raw"
                
                for topic, msg, t in bag.read_messages(topics=[topic_name]):
                    timestamp = t.to_sec()
                    joint_torque = np.array(msg.joint_data.joint_torque)
                    
                    # 转换为扭矩
                    torque_data = self.current_to_torque(joint_torque)
                    current_data = self.torque_to_current(joint_torque)
                    if torque_data is None:
                        continue
                    
                    # 创建扭矩消息并保存到bag
                    torque_msg = Float64MultiArray()
                    torque_msg.data = torque_data.tolist()
                    new_bag.write('/sensor_data_motor/motor_torque', torque_msg, t)

                    current_msg = Float64MultiArray()
                    current_msg.data = current_data.tolist()
                    new_bag.write('/sensor_data_motor/motor_current', current_msg, t)

                    timestamps.append(timestamp)
                    current_data_list.append(joint_torque)
                    torque_data_list.append(torque_data)
                
                # 复制原始bag中的所有话题到新bag
                for topic, msg, t in bag.read_messages():
                    new_bag.write(topic, msg, t)
            
            # # 替换原始文件
            # import shutil
            # shutil.move(temp_bag_path, bag_path)
            
            if not timestamps:
                print(f"警告: 在话题 {topic_name} 中没有找到数据")
                return
            
            print(f"成功读取 {len(timestamps)} 条数据")
            print(f"已将扭矩数据保存到bag文件: {output_path}")

        except Exception as e:
            print(f"处理rosbag时出错: {e}")
            # 清理临时文件
            if os.path.exists(output_path):
                os.remove(output_path)
    
def main():
    parser = argparse.ArgumentParser(description='将rosbag中的电流数据转换为扭矩数据')
    parser.add_argument('-bag_file', help='输入的rosbag文件路径')
    parser.add_argument('-o', '--output', help='输出文件路径')
    
    args = parser.parse_args()
    
    # 创建转换器
    converter = CurrentToTorqueConverter()
    
    # 处理rosbag
    converter.process_rosbag(
        bag_path=args.bag_file,
        output_path=args.output
    )

if __name__ == "__main__":
    main() 