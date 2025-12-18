#!/usr/bin/env python3

import rospy
import sys
import math
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 设置非交互式后端，避免GUI相关错误
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from kuavo_msgs.srv import changeTorsoCtrlMode, changeArmCtrlMode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from kuavo_msgs.msg import sensorsData
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import os

class DataCollector:
    def __init__(self):
        self.base_positions = []
        self.com_positions = []
        self.feet_center_positions = []
        self.joint_positions = []
        self.timestamps = []  # 添加时间戳记录
        self.start_time = None
        self.is_collecting = False
        self.use_direct_foot_data = False
        
        # 为当前测试保存数据（用于单个测试分析）
        self.current_test_base_positions = []
        self.current_test_com_positions = []
        self.current_test_feet_center_positions = []
        self.current_test_timestamps = []
        
        # TF监听器用于获取精确的脚部位置（备用方法）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 创建订阅者
        self.base_sub = rospy.Subscriber('/humanoid_controller/optimizedState_mrt/base/pos_xyz', 
                                       Float64MultiArray, 
                                       self.base_callback)
        self.com_sub = rospy.Subscriber('/humanoid_controller/com/r', 
                                      Float64MultiArray, 
                                      self.com_callback)
        
        # 尝试订阅直接的脚部位置话题
        try:
            self.foot_center_sub = rospy.Subscriber('/humanoid/current_foot_center_pose', 
                                                  Float32MultiArray, 
                                                  self.foot_center_callback)
            self.use_direct_foot_data = True
            rospy.loginfo("使用直接的脚部中心位置数据")
        except:
            # 如果直接脚部位置不可用，回退到传感器数据计算
            self.sensor_sub = rospy.Subscriber('/sensors_data_raw', 
                                             sensorsData, 
                                             self.sensor_callback)
            rospy.loginfo("使用传感器数据计算脚部位置")
        
    def base_callback(self, msg):
        if self.is_collecting:
            # 从Float64MultiArray消息中提取位置数据
            if len(msg.data) >= 3:  # 确保数据包含xyz三个值
                current_time = time.time()
                position = [msg.data[0], msg.data[1], msg.data[2]]
                
                # 保存到总体轨迹
                self.base_positions.append(position)
                self.timestamps.append(current_time)
                
                # 保存到当前测试
                self.current_test_base_positions.append(position)
                self.current_test_timestamps.append(current_time)
            
    def com_callback(self, msg):
        if self.is_collecting:
            # 从Float64MultiArray消息中提取位置数据
            if len(msg.data) >= 3:  # 确保数据包含xyz三个值
                current_time = time.time()
                position = [msg.data[0], msg.data[1], msg.data[2]]
                
                # 保存到总体轨迹
                self.com_positions.append(position)
                
                # 保存到当前测试
                self.current_test_com_positions.append(position)
    
    def foot_center_callback(self, msg):
        """直接从脚部中心位置话题获取数据"""
        if self.is_collecting:
            # 从Float32MultiArray消息中提取脚部中心位置数据
            if len(msg.data) >= 3:  # 确保数据包含xyz三个值
                current_time = time.time()
                position = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
                
                # 保存到总体轨迹
                self.feet_center_positions.append(position)
                
                # 保存到当前测试
                self.current_test_feet_center_positions.append(position)
    
    def sensor_callback(self, msg):
        """只在没有直接脚部位置数据时使用，通过关节角度计算脚部位置"""
        if self.is_collecting and not self.use_direct_foot_data:
            # 提取腿部关节角度（前12个关节）
            if len(msg.joint_data.joint_q) >= 12:
                leg_joints = msg.joint_data.joint_q[:12]
                self.joint_positions.append(leg_joints)
                
                # 尝试使用TF获取精确的脚部位置，如果失败则使用简化计算
                feet_center = self.get_feet_center_from_tf()
                if feet_center is None:
                    # 回退到简化计算方法（从关节角度计算）
                    feet_center = self.calculate_feet_center(leg_joints)
                
                # 保存到总体轨迹
                self.feet_center_positions.append(feet_center)
                
                # 保存到当前测试
                self.current_test_feet_center_positions.append(feet_center)
    
    def get_feet_center_from_tf(self):
        """
        使用TF变换获取精确的脚部位置
        返回两脚中心位置，如果获取失败返回None
        """
        try:
            # 尝试获取左脚和右脚的TF变换
            # 常见的脚部frame名称，可能需要根据实际机器人调整
            left_foot_frames = ['l_foot_link', 'left_foot', 'l_ankle_link', 'left_ankle']
            right_foot_frames = ['r_foot_link', 'right_foot', 'r_ankle_link', 'right_ankle']
            base_frame = 'base_link'  # 或者 'odom', 'world'
            
            left_foot_pos = None
            right_foot_pos = None
            
            # 尝试获取左脚位置
            for frame in left_foot_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(base_frame, frame, rospy.Time(0), rospy.Duration(0.1))
                    left_foot_pos = [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ]
                    break
                except Exception:
                    continue
            
            # 尝试获取右脚位置
            for frame in right_foot_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(base_frame, frame, rospy.Time(0), rospy.Duration(0.1))
                    right_foot_pos = [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ]
                    break
                except Exception:
                    continue
            
            # 如果成功获取到两脚位置，计算中心
            if left_foot_pos and right_foot_pos:
                center_x = (left_foot_pos[0] + right_foot_pos[0]) / 2.0
                center_y = (left_foot_pos[1] + right_foot_pos[1]) / 2.0
                center_z = (left_foot_pos[2] + right_foot_pos[2]) / 2.0
                return [center_x, center_y, center_z]
                
        except Exception as e:
            rospy.logdebug(f"TF获取脚部位置失败: {e}")
        
        return None
    
    def calculate_feet_center(self, leg_joints):
        """
        简化的脚部中心位置计算（备用方法）
        从关节角度计算脚部位置，这里的joint_q是关节角度，不是空间坐标
        """
        # 腿部长度参数（单位：米）- 这些值需要根据实际机器人调整
        THIGH_LENGTH = 0.35    # 大腿长度
        SHIN_LENGTH = 0.35     # 小腿长度
        HIP_WIDTH = 0.2        # 髋部宽度的一半
        
        # 左腿关节角度 (0~5): [l_leg_roll, l_leg_yaw, l_leg_pitch, l_knee, l_foot_pitch, l_foot_roll]
        left_leg = leg_joints[:6]
        # 右腿关节角度 (6~11): [r_leg_roll, r_leg_yaw, r_leg_pitch, r_knee, r_foot_pitch, r_foot_roll]
        right_leg = leg_joints[6:]
        
        # 简化计算：假设机器人在平地上，主要考虑前后和左右位置
        # 注意：这是近似计算，实际的正运动学会更复杂
        
        # 左脚位置估计（使用pitch和knee角度）
        left_foot_x = -THIGH_LENGTH * math.sin(left_leg[2]) - SHIN_LENGTH * math.sin(left_leg[2] + left_leg[3])
        left_foot_y = HIP_WIDTH
        left_foot_z = -THIGH_LENGTH * math.cos(left_leg[2]) - SHIN_LENGTH * math.cos(left_leg[2] + left_leg[3])
        
        # 右脚位置估计
        right_foot_x = -THIGH_LENGTH * math.sin(right_leg[2]) - SHIN_LENGTH * math.sin(right_leg[2] + right_leg[3])
        right_foot_y = -HIP_WIDTH
        right_foot_z = -THIGH_LENGTH * math.cos(right_leg[2]) - SHIN_LENGTH * math.cos(right_leg[2] + right_leg[3])
        
        # 计算两脚中心位置
        center_x = (left_foot_x + right_foot_x) / 2.0
        center_y = (left_foot_y + right_foot_y) / 2.0
        center_z = (left_foot_z + right_foot_z) / 2.0
        
        return [center_x, center_y, center_z]
            
    def start_collection(self):
        # 只清空当前测试的数据，保留总体轨迹数据
        self.current_test_base_positions = []
        self.current_test_com_positions = []
        self.current_test_feet_center_positions = []
        self.current_test_timestamps = []
        
        self.is_collecting = True
        self.start_time = time.time()
        rospy.loginfo("开始收集数据...")
        
    def stop_collection(self):
        self.is_collecting = False
        rospy.loginfo("停止收集数据...")
        return self.analyze_current_test_data()
    
    def analyze_current_test_data(self):
        """分析当前测试的数据"""
        if not self.current_test_base_positions or not self.current_test_com_positions:
            rospy.logwarn("没有收集到足够的数据进行分析")
            return None
            
        base_pos = np.array(self.current_test_base_positions)
        com_pos = np.array(self.current_test_com_positions)
        
        # 如果有脚部中心数据，也包含进来
        feet_center_pos = None
        if self.current_test_feet_center_positions:
            feet_center_pos = np.array(self.current_test_feet_center_positions)
        
        # 确保数据长度一致
        min_len = min(len(base_pos), len(com_pos))
        if feet_center_pos is not None:
            min_len = min(min_len, len(feet_center_pos))
            feet_center_pos = feet_center_pos[:min_len]
            
        base_pos = base_pos[:min_len]
        com_pos = com_pos[:min_len]
        
        # 计算质心与基准位置的偏差
        com_base_deviations = com_pos - base_pos
        
        # 计算质心与脚部中心的偏差（如果有脚部数据）
        com_feet_deviations = None
        if feet_center_pos is not None:
            com_feet_deviations = com_pos - feet_center_pos
        
        # 计算统计信息
        com_base_distances = np.linalg.norm(com_base_deviations, axis=1)
        
        com_feet_distances = None
        if com_feet_deviations is not None:
            com_feet_distances = np.linalg.norm(com_feet_deviations, axis=1)
        
        return {
            'base_positions': base_pos,
            'com_positions': com_pos,
            'feet_center_positions': feet_center_pos,
            'com_base_deviations': com_base_deviations,
            'com_feet_deviations': com_feet_deviations,
            'com_base_distances': com_base_distances,
            'com_feet_distances': com_feet_distances,
            'timestamps': self.current_test_timestamps[:min_len] if self.current_test_timestamps else None
        }
        
    def get_full_trajectory_data(self):
        """获取完整的轨迹数据（所有测试的累积数据）"""
        if not self.base_positions or not self.com_positions:
            rospy.logwarn("没有收集到足够的轨迹数据")
            return None
            
        base_pos = np.array(self.base_positions)
        com_pos = np.array(self.com_positions)
        
        # 如果有脚部中心数据，也包含进来
        feet_center_pos = None
        if self.feet_center_positions:
            feet_center_pos = np.array(self.feet_center_positions)
        
        # 确保数据长度一致
        min_len = min(len(base_pos), len(com_pos))
        if feet_center_pos is not None:
            min_len = min(min_len, len(feet_center_pos))
            feet_center_pos = feet_center_pos[:min_len]
            
        base_pos = base_pos[:min_len]
        com_pos = com_pos[:min_len]
        
        # 计算质心与基准位置的偏差
        com_base_deviations = com_pos - base_pos
        
        # 计算质心与脚部中心的偏差（如果有脚部数据）
        com_feet_deviations = None
        if feet_center_pos is not None:
            com_feet_deviations = com_pos - feet_center_pos
        
        # 计算统计信息
        com_base_distances = np.linalg.norm(com_base_deviations, axis=1)
        
        com_feet_distances = None
        if com_feet_deviations is not None:
            com_feet_distances = np.linalg.norm(com_feet_deviations, axis=1)
        
        return {
            'base_positions': base_pos,
            'com_positions': com_pos,
            'feet_center_positions': feet_center_pos,
            'com_base_deviations': com_base_deviations,
            'com_feet_deviations': com_feet_deviations,
            'com_base_distances': com_base_distances,
            'com_feet_distances': com_feet_distances,
            'timestamps': self.timestamps[:min_len] if self.timestamps else None
        }

def plot_results(data, save_path, test_name=None):
    """绘制并保存分析结果"""
    # 设置matplotlib参数
    plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.family'] = 'sans-serif'
    
    # 创建保存目录
    if test_name:
        test_dir = os.path.join(save_path, test_name)
        os.makedirs(test_dir, exist_ok=True)
    else:
        test_dir = save_path
    
    # 创建原始数据对比图
    plt.figure(figsize=(15, 12))
    
    # X方向位置对比
    plt.subplot(311)
    plt.plot(data['base_positions'][:, 0], label='Base X', color='b', alpha=0.7)
    plt.plot(data['com_positions'][:, 0], label='COM X', color='r', alpha=0.7)
    if 'feet_center_positions' in data:
        plt.plot(data['feet_center_positions'][:, 0], label='Feet Center X', color='g', alpha=0.7)
    plt.xlabel('Sample Points')
    plt.ylabel('Position (m)')
    plt.title('X Position Comparison')
    plt.legend()
    plt.grid(True)
    
    # Y方向位置对比
    plt.subplot(312)
    plt.plot(data['base_positions'][:, 1], label='Base Y', color='b', alpha=0.7)
    plt.plot(data['com_positions'][:, 1], label='COM Y', color='r', alpha=0.7)
    if 'feet_center_positions' in data:
        plt.plot(data['feet_center_positions'][:, 1], label='Feet Center Y', color='g', alpha=0.7)
    plt.xlabel('Sample Points')
    plt.ylabel('Position (m)')
    plt.title('Y Position Comparison')
    plt.legend()
    plt.grid(True)
    
    # Z方向位置对比
    plt.subplot(313)
    plt.plot(data['base_positions'][:, 2], label='Base Z', color='b', alpha=0.7)
    plt.plot(data['com_positions'][:, 2], label='COM Z', color='r', alpha=0.7)
    if 'feet_center_positions' in data:
        plt.plot(data['feet_center_positions'][:, 2], label='Feet Center Z', color='g', alpha=0.7)
    plt.xlabel('Sample Points')
    plt.ylabel('Position (m)')
    plt.title('Z Position Comparison')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/raw_position_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建2D图
    plt.figure(figsize=(15, 5))
    
    # XY平面投影
    plt.subplot(131)
    plt.scatter(data['base_positions'][:, 0], data['base_positions'][:, 1], 
                c='b', label='Base Position', alpha=0.5)
    plt.scatter(data['com_positions'][:, 0], data['com_positions'][:, 1], 
                c='r', label='COM Position', alpha=0.5)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('XY Projection')
    plt.legend()
    plt.grid(True)
    
    # XZ平面投影
    plt.subplot(132)
    plt.scatter(data['base_positions'][:, 0], data['base_positions'][:, 2], 
                c='b', label='Base Position', alpha=0.5)
    plt.scatter(data['com_positions'][:, 0], data['com_positions'][:, 2], 
                c='r', label='COM Position', alpha=0.5)
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title('XZ Projection')
    plt.legend()
    plt.grid(True)
    
    # YZ平面投影
    plt.subplot(133)
    plt.scatter(data['base_positions'][:, 1], data['base_positions'][:, 2], 
                c='b', label='Base Position', alpha=0.5)
    plt.scatter(data['com_positions'][:, 1], data['com_positions'][:, 2], 
                c='r', label='COM Position', alpha=0.5)
    plt.xlabel('Y (m)')
    plt.ylabel('Z (m)')
    plt.title('YZ Projection')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/2d_projections.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建多视图综合图 (2x2子图布局)
    fig = plt.figure(figsize=(16, 12))
    
    # 准备数据
    base_pos = data['base_positions']
    com_pos = data['com_positions']
    feet_pos = data.get('feet_center_positions')
    
    # 子图1: 3D视图 (标准视角)
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    
    # 添加轨迹线
    ax1.plot(base_pos[:, 0], base_pos[:, 1], base_pos[:, 2], 'b-', alpha=0.5, linewidth=1)
    ax1.plot(com_pos[:, 0], com_pos[:, 1], com_pos[:, 2], 'r-', alpha=0.5, linewidth=1)
    if feet_pos is not None:
        ax1.plot(feet_pos[:, 0], feet_pos[:, 1], feet_pos[:, 2], 'g-', alpha=0.5, linewidth=1)
    
    # 添加散点
    ax1.scatter(base_pos[:, 0], base_pos[:, 1], base_pos[:, 2],
               c='b', label='Base Position', alpha=0.7, s=20)
    ax1.scatter(com_pos[:, 0], com_pos[:, 1], com_pos[:, 2],
               c='r', label='COM Position', alpha=0.7, s=20)
    if feet_pos is not None:
        ax1.scatter(feet_pos[:, 0], feet_pos[:, 1], feet_pos[:, 2],
                   c='g', label='Feet Center Position', alpha=0.7, s=20)
    
    # 标记起点和终点
    ax1.scatter(com_pos[0, 0], com_pos[0, 1], com_pos[0, 2], c='red', s=100, marker='o', label='Start')
    ax1.scatter(com_pos[-1, 0], com_pos[-1, 1], com_pos[-1, 2], c='darkred', s=100, marker='s', label='End')
    
    # 设置标准3D视角
    ax1.view_init(elev=20, azim=45)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D View', fontsize=10)
    ax1.legend(fontsize=8)
    ax1.grid(True)
    
    # 子图2: 俯视图 (XY平面 - 从上往下看)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.scatter(base_pos[:, 0], base_pos[:, 1], 
                c='b', label='Base Position', alpha=0.7, s=20)
    ax2.scatter(com_pos[:, 0], com_pos[:, 1], 
                c='r', label='COM Position', alpha=0.7, s=20)
    if feet_pos is not None:
        ax2.scatter(feet_pos[:, 0], feet_pos[:, 1], 
                    c='g', label='Feet Center Position', alpha=0.7, s=20)
    
    # 添加轨迹线
    ax2.plot(base_pos[:, 0], base_pos[:, 1], 'b-', alpha=0.3, linewidth=1)
    ax2.plot(com_pos[:, 0], com_pos[:, 1], 'r-', alpha=0.3, linewidth=1)
    if feet_pos is not None:
        ax2.plot(feet_pos[:, 0], feet_pos[:, 1], 'g-', alpha=0.3, linewidth=1)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY Plane)', fontsize=10)
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # 子图3: 正视图 (XZ平面 - 从前往后看)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.scatter(base_pos[:, 0], base_pos[:, 2], 
                c='b', label='Base Position', alpha=0.7, s=20)
    ax3.scatter(com_pos[:, 0], com_pos[:, 2], 
                c='r', label='COM Position', alpha=0.7, s=20)
    if feet_pos is not None:
        ax3.scatter(feet_pos[:, 0], feet_pos[:, 2], 
                    c='g', label='Feet Center Position', alpha=0.7, s=20)
    
    # 添加轨迹线
    ax3.plot(base_pos[:, 0], base_pos[:, 2], 'b-', alpha=0.3, linewidth=1)
    ax3.plot(com_pos[:, 0], com_pos[:, 2], 'r-', alpha=0.3, linewidth=1)
    if feet_pos is not None:
        ax3.plot(feet_pos[:, 0], feet_pos[:, 2], 'g-', alpha=0.3, linewidth=1)
    
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Front View (XZ Plane)', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # 子图4: 侧视图 (YZ平面 - 从侧面看)
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.scatter(base_pos[:, 1], base_pos[:, 2], 
                c='b', label='Base Position', alpha=0.7, s=20)
    ax4.scatter(com_pos[:, 1], com_pos[:, 2], 
                c='r', label='COM Position', alpha=0.7, s=20)
    if feet_pos is not None:
        ax4.scatter(feet_pos[:, 1], feet_pos[:, 2], 
                    c='g', label='Feet Center Position', alpha=0.7, s=20)
    
    # 添加轨迹线
    ax4.plot(base_pos[:, 1], base_pos[:, 2], 'b-', alpha=0.3, linewidth=1)
    ax4.plot(com_pos[:, 1], com_pos[:, 2], 'r-', alpha=0.3, linewidth=1)
    if feet_pos is not None:
        ax4.plot(feet_pos[:, 1], feet_pos[:, 2], 'g-', alpha=0.3, linewidth=1)
    
    ax4.set_xlabel('Y (m)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title('Side View (YZ Plane)', fontsize=10)
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    # 添加整体标题
    fig.suptitle('Multi-View Trajectory Analysis', fontsize=16, y=0.95)
    
    # 调整子图间距
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    
    plt.savefig(f'{test_dir}/multi_view_trajectory.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建专门的COM和脚部中心分析图表
    create_com_feet_analysis_plot(data, test_dir)
    
    # 生成详细的COM统计报告
    generate_com_statistics_report(data, test_dir)
    
    # 创建时间序列图
    has_feet_data = 'com_feet_deviations' in data and data['com_feet_deviations'] is not None
    num_subplots = 6 if has_feet_data else 3
    plt.figure(figsize=(15, 4 * num_subplots))
    
    # 计算统计值
    com_base_mean = np.mean(data['com_base_deviations'], axis=0)
    com_base_std = np.std(data['com_base_deviations'], axis=0)
    
    # COM与基准位置的X方向偏差
    plt.subplot(num_subplots, 1, 1)
    plt.plot(data['com_base_deviations'][:, 0], label='COM-Base X Deviation', color='b')
    plt.axhline(y=com_base_mean[0], color='r', linestyle='--', label='Mean')
    plt.fill_between(range(len(data['com_base_deviations'])), 
                    com_base_mean[0] - com_base_std[0],
                    com_base_mean[0] + com_base_std[0],
                    alpha=0.2, color='r', label='Std Dev Range')
    plt.xlabel('Sample Points')
    plt.ylabel('X Deviation (m)')
    plt.title('X Direction COM-Base Deviation Time Series')
    plt.legend()
    plt.grid(True)
    
    # COM与基准位置的Y方向偏差
    plt.subplot(num_subplots, 1, 2)
    plt.plot(data['com_base_deviations'][:, 1], label='COM-Base Y Deviation', color='b')
    plt.axhline(y=com_base_mean[1], color='r', linestyle='--', label='Mean')
    plt.fill_between(range(len(data['com_base_deviations'])), 
                    com_base_mean[1] - com_base_std[1],
                    com_base_mean[1] + com_base_std[1],
                    alpha=0.2, color='r', label='Std Dev Range')
    plt.xlabel('Sample Points')
    plt.ylabel('Y Deviation (m)')
    plt.title('Y Direction COM-Base Deviation Time Series')
    plt.legend()
    plt.grid(True)
    
    # COM与基准位置的Z方向偏差
    plt.subplot(num_subplots, 1, 3)
    plt.plot(data['com_base_deviations'][:, 2], label='COM-Base Z Deviation', color='b')
    plt.axhline(y=com_base_mean[2], color='r', linestyle='--', label='Mean')
    plt.fill_between(range(len(data['com_base_deviations'])), 
                    com_base_mean[2] - com_base_std[2],
                    com_base_mean[2] + com_base_std[2],
                    alpha=0.2, color='r', label='Std Dev Range')
    plt.xlabel('Sample Points')
    plt.ylabel('Z Deviation (m)')
    plt.title('Z Direction COM-Base Deviation Time Series')
    plt.legend()
    plt.grid(True)
    
    # 如果有脚部中心数据，添加COM与脚部中心的偏差图
    if has_feet_data:
        com_feet_mean = np.mean(data['com_feet_deviations'], axis=0)
        com_feet_std = np.std(data['com_feet_deviations'], axis=0)
        
        # COM与脚部中心的X方向偏差
        plt.subplot(num_subplots, 1, 4)
        plt.plot(data['com_feet_deviations'][:, 0], label='COM-Feet X Deviation', color='g')
        plt.axhline(y=com_feet_mean[0], color='r', linestyle='--', label='Mean')
        plt.fill_between(range(len(data['com_feet_deviations'])), 
                        com_feet_mean[0] - com_feet_std[0],
                        com_feet_mean[0] + com_feet_std[0],
                        alpha=0.2, color='r', label='Std Dev Range')
        plt.xlabel('Sample Points')
        plt.ylabel('X Deviation (m)')
        plt.title('X Direction COM-Feet Center Deviation Time Series')
        plt.legend()
        plt.grid(True)
        
        # COM与脚部中心的Y方向偏差
        plt.subplot(num_subplots, 1, 5)
        plt.plot(data['com_feet_deviations'][:, 1], label='COM-Feet Y Deviation', color='g')
        plt.axhline(y=com_feet_mean[1], color='r', linestyle='--', label='Mean')
        plt.fill_between(range(len(data['com_feet_deviations'])), 
                        com_feet_mean[1] - com_feet_std[1],
                        com_feet_mean[1] + com_feet_std[1],
                        alpha=0.2, color='r', label='Std Dev Range')
        plt.xlabel('Sample Points')
        plt.ylabel('Y Deviation (m)')
        plt.title('Y Direction COM-Feet Center Deviation Time Series')
        plt.legend()
        plt.grid(True)
        
        # COM与脚部中心的Z方向偏差
        plt.subplot(num_subplots, 1, 6)
        plt.plot(data['com_feet_deviations'][:, 2], label='COM-Feet Z Deviation', color='g')
        plt.axhline(y=com_feet_mean[2], color='r', linestyle='--', label='Mean')
        plt.fill_between(range(len(data['com_feet_deviations'])), 
                        com_feet_mean[2] - com_feet_std[2],
                        com_feet_mean[2] + com_feet_std[2],
                        alpha=0.2, color='r', label='Std Dev Range')
        plt.xlabel('Sample Points')
        plt.ylabel('Z Deviation (m)')
        plt.title('Z Direction COM-Feet Center Deviation Time Series')
        plt.legend()
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/time_series.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建组合偏差图
    fig_height = 16 if has_feet_data else 8
    plt.figure(figsize=(15, fig_height))
    
    if has_feet_data:
        # 如果有脚部数据，创建两个子图
        plt.subplot(2, 1, 1)
    
    # COM与基准位置的偏差
    plt.plot(data['com_base_deviations'][:, 0], label='COM-Base X Deviation', color='r')
    plt.plot(data['com_base_deviations'][:, 1], label='COM-Base Y Deviation', color='g')
    plt.plot(data['com_base_deviations'][:, 2], label='COM-Base Z Deviation', color='b')
    
    # 添加平均值线
    plt.axhline(y=com_base_mean[0], color='r', linestyle='--', alpha=0.5, label='X Mean')
    plt.axhline(y=com_base_mean[1], color='g', linestyle='--', alpha=0.5, label='Y Mean')
    plt.axhline(y=com_base_mean[2], color='b', linestyle='--', alpha=0.5, label='Z Mean')
    
    plt.xlabel('Sample Points')
    plt.ylabel('Deviation (m)')
    plt.title('COM-Base Combined Deviation Time Series')
    plt.legend()
    plt.grid(True)
    
    if has_feet_data:
        # 计算脚部偏差统计
        com_feet_mean = np.mean(data['com_feet_deviations'], axis=0)
        
        # COM与脚部中心的偏差
        plt.subplot(2, 1, 2)
        plt.plot(data['com_feet_deviations'][:, 0], label='COM-Feet X Deviation', color='r')
        plt.plot(data['com_feet_deviations'][:, 1], label='COM-Feet Y Deviation', color='g')
        plt.plot(data['com_feet_deviations'][:, 2], label='COM-Feet Z Deviation', color='b')
        
        # 添加平均值线
        plt.axhline(y=com_feet_mean[0], color='r', linestyle='--', alpha=0.5, label='X Mean')
        plt.axhline(y=com_feet_mean[1], color='g', linestyle='--', alpha=0.5, label='Y Mean')
        plt.axhline(y=com_feet_mean[2], color='b', linestyle='--', alpha=0.5, label='Z Mean')
        
        plt.xlabel('Sample Points')
        plt.ylabel('Deviation (m)')
        plt.title('COM-Feet Center Combined Deviation Time Series')
        plt.legend()
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/combined_deviations.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建偏差分布直方图
    num_rows = 2 if has_feet_data else 1
    plt.figure(figsize=(15, 5 * num_rows))
    
    # COM-Base偏差分布
    # X方向偏差分布
    plt.subplot(num_rows, 3, 1)
    plt.hist(data['com_base_deviations'][:, 0], bins=50, alpha=0.7, color='blue')
    plt.axvline(x=com_base_mean[0], color='r', linestyle='--', label='Mean')
    plt.xlabel('X Deviation (m)')
    plt.ylabel('Frequency')
    plt.title('COM-Base X Direction Deviation Distribution')
    plt.legend()
    plt.grid(True)
    
    # Y方向偏差分布
    plt.subplot(num_rows, 3, 2)
    plt.hist(data['com_base_deviations'][:, 1], bins=50, alpha=0.7, color='blue')
    plt.axvline(x=com_base_mean[1], color='r', linestyle='--', label='Mean')
    plt.xlabel('Y Deviation (m)')
    plt.ylabel('Frequency')
    plt.title('COM-Base Y Direction Deviation Distribution')
    plt.legend()
    plt.grid(True)
    
    # Z方向偏差分布
    plt.subplot(num_rows, 3, 3)
    plt.hist(data['com_base_deviations'][:, 2], bins=50, alpha=0.7, color='blue')
    plt.axvline(x=com_base_mean[2], color='r', linestyle='--', label='Mean')
    plt.xlabel('Z Deviation (m)')
    plt.ylabel('Frequency')
    plt.title('COM-Base Z Direction Deviation Distribution')
    plt.legend()
    plt.grid(True)
    
    # 如果有脚部数据，添加COM-Feet偏差分布
    if has_feet_data:
        # X方向偏差分布
        plt.subplot(num_rows, 3, 4)
        plt.hist(data['com_feet_deviations'][:, 0], bins=50, alpha=0.7, color='green')
        plt.axvline(x=com_feet_mean[0], color='r', linestyle='--', label='Mean')
        plt.xlabel('X Deviation (m)')
        plt.ylabel('Frequency')
        plt.title('COM-Feet X Direction Deviation Distribution')
        plt.legend()
        plt.grid(True)
        
        # Y方向偏差分布
        plt.subplot(num_rows, 3, 5)
        plt.hist(data['com_feet_deviations'][:, 1], bins=50, alpha=0.7, color='green')
        plt.axvline(x=com_feet_mean[1], color='r', linestyle='--', label='Mean')
        plt.xlabel('Y Deviation (m)')
        plt.ylabel('Frequency')
        plt.title('COM-Feet Y Direction Deviation Distribution')
        plt.legend()
        plt.grid(True)
        
        # Z方向偏差分布
        plt.subplot(num_rows, 3, 6)
        plt.hist(data['com_feet_deviations'][:, 2], bins=50, alpha=0.7, color='green')
        plt.axvline(x=com_feet_mean[2], color='r', linestyle='--', label='Mean')
        plt.xlabel('Z Deviation (m)')
        plt.ylabel('Frequency')
        plt.title('COM-Feet Z Direction Deviation Distribution')
        plt.legend()
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/deviation_histograms.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 创建偏差箱线图
    fig_width = 20 if has_feet_data else 10
    plt.figure(figsize=(fig_width, 6))
    
    if has_feet_data:
        # 如果有脚部数据，创建两个子图对比
        plt.subplot(1, 2, 1)
        plt.boxplot([data['com_base_deviations'][:, 0], data['com_base_deviations'][:, 1], data['com_base_deviations'][:, 2]],
                    labels=['X Deviation', 'Y Deviation', 'Z Deviation'])
        plt.title('COM-Base Deviation Boxplot')
        plt.ylabel('Deviation (m)')
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        plt.boxplot([data['com_feet_deviations'][:, 0], data['com_feet_deviations'][:, 1], data['com_feet_deviations'][:, 2]],
                    labels=['X Deviation', 'Y Deviation', 'Z Deviation'])
        plt.title('COM-Feet Center Deviation Boxplot')
        plt.ylabel('Deviation (m)')
        plt.grid(True)
    else:
        # 只有COM-Base数据
        plt.boxplot([data['com_base_deviations'][:, 0], data['com_base_deviations'][:, 1], data['com_base_deviations'][:, 2]],
                    labels=['X Deviation', 'Y Deviation', 'Z Deviation'])
        plt.title('COM-Base Deviation Boxplot')
        plt.ylabel('Deviation (m)')
        plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{test_dir}/deviation_boxplot.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 保存统计数据
    with open(f'{test_dir}/statistics.txt', 'w') as f:
        f.write("=== Statistics ===\n")
        f.write(f"Number of Data Points: {len(data['com_base_deviations'])}\n\n")
        
        f.write("=== COM-Base Deviation Analysis ===\n")
        f.write("=== Mean Deviation ===\n")
        f.write(f"X Direction: {com_base_mean[0]:.6f} m\n")
        f.write(f"Y Direction: {com_base_mean[1]:.6f} m\n")
        f.write(f"Z Direction: {com_base_mean[2]:.6f} m\n\n")
        
        f.write("=== Standard Deviation ===\n")
        f.write(f"X Direction: {com_base_std[0]:.6f} m\n")
        f.write(f"Y Direction: {com_base_std[1]:.6f} m\n")
        f.write(f"Z Direction: {com_base_std[2]:.6f} m\n\n")
        
        f.write("=== Maximum Deviation ===\n")
        f.write(f"X Direction: {np.max(np.abs(data['com_base_deviations'][:, 0])):.6f} m\n")
        f.write(f"Y Direction: {np.max(np.abs(data['com_base_deviations'][:, 1])):.6f} m\n")
        f.write(f"Z Direction: {np.max(np.abs(data['com_base_deviations'][:, 2])):.6f} m\n\n")
        
        f.write("=== Minimum Deviation ===\n")
        f.write(f"X Direction: {np.min(np.abs(data['com_base_deviations'][:, 0])):.6f} m\n")
        f.write(f"Y Direction: {np.min(np.abs(data['com_base_deviations'][:, 1])):.6f} m\n")
        f.write(f"Z Direction: {np.min(np.abs(data['com_base_deviations'][:, 2])):.6f} m\n\n")
        
        f.write("=== Median Deviation ===\n")
        f.write(f"X Direction: {np.median(data['com_base_deviations'][:, 0]):.6f} m\n")
        f.write(f"Y Direction: {np.median(data['com_base_deviations'][:, 1]):.6f} m\n")
        f.write(f"Z Direction: {np.median(data['com_base_deviations'][:, 2]):.6f} m\n\n")
        
        # 如果有脚部中心数据，添加相关统计
        if has_feet_data:
            com_feet_std = np.std(data['com_feet_deviations'], axis=0)
            
            f.write("=== COM-Feet Center Deviation Analysis ===\n")
            f.write("=== Mean Deviation ===\n")
            f.write(f"X Direction: {com_feet_mean[0]:.6f} m\n")
            f.write(f"Y Direction: {com_feet_mean[1]:.6f} m\n")
            f.write(f"Z Direction: {com_feet_mean[2]:.6f} m\n\n")
            
            f.write("=== Standard Deviation ===\n")
            f.write(f"X Direction: {com_feet_std[0]:.6f} m\n")
            f.write(f"Y Direction: {com_feet_std[1]:.6f} m\n")
            f.write(f"Z Direction: {com_feet_std[2]:.6f} m\n\n")
            
            f.write("=== Maximum Deviation ===\n")
            f.write(f"X Direction: {np.max(np.abs(data['com_feet_deviations'][:, 0])):.6f} m\n")
            f.write(f"Y Direction: {np.max(np.abs(data['com_feet_deviations'][:, 1])):.6f} m\n")
            f.write(f"Z Direction: {np.max(np.abs(data['com_feet_deviations'][:, 2])):.6f} m\n\n")
            
            f.write("=== Minimum Deviation ===\n")
            f.write(f"X Direction: {np.min(np.abs(data['com_feet_deviations'][:, 0])):.6f} m\n")
            f.write(f"Y Direction: {np.min(np.abs(data['com_feet_deviations'][:, 1])):.6f} m\n")
            f.write(f"Z Direction: {np.min(np.abs(data['com_feet_deviations'][:, 2])):.6f} m\n\n")
            
            f.write("=== Median Deviation ===\n")
            f.write(f"X Direction: {np.median(data['com_feet_deviations'][:, 0]):.6f} m\n")
            f.write(f"Y Direction: {np.median(data['com_feet_deviations'][:, 1]):.6f} m\n")
            f.write(f"Z Direction: {np.median(data['com_feet_deviations'][:, 2]):.6f} m\n")
        
    # 保存原始数据
    np.save(f'{test_dir}/base_positions.npy', data['base_positions'])
    np.save(f'{test_dir}/com_positions.npy', data['com_positions'])
    np.save(f'{test_dir}/com_base_deviations.npy', data['com_base_deviations'])
    
    if 'feet_center_positions' in data:
        np.save(f'{test_dir}/feet_center_positions.npy', data['feet_center_positions'])
        np.save(f'{test_dir}/com_feet_deviations.npy', data['com_feet_deviations'])

def create_com_feet_analysis_plot(data, test_dir):
    """
    创建专门的COM和脚部中心分析图表（完全按照test.py风格）
    """
    if 'feet_center_positions' not in data:
        print("没有脚部中心数据，跳过COM-脚部分析")
        return
    
    fig = plt.figure(figsize=(16, 12))
    
    # 准备数据 - 完全按照test.py的数据结构
    timestamp = np.arange(len(data['com_positions'])) * 0.1  # 模拟时间戳
    com_pos = data['com_positions']
    feet_center = data['feet_center_positions']
    distance = np.linalg.norm(com_pos[:, :2] - feet_center[:, :2], axis=1)  # 2D距离
    deviation = com_pos[:, :2] - feet_center[:, :2]  # XY偏差
    
    # 1. 3D Trajectory (top left) - 完全按照test.py风格
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(com_pos[:, 0], com_pos[:, 1], com_pos[:, 2], 'r-', linewidth=2, label='COM Trajectory', alpha=0.8)
    ax1.plot(feet_center[:, 0], feet_center[:, 1], feet_center[:, 2], 'g-', linewidth=2, label='Feet Center', alpha=0.8)
    ax1.scatter(com_pos[0, 0], com_pos[0, 1], com_pos[0, 2], c='red', s=100, marker='o')
    ax1.scatter(com_pos[-1, 0], com_pos[-1, 1], com_pos[-1, 2], c='darkred', s=100, marker='s')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory: COM vs Feet Center')
    ax1.legend()
    
    # 2. Distance over time (top center) - 完全按照test.py风格
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(timestamp, distance, 'b-', linewidth=2)
    ax2.fill_between(timestamp, distance, alpha=0.3, color='blue')
    mean_dist = np.mean(distance)
    ax2.axhline(y=mean_dist, color='red', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Distance (m)')
    ax2.set_title('COM-Feet Center 2D Distance')
    ax2.grid(True, alpha=0.3)
    ax2.text(0.02, 0.98, f'Mean: {mean_dist:.3f}m\nMax: {np.max(distance):.3f}m\nStd: {np.std(distance):.3f}m', 
             transform=ax2.transAxes, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # 3. XY Deviation (top right) - 完全按照test.py风格
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(timestamp, deviation[:, 0], 'r-', linewidth=2, label='X Deviation')
    ax3.plot(timestamp, deviation[:, 1], 'g-', linewidth=2, label='Y Deviation')
    ax3.fill_between(timestamp, deviation[:, 0], alpha=0.3, color='red')
    ax3.fill_between(timestamp, deviation[:, 1], alpha=0.3, color='green')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Deviation (m)')
    ax3.set_title('COM-Feet Center XY Deviation')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    
    # 4. 2D Projection (bottom left) - 完全按照test.py风格
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(com_pos[:, 0], com_pos[:, 1], 'r-', linewidth=2, label='COM Projection', alpha=0.8)
    ax4.plot(feet_center[:, 0], feet_center[:, 1], 'g-', linewidth=2, label='Feet Center', alpha=0.8)
    ax4.scatter(com_pos[0, 0], com_pos[0, 1], c='red', s=100, marker='o', label='Start')
    ax4.scatter(com_pos[-1, 0], com_pos[-1, 1], c='darkred', s=100, marker='s', label='End')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_title('2D Ground Projection Comparison')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    # 5. Height comparison (bottom center) - 完全按照test.py风格
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(timestamp, com_pos[:, 2], 'r-', linewidth=2, label='COM Height')
    ax5.plot(timestamp, feet_center[:, 2], 'g-', linewidth=2, label='Feet Center Height')
    ax5.fill_between(timestamp, com_pos[:, 2], alpha=0.3, color='red')
    ax5.fill_between(timestamp, feet_center[:, 2], alpha=0.3, color='green')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Height (m)')
    ax5.set_title('Height Comparison')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 6. Statistical distribution (bottom right) - 完全按照test.py风格
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.hist(distance, bins=30, alpha=0.7, color='blue', edgecolor='black')
    ax6.axvline(np.mean(distance), color='red', linestyle='--', label=f'Mean: {np.mean(distance):.3f}m')
    ax6.set_xlabel('Distance (m)')
    ax6.set_ylabel('Frequency')
    ax6.set_title('Distance Distribution')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存图像 - 按照test.py的命名方式
    plt.savefig(f'{test_dir}/com_analysis_summary.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"COM分析总结图已保存: {test_dir}/com_analysis_summary.png")

def generate_com_statistics_report(data, test_dir):
    """
    生成详细的COM统计报告（按照test.py风格）
    """
    if 'feet_center_positions' not in data:
        return
    
    timestamp = np.arange(len(data['com_positions'])) * 0.1
    com_pos = data['com_positions']
    feet_center = data['feet_center_positions']
    distance = np.linalg.norm(com_pos[:, :2] - feet_center[:, :2], axis=1)
    deviation = com_pos[:, :2] - feet_center[:, :2]
    
    # Calculate statistics - 按照test.py的计算方式
    duration = timestamp[-1] - timestamp[0]
    sample_rate = len(timestamp) / duration
    
    # Distance statistics
    dist_mean = np.mean(distance)
    dist_std = np.std(distance)
    dist_max = np.max(distance)
    dist_min = np.min(distance)
    
    # Deviation statistics
    x_dev_mean = np.mean(deviation[:, 0])
    x_dev_std = np.std(deviation[:, 0])
    y_dev_mean = np.mean(deviation[:, 1])
    y_dev_std = np.std(deviation[:, 1])
    
    # Height statistics
    com_height_mean = np.mean(com_pos[:, 2])
    com_height_std = np.std(com_pos[:, 2])
    feet_height_mean = np.mean(feet_center[:, 2])
    height_diff_mean = np.mean(com_pos[:, 2] - feet_center[:, 2])
    
    # Generate report - 完全按照test.py的格式
    report = f"""
==========================================
COM Analysis Statistics Report
==========================================

Data Overview:
--------------
Duration: {duration:.2f} seconds
Sample Points: {len(timestamp)}
Sample Rate: {sample_rate:.1f} Hz

COM-Feet Center Distance Analysis:
-----------------------------------
Mean Distance: {dist_mean:.4f} m
Standard Deviation: {dist_std:.4f} m
Maximum Distance: {dist_max:.4f} m
Minimum Distance: {dist_min:.4f} m

XY Deviation Analysis:
----------------------
X Deviation - Mean: {x_dev_mean:.4f} m, Std: {x_dev_std:.4f} m
Y Deviation - Mean: {y_dev_mean:.4f} m, Std: {y_dev_std:.4f} m

Height Analysis:
---------------
COM Height - Mean: {com_height_mean:.4f} m, Std: {com_height_std:.4f} m
Feet Center Height - Mean: {feet_height_mean:.4f} m
Average Height Difference: {height_diff_mean:.4f} m

Stability Assessment:
--------------------
Distance Variability: {(dist_std/dist_mean)*100:.2f}%
X Deviation Spread: ±{x_dev_std:.4f} m
Y Deviation Spread: ±{y_dev_std:.4f} m

==========================================
"""
    
    print(report)
    
    # Save report to file - 按照test.py的保存方式
    report_path = f'{test_dir}/com_analysis_report.txt'
    with open(report_path, 'w') as f:
        f.write(report)
    print(f"Detailed report saved to: {report_path}")
    
    return report

def enable_control_mode():
    """Enable the necessary control modes for arm control"""
    try:
        # Enable mobile manipulator MPC control
        rospy.wait_for_service('/mobile_manipulator_mpc_control', timeout=5.0)
        mpc_control = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeTorsoCtrlMode)
        mpc_control(control_mode=1)
        
        # Enable humanoid arm control mode
        rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=5.0)
        arm_ctrl = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
        arm_ctrl(control_mode=2)  # 设置为外部控制模式
        
        rospy.loginfo("Successfully enabled all control modes")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to enable control modes: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Service not available: {e}")
        return False

def publish_visualization_markers(marker_pub, left_pos, right_pos, quat_left, quat_right):
    """Publish visualization markers for RViz"""
    marker_array = MarkerArray()
    
    # Create marker for left hand
    left_marker = Marker()
    left_marker.header.frame_id = "odom"
    left_marker.header.stamp = rospy.Time.now()
    left_marker.ns = "hand_position_markers"
    left_marker.id = 0
    left_marker.type = Marker.SPHERE
    left_marker.action = Marker.ADD
    
    # Set position and orientation
    left_marker.pose.position.x = left_pos[0]
    left_marker.pose.position.y = left_pos[1]
    left_marker.pose.position.z = left_pos[2]
    left_marker.pose.orientation.x = quat_left[0]
    left_marker.pose.orientation.y = quat_left[1]
    left_marker.pose.orientation.z = quat_left[2]
    left_marker.pose.orientation.w = quat_left[3]
    
    # Set scale
    left_marker.scale.x = 0.05
    left_marker.scale.y = 0.05
    left_marker.scale.z = 0.05
    
    # Set color (blue for left hand)
    left_marker.color.r = 0.0
    left_marker.color.g = 0.0
    left_marker.color.b = 1.0
    left_marker.color.a = 0.8
    
    # Create marker for right hand
    right_marker = Marker()
    right_marker.header.frame_id = "odom"
    right_marker.header.stamp = rospy.Time.now()
    right_marker.ns = "hand_position_markers"
    right_marker.id = 1
    right_marker.type = Marker.SPHERE
    right_marker.action = Marker.ADD
    
    # Set position and orientation
    right_marker.pose.position.x = right_pos[0]
    right_marker.pose.position.y = right_pos[1]
    right_marker.pose.position.z = right_pos[2]
    right_marker.pose.orientation.x = quat_right[0]
    right_marker.pose.orientation.y = quat_right[1]
    right_marker.pose.orientation.z = quat_right[2]
    right_marker.pose.orientation.w = quat_right[3]
    
    # Set scale
    right_marker.scale.x = 0.05
    right_marker.scale.y = 0.05
    right_marker.scale.z = 0.05
    
    # Set color (red for right hand)
    right_marker.color.r = 1.0
    right_marker.color.g = 0.0
    right_marker.color.b = 0.0
    right_marker.color.a = 0.8
    
    # Add both markers to the array
    marker_array.markers.append(left_marker)
    marker_array.markers.append(right_marker)
    
    # Publish the marker array
    marker_pub.publish(marker_array)

def publish_arm_joints(pub, marker_pub, joint_angles_left, joint_angles_right):
    """Publish arm joint angles using JointState message"""
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    
    # 设置关节名称
    msg.name = [
        "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", 
        "zarm_l5_link", "zarm_l6_link", "zarm_l7_link",
        "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link", 
        "zarm_r5_link", "zarm_r6_link", "zarm_r7_link"
    ]
    
    # 设置关节位置（角度转换为弧度）
    # msg.position = [math.radians(angle) for angle in joint_angles_left + joint_angles_right]
    msg.position = joint_angles_left + joint_angles_right
    
    # 发布消息
    pub.publish(msg)
    
    # 发布可视化标记
    # 使用默认位置和方向进行可视化
    left_pos = [0.3692052960395813, 0.43259960412979126, 0.8304591178894043]
    right_pos = [0.3692052960395813, -0.43259960412979126, 0.8304591178894043]
    quat = [0, -0.67566370964, 0, 0.73720997571945]  # 默认方向
    publish_visualization_markers(marker_pub, left_pos, right_pos, quat, quat)

def generate_sine_wave(amplitude, frequency, phase, time, hold_time=2.0):
    """Generate a sine wave value with hold at peak"""
    cycle_time = 1.0 / frequency
    if time < cycle_time - hold_time:
        return amplitude * math.sin(2 * math.pi * frequency * time + phase)
    else:
        return amplitude * math.sin(2 * math.pi * frequency * (cycle_time - hold_time) + phase)

def generate_half_sine_wave(amplitude, frequency, phase, time):
    """Generate half a sine wave (0 to pi)"""
    # 半个周期，所以时间被压缩到一半
    return amplitude * math.sin(math.pi * frequency * time + phase)

def generate_forward_backward_motion(amplitude, time, cycle_duration=2.0, num_cycles=3, reverse=False):
    """
    生成向前90度然后向后90度的运动，重复指定次数
    :param amplitude: 运动幅度（90度）
    :param time: 当前时间
    :param cycle_duration: 一个完整周期的时间（向前+向后）
    :param num_cycles: 重复次数
    :param reverse: 是否反向运动（True时先向后再向前）
    :return: 关节角度
    """
    total_duration = cycle_duration * num_cycles
    
    # 如果超过总时间，保持在最后位置
    if time >= total_duration:
        return 0.0
    
    # 计算当前在第几个周期内
    current_cycle_time = time % cycle_duration
    half_cycle = cycle_duration / 2.0
    
    if not reverse:
        # 正向运动：先向前，再向后
        if current_cycle_time < half_cycle:
            # 向前运动：0 -> 90度
            progress = current_cycle_time / half_cycle
            return amplitude * progress
        else:
            # 向后运动：90度 -> -90度
            progress = (current_cycle_time - half_cycle) / half_cycle
            return amplitude * (1.0 - 2.0 * progress)
    else:
        # 反向运动：先向后，再向前
        if current_cycle_time < half_cycle:
            # 向后运动：0 -> -90度
            progress = current_cycle_time / half_cycle
            return -amplitude * progress
        else:
            # 向前运动：-90度 -> 90度
            progress = (current_cycle_time - half_cycle) / half_cycle
            return amplitude * (-1.0 + 2.0 * progress)

def main():
    # Initialize ROS node
    rospy.init_node('hand_position_control', anonymous=True)
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 创建数据保存目录
    save_dir = os.path.join(script_dir, f'experiment_data_{time.strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(save_dir, exist_ok=True)
    rospy.loginfo(f"数据将保存在: {save_dir}")
    
    # 创建数据收集器
    data_collector = DataCollector()
    
    # Create publishers
    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    marker_pub = rospy.Publisher('/hand_position_markers', MarkerArray, queue_size=10)
    
    # 等待话题可用
    rospy.loginfo("等待话题就绪...")
    rospy.wait_for_message('/humanoid_controller/optimizedState_mrt/base/pos_xyz', Float64MultiArray, timeout=10.0)
    rospy.wait_for_message('/humanoid_controller/com/r', Float64MultiArray, timeout=10.0)
    
    # 尝试等待脚部位置话题
    foot_data_available = False
    try:
        rospy.wait_for_message('/humanoid/current_foot_center_pose', Float32MultiArray, timeout=5.0)
        rospy.loginfo("直接脚部中心位置话题可用，将使用精确的脚部位置数据")
        foot_data_available = True
    except rospy.ROSException:
        try:
            rospy.wait_for_message('/sensors_data_raw', sensorsData, timeout=5.0)
            rospy.loginfo("传感器数据话题可用，将从关节角度计算脚部位置")
            foot_data_available = True
        except rospy.ROSException:
            rospy.logwarn("脚部位置相关话题不可用，将只分析质心与基准位置的偏差")
    
    rospy.loginfo("话题已就绪")
    
    rospy.sleep(1.0)  # Wait for publishers to initialize
    
    print("\n=== 关节运动测试 ===")
    print("按 Ctrl+C 退出")
    print("将先进行单关节测试，然后进行对称关节对测试")
    print("===========================\n")
    
    # 开始数据收集
    data_collector.start_collection()
    rospy.loginfo("数据收集器已启动")
    
    # 定义每个关节的正弦运动参数 (振幅, 频率, 相位)
    joint_params = {
        # 左臂关节参数
        'left': [
            (90.0, 0.5, -math.pi/2),    # zarm_l1_link
            (90.0, 0.5, math.pi/4),     # zarm_l2_link
            (90.0, 0.5, math.pi/2),     # zarm_l3_link
            (90.0, 0.5, 3*math.pi/4),   # zarm_l4_link (跳过)
            (90.0, 0.5, -math.pi),      # zarm_l5_link
            (90.0, 0.5, -5*math.pi/4),  # zarm_l6_link
            (90.0, 0.5, 3*math.pi/2),   # zarm_r7_link
        ],
        # 右臂关节参数
        'right': [
            (90.0, 0.5, -math.pi/2),    # zarm_r1_link
            (90.0, 0.5, -math.pi/4),    # zarm_r2_link
            (90.0, 0.5, -math.pi/2),    # zarm_r3_link
            (90.0, 0.5, 3*math.pi/4),   # zarm_r4_link (跳过)
            (90.0, 0.5, -math.pi),      # zarm_r5_link
            (90.0, 0.5, 5*math.pi/4),   # zarm_r6_link
            (90.0, 0.5, 3*math.pi/2),   # zarm_r7_link
        ]
    }
    
    # 测试时间（秒）
    TEST_DURATION = 3
    PAUSE_DURATION = 1  # 动作间停顿时间
    current_joint_index = 0
    current_test_phase = 0  # 0: 单关节测试, 1: 对称关节对测试
    start_time = time.time()
    rate = rospy.Rate(10)  # 10Hz
    
    def is_skipped_joint(joint_idx):
        return joint_idx == 3  # 跳过第四组关节
    
    # 新增：对称关节测试的配置
    def get_symmetric_test_duration(joint_idx):
        """根据关节索引返回对称测试的持续时间"""
        if joint_idx == 0:  # 第一组：双手同步向前90度然后向后90度，重复3次
            return 6.0  # 每个周期2秒，重复3次，总共6秒
        elif joint_idx == 1:  # 第二组：左手和右手都执行3次90度运动，右手方向相反
            return 6.0  # 双手同步运动，需要6秒完成3次循环
        else:
            return 0  # 其他组不做测试
    
    def should_skip_symmetric_test(joint_idx):
        """判断是否跳过对称关节测试"""
        return joint_idx >= 2  # 只测试第一组和第二组
    
    def get_next_test():
        nonlocal current_joint_index, current_test_phase
        
        if current_test_phase == 0:  # 单关节测试
            current_joint_index += 1
            if current_joint_index >= len(joint_params['left']) * 2:  # 所有单关节测试完成
                current_test_phase = 1  # 切换到对称关节对测试
                current_joint_index = 0
                print("\n=== 开始对称关节对测试 ===")
                print("第一组：双手同步向前90度然后向后90度，重复3次(6秒)")
                print("第二组：左手和右手都执行3次90度运动，右手方向相反(6秒)")
                print("其他组：跳过测试")
            else:
                print(f"\n开始测试第 {current_joint_index + 1} 个单关节")
        else:  # 对称关节对测试
            current_joint_index += 1
            if current_joint_index >= len(joint_params['left']):  # 所有对称关节对测试完成
                return True  # 表示所有测试完成
            elif should_skip_symmetric_test(current_joint_index):
                print(f"\n跳过第 {current_joint_index + 1} 对对称关节测试")
            else:
                duration = get_symmetric_test_duration(current_joint_index)
                print(f"\n开始测试第 {current_joint_index + 1} 对对称关节 (测试时长: {duration}秒)")
        return False
    
    try:
        while not rospy.is_shutdown():
            current_time = time.time() - start_time
            
            # 获取当前测试的持续时间
            if current_test_phase == 0:  # 单关节测试
                test_duration = TEST_DURATION
            else:  # 对称关节对测试
                test_duration = get_symmetric_test_duration(current_joint_index)
                # 如果当前关节组被跳过，立即进入下一个
                if should_skip_symmetric_test(current_joint_index):
                    test_duration = 0
            
            # 如果当前测试完成，切换到下一个
            if current_time >= test_duration:
                # 回到初始位置并等待
                left_joint_angles = [0.0] * len(joint_params['left'])
                right_joint_angles = [0.0] * len(joint_params['right'])
                publish_arm_joints(pub, marker_pub, left_joint_angles, right_joint_angles)
                rospy.sleep(PAUSE_DURATION)
                
                # 保存当前测试的数据
                results = data_collector.stop_collection()
                if results:
                    # 生成测试名称
                    if current_test_phase == 0:
                        is_left_arm = current_joint_index < len(joint_params['left'])
                        joint_idx = current_joint_index if is_left_arm else current_joint_index - len(joint_params['left'])
                        test_name = f"single_joint_{'left' if is_left_arm else 'right'}_{joint_idx + 1}"
                    else:
                        test_name = f"paired_joints_{current_joint_index + 1}"
                    
                    plot_results(results, save_dir, test_name)
                    print(f"\n数据已保存到目录: {os.path.join(save_dir, test_name)}")
                
                # 获取下一个测试
                test_completed = get_next_test()
                
                # 如果所有测试完成，退出
                if test_completed:
                    print("\n所有测试完成")
                    break
                
                # 如果是要跳过的关节，直接进入下一个
                if is_skipped_joint(current_joint_index):
                    get_next_test()
                
                # 开始新的数据收集
                data_collector.start_collection()
                start_time = time.time()
                current_time = 0
            
            # 初始化所有关节角度为0
            left_joint_angles = [0.0] * len(joint_params['left'])
            right_joint_angles = [0.0] * len(joint_params['right'])
            
            if current_test_phase == 0:  # 单关节测试
                # 确定当前测试的是左臂还是右臂的关节
                is_left_arm = current_joint_index < len(joint_params['left'])
                joint_idx = current_joint_index if is_left_arm else current_joint_index - len(joint_params['left'])
                
                # 只设置当前测试的关节角度
                if is_left_arm:
                    amp, freq, phase = joint_params['left'][joint_idx]
                    left_joint_angles[joint_idx] = generate_sine_wave(amp, freq, phase, current_time)
                else:
                    amp, freq, phase = joint_params['right'][joint_idx]
                    right_joint_angles[joint_idx] = generate_sine_wave(amp, freq, phase, current_time)
            else:  # 对称关节对测试
                # 只有不被跳过的关节组才执行测试
                if not should_skip_symmetric_test(current_joint_index):
                    # 设置当前测试的对称关节对的角度
                    amp_left, freq_left, phase_left = joint_params['left'][current_joint_index]
                    amp_right, freq_right, phase_right = joint_params['right'][current_joint_index]
                    
                    if current_joint_index == 0:  # 第一组：向前90度然后向后90度，重复3次
                        left_joint_angles[current_joint_index] = generate_forward_backward_motion(90.0, current_time, cycle_duration=2.0, num_cycles=3)
                        right_joint_angles[current_joint_index] = generate_forward_backward_motion(90.0, current_time, cycle_duration=2.0, num_cycles=3)
                    elif current_joint_index == 1:  # 第二组：左手和右手都执行三次同步90度运动，右手方向相反
                        # 左手：执行3次90度运动（正方向）
                        left_joint_angles[current_joint_index] = generate_forward_backward_motion(90.0, current_time, cycle_duration=2.0, num_cycles=3, reverse=False)
                        
                        # 右手：执行3次90度运动（反方向）
                        right_joint_angles[current_joint_index] = generate_forward_backward_motion(90.0, current_time, cycle_duration=2.0, num_cycles=3, reverse=True)
            
            # 发布关节角度
            publish_arm_joints(pub, marker_pub, left_joint_angles, right_joint_angles)
            
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n退出...")
        # 停止数据收集并分析
        results = data_collector.stop_collection()
        if results:
            plot_results(results, save_dir, "interrupted_test")
            print(f"\n数据已保存到目录: {os.path.join(save_dir, 'interrupted_test')}")
    except Exception as e:
        print(f"错误: {e}")
        # 确保在发生错误时也保存数据
        results = data_collector.stop_collection()
        if results:
            plot_results(results, save_dir, "error_test")
            print(f"\n错误数据已保存到目录: {os.path.join(save_dir, 'error_test')}")
    
    # 生成完整轨迹分析（所有测试的累积数据）
    print("\n=== 生成完整轨迹分析 ===")
    full_trajectory_data = data_collector.get_full_trajectory_data()
    if full_trajectory_data:
        # 生成完整轨迹的综合分析图
        plot_results(full_trajectory_data, save_dir, "full_trajectory_analysis")
        
        # 生成COM和脚部中心分析
        if 'feet_center_positions' in full_trajectory_data and full_trajectory_data['feet_center_positions'] is not None:
            create_com_feet_analysis_plot(full_trajectory_data, save_dir)
            generate_com_statistics_report(full_trajectory_data, save_dir)
        
        print(f"\n完整轨迹分析已保存到目录: {os.path.join(save_dir, 'full_trajectory_analysis')}")
        print("包含所有测试的累积轨迹数据和详细分析")
    else:
        print("\n没有足够的轨迹数据进行完整分析")

if __name__ == '__main__':
    main() 