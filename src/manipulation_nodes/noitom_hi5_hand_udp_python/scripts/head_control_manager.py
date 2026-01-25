#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部控制管理器
实现四种头部控制模式：
1. 固定头部控制模式
2. 自动跟踪主动手控制模式
3. 固定主手控制模式
4. VR随动控制模式
"""

import rospy
import math
import numpy as np
from kuavo_ros_interfaces.msg import robotHeadMotionData


class HeadControlMode:
    """头部控制模式枚举"""
    FIXED = 0              # 固定头部控制模式
    AUTO_TRACK_ACTIVE = 1  # 自动跟踪主动手控制模式
    FIXED_MAIN_HAND = 2    # 固定主手控制模式
    VR_FOLLOW = 3          # VR随动控制模式
    
    @staticmethod
    def from_string(mode_str):
        """
        从字符串转换为模式枚举值
        
        Args:
            mode_str: 模式字符串 ("fixed", "auto_track_active", "fixed_main_hand", "vr_follow")
            
        Returns:
            HeadControlMode枚举值，如果无效则返回None
        """
        mode_map = {
            "fixed": HeadControlMode.FIXED,
            "auto_track_active": HeadControlMode.AUTO_TRACK_ACTIVE,
            "fixed_main_hand": HeadControlMode.FIXED_MAIN_HAND,
            "vr_follow": HeadControlMode.VR_FOLLOW
        }
        return mode_map.get(mode_str.lower(), None)
    
    @staticmethod
    def to_string(mode):
        """
        从模式枚举值转换为字符串
        
        Args:
            mode: HeadControlMode枚举值
            
        Returns:
            模式字符串
        """
        mode_map = {
            HeadControlMode.FIXED: "fixed",
            HeadControlMode.AUTO_TRACK_ACTIVE: "auto_track_active",
            HeadControlMode.FIXED_MAIN_HAND: "fixed_main_hand",
            HeadControlMode.VR_FOLLOW: "vr_follow"
        }
        return mode_map.get(mode, "unknown")


class HeadControlManager:
    """头部控制管理器"""
    
    def __init__(self, config_manager=None):
        """
        初始化头部控制管理器
        
        Args:
            config_manager: Quest3VrConfig 配置管理器实例
        """
        self.config_manager = config_manager
        self.mode = HeadControlMode.VR_FOLLOW  # 默认VR随动模式
        self.fixed_main_hand = "right"  # 固定主手模式时跟踪的手（"left" 或 "right"）
        
        # 头部目标位置（度）
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        
        # 平滑滤波参数（将从配置读取，这里只是占位符）
        self.smoothing_factor = None
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        # 主动手检测参数（将从配置读取，这里只是占位符）
        self.active_hand_threshold = None
        self.left_hand_moving = False
        self.right_hand_moving = False
        self.last_left_hand_pos = None
        self.last_right_hand_pos = None
        
        # 头部关节限制（度）（将从配置读取，这里只是占位符）
        self.yaw_limit = None
        self.pitch_limit = None
        
        # 模式切换时的平滑过渡标志
        self.mode_changed = False
        self.last_mode = None
        
    def set_mode(self, mode, fixed_hand="right"):
        """
        设置头部控制模式
        
        Args:
            mode: HeadControlMode 枚举值
            fixed_hand: 固定主手模式时的手（"left" 或 "right"）
        """
        if mode != self.mode:
            self.last_mode = self.mode
            self.mode_changed = True
            rospy.loginfo(f"Head control mode changed from {self.last_mode} to {mode}")
        
        self.mode = mode
        if mode == HeadControlMode.FIXED_MAIN_HAND:
            self.fixed_main_hand = fixed_hand
        rospy.loginfo(f"Head control mode set to: {mode}, fixed_hand: {fixed_hand}")
    
    def set_joint_limits(self, yaw_limit, pitch_limit):
        """
        设置头部关节限制
        
        Args:
            yaw_limit: [min, max] yaw角度限制（度）
            pitch_limit: [min, max] pitch角度限制（度）
        """
        self.yaw_limit = yaw_limit
        self.pitch_limit = pitch_limit
    
    def set_smoothing_factor(self, factor):
        """
        设置平滑滤波系数
        
        Args:
            factor: 平滑系数（0-1，越小越平滑）
        """
        self.smoothing_factor = max(0.0, min(1.0, factor))
    
    def set_active_hand_threshold(self, threshold):
        """
        设置主动手检测阈值
        
        Args:
            threshold: 手部移动阈值（米）
        """
        self.active_hand_threshold = threshold
    
    def calculate_head_pose_from_hand(self, hand_tcp_pos, head_pos):
        """
        从机器人本体手部末端TCP位置计算头部目标姿态
        
        Args:
            hand_tcp_pos: 机器人手部末端TCP位置 [x, y, z] (相对于base_link，单位：米)
            head_pos: 机器人头部位置 [x, y, z] (相对于base_link，单位：米)
            
        Returns:
            (yaw, pitch): 头部目标角度（度）
        """
        if hand_tcp_pos is None or head_pos is None:
            return None, None
        
        # 计算机器人本体手部末端TCP到头的相对距离
        dx = hand_tcp_pos[0] - head_pos[0]
        dy = hand_tcp_pos[1] - head_pos[1]
        dz = hand_tcp_pos[2] - head_pos[2]
        
        # 计算yaw和pitch（度）
        yaw = math.degrees(math.atan2(dy, dx))
        distance_xy = math.sqrt(dx*dx + dy*dy)
        pitch = -math.degrees(math.atan2(dz, distance_xy))
        
        # 限制范围
        yaw = max(self.yaw_limit[0], min(self.yaw_limit[1], yaw))
        pitch = max(self.pitch_limit[0], min(self.pitch_limit[1], pitch))
        
        return yaw, pitch
    
    def detect_active_hand(self, left_hand_pos, right_hand_pos):
        """
        检测主动手
        
        Args:
            left_hand_pos: 左手位置 [x, y, z]
            right_hand_pos: 右手位置 [x, y, z]
            
        Returns:
            "left", "right", 或 None
        """
        left_movement = 0.0
        right_movement = 0.0
        
        # 检测左手是否移动
        if left_hand_pos is not None:
            if self.last_left_hand_pos is not None:
                left_movement = math.sqrt(sum((a-b)**2 for a, b in 
                                            zip(left_hand_pos, self.last_left_hand_pos)))
                self.left_hand_moving = left_movement > self.active_hand_threshold
            else:
                self.left_hand_moving = False
                self.last_left_hand_pos = left_hand_pos
        else:
            self.left_hand_moving = False
        
        # 检测右手是否移动
        if right_hand_pos is not None:
            if self.last_right_hand_pos is not None:
                right_movement = math.sqrt(sum((a-b)**2 for a, b in 
                                             zip(right_hand_pos, self.last_right_hand_pos)))
                self.right_hand_moving = right_movement > self.active_hand_threshold
            else:
                self.right_hand_moving = False
                self.last_right_hand_pos = right_hand_pos
        else:
            self.right_hand_moving = False
        
        # 更新上一帧位置
        if left_hand_pos is not None:
            self.last_left_hand_pos = left_hand_pos
        if right_hand_pos is not None:
            self.last_right_hand_pos = right_hand_pos
        
        # 确定主动手（优先选择移动的手）
        if self.left_hand_moving and not self.right_hand_moving:
            return "left"
        elif self.right_hand_moving and not self.left_hand_moving:
            return "right"
        elif self.left_hand_moving and self.right_hand_moving:
            # 两只手都在移动，选择移动距离更大的
            return "left" if left_movement > right_movement else "right"
        else:
            return None  # 没有主动手
    
    def smooth_update(self, target_yaw, target_pitch):
        """
        平滑更新头部目标位置
        
        Args:
            target_yaw: 目标yaw（度）
            target_pitch: 目标pitch（度）
        """
        # 模式切换时，重置滤波器状态以实现平滑过渡
        if self.mode_changed:
            self.current_yaw = target_yaw
            self.current_pitch = target_pitch
            self.mode_changed = False
            rospy.loginfo("Head control mode transition: resetting filter state")
        
        # 一阶低通滤波
        self.current_yaw = (1 - self.smoothing_factor) * self.current_yaw + \
                          self.smoothing_factor * target_yaw
        self.current_pitch = (1 - self.smoothing_factor) * self.current_pitch + \
                            self.smoothing_factor * target_pitch
        
        self.target_yaw = self.current_yaw
        self.target_pitch = self.current_pitch
    
    def update(self, left_hand_tcp_pos=None, right_hand_tcp_pos=None, head_pos=None):
        """
        更新头部控制
        
        Args:
            left_hand_tcp_pos: 机器人左手末端TCP位置 [x, y, z]（相对于base_link，单位：米）
            right_hand_tcp_pos: 机器人右手末端TCP位置 [x, y, z]（相对于base_link，单位：米）
            head_pos: 机器人头部位置 [x, y, z]（相对于base_link，单位：米）
        """
        # 如果head_pos为None，无法计算，直接返回
        if head_pos is None:
            return  # 需要从TF树获取头部位置
        
        if self.mode == HeadControlMode.FIXED:
            # 固定模式：将头部yaw和pitch设置为0（正前方）
            target_yaw = 0.0
            target_pitch = 0.0
            # 平滑更新到目标位置
            self.smooth_update(target_yaw, target_pitch)
            
        elif self.mode == HeadControlMode.AUTO_TRACK_ACTIVE:
            # 自动跟踪主动手模式
            if left_hand_tcp_pos is not None and right_hand_tcp_pos is not None:
                active_hand = self.detect_active_hand(left_hand_tcp_pos, right_hand_tcp_pos)
                if active_hand:
                    # 计算目标姿态：计算机器人本体手部末端TCP到头的相对距离
                    if active_hand == "left":
                        target_yaw, target_pitch = self.calculate_head_pose_from_hand(
                            left_hand_tcp_pos, head_pos)
                    else:
                        target_yaw, target_pitch = self.calculate_head_pose_from_hand(
                            right_hand_tcp_pos, head_pos)
                    
                    if target_yaw is not None and target_pitch is not None:
                        # 平滑更新
                        self.smooth_update(target_yaw, target_pitch)
                # 如果没有主动手，保持当前目标位置不变
                    
        elif self.mode == HeadControlMode.FIXED_MAIN_HAND:
            # 固定主手模式
            if self.fixed_main_hand == "left" and left_hand_tcp_pos is not None:
                target_yaw, target_pitch = self.calculate_head_pose_from_hand(
                    left_hand_tcp_pos, head_pos)
                if target_yaw is not None and target_pitch is not None:
                    self.smooth_update(target_yaw, target_pitch)
            elif self.fixed_main_hand == "right" and right_hand_tcp_pos is not None:
                target_yaw, target_pitch = self.calculate_head_pose_from_hand(
                    right_hand_tcp_pos, head_pos)
                if target_yaw is not None and target_pitch is not None:
                    self.smooth_update(target_yaw, target_pitch)
                
        elif self.mode == HeadControlMode.VR_FOLLOW:
            # VR随动模式：由外部直接调用pub_head_motion_data处理
            # 这里不需要更新
            pass
    
    def publish_head_command(self, head_pub):
        """
        发布头部控制命令
        
        Args:
            head_pub: robotHeadMotionData 发布器
        """
        msg = robotHeadMotionData()
        msg.joint_data = [self.target_yaw, self.target_pitch]
        head_pub.publish(msg)
    
    def get_current_target(self):
        """
        获取当前目标位置
        
        Returns:
            (yaw, pitch): 当前目标角度（度）
        """
        return self.target_yaw, self.target_pitch
    
    def reset(self):
        """重置管理器状态"""
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.last_left_hand_pos = None
        self.last_right_hand_pos = None
        self.left_hand_moving = False
        self.right_hand_moving = False
        self.mode_changed = False
        rospy.loginfo("Head control manager reset")
