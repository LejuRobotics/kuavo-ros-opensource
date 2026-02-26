#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
关键帧插值工具函数

- generate_time_grid(keyframes, rate_hz): 生成时间网格
- interpolate_keyframes(keyframes, rate_hz): 对关键帧进行线性插值
"""
import numpy as np


def clamp(v, lo, hi):
    """限制值在范围内"""
    return max(lo, min(hi, v))


def generate_time_grid(keyframes, rate_hz):
    """
    生成时间网格
    
    Args:
        keyframes: 关键帧列表
        rate_hz: 采样频率（Hz）
    
    Returns:
        list: 时间点列表
    """
    if not keyframes:
        return []
    
    t_start = float(keyframes[0]['time'])
    t_end = float(keyframes[-1]['time'])
    dt = 1.0 / max(rate_hz, 1e-6)
    
    t = t_start
    times = []
    while t < t_end - 1e-9:
        times.append(round(t, 6))
        t += dt
    times.append(round(t_end, 6))
    
    return times


def linear_interpolate(p0, p1, t):
    """
    线性插值
    
    Args:
        p0: 起始点
        p1: 结束点
        t: 插值参数 [0, 1]
    
    Returns:
        np.ndarray: 插值结果
    """
    t = clamp(t, 0.0, 1.0)
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    
    # 线性插值公式: result = (1 - t) * p0 + t * p1
    return (1.0 - t) * p0 + t * p1


def interpolate_keyframes(keyframes, rate_hz=100):
    """
    对关键帧进行线性插值
    
    Args:
        keyframes: 关键帧列表，每个关键帧包含：
            - time: 时间
            - torso_pose: 躯干位姿 [x, y, z, roll, pitch, yaw]
            - joint_angles: 14个关节角度
        rate_hz: 采样频率（Hz），默认100Hz
    
    Returns:
        list: 插值后的序列，每个元素包含：
            - time: 时间
            - torso_pose: 躯干位姿
            - joint_angles: 关节角度
    """
    if len(keyframes) < 2:
        # 如果关键帧少于2个，直接返回
        return keyframes
    
    # 生成时间网格
    times = generate_time_grid(keyframes, rate_hz)
    
    # 提取关键帧的时间和数值
    kf_times = [kf['time'] for kf in keyframes]
    kf_torso_poses = [np.asarray(kf['torso_pose'], dtype=float) for kf in keyframes]
    kf_joint_angles = [np.asarray(kf['joint_angles'], dtype=float) for kf in keyframes]
    
    interpolated_sequence = []
    n_kf = len(keyframes)
    
    for t in times:
        # 找到当前时间所在的区间
        if t <= kf_times[0]:
            # 在第一个关键帧之前，使用第一个关键帧
            torso_pose = kf_torso_poses[0]
            joint_angles = kf_joint_angles[0]
        elif t >= kf_times[-1]:
            # 在最后一个关键帧之后，使用最后一个关键帧
            torso_pose = kf_torso_poses[-1]
            joint_angles = kf_joint_angles[-1]
        else:
            # 找到对应的区间
            segment_idx = 0
            for i in range(n_kf - 1):
                if kf_times[i] <= t <= kf_times[i + 1]:
                    segment_idx = i
                    break
            
            t0 = kf_times[segment_idx]
            t1 = kf_times[segment_idx + 1]
            
            # 归一化插值参数
            u = clamp((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0)
            
            # 获取当前段的起始点和结束点
            p0_torso = kf_torso_poses[segment_idx]
            p1_torso = kf_torso_poses[segment_idx + 1]
            
            p0_joints = kf_joint_angles[segment_idx]
            p1_joints = kf_joint_angles[segment_idx + 1]
            
            # 线性插值
            torso_pose = linear_interpolate(p0_torso, p1_torso, u)
            joint_angles = linear_interpolate(p0_joints, p1_joints, u)
        
        interpolated_sequence.append({
            'time': t,
            'torso_pose': torso_pose,
            'joint_angles': joint_angles,
        })
    
    return interpolated_sequence

