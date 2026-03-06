#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- default_config_path(): 返回默认配置文件路径
- load_keyframes(config_path): 读取 YAML 关键帧配置

YAML 关键帧格式至少包含：
    - time: float
    - torso_pose: [x, y, z, roll, pitch, yaw]
    - joint_angles: [j1, j2, ..., j14]  # 14个关节角度

可选标签字段（任意其一存在即视为被标记）：
    - mark | flag | label | tag | marker | signal | emit_flag

加载后每个 keyframe 为：
{
    'time': float,
    'torso_pose': np.ndarray(6),  # [x, y, z, roll, pitch, yaw]
    'joint_angles': np.ndarray(14),  # 14个关节角度
    'mark': bool  # 若在 YAML 中提供了任一标签字段，则为 True，否则 False
}
"""
import os
import yaml
import numpy as np


def default_config_path():
    """返回默认配置文件路径"""
    return os.path.join(os.path.dirname(__file__), "../config/target_joint.yaml")


def load_keyframes(config_path=None):
    """
    读取关键帧配置并进行基本整理。
    
    Args:
        config_path (str, optional): 配置文件路径。如果为 None，则使用默认路径。
    
    Returns:
        list: 关键帧列表，每个关键帧包含：
            - time (float): 时间（秒）
            - torso_pose (np.ndarray): 躯干位姿 [x, y, z, roll, pitch, yaw]
            - joint_angles (np.ndarray): 14个关节角度
            - mark (bool): 是否标记
    
    Raises:
        ValueError: 配置文件格式错误或缺少必需字段
        FileNotFoundError: 配置文件不存在
    """
    if config_path is None:
        config_path = default_config_path()
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    
    traj = cfg.get('trajectory', [])
    if not traj:
        raise ValueError('配置文件中 trajectory 为空')
    
    # 校验和规范化关键帧
    keyframes = []
    for idx, kf in enumerate(traj):
        # 校验 time 字段
        if 'time' not in kf or kf['time'] is None or (isinstance(kf['time'], str) and kf['time'] == ''):
            raise ValueError(f'第 {idx} 个关键帧缺少有效的 time 字段')
        
        time_val = float(kf['time'])
        
        # 校验 torso_pose 字段
        if 'torso_pose' not in kf:
            raise ValueError(f'第 {idx} 个关键帧缺少 torso_pose 字段')
        
        torso_pose = np.array(kf['torso_pose'], dtype=float)
        if len(torso_pose) != 6:
            raise ValueError(f'第 {idx} 个关键帧 torso_pose 长度应为 6，实际为 {len(torso_pose)}')
        
        # 校验 joint_angles 字段
        if 'joint_angles' not in kf:
            raise ValueError(f'第 {idx} 个关键帧缺少 joint_angles 字段')
        
        joint_angles = np.array(kf['joint_angles'], dtype=float)
        if len(joint_angles) != 14:
            raise ValueError(f'第 {idx} 个关键帧 joint_angles 长度应为 14，实际为 {len(joint_angles)}')
        
        # 处理可选标签字段
        mark_fields = ['mark', 'flag', 'label', 'tag', 'marker', 'signal', 'emit_flag']
        mark_val = False
        for mf in mark_fields:
            if mf in kf and kf[mf] is not None:
                try:
                    # 支持 bool / 数字 / 字符串("true"/"1")
                    v = kf[mf]
                    if isinstance(v, str):
                        vnorm = v.strip().lower()
                        mark_val = vnorm in ('1', 'true', 'yes', 'y', 'on')
                    else:
                        mark_val = bool(v)
                except Exception:
                    mark_val = False
                break
        
        keyframes.append({
            'time': time_val,
            'torso_pose': torso_pose,
            'joint_angles': joint_angles,
            'mark': bool(mark_val),
        })
    
    # 按时间排序
    keyframes.sort(key=lambda x: x['time'])
    
    return keyframes

