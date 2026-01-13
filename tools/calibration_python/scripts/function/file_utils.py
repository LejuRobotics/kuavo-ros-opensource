#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件路径工具函数：
- 获取配置目录路径
- 其他文件/路径相关工具函数
"""

import os


def get_config_dir(script_file=None):
    """
    获取 config 目录路径，如果不存在则创建
    
    说明：
        相对于脚本所在目录的上一级目录下的 config 目录
        例如：scripts/generate_arm_joints.py -> calibration_python/config/
    
    Args:
        script_file: 可选，调用脚本的 __file__ 路径。如果不提供，将尝试从调用栈获取
    
    Returns:
        str: config 目录的绝对路径
    """
    if script_file is None:
        # 尝试从调用栈获取调用者的 __file__
        import inspect
        frame = inspect.currentframe()
        try:
            caller_frame = frame.f_back
            while caller_frame:
                caller_file = caller_frame.f_globals.get('__file__')
                if caller_file and not caller_file.endswith('file_utils.py'):
                    script_file = caller_file
                    break
                caller_frame = caller_frame.f_back
        finally:
            del frame
    
    if script_file:
        script_dir = os.path.dirname(os.path.abspath(script_file))
        # config 目录在 scripts 目录的上一级
        config_dir = os.path.join(os.path.dirname(script_dir), 'config')
    else:
        # 如果无法获取，使用当前工作目录的上一级
        config_dir = os.path.join(os.path.dirname(os.getcwd()), 'config')
    
    os.makedirs(config_dir, exist_ok=True)
    return config_dir


def format_with_sign(value, precision=1):
    """
    格式化数值，保留符号位（正数前面加空格，负数用负号）
    
    Args:
        value: 要格式化的数值
        precision: 小数位数
    
    Returns:
        str: 格式化后的字符串
    """
    if value < 0:
        return f"{value:.{precision}f}"
    else:
        return f" {value:.{precision}f}"

