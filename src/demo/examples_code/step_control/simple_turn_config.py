#!/usr/bin/env python3
"""
简单转向配置文件：用于两步转向的预设参数
"""

# 简单转向预设配置
SIMPLE_TURN_CONFIGS = {
    # 小角度转向
    "turn_15": {
        "angle": 15,
        "description": "15度小幅转向"
    },
    "turn_30": {
        "angle": 30,
        "description": "30度中等转向"
    },
    "turn_45": {
        "angle": 45,
        "description": "45度转向"
    },
    "turn_60": {
        "angle": 60,
        "description": "60度大幅转向"
    },
    "turn_90": {
        "angle": 90,
        "description": "90度直角转向"
    },
    
    # 逆时针转向
    "turn_left_30": {
        "angle": -30,
        "description": "30度左转"
    },
    "turn_left_45": {
        "angle": -45,
        "description": "45度左转"
    },
    "turn_left_90": {
        "angle": -90,
        "description": "90度左转"
    },
}

# 默认参数
DEFAULT_PARAMS = {
    "dt": 0.4,              # 每步时间间隔
    "is_left_first": None,  # 自动判断（顺时针右脚先，逆时针左脚先）
    "torso_height": 0.0,    # 躯干高度
}

def get_simple_config(config_name):
    """
    获取简单转向配置。
    
    参数：
    - config_name: 配置名称
    
    返回：
    - 配置字典，如果不存在则返回None
    """
    if config_name in SIMPLE_TURN_CONFIGS:
        config = SIMPLE_TURN_CONFIGS[config_name].copy()
        config.update(DEFAULT_PARAMS)
        return config
    return None

def list_simple_configs():
    """
    列出所有简单转向配置。
    """
    print("\033[94m=== 简单转向配置 (两步完成) ===\033[0m")
    for name, config in SIMPLE_TURN_CONFIGS.items():
        angle = config["angle"]
        direction = "顺时针" if angle > 0 else "逆时针"
        desc = config["description"]
        print(f"\033[92m{name:15}\033[0m: {desc} ({abs(angle)}° {direction})")

def validate_simple_config(angle, dt):
    """
    验证简单转向配置。
    
    参数：
    - angle: 转向角度
    - dt: 时间间隔
    
    返回：
    - (is_valid, message): 验证结果和消息
    """
    if abs(angle) > 120:
        return False, f"转向角度过大: {abs(angle)}° > 120°，建议使用多步转向"
    
    if dt < 0.2:
        return False, f"时间间隔太短: {dt}s < 0.2s"
    
    if dt > 1.0:
        return False, f"时间间隔太长: {dt}s > 1.0s"
    
    if abs(angle) < 5:
        return False, f"转向角度太小: {abs(angle)}° < 5°"
    
    return True, "配置验证通过"