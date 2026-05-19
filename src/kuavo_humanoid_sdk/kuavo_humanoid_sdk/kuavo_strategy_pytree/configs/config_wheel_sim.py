import numpy as np


class config:
    class common:
        """通用配置"""
        box_width = 0.4  # 箱子宽度，单位米
        is_chassis_move_sync = False  # 底盘移动x,y,yaw三轴运动是否同步（True=同步，False=非同步）
        box_weight_kg = 14.0  # 箱子重量，单位千克

        contact_force_weight_transition_time = 0.5
        contact_force_interpolation_speed = 50.0
        
        tag_euler_error = 15.0 # 相机识别结果与预期 tag_euler_world 的角度误差阈值（度）
        tag_euler_error_validate_yaw = True # 是否审查 yaw 角误差，（True：审查 roll+pitch+yaw，False：仅审查 roll+pitch）

        head_move_delay = 0.8 # 头部扫描后延时，单位秒

    class fake_tags:
        """虚假 tag 配置（跳过视觉识别，用于测试）"""
        enable = False  # True 时跳过视觉识别
        pick_pos = (0.0, -1.755, 0.885)   # pick tag 虚假位置 (x,y,z) 米，ODOM 系
        pick_euler = (90, 0, 180)     # pick tag 虚假姿态 (roll,pitch,yaw) 度
        place_pos = (0.0, 1.645, 1.435)   # place tag 虚假位置
        place_euler = (90, 0, 0)      # place tag 虚假姿态

    class pick:
        """搬框配置"""
        tag_id = 1
        tag_euler_world = (90, 0, 180)  # 初始姿态猜测，单位欧拉角（弧度）

        stand_in_tag_pos = (0.0, 0.0, 0.6)  # 站立位置在目标标签中的位置，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 站立位置在目标标签中的姿态，单位欧拉角（弧度）
        box_behind_tag = 0.1  # 箱子在tag后面的距离，单位米
        box_beneath_tag = 0.0  # 箱子在tag下方的距离，单位米
        box_left_tag = -0.0  # 箱子在tag左侧的距离，单位米

    class place:
        """放框配置"""
        tag_id = 0
        tag_euler_world = (90, 0, 0)  # 放置位置姿态猜测，单位欧拉角（弧度）

        stand_in_tag_pos = (0.0, 0.0, 0.6)  # 放置位置站立位置在目标标签中的位置，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 放置位置站立位置在目标标签中的姿态，单位欧拉角（弧度）
        box_behind_tag = 0.1  # 箱子在tag后面的距离，单位米
        box_beneath_tag = 0.5  # 箱子在tag下方的距离，单位米
        box_left_tag = 0.0  # 箱子在tag左侧的距离，单位米
