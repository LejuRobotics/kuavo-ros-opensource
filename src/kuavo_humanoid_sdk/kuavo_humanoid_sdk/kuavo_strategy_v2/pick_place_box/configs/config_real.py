import numpy as np

class config:
    class common:
        """通用配置"""
        step_back_distance = 0.5
        enable_head_tracking = True
        half_fov = 60  # 半视场角度，单位度
        walk_timeout = 50  # 走路事件的超时时间，单位秒
        head_timeout = 30  # 头部移动事件的超时时间，单位秒
        arm_timeout = 50  # 手臂移动事件的超时时间，单位秒

        walk_yaw_threshold = 0.1  # 走路事件的偏航角度阈值，单位弧度
        walk_pos_threshold = 0.2  # 走路事件的位置阈值，单位米

        head_search_yaws = [12, -12]  # 头部搜索的偏航角度范围，单位度
        head_search_pitchs = [-30, -15, 0, 15, 30]  # 头部搜索的俯仰角度范围，单位度
        rotate_body = True  # 允许身体旋转以寻找目标

        arm_control_mode = 'fixed_base'  # 手臂控制模式，使用Manipulation MPC控制模式; fixed_base; manipulation_mpc
        arm_pos_threshold = 0.15  # 手臂位置阈值，单位米
        arm_angle_threshold = np.deg2rad(20)  # 手臂角度阈值，单位弧度

        enable_percep_when_walking = True # 是否在走路时启用感知(边走边看)

        box_width = 0.37  # 米
        box_mass = 1.5 # kg，假设一个较重的箱子

        walk_use_cmd_vel = True  # 是否使用cmd_vel控制走路

    class pick:
        """搬框配置"""
        tag_id = 8
        tag_pos_world = (0, -10, 0)  # 初始位置猜测，单位米
        tag_euler_world = (0, 0, 0)  # 初始姿态猜测，单位欧拉角（弧度）
        box_in_tag_pos = (0.0, 0.0, 0.0)  # 箱子在目标标签中的位置猜测，单位米
        box_in_tag_euler = (0.0, 0.0, 0.0)  # 箱子在目标标签中的姿态猜测，单位欧拉角（弧度）

        stand_in_tag_pos = (0.0, 0.0, 0.40)  # 站立位置在目标标签中的位置猜测，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）

        box_behind_tag = 0.13  # 箱子在tag后面的距离，单位米
        box_beneath_tag = -0.04  # 箱子在tag下方的距离，单位米
        box_left_tag = 0.0  # 箱子在tag左侧的距离，单位米

        force_ratio_z = 0.34
        lateral_force = 10.0  # 侧向夹持力，单位N

    class place:
        """放框配置"""
        tag_id = 1
        tag_pos_world = (0, 10, 0)  # 放置位置猜测，单位米
        tag_euler_world = (0, 0, 0)  # 放置位置姿态猜测，单位欧拉角（弧度）
        stand_in_tag_pos = (0.0, 0.0, 0.65)  # 放置位置站立位置在目标标签中的位置猜测，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 放置位置站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）

        box_behind_tag = -0.05  # 箱子在tag后面的距离，单位米
        box_beneath_tag = 0.60  # 箱子在tag下方的距离，单位米
        box_left_tag = 0.0  # 箱子在tag左侧的距离，单位米1

        force_ratio_z = 0.34
        lateral_force = 10.0  # 侧向夹持力，单位N