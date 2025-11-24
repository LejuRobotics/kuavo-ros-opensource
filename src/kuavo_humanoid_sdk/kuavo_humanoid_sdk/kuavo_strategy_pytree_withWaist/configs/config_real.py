import numpy as np

class config:
    class common:
        """通用配置"""
        step_back_distance = 0.0
        enable_head_tracking = True
        half_fov = 60  # 半视场角度，单位度
        walk_timeout = 50  # 走路事件的超时时间，单位秒
        head_timeout = 30  # 头部移动事件的超时时间，单位秒
        arm_timeout = 50  # 手臂移动事件的超时时间，单位秒

        walk_yaw_threshold = np.deg2rad(5)  # 走路事件的偏航角度阈值，单位弧度
        walk_pos_threshold = 0.15  # 走路事件的位置阈值，单位米

        head_search_yaws = [12, -12]  # 头部搜索的偏航角度范围，单位度
        head_search_pitchs = [-30, -15, 0, 15, 30]  # 头部搜索的俯仰角度范围，单位度
        rotate_body = False  # 允许身体旋转以寻找目标

        arm_control_mode = 'fixed_base'  # 手臂控制模式，使用Manipulation MPC控制模式; fixed_base; manipulation_mpc
        arm_pos_threshold = 0.15  # 手臂位置阈值，单位米
        arm_angle_threshold = np.deg2rad(20)  # 手臂角度阈值，单位弧度

        enable_percep_when_walking = False # 是否在走路时启用感知(边走边看)

        box_width = 0.37  # 米
        box_length = 0.25
        box_mass = 1.5 # kg，假设一个较重的箱子

        walk_use_cmd_vel = True  # 是否使用cmd_vel控制走路
        enable_step_pause = False  # 是否启用步骤间暂停功能

    class pick:
        """搬框配置"""
        tag_id = 1
        box_width = 0.37  # 米
        box_length = 0.25
        tag_pos_world = (10, 0, 0)  # 初始位置猜测，单位米
        tag_euler_world = (0, 0, 0)  # 初始姿态猜测，单位欧拉角（弧度）
        box_in_tag_pos = (0.0, 0.0, 0.0)  # 箱子在目标标签中的位置猜测，单位米
        box_in_tag_euler = (0.0, 0.0, 0.0)  # 箱子在目标标签中的姿态猜测，单位欧拉角（弧度）

        stand_in_tag_pos = (-(0.3 + box_length/2), 0.0, -box_width/2)  # 站立位置在目标标签中的位置猜测，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）

        hand_pitch_degree = 0.0  # 手臂pitch角度（相比水平, 下倾是正），单位度
        tag_dx = 0.05  # 箱子在tag坐标系x方向偏移的距离，单位米
        tag_dy = 0.1  # 箱子在tag坐标系y方向偏移的距离，单位米
        tag_dz = 0.0 - box_width/2  # 箱子在tag坐标系z方向偏移的距离，单位米

        force_ratio_z = 0.0
        lateral_force = 10.0  # 侧向夹持力，单位N

        waist_pos = -90.0 #转腰角度
        side_step_distance = 0.0  # 侧移距离，单位米（正值为向右移动）

    class place:
        """放框配置"""
        tag_id = 0
        box_width = 0.37  # 米
        box_length = 0.25
        tag_pos_world = (10, 0, 0)  # 放置位置猜测，单位米
        tag_euler_world = (0, 0, 0)  # 放置位置姿态猜测，单位欧拉角（弧度）
        stand_in_tag_pos = (0.4 + box_length/2, 0.0, -box_width/2)  # 放置位置站立位置在目标标签中的位置猜测，单位米
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # 放置位置站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）

        tag_dx = box_length/2  # 箱子在tag坐标系x方向偏移的距离，单位米
        tag_dy = 0.1  # 箱子在tag坐标系y方向偏移的距离，单位米
        tag_dz = 0.0 - box_width/2  # 箱子在tag坐标系z方向偏移的距离，单位米

        force_ratio_z = 0.0
        lateral_force = 10.0  # 侧向夹持力，单位N

        waist_pos = 90.0 #转腰角度
        side_step_distance = -0.4  # 侧移距离，单位米（正值为向右移动）