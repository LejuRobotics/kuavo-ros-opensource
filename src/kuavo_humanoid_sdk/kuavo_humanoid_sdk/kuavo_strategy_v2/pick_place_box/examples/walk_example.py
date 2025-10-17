import numpy as np

# from configs.config_real import config
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup import init_logging
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy import (
    search_tag_with_head,
    walk_approach_target_with_perception_loop,
    grab_box_and_backward,
    place_box_and_backward,
    return_to_idle
)

# 初始化机器人
robot_sdk = RobotSDK()
walk_event = EventWalkToPose(
    robot_sdk=robot_sdk,
    timeout=20,  # 走路事件的超时时间，单位秒
    yaw_threshold=np.deg2rad(10),  # 走路事件的偏航角度阈值，单位弧度
    pos_threshold=0.2,  # 走路事件的位置阈值，单位米
    control_mode='cmd_pos_world'  # 使用世界坐标系的命令位置控制模式
)

percep_event = EventPercep(
    robot_sdk=robot_sdk,
    half_fov=60,  # 半视场角度，单位度
    timeout=np.inf,  # 头部移动事件的超时时间，单位秒
)

fake_target_tag = Tag(
    id=1,  # 假设目标箱子的ID为1
    pose=Pose.from_euler(
        pos=(1.5, 0.0, 0.96),  # 初始位置猜测，单位米
        euler=(90, 0, -90),  # 初始姿态猜测，单位欧拉角（弧度）
        frame=Frame.ODOM,  # 使用里程计坐标系
        degrees=True
    )
)

success, latest_tag = walk_approach_target_with_perception_loop(
    walk_event=walk_event,
    percep_event=percep_event,
    tag=fake_target_tag,
    stand_pose_in_tag=Pose.from_euler(
        pos=(0.0, 0.0, 0.40),  # 站立位置在目标标签中的位置猜测，单位米
        euler=(-np.deg2rad(90), np.deg2rad(90), 0.0),  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
        frame=Frame.TAG,  # 使用标签坐标系
        degrees=False
    ),
    enable_percep_when_walking=False
)

