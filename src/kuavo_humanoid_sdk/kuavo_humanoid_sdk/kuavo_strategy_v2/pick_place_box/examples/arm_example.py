import numpy as np

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

robot_sdk = RobotSDK()

walk_event = EventWalkToPose(
    robot_sdk=robot_sdk,
    timeout=20,  # 走路事件的超时时间，单位秒
    yaw_threshold=np.deg2rad(10),  # 走路事件的偏航角度阈值，单位弧度
    pos_threshold=0.2,  # 走路事件的位置阈值，单位米
    control_mode='cmd_pos_world'  # 使用世界坐标系的命令位置控制模式
)

arm_event = EventArmMoveKeyPoint(
    robot_sdk=robot_sdk,
    timeout=50,  # 手臂移动事件的超时时间，单位秒
    arm_control_mode='manipulation_mpc',  # 手臂控制模式
    pos_threshold=0.15,  # 手臂位置阈值，单位米
    angle_threshold=np.deg2rad(15),  # 手臂角度阈值，单位弧度
)

fake_target_tag = Tag(
    id=1,  # 假设目标箱子的ID为1
    pose=Pose.from_euler(
        pos=(0.3, 0.0, 0.96),  # 初始位置猜测，单位米
        euler=(90, 0, -90),  # 初始姿态猜测，单位欧拉角（弧度）
        frame=Frame.ODOM,  # 使用里程计坐标系
        degrees=True
    )
)

success = grab_box_and_backward(
    walk_event=walk_event,
    arm_event=arm_event,
    box_width=0.2,
    box_behind_tag=0.0,  # 箱子在tag后面的距离，单位米
    box_beneath_tag=0.0,  # 箱子在tag下方的距离，单位米
    box_left_tag=0.0,  # 箱子在tag左侧的距离，单位米
    tag=fake_target_tag,
    step_back_distance=0.5,  # 搬起后向后平移的距离，单位米

    box_mass=10,  # 假设箱子质量，单位kg，用来计算纵向wrench
    force_ratio_z=0,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
    lateral_force=0,  # 侧向夹持力，单位N
)

print("Grab box success:", success)
