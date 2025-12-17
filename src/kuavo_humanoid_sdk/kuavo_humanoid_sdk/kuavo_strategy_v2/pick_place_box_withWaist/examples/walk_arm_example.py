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
    walk_and_move_arm_together,
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
    arm_control_mode='fixed_base',  # 手臂控制模式
    pos_threshold=0.20,  # 手臂位置阈值，单位米
    angle_threshold=np.deg2rad(20),  # 手臂角度阈值，单位弧度
)

success = walk_and_move_arm_together(
    walk_event=walk_event,
    arm_event=arm_event,
)

while True:
    print(f'=------ loop ------=')
print("walk arm success:", success)
