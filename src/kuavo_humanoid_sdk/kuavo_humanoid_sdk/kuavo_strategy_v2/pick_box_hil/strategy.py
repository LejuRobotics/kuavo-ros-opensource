import time
from typing import List, Dict, Any, Tuple
import numpy as np


from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus


def move_arm_with_hil(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        step_back_distance: float,  # 向后平移的距离，单位米
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None  # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
):
    """
    使用HIL（Human-in-the-loop）方式移动手臂。即：每到arm_traj中的一个关键点之后，会进入到hil阶段，此时可以通过键盘来移动手臂，
    """
    arm_event.open()  # 打开手臂事件
    if not arm_event.set_target(arm_traj, arm_wrench=arm_wrench, tag=tag):
        print("❌ 设置手臂key point失败")
        return False

    while True:
        arm_status = arm_event.step()
        if arm_status != EventStatus.RUNNING:
            break

    if arm_status != EventStatus.SUCCESS:
        print("❌ 手臂移动失败，退出策略。")
        arm_event.close()
        return False

    print("✅ 已成功移动手臂，开始向后平移...")
    arm_event.close()

    walk_event.open()
    walk_event.set_control_mode('cmd_pos')  # 使用相对位置控制模式

    walk_event.set_target(
        Pose(
            pos=(-step_back_distance, 0., 0.),  # 向后平移
            quat=(0, 0, 0, 1),  # 保持姿态不变
            frame=Frame.BASE  # 使用基座坐标系
        )
    )

    while True:
        walk_status = walk_event.step()
        if walk_status != EventStatus.RUNNING:
            break

    if walk_status != EventStatus.SUCCESS:
        print("❌ 向后平移失败，退出策略。")
        walk_event.close()
        return False

    print("✅ 已成功向后平移，策略完成。")
    walk_event.close()
    return True