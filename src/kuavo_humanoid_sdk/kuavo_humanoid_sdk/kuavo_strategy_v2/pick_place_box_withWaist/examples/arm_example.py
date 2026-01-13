import time
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
        pos=(0.5, 0.0, 0.9),  # 初始位置猜测，单位米
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
    step_back_distance=0.3,  # 搬起后向后平移的距离，单位米

    box_mass=10,  # 假设箱子质量，单位kg，用来计算纵向wrench
    force_ratio_z=0,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
    lateral_force=0,  # 侧向夹持力，单位N
)

print("Grab box success:", success)

input("按回车：手臂回到零点")
# # 获取当前手臂关节位置并进行插值到零位，避免运动过快
current_joint_positions = robot_sdk.state.arm_joint_state().position  # 手臂关节位置
target_joint_positions = [0.0] * 14  # 目标零位
print(f"当前手臂关节位置：{current_joint_positions}")
# 设置插值步数，控制运动速度
interpolation_steps = 50
robot_sdk.control.control_arm_joint_positions(
        joint_positions=current_joint_positions
    )
for i in range(interpolation_steps + 1):
    # 线性插值计算中间位置
    alpha = i / interpolation_steps
    interpolated_positions = [
        current_pos + alpha * (target_pos - current_pos)
        for current_pos, target_pos in zip(current_joint_positions, target_joint_positions)
    ]
    
    # 控制手臂到插值位置
    robot_sdk.control.control_arm_joint_positions(
        joint_positions=interpolated_positions
    )
    
    # 短暂延时确保运动平滑
    time.sleep(0.02)
current_jointsPos = robot_sdk.state.arm_joint_state().position  # 手臂关节位置
print(f"最终期望关节位置：{interpolated_positions}")
print(f"最终手臂关节位置：{current_jointsPos}")

input("按回车：手臂回到初始摆动状态")
arm_event.arm_reset()