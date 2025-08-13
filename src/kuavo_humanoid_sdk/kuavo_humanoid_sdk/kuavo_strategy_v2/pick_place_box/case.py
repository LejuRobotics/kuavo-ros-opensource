from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup import init_logging
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

import numpy as np
import os, sys
mother_dir = os.path.dirname(os.path.abspath(__file__))

log_path = init_logging(log_dir=os.path.join(mother_dir, "logs"), filename_prefix="grab_box_v2", enable=True)

from configs.config_sim import config
# from configs.config_real import config

from strategy import (
    search_tag_with_head,
    walk_approach_target_with_perception_loop,
    grab_box_and_backward,
    place_box_and_backward,
    return_to_idle
)


def test_arm_only():
    """
    测试仅使用手臂的功能。
    """
    robot_sdk = RobotSDK()

    # 初始化事件
    walk_event = EventWalkToPose(
        robot_sdk=robot_sdk,
        timeout=config.common.walk_timeout,  # 走路事件的超时时间，单位秒
        yaw_threshold=config.common.walk_yaw_threshold,  # 走路事件的偏航角度阈值，单位弧度
        pos_threshold=config.common.walk_pos_threshold,  # 走路事件的位置阈值，单位米
        control_mode='cmd_pos_world'  # 使用世界坐标系的命令位置控制模式
    )
    head_event = EventHeadMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.head_timeout,  # 头部移动事件的超时时间，单位秒
    )
    percep_event = EventPercep(
        robot_sdk=robot_sdk,
        half_fov=config.common.half_fov,  # 半视场角度，单位度
        timeout=np.inf,  # 头部移动事件的超时时间，单位秒
    )
    arm_event = EventArmMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.arm_timeout,  # 手臂移动事件的超时时间，单位秒
        arm_control_mode=config.common.arm_control_mode,  # 手臂控制模式
        pos_threshold=config.common.arm_pos_threshold,  # 手臂位置阈值，单位米
        angle_threshold=config.common.arm_angle_threshold,  # 手臂角度阈值，单位弧度
    )

    fake_target_tag = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
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
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=fake_target_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.pick.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.pick.lateral_force,  # 侧向夹持力，单位N
    )

def grab_one_box(user_input=True):
    """
    执行抓取一个箱子的完整策略。

    参数：
        user_input (bool): 是否等待用户输入以继续每个步骤。

    返回：
        bool: 是否成功完成策略。
    """
    # 1. 尋找箱子

    # 初始化机器人
    robot_sdk = RobotSDK()

    # 初始化事件
    walk_event = EventWalkToPose(
        robot_sdk=robot_sdk,
        timeout=config.common.walk_timeout,  # 走路事件的超时时间，单位秒
        yaw_threshold=config.common.walk_yaw_threshold,  # 走路事件的偏航角度阈值，单位弧度
        pos_threshold=config.common.walk_pos_threshold,  # 走路事件的位置阈值，单位米
        control_mode='cmd_pos_world'  # 使用世界坐标系的命令位置控制模式
    )
    head_event = EventHeadMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.head_timeout,  # 头部移动事件的超时时间，单位秒
    )
    percep_event = EventPercep(
        robot_sdk=robot_sdk,
        half_fov=config.common.half_fov,  # 半视场角度，单位度
        timeout=np.inf,  # 头部移动事件的超时时间，单位秒
    )
    arm_event = EventArmMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.arm_timeout,  # 手臂移动事件的超时时间，单位秒
        arm_control_mode=config.common.arm_control_mode,  # 手臂控制模式
        pos_threshold=config.common.arm_pos_threshold,  # 手臂位置阈值，单位米
        angle_threshold=config.common.arm_angle_threshold,  # 手臂角度阈值，单位弧度
    )

    # 1. -------------------- 使用头部寻找目标位置 --------------------
    
    success, target_tag = search_tag_with_head(
        robot_sdk=robot_sdk,
        walk_event=walk_event,
        head_event=head_event,
        percep_event=percep_event,

        init_tag_guess=Tag(
            id=config.pick.tag_id,  # 假设目标箱子的ID为1
            pose=Pose.from_euler(
                pos=config.pick.tag_pos_world,  # 初始位置猜测，单位米
                euler=config.pick.tag_euler_world,  # 初始姿态猜测，单位四元数
                frame=Frame.ODOM,  # 使用里程计坐标系
                degrees=False
            )
        ),

        head_search_yaws=config.common.head_search_yaws,  # 头部搜索的偏航角度范围，单位度
        head_search_pitchs=config.common.head_search_pitchs,  # 头部搜索的俯仰角度范围，单位度
        enable_head_tracking=config.common.enable_head_tracking,  # 是否启用头部追踪
        rotate_body=config.common.rotate_body,  # 是否允许身体旋转以寻找目标
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能找到目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    print(f"找到目标Tag，ID: {target_tag.id}, 位置: {target_tag.pose}")
    if user_input:
        input("准备接近搬起Tag，按回车键继续... \n")

    # 2. -------------------- 走路接近目标位置 --------------------

    arm_event.arm_reset()
    robot_sdk.control.control_arm_joint_positions(
        joint_positions=[0.0] * 14 # 手臂站立位置的关节角度，单位弧度
    )

    success, latest_tag = walk_approach_target_with_perception_loop(
        walk_event=walk_event,
        percep_event=percep_event,
        tag=target_tag,
        stand_pose_in_tag=Pose.from_euler(
            pos=config.pick.stand_in_tag_pos,  # 站立位置在目标标签中的位置猜测，单位米
            euler=config.pick.stand_in_tag_euler,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
            frame=Frame.TAG,  # 使用标签坐标系
            degrees=False
        ),
        enable_percep_when_walking=config.common.enable_percep_when_walking,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能接近目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    print(f"已接近目标Tag，ID: {latest_tag.id}, 位置: {latest_tag.pose}")
    if user_input:
        input("  准备搬框，按回车键继续... \n")

    # 3. -------------------- 移动手臂搬框 --------------------

    fake_target_tag = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
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
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=latest_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.pick.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.pick.lateral_force,  # 侧向夹持力，单位N
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能搬起来目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    if user_input:
        input("准备寻找放置地点，按回车键继续... \n")

    # 4. -------------------- 尋找放置地點 --------------------

    success, target_tag = search_tag_with_head(
        robot_sdk=robot_sdk,
        walk_event=walk_event,
        head_event=head_event,
        percep_event=percep_event,

        init_tag_guess=Tag(
            id=config.place.tag_id,  # 假设目标箱子的ID为1
            pose=Pose.from_euler(
                pos=config.place.tag_pos_world,  # 初始位置猜测，单位米
                euler=config.place.tag_euler_world,  # 初始姿态猜测，单位四元数
                frame=Frame.ODOM,  # 使用里程计坐标系
                degrees=False
            )
        ),

        head_search_yaws=config.common.head_search_yaws,  # 头部搜索的偏航角度范围，单位度
        head_search_pitchs=config.common.head_search_pitchs,  # 头部搜索的俯仰角度范围，单位度
        enable_head_tracking=config.common.enable_head_tracking,  # 是否启用头部追踪
        rotate_body=config.common.rotate_body,  # 是否允许身体旋转以寻找目标
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能找到目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    print(f"找到目标Tag，ID: {target_tag.id}, 位置: {target_tag.pose}")
    if user_input:
        input("准备接近放置Tag，按回车键继续... \n")

    # 5. -------------------- 走路接近放置位置 --------------------

    success, latest_tag = walk_approach_target_with_perception_loop(
        walk_event=walk_event,
        percep_event=percep_event,
        tag=target_tag,
        stand_pose_in_tag=Pose.from_euler(
            pos=config.place.stand_in_tag_pos,  # 站立位置在目标标签中的位置猜测，单位米
            euler=config.place.stand_in_tag_euler,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
            frame=Frame.TAG,  # 使用标签坐标系
            degrees=False
        ),
        enable_percep_when_walking=config.common.enable_percep_when_walking,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能接近目标地点，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    print(f"已接近目标Tag，ID: {latest_tag.id}, 位置: {latest_tag.pose}")
    if user_input:
        input("准备放框，按回车键继续... \n")

    # 6. -------------------- 放下箱子并向后平移 --------------------
    success = place_box_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.place.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.place.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=latest_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.place.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.place.lateral_force,  # 侧向夹持力，单位N,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel
    )

    if not success:
        print("未能放下目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    if user_input:
        input("准备回到初始位置，按回车键继续...\n")

    # end. -------------------- 回到初始位置 --------------------
    # success = return_to_idle(
    #     walk_event=walk_event,
    # )

    # if not success:
    #     print("未能回到初始位置，退出策略。")
    #     return False
    return True


if __name__ == "__main__":
    for eps in range(10):
        print(f"### 案例开始: {eps} ###")
        res = grab_one_box(user_input=True)
        print(f"### 案例结束: {eps} ###")

        if not res:
            break

    # test_arm_only(）