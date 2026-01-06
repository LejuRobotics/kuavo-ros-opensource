from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup import init_logging
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventPercep, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.walk_events_lb import EventWalkToPose  # 使用轮臂版本
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.arm_events_lb import EventArmMoveKeyPoint  # 使用轮臂版本
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvRequest

import rospy
import numpy as np
import os, sys
mother_dir = os.path.dirname(os.path.abspath(__file__))

log_path = init_logging(log_dir=os.path.join(mother_dir, "logs"), filename_prefix="grab_box_v2", enable=True)

from configs.config_sim import config
# from configs.config_real import config

from lb_strategy import (
    search_tag_with_head,
    walk_approach_target_with_perception_loop,
    grab_box_and_backward,
    place_box_and_backward,
    return_to_idle,
    base_link_leg_ik_service,
    arm_reset,
    reset_head_pos
)

def test_arm_place(user_input=True, use_faker=True):
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
        use_relative_control=config.common.use_relative_control  # 坐标系控制模式
    )

        # 2. -------------------- 确定搬运目标箱子的base_link位姿 --------------------
    fake_target_tag_place_1 = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
        pose=Pose.from_euler(
            pos=(0.60, 0.0, 1.35),  # 初始位置猜测，单位米
            euler=(90, 0, -90),  # 初始姿态猜测，单位欧拉角（弧度）
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=True
        )
    )

    # 1. -------------------- 确定base_link当前位姿 --------------------
    success = place_box_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.place.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.place.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=fake_target_tag_place_1,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米
        back_direction=[-1, 0, 0], # 向后平移的方向，当前x轴负方向

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.place.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.place.lateral_force,  # 侧向夹持力，单位N,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        user_input=user_input
    )

    if not success:
        print("未能放下目标箱子，退出策略。")
        return False

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    # if user_input:
    #     input("准备回到初始位置，按回车键继续...\n")

    return True

def test_arm_pick(user_input=True, use_faker=True):
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
        use_relative_control=config.common.use_relative_control  # 坐标系控制模式
    )

    fake_target_tag = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
        pose=Pose.from_euler(
            pos=(0.5, 0.0, 0.75),  # 初始位置猜测，单位米
            euler=(90, 0, -90),  # 初始姿态猜测，单位欧拉角（弧度）
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=True
        )
    )

    target_tag = None

    if use_faker:
        print("使用虚假目标fake_target_tag")
        target_tag = fake_target_tag
    else:
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
    
    # rospy.sleep(2)

    # if user_input:
    #     input("准备搬起目标箱子，按回车键继续... \n")    

    success = grab_box_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=target_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米
        back_direction=[-1, 0, 0], # 向后平移的方向，当前x轴负方向

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.pick.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.pick.lateral_force,  # 侧向夹持力，单位N

        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        user_input=user_input
    )

    if not success:
        print("未能搬起目标箱子，退出策略。")
        return False

    
    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")

    return True


def return_init_pos(walk_event: EventWalkToPose, percep_event: EventPercep):
    
    init_target_tag = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
        pose=Pose.from_euler(
            pos=(0.0, 0.0, 0.0),  # 初始位置猜测，单位米
            euler=(90, 0, -90),  # 初始姿态猜测，单位欧拉角（弧度）
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=True
        )
    )

    success, latest_tag = walk_approach_target_with_perception_loop(
        walk_event=walk_event,
        percep_event=percep_event,
        tag=init_target_tag,
        stand_pose_in_tag=Pose.from_euler(
            pos=(0.0, 0.0, 0.0),  # 站立位置在目标标签中的位置猜测，单位米
            euler=(-np.deg2rad(90), np.deg2rad(90), 0.0),  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
            frame=Frame.TAG,  # 使用标签坐标系
            degrees=False
        ),
        enable_percep_when_walking=config.common.enable_percep_when_walking,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        walk_time=5.0
    )

    return success


def grab_box_case(user_input=True, use_faker=True, should_return_init_pos=True):
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
            pos=(-0.00657065, -1.77025328, 0.70921274),  # 初始位置猜测，单位米
            euler=(90, 0, -180),  # 初始姿态猜测，单位欧拉角（弧度）
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=True
        )
    )

    fake_target_tag_place_1 = Tag(
        id=config.pick.tag_id,  # 假设目标箱子的ID为1
        pose=Pose.from_euler(
            pos=(-0.09265887, 1.67392445, 1.24421732),  # 初始位置猜测，单位米
            euler=(90, 0, 0),  # 初始姿态猜测，单位欧拉角（弧度）
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=True
        )
    )

    target_tag = None
    place_tag = None
    transform_odom_to_base = arm_event.get_base_to_odom_transform()
    print(f"🔵 当前odom_to_base姿态: {transform_odom_to_base.trans_pose}")

    # if user_input:
    #     input("准备获取Tag数据，按回车键继续... \n")

    if use_faker:
        print("使用虚假目标fake_target_tag")
        target_tag = fake_target_tag
        place_tag = fake_target_tag_place_1
    else:
        success, target_tag = search_tag_with_head(
            robot_sdk=robot_sdk,
            walk_event=walk_event,
            head_event=head_event,
            percep_event=percep_event,
            arm_event=arm_event,

            init_tag_guess=Tag(
                id=config.pick.tag_id,  # 假设目标箱子的ID为1
                pose=Pose.from_euler(
                    pos=config.pick.tag_pos_world,  # 初始位置猜测，单位米
                    euler=config.pick.tag_euler_world,  # 初始姿态猜测，单位四元数
                    frame=Frame.ODOM,  # 使用里程计坐标系
                    degrees=False
                )
            ),

            head_search_yaws=[-80, -88],  # 头部搜索的偏航角度范围，单位度
            head_search_pitchs=config.common.head_search_pitchs,  # 头部搜索的俯仰角度范围，单位度
            enable_head_tracking=config.common.enable_head_tracking,  # 是否启用头部追踪
            rotate_body=config.common.rotate_body,  # 是否允许身体旋转以寻找目标
            walk_use_cmd_vel=config.common.walk_use_cmd_vel
        )

        if not success:
            print("未能找到目标箱子tag，退出策略。")
            return False

        success, place_tag = search_tag_with_head(
            robot_sdk=robot_sdk,
            walk_event=walk_event,
            head_event=head_event,
            percep_event=percep_event,
            arm_event=arm_event,

            init_tag_guess=Tag(
                id=config.place.tag_id,  # 假设目标箱子的ID为1
                pose=Pose.from_euler(
                    pos=config.place.tag_pos_world,  # 初始位置猜测，单位米
                    euler=config.place.tag_euler_world,  # 初始姿态猜测，单位四元数
                    frame=Frame.ODOM,  # 使用里程计坐标系
                    degrees=False
                )
            ),

            head_search_yaws=[80, 88],  # 头部搜索的偏航角度范围，单位度
            head_search_pitchs=config.common.head_search_pitchs,  # 头部搜索的俯仰角度范围，单位度
            enable_head_tracking=config.common.enable_head_tracking,  # 是否启用头部追踪
            rotate_body=config.common.rotate_body,  # 是否允许身体旋转以寻找目标
            walk_use_cmd_vel=config.common.walk_use_cmd_vel
        )

        if not success:
            print("未能找到放置位tag，退出策略。")
            return False

    # 头部回零
    reset_head_pos(head_event=head_event)

    # 手臂回到初始位姿
    arm_reset(arm_event=arm_event)

    ## 在这里添加键盘事件，按特定按键才能继续
    print("======================================================")
    print(f"找到目标Tag，ID: {target_tag.id}, 位置: {target_tag.pose}")
    print(f"找到放置位Tag，ID: {place_tag.id}, 位置: {place_tag.pose}")

    if user_input:
        input("准备移动到搬起箱子位置，按回车键继续... \n")

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
        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        walk_time=5.0
    )

    if not success:
        print("未能接近目标地点，退出策略。")
        return False

    if user_input:
        input("准备搬起目标箱子，按回车键继续... \n")    

    success = grab_box_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        box_width=config.common.box_width,
        box_behind_tag=config.pick.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.pick.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.pick.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=target_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米
        back_direction=[-1, 0, 0], # 向后平移的方向，当前x轴负方向

        box_mass=config.common.box_mass,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z=config.pick.force_ratio_z,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force=config.pick.lateral_force,  # 侧向夹持力，单位N

        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        user_input=user_input
    )

    if not success:
        print("未能搬起目标箱子，退出策略。")
        return False

    if user_input:
        input("准备移动到放下箱子位置，按回车键继续... \n")


    # 2. -------------------- 确定搬运目标箱子的base_link位姿 --------------------
    success, latest_tag = walk_approach_target_with_perception_loop(
        walk_event=walk_event,
        percep_event=percep_event,
        tag=place_tag,
        stand_pose_in_tag=Pose.from_euler(
            pos=config.place.stand_in_tag_pos,  # 站立位置在目标标签中的位置猜测，单位米
            euler=config.place.stand_in_tag_euler,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
            frame=Frame.TAG,  # 使用标签坐标系
            degrees=False
        ),
        enable_percep_when_walking=config.common.enable_percep_when_walking,
        walk_use_cmd_vel=config.common.walk_use_cmd_vel,
        walk_time=10.0
    )

    if not success:
        print("未能接近目标地点，退出策略。")
        return False

    if user_input:
        input("准备放下目标箱子，按回车键继续... \n")

    success = place_box_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        box_width=config.common.box_width,
        box_behind_tag=config.place.box_behind_tag,  # 箱子在tag后面的距离，单位米
        box_beneath_tag=config.place.box_beneath_tag,  # 箱子在tag下方的距离，单位米
        box_left_tag=config.place.box_left_tag,  # 箱子在tag左侧的距离，单位米
        tag=place_tag,
        step_back_distance=config.common.step_back_distance,  # 搬起后向后平移的距离，单位米
        back_direction=[-1, 0, 0], # 向后平移的方向，当前x轴负方向

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

    if should_return_init_pos:
        if user_input:
            input("准备回到初始位置，按回车键继续...\n")

        success = return_init_pos(walk_event=walk_event, percep_event=percep_event)
        if not success:
            print("未能回到初始位置，退出策略。")
            return False

        arm_reset(arm_event=arm_event)

    return True


if __name__ == "__main__":
    rospy.init_node('test_leg_ik_service', anonymous=True)
    rospy.loginfo("等待腿部IK服务...")

    for eps in range(2):
        print(f"### 案例开始: {eps} ###")
        # res = grab_box_case(user_input=False, use_faker=True, should_return_init_pos=True)
        # res = test_arm_place(user_input=False, use_faker=True)
        res = test_arm_pick(user_input=False, use_faker=True)
        print(f"### 案例结束: {eps} ###")

        if not res:
            break
