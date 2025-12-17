import time
from typing import List, Dict, Any, Tuple
import numpy as np
from std_msgs.msg import Float64MultiArray
import rospy
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus

"""
策略編寫眼原則：
1. 策略和策略之間不能有隱藏狀態傳遞。變量必須顯示傳遞。这样做是确保每个策略可以通过构造输入来单独启用。
2. 事件：事件实例可以复用，但是不能跨策略输送状态
3. 事件的抽象： 有开始、过程、和终止的事情，并且结束需要返回一个状态。事件内部可以是阻塞的也可以是非阻塞的。
如果事件是非阻塞的，则可能出现多个事件同时处于运行的状态。
4. 例如，移动寻找tag这个策略由三个事件参与：移动事件，感知事件，动头事件
5. 每个事件可以单独被测试。因为事件有定义良好的target
"""


def search_tag_with_head(
        robot_sdk: RobotSDK,
        walk_event: EventWalkToPose,
        head_event: EventHeadMoveKeyPoint,  # 可选的头部移动事件
        percep_event: EventPercep,  # 可选的感知事件

        init_tag_guess: Tag,  # 初始的tag信息猜測

        head_search_yaws: List[float],
        head_search_pitchs: List[float],

        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
        enable_head_tracking: bool = True,  # 是否启用头部追踪
        rotate_body: bool = True,  # 是否允许身体旋转

) -> (bool, Tag):
    """
    使用头部寻找目标位置。

    参数：
        robot_sdk (RobotSDK): 机器人SDK实例。
        walk_event (EventWalkToPose): 走路事件。
        head_event (EventHeadMoveKeyPoint): 头部移动事件。
        percep_event (EventPercep): 感知事件。
        init_tag_guess (Tag): 初始的tag信息猜测。
        head_search_yaws (List[float]): 头部搜索的yaw角度列表。
        head_search_pitchs (List[float]): 头部搜索的pitch角度列表。
        enable_head_tracking (bool): 是否启用头部追踪。
        rotate_body (bool): 是否允许身体旋转。

    返回：
        Tuple[bool, Tag]: 搜索结果和目标Tag。
    """
    # 初始化事件
    # 判断是否在fov里
    robot_sdk.control.disable_head_tracking()

    # 生成头部搜索的角度组合
    head_search_yaw_pitch = []
    for pitch_deg in head_search_pitchs:
        # 遍历yaw角度进行左右扫描
        for yaw_deg in head_search_yaws:
            head_search_yaw_pitch.append((np.deg2rad(yaw_deg), np.deg2rad(pitch_deg)))

    head_event.open()
    percep_event.open()

    percep_event.set_target(init_tag_guess.id)
    head_event.set_target(head_search_yaw_pitch)  # 设置头部搜索的位姿

    # 1. 检查是否在fov里
    is_in_fov, target_direction = percep_event.check_in_fov(init_tag_guess)

    if not is_in_fov and rotate_body:
        # 如果目标不在FOV内且允许旋转身体，先旋转机器人身体
        walk_event.open()
        walk_event.utils_enable_base_pitch_limit(True)  # 启用基座俯仰限制
        if walk_use_cmd_vel:
            walk_event.set_control_mode('cmd_vel')
        target_pose = Pose.from_euler(
            pos=robot_sdk.state.robot_position(),
            euler=(0, 0, target_direction),  # 只旋转yaw角度
            frame=Frame.ODOM,  # 使用里程计坐标系
            degrees=False
        )
        walk_event.set_target(target_pose)

        while True:
            walk_status = walk_event.step()
            if walk_status != EventStatus.RUNNING:
                break

        walk_event.close()

        if walk_status == EventStatus.SUCCESS:
            print("✅ 已成功旋转机器人身体到目标位置。")
        else:
            return False, None  # 走路事件失败，返回失败状态

    while True:
        head_status = head_event.step()
        _ = percep_event.step()
        if percep_event.new_tag_pose_came():
            print("🔵 感知到新的Tag位置，更新目标位置。")
            target_tag = percep_event.get_tag_in_world()
            if enable_head_tracking:
                robot_sdk.control.enable_head_tracking(target_tag.id)
            res = True, target_tag

            head_event.close()
            percep_event.close()
            return res

        if head_status != EventStatus.RUNNING:
            break

    res = False, None

    head_event.close()
    percep_event.close()

    return res


def walk_approach_target_with_perception_loop(
        walk_event: EventWalkToPose,
        percep_event: EventPercep,
        tag: Tag,
        stand_pose_in_tag: Pose,  # 最終站立位置在目标标签中的位姿
        enable_percep_when_walking: bool,  # 是否在走路时启用感知闭环（边走边看）
        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
        use_vison = True,  # 是否使用视觉闭环
        waist_pos = 0.0
):
    """
    走路接近目标，同时视觉闭环。

    参数：
        walk_event (EventWalkToPose): 走路事件。
        percep_event (EventPercep): 感知事件。
        tag (Tag): 目标标签。
        stand_pose_in_tag (Pose): 最终站立位置在目标标签中的位姿。
        enable_percep_when_walking (bool): 是否在走路时启用感知闭环。

    返回：
        Tuple[bool, Tag]: 是否成功接近目标和目标Tag。
    """
    if use_vison == True:
        percep_event.open()
    walk_event.open()

    walk_event.utils_enable_base_pitch_limit(True)

    if walk_use_cmd_vel:
        walk_event.set_control_mode('cmd_vel')  # 使用相对位置控制模式

    # # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
    stand_pose_in_world = percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
    print(f'🔵 目标位置在世界坐标系中的位置: {stand_pose_in_world}')
    walk_event.set_target(stand_pose_in_world)

    if use_vison == True:
        percep_event.set_timeout(np.inf)
        percep_event.set_target(tag.id)
    waist_pub = rospy.Publisher('/robot_waist_motion_data', Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()
    msg.data = [waist_pos]
    # 事件之间的交互逻辑
    while True:
        walk_status = walk_event.step()
        waist_pub.publish(msg)
        if walk_status != EventStatus.RUNNING:
            print("走路事件未在运行状态，退出接近目标位置。")
            break
        if use_vison == True:
            _ = percep_event.step()  # 更新感知事件状态
            if enable_percep_when_walking:
                if percep_event.new_tag_pose_came():  # 检查是否有新的Tag
                    tag = percep_event.get_tag_in_world()  # 获取目标位置
                    # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
                    stand_pose_in_world = percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
                    walk_event.set_target(stand_pose_in_world)

        time.sleep(0.1)

    walk_event.close()  # 停止走路事件
    percep_event.close()  # 停止感知事件

    if walk_status == EventStatus.SUCCESS:
        print(f"✅ 已成功走到目标位置 {stand_pose_in_world}")
        return True, tag
    elif walk_status == EventStatus.FAILED:
        print(f"❌ 走到目标位置失败，退出。目标位置: {stand_pose_in_world}")
    elif walk_status == EventStatus.TIMEOUT:
        print(f"❌ 走到目标位置超时，退出。目标位置: {stand_pose_in_world}")

    return False, tag


def move_arm_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        step_back_distance: float,  # 向后平移的距离，单位米
        side_shift_distance: float,  # 侧移距离，单位米
        walk_use_cmd_vel: bool = False, # 是否使用cmd_vel控制走路
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None  # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
):
    """
    抓起箱子同时向后平移。

    参数：
        walk_event (EventWalkToPose): 走路事件。
        arm_event (EventArmMoveKeyPoint): 手臂移动事件。
        arm_traj (Tuple[List[Pose], List[Pose]]): 手臂轨迹，分别存放左臂和右臂的数据。
        step_back_distance (float): 向后平移的距离，单位米。
        tag (Tag): 可选的目标标签。
        arm_wrench (Tuple[List, List]): 可选的手臂扭矩数据。

    返回：
        bool: 是否成功完成操作。
    """
    arm_event.open()  # 打开手臂事件
    arm_traj1 = [arm_traj[0][0:3], arm_traj[1][0:3]]
    arm_wrench1 = [arm_wrench[0][0:3], arm_wrench[1][0:3]]
    arm_traj2 = [arm_traj[0][3:], arm_traj[1][3:]]
    arm_wrench2 = [arm_wrench[0][3:], arm_wrench[1][3:]]
    if not arm_event.set_target(arm_traj1, arm_wrench=arm_wrench1, tag=tag):
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
    
    arm_event.open()  # 打开手臂事件
    if not arm_event.set_target(arm_traj2, arm_wrench=arm_wrench2, tag=tag):
        print("❌ 设置手臂key point失败")
        return False
    walk_event.open()

    walk_event.utils_enable_base_pitch_limit(False)

    if walk_use_cmd_vel:
        walk_event.set_control_mode('cmd_vel')  # 使用相对位置控制模式
    else:
        walk_event.set_control_mode('cmd_pos')

    walk_event.set_target(
        Pose(
            pos=(-step_back_distance, side_shift_distance, 0.),  # 向后平移
            quat=(0, 0, 0, 1),  # 保持姿态不变
            frame=Frame.BASE  # 使用基座坐标系
        )
    )

    while True:
        walk_status = walk_event.step()
        arm_status = arm_event.step()  # 同时更新手臂状态

        if walk_status != EventStatus.RUNNING and arm_status != EventStatus.RUNNING:
            break

    if walk_status != EventStatus.SUCCESS:
        print("❌ 向后平移失败，退出策略。")
        walk_event.close()
        return False

    if arm_status != EventStatus.SUCCESS:
        print("❌ 手臂移动失败，退出策略。")
        arm_event.close()
        return False

    print("✅ 已成功向后平移，策略完成。")
    walk_event.close()
    arm_event.close()
    return True


def walk_and_move_arm_together(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint
        ) -> bool:
    arm_event.open(control_frame=Frame.BASE)  # 打开手臂事件，设置局部控制
    # while True:
    #     print(f'=------ loop opened ------=')
    left_arm_poses = [
        Pose.from_euler(pos=(0.0, 0.4, -0.1), euler=(0, 0, 0), degrees=True, frame=Frame.BASE),
        Pose.from_euler(pos=(0.4, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    right_arm_poses = [
        Pose.from_euler(pos=(0.0, -0.4, -0.1), euler=(0, 0, 0), degrees=True, frame=Frame.BASE),
        Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]

    arm_traj = (left_arm_poses, right_arm_poses)

    if not arm_event.set_target(arm_traj):
        print("❌ 设置手臂key point失败")
        return False

    walk_event.open()
    #
    walk_event.utils_enable_base_pitch_limit(False)

    walk_event.set_control_mode('cmd_pos_world')  # 使用相对位置控制模式
    #
    walk_event.set_target(
        Pose.from_euler(pos=(3.0, 0., 0.), euler=(0, 0, 0), degrees=True, frame=Frame.ODOM)
        # Pose(
        #     pos=(3.0, 0., 0.),  # 向后平移
        #     quat=(0, 0, 0, 1),  # 保持姿态不变
        #     frame=Frame.ODOM  # 使用基座坐标系
        # )
    )

    while True:
        walk_status = walk_event.step()
        arm_status = arm_event.step()  # 同时更新手臂状态

        if walk_status != EventStatus.RUNNING and arm_status != EventStatus.RUNNING:
            break

    if walk_status != EventStatus.SUCCESS:
        print("❌ 向后平移失败，退出策略。")
        # walk_event.close()
        arm_event.close()
        return False

    if arm_status != EventStatus.SUCCESS:
        print("❌ 手臂移动失败，退出策略。")
        # walk_event.close()
        arm_event.close()
        return False

    print("✅ 已成功向后平移，策略完成。")
    # walk_event.close()
    arm_event.close()
    return True


def grab_box_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        step_back_distance: float,  # 向后平移的距离，单位米
        side_shift_distance: float,  # 侧移距离，单位米
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息

        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米

        box_mass: float, # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z: float,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force: float,  # 侧向夹持力，单位N

        hand_pitch_degree: float = 0.0,  # 手臂pitch角度（相比水平, 下倾是正），单位度
        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
) -> bool:
    """
    抓取箱子并向后移动。

    参数：
        walk_event (EventWalkToPose): 走路事件。
        arm_event (EventArmMoveKeyPoint): 手臂移动事件。
        step_back_distance (float): 向后平移的距离，单位米。
        tag (Tag): 目标标签。
        box_width (float): 箱子宽度。
        box_behind_tag (float): 箱子在tag后面的距离，单位米。
        box_beneath_tag (float): 箱子在tag下方的距离，单位米。
        box_left_tag (float): 箱子在tag左侧的距离，单位米。
        box_mass (float): 箱子质量，单位kg。
        force_ratio_z (float): 纵向力经验系数。
        lateral_force (float): 侧向夹持力，单位N。

    返回：
        bool: 是否成功完成操作。
    """
    # =================== 计算每个关键点的手臂位姿（Pose） =================== #
    pick_left_arm_poses = [
        # # 1. 预抓取点位
        Pose.from_euler(pos=(-box_width*3/2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.20, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, box_width / 2, 0.3), euler=(0, -90 + hand_pitch_degree , 0), degrees=True, frame=Frame.BASE)]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(box_width*3/2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.20, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        Pose.from_euler(pos=(0.5, -box_width / 2, 0.3), euler=(0, -90 + hand_pitch_degree , 0), degrees=True, frame=Frame.BASE),
        ]
    
    pose4_left = Pose.from_euler(pos=(0.5, box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree , 0), degrees=True, frame=Frame.BASE)
    pose4_right = Pose.from_euler(pos=(0.5, -box_width / 2, 0.4), euler=(0, -90 + hand_pitch_degree , 0), degrees=True, frame=Frame.BASE)
    pose4_left_new = transform_by_yaw(pose4_left, -90)
    pose4_right_new = transform_by_yaw(pose4_right, -90)

    pickWithWaist_left_arm_poses = [
        # # 1. 预抓取点位
        Pose.from_euler(pos=(-box_width*3/2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.20, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        pose4_left_new,
        
        pose4_left
        ]

    pickWithWaist_right_arm_poses = [
        Pose.from_euler(pos=(box_width*3/2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.20, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 4. 收臂点位
        pose4_right_new,
        
        pose4_right
        ]
    
    # ================ 计算每个关键点的力控目标（wrench） ================ #
    # 计算夹持力参数
    g = 9.8  # 重力加速度

    # 计算基础Z向力（考虑安全系数和经验比例）
    force_z = -abs(box_mass * g * force_ratio_z)

    # 判断是否为仿真模式
    left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
    right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

    pick_left_arm_wrench = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
        [0, left_force,  force_z, 0, 0, 0],  # 第三关键点的扭矩
        [0, left_force,  force_z, 0, 0, 0],   # 第四关键点的扭矩
        [0, left_force,  force_z, 0, 0, 0]
    ]

    pick_right_arm_wrench = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
        [0, right_force, force_z, 0, 0, 0],  # 第三关键点的扭矩
        [0, right_force, force_z, 0, 0, 0],   # 第四关键点的扭矩
        [0, left_force,  force_z, 0, 0, 0]
    ]

    arm_traj = (pickWithWaist_left_arm_poses, pickWithWaist_right_arm_poses)
    arm_wrench = (pick_left_arm_wrench, pick_right_arm_wrench)  # 手臂扭矩数据
    success = move_arm_and_backward(walk_event, arm_event, arm_traj, step_back_distance, side_shift_distance, tag=tag, arm_wrench=arm_wrench, walk_use_cmd_vel=walk_use_cmd_vel)
    return success


def place_box_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        # arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        step_back_distance: float,  # 向后平移的距离，单位米
        side_shift_distance: float,  # 侧移距离，单位米
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息

        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米

        box_mass: float,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z: float,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force: float,  # 侧向夹持力，单位N

        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
):
    """
    放置箱子并向后移动。

    参数：
        walk_event (EventWalkToPose): 走路事件。
        arm_event (EventArmMoveKeyPoint): 手臂移动事件。
        step_back_distance (float): 向后平移的距离，单位米。
        tag (Tag): 目标标签。
        box_width (float): 箱子宽度。
        box_behind_tag (float): 箱子在tag后面的距离，单位米。
        box_beneath_tag (float): 箱子在tag下方的距离，单位米。
        box_left_tag (float): 箱子在tag左侧的距离，单位米。
        box_mass (float): 箱子质量，单位kg。
        force_ratio_z (float): 纵向力经验系数。
        lateral_force (float): 侧向夹持力，单位N。

    返回：
        bool: 是否成功完成操作。
    """
    waist_pub = rospy.Publisher('/robot_waist_motion_data', Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()
    msg.data = [90.0]
    waist_pub.publish(msg)
    time.sleep(2)
    # =================== 计算每个关键点的手臂位姿（Pose） =================== #
    # 生成放下箱子的手臂轨迹
    place_left_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width*3/2 - 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),

        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag - 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width*3/2 + 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),

        # 4. 收臂点位
        Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
    ]  # 手臂关键点数据，假设为空列表

    pose4_left = Pose.from_euler(pos=(0.4, 0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE)
    pose4_right = Pose.from_euler(pos=(0.4, -0.4, 0.1), euler=(0, -90, 0), degrees=True, frame=Frame.BASE)
    pose4_left_new = transform_by_yaw(pose4_left, 90)
    pose4_right_new = transform_by_yaw(pose4_right, 90)

    pickWithWaist_left_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(-box_width*3/2 - 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        pose4_left_new,
        
        # pose4_left
        ]

    pickWithWaist_right_arm_poses = [
        # 1. 上方点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag - 0.03, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 打开点位
        Pose.from_euler(pos=(box_width*3/2 + 0.2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 4. 收臂点位
        pose4_right_new,
        
        # pose4_right
        ]

    # ================ 计算每个关键点的力控目标（wrench） ================ #
    g = 9.8  # 重力加速度

    # 计算基础Z向力（考虑安全系数和经验比例）
    force_z = -abs(box_mass * g * force_ratio_z)

    # 判断是否为仿真模式
    left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
    right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

    place_left_arm_wrench = [
        [0, left_force,  force_z, 0, 0, 0],
        [0, left_force,  force_z, 0, 0, 0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    place_right_arm_wrench = [
        [0, right_force, force_z, 0, 0, 0],
        [0, right_force, force_z, 0, 0, 0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    arm_traj = (pickWithWaist_left_arm_poses, pickWithWaist_right_arm_poses)
    arm_wrench = (place_left_arm_wrench, place_right_arm_wrench)  # 手臂扭矩数据

    success = move_arm_and_backward(walk_event, arm_event, arm_traj, step_back_distance, side_shift_distance, tag=tag, arm_wrench=arm_wrench, walk_use_cmd_vel=walk_use_cmd_vel)

    msg.data = [0.0]
    waist_pub.publish(msg)
    return success


def return_to_idle(
        walk_event: EventWalkToPose,
):
    """
    返回到空闲状态。

    参数：
        walk_event (EventWalkToPose): 走路事件。

    返回：
        bool: 是否成功返回到空闲状态。
    """
    walk_event.open()

    origin_pose = Pose(
        pos=(0, 0, 0),  # 假设原点位置为(0, 0, 0)
        quat=(0, 0, 0, 1),  # 假设原点姿态为单位四元数
        frame=Frame.ODOM  # 使用里程计坐标系
    )

    walk_event.set_target(origin_pose)  # 设置目标为原点位置

    while True:
        walk_status = walk_event.step()

        if not walk_status == EventStatus.RUNNING:
            break

    # 检查走路事件的状态
    if walk_status == EventStatus.SUCCESS:
        print("✅ 已成功返回待命位置。")
        return True

    print("❌ 超时或失败，无法返回待命位置。")

    return False  # 返回失败状态

def transform_by_yaw(pose: Pose, yaw_deg: float) -> Pose:
    yaw_rad = np.deg2rad(yaw_deg)
    x, y, z = pose.pos
    x_new = x * np.cos(yaw_rad) - y * np.sin(yaw_rad)
    y_new = x * np.sin(yaw_rad) + y * np.cos(yaw_rad)
    euler = pose.get_euler(degrees=True)
    return Pose.from_euler(pos=(x_new, y_new, z), 
                            euler=(euler[0], euler[1], euler[2] + yaw_deg), 
                            frame=pose.frame, degrees=True)
