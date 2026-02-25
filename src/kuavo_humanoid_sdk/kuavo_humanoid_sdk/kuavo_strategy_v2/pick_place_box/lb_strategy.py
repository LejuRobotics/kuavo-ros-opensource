import time
from typing import List, Dict, Any, Tuple
import numpy as np

import rospy
import tf
from scipy.spatial.transform import Rotation as R
from kuavo_msgs.srv import lbBaseLinkPoseCmdSrv, lbBaseLinkPoseCmdSrvRequest
from kuavo_msgs.srv import lbLegControlSrv, lbLegControlSrvRequest

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus

from configs.config_sim import config

"""
策略編寫眼原則：
1. 策略和策略之間不能有隱藏狀態傳遞。變量必須顯示傳遞。这样做是确保每个策略可以通过构造输入来单独启用。
2. 事件：事件实例可以复用，但是不能跨策略输送状态
3. 事件的抽象： 有开始、过程、和终止的事情，并且结束需要返回一个状态。事件内部可以是阻塞的也可以是非阻塞的。
如果事件是非阻塞的，则可能出现多个事件同时处于运行的状态。
4. 例如，移动寻找tag这个策略由三个事件参与：移动事件，感知事件，动头事件
5. 每个事件可以单独被测试。因为事件有定义良好的target
"""

def reset_head_pos(head_event: EventHeadMoveKeyPoint):
    """头部回到零位"""
    print("🔵 头部回零...")
    head_event.open()
    head_event.set_target([(0.0, 0.0)])  # 设置头部目标为零位

    # 循环执行直到完成
    while True:
        head_status = head_event.step()
        if head_status != EventStatus.RUNNING:
            break

    head_event.close()
    print("✅ 头部已回零")

def search_tag_with_head(
        robot_sdk: RobotSDK,
        walk_event: EventWalkToPose,
        head_event: EventHeadMoveKeyPoint,  # 可选的头部移动事件
        percep_event: EventPercep,  # 可选的感知事件
        arm_event: EventArmMoveKeyPoint,  # 手臂事件，用于获取坐标变换

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
    print(f"目标是否在FOV内: {is_in_fov}")
    print(f"目标方向: {target_direction}")

    if not is_in_fov and rotate_body:
        print("目标不在FOV内，旋转机器人身体...")
        # 如果目标不在FOV内且允许旋转身体，先旋转机器人身体
        walk_event.open()
        # walk_event.utils_enable_base_pitch_limit(True)  # 启用基座俯仰限制
        if walk_use_cmd_vel:
            print("使用cmd_vel控制走路")
            walk_event.change_control_mode('BaseOnly')
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
            transform_odom_to_base = arm_event.get_base_to_odom_transform()
            print(f"🔵 当前odom_to_base姿态: {transform_odom_to_base.trans_pose}")
        else:
            return False, None  # 走路事件失败，返回失败状态
    else:
        transform_odom_to_base = arm_event.get_base_to_odom_transform()
        print(f"🔵 当前odom_to_base姿态: {transform_odom_to_base.trans_pose}")

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
        walk_time: float = 3.0 # 走路时间，单位秒
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

    percep_event.open()
    walk_event.open()

    walk_event.change_control_mode('BaseOnly')
    # walk_event.utils_enable_base_pitch_limit(True)

    # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
    stand_pose_in_world = percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
    print(f'🔵 站立位置在世界坐标系中的位置: {stand_pose_in_world}')
    walk_event.set_target(stand_pose_in_world)

    percep_event.set_timeout(np.inf)
    percep_event.set_target(tag.id)

    # 事件之间的交互逻辑
    while True:
        walk_status = walk_event.step(walk_time=walk_time)
        _ = percep_event.step()  # 更新感知事件状态

        if walk_status != EventStatus.RUNNING:
            print("走路事件未在运行状态，退出接近目标位置。")
            break

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
        print(f"✅ 已成功走到搬运站立位置 {stand_pose_in_world}")
        return True, tag
    elif walk_status == EventStatus.FAILED:
        print(f"❌ 走到目标位置失败，退出。目标位置: {stand_pose_in_world}")
    elif walk_status == EventStatus.TIMEOUT:
        print(f"❌ 走到目标位置超时，退出。目标位置: {stand_pose_in_world}")

    return False, tag

def arm_reset(arm_event: EventArmMoveKeyPoint):

    print("🔵 手臂回到设定的初始位姿")
    arm_event.open()
    arm_event.set_arm_control_mode('fixed_base')
    init_joint_pos = [-0.0, 0.4, 0.2, -1.5, -0.0, -0.0, -0.0,
                        -0.0, -0.4, -0.2, -1.5, 0.0, -0.0, -0.0]
    arm_event.robot_sdk.control.control_arm_joint_positions(joint_positions=init_joint_pos)
    time.sleep(1.0)
    arm_event.close()

def move_arm_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        step_back_distance: float,  # 向后平移的距离，单位米
        back_direction: List[float], # 向后平移的方向，向量比例，默认是x轴正方向
        arm_control_type: str = 'endff', # 手臂控制类型，joint: 关节位置控制，endff: 末端执行器控制
        walk_use_cmd_vel: bool = False, # 是否使用cmd_vel控制走路
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None, # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
        user_input: bool = False, # 是否需要用户输入
        leg_control_time = 3.0,
        pick_or_place: bool = False
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
    if not arm_event.set_target(arm_traj, arm_wrench=arm_wrench, tag=tag):
        print("❌ 设置手臂key point失败")
        return False

    traj_result = arm_event.general_traj(segment_sleep_time=0.0)
    if traj_result is None:
        print("❌ 生成手臂轨迹失败（逆解失败）")
        return False

    if pick_or_place:
        arm_delay_time = [0.5, 0.5, 0.2]
        arm_event.set_delay_time(arm_delay_time)
    else:
        arm_delay_time = [3, 0.5, 0.5]
        arm_event.set_delay_time(arm_delay_time)
    if arm_control_type == 'endff':
      arm_event.set_arm_control_mode('manipulation_mpc')
    elif arm_control_type == 'joint':
      arm_event.set_arm_control_mode('fixed_base')
    else:
      print("❌ 手臂控制类型错误，退出策略。")
      arm_event.close()
      return False

    while True:
        if arm_control_type == 'endff':
          arm_status = arm_event.step_endff()
        elif arm_control_type == 'joint':
          arm_status = arm_event.step()

        if arm_status != EventStatus.RUNNING:
            break

    if arm_status != EventStatus.SUCCESS:
        print("❌ 手臂移动失败，退出策略。")
        arm_event.close()
        return False

    print("✅ 已成功移动手臂，开始向后平移...")
    arm_event.close()
    time.sleep(0.1)

    walk_event.open()
    walk_event.change_control_mode('BaseOnly')
    # walk_event.utils_enable_base_pitch_limit(False)

    walk_event.set_control_mode('cmd_vel')  # 使用相对位置控制模式
    walk_event.set_target(
        Pose(
            pos=(step_back_distance * back_direction[0], step_back_distance * back_direction[1], 0.),  # 向后平移
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

def grab_box_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息

        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米

        box_mass: float, # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z: float,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force: float,  # 侧向夹持力，单位N

        step_back_distance: float,  # 向后平移的距离，单位米
        back_direction: List[float] = [1, 0, 0], # 向后平移的方向，向量比例，默认是x轴正方向

        hand_pitch_degree: float = 0.0,  # 手肘俯仰角度，单位度
        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
        user_input: bool = False # 是否需要用户输入
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
        Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG)
        ]

    pick_right_arm_poses = [
        Pose.from_euler(pos=(box_width*3/2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 2. 并拢点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag, -box_behind_tag), euler=(0, 0, 90), degrees=True,
                        frame=Frame.TAG),
        # 3. 抬升点位
        Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.2, -box_behind_tag), euler=(0, 0, 90),
                        degrees=True, frame=Frame.TAG)
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
        # [0, left_force,  force_z, 0, 0, 0]   # 第四关键点的扭矩
    ]

    pick_right_arm_wrench = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
        [0, right_force, force_z, 0, 0, 0],  # 第三关键点的扭矩
        # [0, right_force, force_z, 0, 0, 0]   # 第四关键点的扭矩
    ]

    arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
    arm_wrench = (pick_left_arm_wrench, pick_right_arm_wrench)  # 手臂扭矩数据
    success = move_arm_and_backward(walk_event, arm_event, arm_traj, step_back_distance, back_direction, arm_control_type='joint', tag=tag, arm_wrench=arm_wrench, walk_use_cmd_vel=walk_use_cmd_vel,user_input=user_input,leg_control_time=2.0, pick_or_place=True)

    if not success:
        print("❌ 抓取和后退失败")
        return False

    return success


def place_box_and_backward(
        walk_event: EventWalkToPose,
        arm_event: EventArmMoveKeyPoint,
        # arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息

        box_width: float,
        box_behind_tag: float,  # 箱子在tag后面的距离，单位米
        box_beneath_tag: float,  # 箱子在tag下方的距离，单位米
        box_left_tag: float,  # 箱子在tag左侧的距离，单位米

        box_mass: float,  # 假设箱子质量，单位kg，用来计算纵向wrench
        force_ratio_z: float,  # 经验系数（根据1.5kg对应5N得出：5/(1.5*9.8)≈0.34
        lateral_force: float,  # 侧向夹持力，单位N

        step_back_distance: float,  # 向后平移的距离，单位米
        back_direction: List[float] = [1, 0, 0], # 向后平移的方向，向量比例，默认是x轴正方向

        walk_use_cmd_vel: bool = False,  # 是否使用cmd_vel控制走路
        user_input: bool = False # 是否需要用户输入
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

    # =================== 计算每个关键点的手臂位姿（Pose） =================== #
    # 生成放下箱子的手臂轨迹（都使用BASE坐标系，相对底盘）
    place_left_arm_poses = [

        # 0. 零时添加
        # Pose.from_euler(pos=(0.5, box_width / 2 - 0.03, 0.2), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),

        # 1. 预放置点位（稍高，准备放下）
        # Pose.from_euler(pos=(0.6, box_width / 2 - 0.07, 0.3), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
        # 2. 放下点位（降低高度，放在目标位置）
        # Pose.from_euler(pos=(0.6, box_width / 2 - 0.02, 0.25), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
        # 3. 打开点位（手臂向外张开，松开箱子）
        Pose.from_euler(pos=(0.6, box_width / 2 + 0.15, 0.18), euler=(0, -90, 0), degrees=True, frame=Frame.BASE)
    ]
    place_right_arm_poses = [
        # 1. 预放置点位（稍高，准备放下）
        # Pose.from_euler(pos=(0.6, -box_width / 2 + 0.07, 0.3), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
        # 2. 放下点位（降低高度，放在目标位置）
        # Pose.from_euler(pos=(0.6, -box_width / 2 + 0.02, 0.25), euler=(0, -90, 0), degrees=True, frame=Frame.BASE),
        # 3. 打开点位（手臂向外张开，松开箱子）
        Pose.from_euler(pos=(0.6, -box_width / 2 - 0.15, 0.18), euler=(0, -90, 0), degrees=True, frame=Frame.BASE)
    ]  # 手臂关键点数据，假设为空列表


    # ================ 计算每个关键点的力控目标（wrench） ================ #
    g = 9.8  # 重力加速度

    # 计算基础Z向力（考虑安全系数和经验比例）
    force_z = -abs(box_mass * g * force_ratio_z)

    # 判断是否为仿真模式
    left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
    right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

    place_left_arm_wrench = [
        [0, left_force,  force_z, 0, 0, 0],  # 第1点：预放置点位，保持夹持力
        # [0, left_force,  force_z, 0, 0, 0],  # 第2点：放下点位，保持夹持力
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # 第3点：打开点位，无力控
    ]

    place_right_arm_wrench = [
        [0, right_force, force_z, 0, 0, 0],  # 第1点：预放置点位，保持夹持力
        # [0, right_force, force_z, 0, 0, 0],  # 第2点：放下点位，保持夹持力
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # 第3点：打开点位，无力控
    ]

    arm_traj = (place_left_arm_poses, place_right_arm_poses)
    arm_wrench = (place_left_arm_wrench, place_right_arm_wrench)  # 手臂扭矩数据

    success = move_arm_and_backward(walk_event, arm_event, arm_traj, step_back_distance, back_direction, arm_control_type='joint', tag=tag, arm_wrench=arm_wrench, walk_use_cmd_vel=walk_use_cmd_vel, user_input=user_input)

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


def execute_leg_control(target_joints, duration=3.0):
    """
    执行腿部控制服务

    参数：
        target_joints: 目标关节角度列表

    返回：
        bool: 控制服务是否成功
    """
    # try:
    #     # 等待腿部控制服务可用
    #     rospy.loginfo("等待腿部控制服务...")
    #     rospy.wait_for_service('/lb_leg_control_srv', timeout=10.0)

    #     # 创建服务代理
    #     control_service = rospy.ServiceProxy('/lb_leg_control_srv', lbLegControlSrv)

    #     # 创建控制服务请求
    #     control_request = lbLegControlSrvRequest()
    #     control_request.target_joints = target_joints
    #     control_request.duration = duration

    #     rospy.loginfo(f"发送控制请求，目标关节角度: {control_request.target_joints}")

    #     # 调用控制服务
    #     control_response = control_service(control_request)

    #     if control_response.success:
    #         rospy.loginfo("控制服务调用成功！机器人开始执行目标关节角度")
    #         return True
    #     else:
    #         rospy.logwarn("控制服务调用失败")
    #         return False

    # except rospy.ServiceException as e:
    #     rospy.logerr(f"控制服务调用失败: {e}")
    #     return False


def base_link_leg_ik_service(arm_event: EventArmMoveKeyPoint,
                             with_chassis = True,
                             chassis_info = None,
                             q_lb = None,
                             refer_pose: Pose = None,
                             torso_pose_in_refer: Pose = None,
                             control_type = 0, # 0-底盘优先，1-腰部优先
                             duration = 3.0,
                             pick_or_place : bool = True
):
    """测试腿部IK服务"""

    if refer_pose is None:
        print("🔴 未提供refer_pose")
        return False

    # 根据use_relative_control配置决定底盘信息来源
    if arm_event.use_relative_control:
        # 相对底座控制模式：使用固定底盘位姿
        chassis_info = [0.0, 0.0, 0.0]  # 固定在原点
        with_chassis = True  # 使用外部传入的底盘信息
        print(f"🔵 使用相对底座控制模式，固定chassis_info [x, y, yaw]: {chassis_info}")
    else:
        # 世界坐标系控制模式
        if chassis_info is None:
            # 如果没有传入chassis_info，从robot_sdk获取
            chassis_pos = arm_event.robot_sdk.state.robot_position()
            chassis_quat = arm_event.robot_sdk.state.robot_orientation()

            # 四元数格式: (x, y, z, w)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(chassis_quat)

            # 将odom的姿态组成[x, y, yaw]数据赋值给chassis_info
            chassis_info = [chassis_pos[0], chassis_pos[1], yaw]
            print(f"🔵 从robot_sdk中获取chassis_info [x, y, yaw]: {chassis_info}")
            with_chassis = True
        # 如果传入了chassis_info，保持with_chassis的原始值

    if q_lb is None:
        first_four_joint_angles = arm_event.robot_sdk.state.joint_state.position
        q_lb = [first_four_joint_angles[0], first_four_joint_angles[1], first_four_joint_angles[2], first_four_joint_angles[3]]
        print(f"🔵 从robot_sdk中获取q_lb: {q_lb}")

    rospy.wait_for_service('/lb_optimization_ik_service', timeout=10.0)

    # 创建服务代理
    ik_service = rospy.ServiceProxy('/lb_optimization_ik_service', lbBaseLinkPoseCmdSrv)

    # 创建测试请求
    request = lbBaseLinkPoseCmdSrvRequest()

    # 设置请求参数
    request.with_chassis = with_chassis
    request.chassis_info = chassis_info  # [x, y, yaw]
    request.q_lb = q_lb  # [knee, leg, waist_pitch, waist_yaw]
    request.control_type = control_type

    # 根据torso_pose_in_refer的frame选择合适的转换方式
    if torso_pose_in_refer.frame == Frame.TAG:
        # TAG坐标系：使用refer_pose作为变换矩阵
        transform = Transform3D(
            trans_pose=refer_pose,
            source_frame=Frame.TAG,
            target_frame=Frame.ODOM
        )
        print(f"🔵 使用TAG坐标系转换")
        print(f"🔵 transform: {transform.trans_pose.pos}, {transform.trans_pose.quat}")
        # print(f"🔵 refer_pose: {refer_pose.pos}, {refer_pose.quat}")
        print(f"🔵 torso_pose_in_refer: {torso_pose_in_refer.pos}, {torso_pose_in_refer.quat}")
    elif torso_pose_in_refer.frame == Frame.BASE:
        # BASE坐标系：直接使用，不需要转换
        torse_pose_in_world = torso_pose_in_refer
        print("🔵 使用BASE坐标系直接转换")
    else:
        # 其他坐标系：使用refer_pose的frame作为source_frame
        transform = Transform3D(
            trans_pose=refer_pose,
            source_frame=refer_pose.frame,
            target_frame=Frame.ODOM
        )
        print(f"🔵 使用其他坐标系转换refer_pose.frame: {refer_pose.frame}")

    if torso_pose_in_refer.frame != Frame.BASE:
        torse_pose_in_world = transform.apply_to_pose(torso_pose_in_refer)
    q = torse_pose_in_world.quat  # [w,x,y,z]
    yaw = R.from_quat(q).as_euler('zyx')[0]
    x, y, z, w = R.from_euler('z', yaw).as_quat();
    if torse_pose_in_world.pos[0] < 0.05:
        base_link = [0.0480578, 0.000463839, torse_pose_in_world.pos[2], w, x, y, z]
        print(f"🔵 torso初始化位置")
    else:
        base_link = [torse_pose_in_world.pos[0], torse_pose_in_world.pos[1], torse_pose_in_world.pos[2], w, x, y, z]
    print(f"🔵 torso在world坐标系中的位姿: {base_link}")

    # 设置目标base_link位姿 [x, y, z, qw, qx, qy, qz]
    request.base_link = base_link

    rospy.loginfo("发送IK请求...")
    rospy.loginfo(f"with_chassis: {request.with_chassis}")
    rospy.loginfo(f"chassis_info: {request.chassis_info}")
    rospy.loginfo(f"q_lb: {request.q_lb}")
    rospy.loginfo(f"control_type: {request.control_type}")
    rospy.loginfo(f"base_link: {request.base_link}")

    for attempt in range(3):
        try:
            rospy.loginfo(f"尝试IK求解 (第{attempt + 1}次)")
            # 调用IK服务
            ik_response = ik_service(request)

            if ik_response.success:
                rospy.loginfo("IK求解成功！")
                rospy.loginfo(f"腿部关节角度: {ik_response.lb_leg}")
                rospy.loginfo(f"求解耗时: {ik_response.time_cost:.2f} ms")

                # 获得逆解结果后，调用控制服务执行这些关节角度
                rospy.loginfo("开始执行控制服务...")

                # 调用新提取的执行函数
                control_success = execute_leg_control(ik_response.lb_leg, duration)

                if control_success:
                    rospy.loginfo("腿部控制执行完成")
                    return True
                else:
                    rospy.logwarn("腿部控制执行失败")
                    return False
            else:
                rospy.logwarn(f"IK求解失败 (第{attempt + 1}次)")
                rospy.logwarn(f"求解耗时: {ik_response.time_cost:.2f} ms")
                if attempt < 2:  # 还有重试机会
                    time.sleep(0.01)
                else:
                    return False
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败 (第{attempt + 1}次): {e}")
            if attempt < 2:  # 还有重试机会
                time.sleep(0.01)
            else:
                return False