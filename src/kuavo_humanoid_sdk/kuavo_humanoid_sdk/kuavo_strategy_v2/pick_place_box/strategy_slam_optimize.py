from re import T
import time
from typing import List, Dict, Any, Tuple

import numpy as np
import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray, String, Bool, Int32, Float32, Int8
import multiprocessing
# 导入配置文件
from .configs.config_real_optimize import config

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState,LejuClaw
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode, KuavoManipulationMpcFrame, KuavoPose
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest, controlLejuClawResponse
from geometry_msgs.msg import Twist


# from nav_straight_walk.srv import SetPose2D,SetPose2DRequest
from std_srvs.srv import Trigger,TriggerRequest

import threading



"""
策略编写原则：

1、策略间无隐藏状态传递：策略和策略之间不能有隐藏的状态传递。所有变量必须显式传递。这样做是为了确保每个策略可以通过构造输入来单独启用和测试。
2、事件实例的复用：事件实例可以复用，但不能跨策略传递状态。每个策略应管理自己的事件状态。
3、事件的抽象：
    事件应包含开始、过程和终止三个阶段，并在结束时返回一个明确的状态。
    事件可以是阻塞的（执行时独占资源，直到完成）或非阻塞的（允许与其他事件并发执行）。
    如果是非阻塞事件，可能出现多个事件同时运行的情况。
4、示例：比如“移动寻找Tag”策略可能由三个事件组成：移动事件、感知事件和动头事件，它们协同完成任务。
5、事件的可测试性：每个事件应能单独测试，因为它有定义良好的目标（Target）和明确的输入输出
"""

def control_leju_claw(position, velocity=[50, 50], effort=[1.0, 1.0],
                    claw_names=['left_claw', 'right_claw'],
                    service_name='/control_robot_leju_claw'):
    """
    控制乐居爪子的优雅封装函数

    参数:
        position: 爪子位置列表 [left_pos, right_pos]
        velocity: 爪子速度列表 [left_vel, right_vel] (默认[50, 50])
        effort: 爪子力度列表 [left_effort, right_effort] (默认[1.0, 1.0])
        claw_names: 爪子名称列表 (默认['left_claw', 'right_claw'])
        service_name: 服务名称 (默认'/control_robot_leju_claw')
    """
    try:
        # 确保服务可用
        rospy.wait_for_service(service_name, timeout=1.0)

        # 创建请求消息
        claw_control_msg = controlLejuClawRequest()
        claw_control_msg.data.name = claw_names
        claw_control_msg.data.position = position
        claw_control_msg.data.velocity = velocity
        claw_control_msg.data.effort = effort

        # 调用服务
        service_proxy = rospy.ServiceProxy(service_name, controlLejuClaw)
        response = service_proxy(claw_control_msg)
        return response

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
    except rospy.ROSException as e:
        rospy.logerr(f"Service wait timeout: {e}")
        return None

# 预定义常用动作
CLAW_OPEN = [35, 40]
CLAW_PRE = [35, 40]
CLAW_CLOSE = [90, 40]


class PoseController:
    """
    PoseController 用于控制机器人整体姿态的简易控制器。

    该类通过 ROS 话题 `/cmd_pose` 发布 Twist 消息来控制机器人身体姿态，
    提供了弯腰和起立等常见动作的封装方法，便于在高层策略中调用。

    属性：
        cmd_pose_pub (rospy.Publisher): ROS 发布器，向 `/cmd_pose` 话题发送 Twist 消息。

    方法：
        _create_pose_msg(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
            内部方法，生成一个带有指定线速度和角速度的 Twist 消息。

        bend_down(angular_y=0.2):
            执行弯腰动作，默认绕 Y 轴旋转 0.2 弧度。

        stand_up():
            执行起立动作，恢复为全零姿态。
    """
    def __init__(self):
        self.cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)

    def _create_pose_msg(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                        angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """创建并返回一个Twist消息"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        return msg

    def bend_down(self, angular_y=0.2):
        """弯腰动作"""
        pose_msg = self._create_pose_msg(angular_y=angular_y)
        self.cmd_pose_pub.publish(pose_msg)
        rospy.loginfo("Executing bend down action")

    def stand_up(self):
        """起立动作"""
        pose_msg = self._create_pose_msg()  # 全部为0的默认值
        self.cmd_pose_pub.publish(pose_msg)
        rospy.loginfo("Executing stand up action")

controller = PoseController()

def build_pose_from_tag(tag_position, pos_offset, euler_deg):
    """根据 tag 的位置与给定偏移/欧拉角构造 Pose（Frame.BASE）"""
    return Pose.from_euler(
        pos=(
            tag_position.x + pos_offset[0],
            tag_position.y + pos_offset[1],
            tag_position.z + pos_offset[2],
        ),
        euler=euler_deg,
        degrees=True,
        frame=Frame.BASE,
    )

def start_navigation(x, y, theta, is_keep_yaw=False):
    """
    Start navigation to the specified 2D pose

    Args:
        x (float): Target x position
        y (float): Target y position
        theta (float): Target orientation in radians

    Returns:
        bool: True if navigation started successfully, False otherwise
    """
    try:
        set_nav_goal_2d_srv_name = '/set_nav_goal_2D'
        set_goal_client = rospy.ServiceProxy(set_nav_goal_2d_srv_name, SetPose2D)
        req = SetPose2DRequest()
        req.pose.x = x
        req.pose.y = y
        req.pose.theta = theta
        req.is_keep_yaw = is_keep_yaw

        req.use_final_adjustment = False  # 启用最终调整
        req.use_user_set_reach_goal_safe_region_radius = False  # 使用默认安全区域半径
        req.reach_goal_safe_region_radius = 0.1

        resp = set_goal_client(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

def stop_navigation():
    """
    Cancel current navigation task

    Returns:
        bool: True if cancellation was successful, False otherwise
    """
    try:
        stop_nav_goal_2d_srv_name = '/stop_nav_goal_2D'
        cancel_goal_client = rospy.ServiceProxy(stop_nav_goal_2d_srv_name, Trigger)
        req = TriggerRequest()
        resp = cancel_goal_client(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

def move_backward(nav, step_back_distance=0.6,direction='pick', is_keep_yaw=True):
    """
    Move backward for pick or place operation.

    Args:
        nav: Target navigation coordinates (x, y, yaw)
        direction: 'pick' to subtract step_back_distance, 'place' to add
        is_keep_yaw: Whether to maintain the yaw orientation
    """
    if direction == 'pick':
        new_x = nav[0] - step_back_distance
    elif direction == 'place':
        new_x = nav[0] + step_back_distance
    else:
        rospy.logerr(f"Invalid direction: {direction}. Use 'pick' or 'place'")
        return False

    success = start_navigation(new_x, nav[1], nav[2], is_keep_yaw=is_keep_yaw)
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    return success

def move_forward_backward(nav, step_distance=0, direction='forward', is_keep_yaw=True):
    """
    Move backward or forward for pick or place operation.

    Args:
        nav: Target navigation coordinates (x, y, yaw)
        step_distance: Distance to move (positive)
        direction: 'forward' or 'backward'
        is_keep_yaw: Whether to maintain the yaw orientation
    """
    current_x, current_y, current_yaw = nav

    if direction == 'forward':
        delta_x = step_distance * math.cos(current_yaw)
        delta_y = step_distance * math.sin(current_yaw)
    elif direction == 'backward':
        delta_x = -step_distance * math.cos(current_yaw)
        delta_y = -step_distance * math.sin(current_yaw)
    else:
        rospy.logerr(f"Invalid direction: {direction}. Use 'forward' or 'backward'")
        return False
    new_x = current_x + delta_x
    new_y = current_y + delta_y

    success = start_navigation(new_x, new_y, current_yaw, is_keep_yaw=is_keep_yaw)
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    return success

def arm_prepare_for_grab(robot_sdk: RobotSDK):
    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()

    target_poses = [[0.5,[15.937308712342915, -0.6338455017979093, 8.567882779750494, -42.68665724749538, -86.2037426014297, -25.288238567063964, -7.715558515999674, 1.8702293616666483, -1.4645594234904613, 1.0054488324642354, -1.3987896969101004, 0.5247115793417383, -0.19655156677224864, -0.17645495264345115]]
    ]

    execute_joint_trajectory(robot_sdk, target_poses)
    time.sleep(1.3)


def grab_object_and_backward(
        second_row:List[int],
        fourth_row:List[int],
        fifth_row:List[int],
        sixth_row:List[int],
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        step_back_distance: float,  # 向后平移的距离，单位米
        target_tag_id: int,
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息
        enable_arm_control: bool = True,  # 是否启用胳膊和灵巧手控制
        enable_backward: bool = True,
        nav_position: Tuple[List, List] = None,
) -> bool:
    """
    执行“抓取目标物体并向后平移”的复合策略。

    本函数先通过指定的 AprilTag ID 定位物体，必要时控制机器人下蹲，
    然后驱动手臂完成抓取动作，最后可选地向后平移一定距离，用于将物体搬运至安全位置。

    参数：
        fourth_row (List[int]): 需要弯腰执行的 ID 列表
        fifth_row (List[int]): 需要下蹲执行的 Tag ID 列表。
        robot_sdk (RobotSDK): 机器人 SDK 实例，封装底层控制接口。
        arm_event (EventArmMoveKeyPoint): 手臂运动事件实例，用于执行关键点轨迹。
        step_back_distance (float): 向后平移的距离（米）。
        target_tag_id (int): 目标物体对应的 AprilTag ID。
        tag (Tag): 目标标签对象，包含位姿信息。如果为 None，将自动订阅检测结果。
        enable_arm_control (bool, 可选): 是否启用手臂和夹爪执行抓取，默认 True。
        enable_backward (bool, 可选): 是否在抓取后执行向后平移，默认 True。
        nav_position (tuple[list, list], 可选): 抓取完成后导航的目标位置，格式 (x, y, yaw)，默认 None。

    返回：
        bool: True 表示抓取并后退成功；False 表示执行失败（如未检测到目标或手臂动作失败）。
    """

    # 如果不启用胳膊控制，直接执行后退操作
    if not enable_arm_control:
        print("🔵 跳过胳膊和灵巧手控制，直接执行后退操作...")
        move_backward(nav_position,step_back_distance,direction='pick',is_keep_yaw=True)
        return True
    # =================== 计算每个关键点的手臂位姿（Pose） —— 数据驱动 =================== #
    # 使用配置中的行定义构造计划，避免重复硬编码
    row_plans = []
    for key, ids in (
        ('second_row', second_row),
        ('fourth_row', fourth_row),
        ('fifth_row', fifth_row),
        ('sixth_row', sixth_row),
    ):
        cfg = config.pick.row_configs[key]
        squat_tuple = None
        if cfg['squat'] is not None:
            squat_tuple = (cfg['squat']['height'], cfg['squat']['pitch'])
        row_plans.append({
            'key': key,
            'ids': ids,
            'tag_offset': cfg['tag_offset'],
            'squat': squat_tuple,
            'motion_delay': cfg['motion_delay'],
            'head_pitch_deg': cfg['head_pitch_deg'],
            'debug_row_name': cfg['debug_row_name'],
        })

    selected_plan = {
        'key': 'third_row',
        'ids': None,
        'tag_offset': 0,
        'squat': None,
        'motion_delay': 0.0,
        'head_pitch_deg': None,
        'debug_row_name': '第3排',
    }

    for plan in row_plans:
        if target_tag_id in plan['ids']:
            selected_plan = plan
            break

    start_time = time.time()
    # 预动作：下蹲或低头（允许并行）
    threads = []

    if selected_plan['squat'] is not None:
        def squat_task_func_():
            height, pitch = selected_plan['squat']
            time.sleep(1.5)
            robot_sdk.control.squat(height, pitch)
            time.sleep(selected_plan['motion_delay'])

        squat_thread = threading.Thread(target=squat_task_func_, name='pre_action_squat')
        squat_thread.start()
        threads.append(squat_thread)

    if selected_plan['head_pitch_deg'] is not None:
        def head_task_func_():
            robot_sdk.control.control_head(
                yaw=np.deg2rad(0),
                pitch=np.deg2rad(selected_plan['head_pitch_deg'])
            )
            time.sleep(1)

        head_thread = threading.Thread(target=head_task_func_, name='pre_action_head')
        head_thread.start()
        threads.append(head_thread)
    # 关闭线程
    for t in threads:
        t.join()

    print(f"############ 第二步中 1 预动作：下蹲或低头 耗时: {time.time() - start_time} 秒")


    query_id = target_tag_id - selected_plan['tag_offset'] if selected_plan['tag_offset'] else target_tag_id

    print("query_id", query_id)
    print("selected_plan", selected_plan)

    start_time = time.time()
    tag_info = get_stable_tag_position(target_id=query_id, scan_count=5, max_deviation=0.03)
    print(f"############ 第二步中 2 获取目标Tag位置 耗时: {time.time() - start_time} 秒")

    if not tag_info or tag_info[0] != query_id:
        print(f"❌ 未检测到目标Tag或ID不匹配：期望 {query_id}，实际 {tag_info[0] if tag_info else 'None'}")
        return False

    print(f"############################抓取{selected_plan['debug_row_name']}", tag_info[0], query_id)

    offsets = config.pick.pick_pose_offsets[selected_plan['key']]

    pick_left_arm_poses = [
        build_pose_from_tag(tag_info[1], offsets['pre_grasp']['pos_offset'], offsets['pre_grasp']['euler_deg']),
        build_pose_from_tag(tag_info[1], offsets['grasp']['pos_offset'], offsets['grasp']['euler_deg']),
    ]
    pick_right_arm_poses = [
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
    ]

    print("pick_left_arm_poses", pick_left_arm_poses)


    arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
    success = pick_move_arm_and_backward(second_row,fourth_row,fifth_row, sixth_row,tag_info, target_tag_id,robot_sdk, arm_event, arm_traj, step_back_distance, tag=tag, enable_backward=enable_backward,nav_position=nav_position)

    return success


def pick_move_arm_and_backward(
        second_row:List[int],
        fourth_row:List[int],
        fifth_row:List[int],
        sixth_row:List[int],
        tag_info:int,
        target_tag_id:int,
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # 分别存放左臂和右臂的list数据，frame可以是odom或者bask_link
        step_back_distance: float,  # 向后平移的距离，单位米
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None,  # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
        enable_backward: bool = True,
        nav_position: Tuple[List, List] = None,
):
    """
    抓起物体同时向后平移。

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
    start_time = time.time()

    # ================ 夹爪准备 + 手臂并行 ================ #
    threads = []
    arm_status_holder = {'status': None}

    def claw_open_task():
        control_leju_claw(position=CLAW_PRE)

    def arm_move_task():
        arm_event.open()
        if not arm_event.set_target(arm_traj, arm_wrench=arm_wrench,  tag=tag):
            print("❌ 设置手臂key point失败")
            arm_status_holder['status'] = EventStatus.FAILED
            arm_event.close()
            return
        while True:
            arm_status = arm_event.step()
            if arm_status != EventStatus.RUNNING:
                break
        arm_status_holder['status'] = arm_status
        arm_event.close()

        # 夹爪关闭
        control_leju_claw(position=CLAW_CLOSE, velocity=[90, 50])
        time.sleep(0.5)

    threads.append(threading.Thread(target=claw_open_task, name='prep_claw_open'))
    threads.append(threading.Thread(target=arm_move_task, name='move_arm_task'))

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    if arm_status_holder['status'] != EventStatus.SUCCESS:
        print("❌ 手臂移动失败，退出策略。")
        return False

    print("✅ 已成功移动手臂，开始向后平移...")



    print(f"✅ ############ 第二步中 3 夹取物品，手臂开始向前...{time.time() - start_time}")

    # =================== 收臂与观察 —— 数据驱动 =================== #
    row_plans = []
    for key, ids in (
        ('second_row', second_row),
        ('fourth_row', fourth_row),
        ('fifth_row', fifth_row),
        ('sixth_row', sixth_row),
    ):
        cfg = config.pick.row_configs[key]
        row_plans.append({ 'key': key, 'ids': ids, 'debug_row_name': cfg['debug_row_name'] })

    selected_plan = { 'key': 'third_row', 'ids': None, 'debug_row_name': '第3排' }
    for plan in row_plans:
        if target_tag_id in plan['ids']:
            selected_plan = plan
            break

    tag_offset = config.pick.row_configs[selected_plan['key']]['tag_offset'] if selected_plan['key'] in config.pick.row_configs else 0
    print(f"############################收臂{selected_plan['debug_row_name']}", tag_info[0], target_tag_id - tag_offset)

    offsets = config.pick.post_grasp_offsets[selected_plan['key']]
    chest_left_pose = [
        build_pose_from_tag(tag_info[1], offsets['withdraw']['pos_offset'], offsets['withdraw']['euler_deg']),
        Pose.from_euler(pos=offsets['observe']['pos_abs'], euler=offsets['observe']['euler_deg'], degrees=True,
                        frame=Frame.BASE),
    ]

    start_time = time.time()

    chest_right_pose = [
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
    ]
    chest_traj = (chest_left_pose, chest_right_pose)

    def move_arm_to_chest(arm_event, chest_traj, tag):

        arm_event.open()
        if not arm_event.set_target(chest_traj, tag=tag):
            print("❌ 设置收回手臂key point失败")
            return False

        while True:
            arm_status = arm_event.step()
            if arm_status != EventStatus.RUNNING:
                break
        arm_event.close()
        time.sleep(1)

    def squat_stand(selected_plan, robot_sdk):
        if selected_plan['key'] in config.pick.row_configs:
            cfg = config.pick.row_configs[selected_plan['key']]
            print(f"############################起立{selected_plan['debug_row_name']}", cfg['stand'])

            if cfg['stand'] is not None:
                time.sleep(2)
                height, pitch, delay = cfg['stand']['height'], cfg['stand']['pitch'], cfg['stand_delay']
                robot_sdk.control.squat(height, pitch)
                time.sleep(delay)

    arm_thread = threading.Thread(target=move_arm_to_chest, args=(arm_event, chest_traj, tag))
    squat_thread = threading.Thread(target=squat_stand, args=(selected_plan, robot_sdk))
    arm_thread.start()
    squat_thread.start()
    arm_thread.join()
    squat_thread.join()

    print(f" ############ 第二步 4 抓取完回到胸前 耗时: {time.time() - start_time} 秒")

   # input(" 抓取完回到胸前，按回车继续，获取手臂的角度值 \n")

    print("策略完成。")
    return True


def move_arm_to_head_position(robot_sdk: RobotSDK, target_poses: List[Tuple[float, List[float]]]=None):
    """
    将手臂移动到头部附近的位置。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例。
        target_poses (List[Tuple[float, List[float]]]): 目标位置列表，每个元素包含时间和关节角度列表。
    """
    # 开始移动手臂到头部附近
    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()

    target_poses = [
        [0.8,[-30, -20, -20, -90-10, 0, -15, 3, 2.5, -1.5, 1.3, -2, 0.17, 0.065, 0.37]],
        # [0.8,[-85, 32.5-20, -54, -96.4, 26.6, -6.7, 3, 2.5, -1.5, 1.3, -2, 0.17, 0.065, 0.37]],
    ]

    execute_joint_trajectory(robot_sdk, target_poses)

def move_arm_to_head_position_for_detector(robot_sdk: RobotSDK, target_poses: List[Tuple[float, List[float]]]=None):
    """
    将手臂移动到头部附近的位置。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例。
        target_poses (List[Tuple[float, List[float]]]): 目标位置列表，每个元素包含时间和关节角度列表。
    """
    # 开始移动手臂到头部附近
    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()

    target_poses = [
            [0.4, [-79, 42.5, -54, -126.4, 26.6, -6.7, -4, -65.1, -20.9, 78.59, -77.92, 9.13, -0.37, 7.6]],
            [0.8, [-85, 42.5, -54, -126.4, 26.6, -6.7, 3, 2.5, -1.5, 1.3, -2, 0.17, 0.065, 0.37]],
    ]

    execute_joint_trajectory(robot_sdk, target_poses)


def detect_18bit_barcode(robot_sdk,  max_retries=3):

    for i in range(max_retries):
        print(f"尝试第{i}次获取条形码")
        result = subscribe_18bit_barcode_info()
        if result is not None and "No_18bit_Barcode" not in result:  # 根据实际情况调整空值判断
            return result

        print(f"第{i + 1}次扫描失败，准备重试...")
        time.sleep(0.05)

    return "Scan_Failed_After_Retries"

def place_arm_from_head_position(robot_sdk: RobotSDK, target_poses: List[Tuple[float, List[float]]]=None):
    """
    将手臂从头部附近移开。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例。
        target_poses (List[Tuple[float, List[float]]]): 目标位置列表，每个元素包含时间和关节角度列表。
    """
    # 将手臂从头部附近移开放下
    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()

    target_poses = [
            [0.5,[-9.318774930143135, 15.97743809386955, -35.932496082596046, -70.59795547377867, -86.22546969950477, 33.02592399940572, -39.3642811121513, 4.835923944612294, -0.41531325050333595, 0.8960756359245267, -1.9451379955373753, -0.1094481700510028, -8.11787725087782e-06, 0.28413018561886666]],
            [0.8,[-21.76476148694459, 18.425454645052362, -47.34217269017093, -71.88755302880611, -86.22547623989553, 21.17957044477354, -40.41334952520176, 4.796660829585416, -0.4153077714450863, 0.8962580456328074, -1.943636476729208, -0.10931241094528285, -1.303784962296106e-05, 0.28260533641728836]],
    ]

    execute_joint_trajectory(robot_sdk, target_poses)


def place_object_and_backward(
        robot_sdk:RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        step_back_distance: float,  # 向后平移的距离，单位米
        tag: Tag,  # 可选的目标标签，用于获取位置和姿态信息
        enable_arm_control: bool = True,  # 是否启用胳膊和灵巧手控制，默认False。
        enable_backward:bool = True,
        palce_position:Tuple[List, List] = None,
):
    """
    执行“放置物体并向后平移”的复合策略。

    本函数先将抓取的物体移动至放置位置，并根据需要控制夹爪释放，
    然后可选地向后平移指定距离，以便让机器人与目标保持安全距离。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例，封装底层控制接口。
        arm_event (EventArmMoveKeyPoint): 手臂运动事件实例，用于执行手臂放置动作。
        step_back_distance (float): 放置完成后向后平移的距离（米）。
        tag (Tag): 目标标签对象，包含位姿信息。若为 None，则放置位置需通过 palce_position 指定。
        enable_arm_control (bool, 可选): 是否启用手臂和夹爪完成放置动作，默认 True。
        enable_backward (bool, 可选): 是否在放置完成后执行后退动作，默认 True。
        palce_position (tuple[list, list], 可选): 放置动作完成后导航的目标位置 (x, y, yaw)，默认 None。

    返回：
        bool: True 表示成功完成放置与后退流程；False 表示执行失败。
    """

    # 如果不启用胳膊控制，直接执行后退操作
    if not enable_arm_control:
        print("🔵 跳过胳膊和灵巧手控制，直接执行后退操作...")

        move_backward(palce_position,step_back_distance,direction='place',is_keep_yaw=True)
        return True

    success = place_move_arm_and_backward(robot_sdk)

    return success

def place_move_arm_and_backward(
        robot_sdk: RobotSDK
    ):
    """
    放置物品并向后平移。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例。

    返回：
        bool: 是否成功完成操作。
    """

    print("✅ 弯腰放置！！！ ")
    start_time = time.time()
    ########################################改为关节角度控制手臂放置测试#########################################
    def robot_arm_place_execute(robot_sdk: RobotSDK):
        robot_arm=KuavoRobotArm()
        robot_arm.set_external_control_arm_mode()

        target_poses = [
                [0.5,[-55.948581, 11.847661, -59.109643, -64.087237, -19.720226, -8.202122, -20.115264, 27.047374, -11.406113, 11.485359, -56.491676, -13.801708, -1.090193, 8.369241]],
                [0.8,[-58.980504, -12.534475, -67.546314, -17.813916, -59.497553, 19.461069, -24.524301, 25.500536, -7.994606, 12.113954, -51.104790, -11.337213, -3.625663, 7.541371]],
                [1.1,[-56.032080, -10.262997, -63.050543, -15.809181, -50.157864, -7.583795, -27.906737, 24.703313, -7.701698, 12.249448, -49.079786, -11.744624, -2.908911, 6.466917]],
                [1.3,[-56.143881, -10.302724, -62.923698, -16.181655, -49.867205, -9.029821, -27.759823, 24.589863, -7.718482, 12.225287, -48.762838, -11.986387, -2.829542, 6.420256]],
        ]
        execute_joint_trajectory(robot_sdk,target_poses)
        time.sleep(1.3)

    ########放置弯腰0.2########################
    def place_bend_down():
        time.sleep(1.3)
        robot_sdk.control.squat(config.place.place_squat_height_pick, config.place.place_squat_pitch_pick)
        time.sleep(0.5)

    def control_leju_claw_func(position: List[int]):
        print("✅ 开夹爪！！！ ")
        #########🕉️🕉️🕉️开夹爪#####################
        time.sleep(1.3)
        control_leju_claw(position=CLAW_OPEN)
        time.sleep(0.5)

    thread_robot_arm_place_execute = threading.Thread(target=robot_arm_place_execute, args=(robot_sdk,))
    thread_place_bend = threading.Thread(target=place_bend_down)
    thread_control_leju_claw = threading.Thread(target=control_leju_claw_func, args=(CLAW_OPEN,))

    thread_robot_arm_place_execute.start()
    thread_place_bend.start()
    thread_control_leju_claw.start()
    thread_robot_arm_place_execute.join()
    thread_place_bend.join()
    thread_control_leju_claw.join()

    print(f" 第六步中 1 弯腰放置 耗时: {time.time() - start_time} 秒")


    start_time = time.time()

    ###########放置弯腰后起立 ########################
    def place_stand_up():
        time.sleep(1)
        robot_sdk.control.squat(config.place.place_stand_height_pick, config.place.place_stand_pitch_pick)
        time.sleep(1)
        robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))


    ########################################改为关节角度控制收回手臂到胸前->下垂测试######################
    def move_arm_backward_after_place():
        target_poses = [
            [0.2,[-39.276179, 17.997447, -62.841195, -76.585956, -47.677527, -17.101594, -13.604045, 28.374740, -11.619686, 13.296413, -56.728886, -14.791013, -2.110495, 14.649980]],
            [0.4,[-33.783113, 30.951001, -62.474620, -93.021795, -52.939490, -6.111006, -18.186877, 25.932421, -10.528018, 12.913447, -51.203134, -15.100647, -1.816185, 13.363724]],
            [0.6,[-13.952884, 36.701338, -54.991005, -99.395940, -47.932208, 6.417060, -9.210062, 23.318515, -9.138297, 11.900506, -46.785559, -14.979767, -1.638199, 11.195697]],
            [0.8,[2.861067, 33.726738, -48.653137, -96.991292, -42.766185, 10.552176, -1.193767, 23.862625, -9.142449, 12.187975, -46.781768, -14.938248, -1.710637, 10.138234]],
            [1.0,[10.573113, 4.605678, -20.807334, -20.349897, -9.777089, 12.313784, -9.395159, 10.291423, -4.738348, 11.655110, -20.388633, -15.214493, -1.107452, 7.228860]],
        ]
        # target_poses = [
        # [0.5, [21.573113, 8.605678, -20.807334, -47.349897, -9.777089, 12.313784, -9.395159, 24.291423, -8.738348, 11.655110, -46.388633, -15.214493, -1.107452, 7.228860]],
        # ]
        execute_joint_trajectory(robot_sdk,target_poses)
        time.sleep(1)

    thread_place_stand_up = threading.Thread(target=place_stand_up)
    thread_move_arm_backward_after_place = threading.Thread(target=move_arm_backward_after_place)
    thread_place_stand_up.start()
    thread_move_arm_backward_after_place.start()
    thread_place_stand_up.join()
    thread_move_arm_backward_after_place.join()

    print(f" 第六步 抓取完成起身耗时: {time.time() - start_time} 秒")

    return True

def execute_joint_trajectory(robot_sdk: RobotSDK, target_poses: list):
    """
    Execute a joint trajectory on the robot arm.

    Args:
        robot_sdk: Instance of RobotSDK
        target_poses: List of target poses, where each pose is a list in format [time, [joint_angles_in_degrees]]
                      Example: [[1.0, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
                               [2.5, [30, 10, 5, -25, 5, 5, 5, 25, 5, 5, -25, 5, 5, 5]]]
    """
    if not target_poses:
        print("Error: target_poses is empty!")
        return False

    times = [pose[0] for pose in target_poses]

    # Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]

    print("🚀🚀🚀🚀🚀🚀")
    if not robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")
        return False

    return True

def subscribe_and_print_tag_info(target_tag_id=None):
    """
    订阅'/robot_tag_info_smt'话题并打印出检测到的标签的位姿信息。
    如果指定了target_tag_id，则只返回该ID的标签信息。

    参数:
        target_tag_id: 目标标签ID，如果为None则打印所有标签信息

    返回:
        tuple: 如果找到目标标签，返回(tag_id, position, orientation)，否则返回None
    """
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('tag_info_subscriber', anonymous=True)
    try:
        msg = rospy.wait_for_message('/robot_tag_info', AprilTagDetectionArray, timeout=5.0)
        for detection in msg.detections:
            tag_id = detection.id[0] if detection.id else "Unknown"
            if tag_id == target_tag_id:
                print("二维码坐标:",tag_id, detection.pose.pose.pose.position, detection.pose.pose.pose.orientation)
                return (tag_id, detection.pose.pose.pose.position, detection.pose.pose.pose.orientation)
    except rospy.ROSException:
        print("Timeout while waiting for tag message")
    return None

def wait_tag_ids(color):
    """
    阻塞直到收到 ID 列表
    return  -> list[int]
    """
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('tag_info_subscriber', anonymous=True)
    topic = f"/{color}_tag_ids"          # -> /red_tag_ids 或 /blue_tag_ids
    rospy.loginfo(f"[grasp] 等待 {topic} …")
    msg = rospy.wait_for_message(topic, Int32MultiArray)
    if not msg.data:
        rospy.logerr(f"[grasp] {topic} 空列表！")
        raise RuntimeError("empty tag id list")
    rospy.loginfo(f"[grasp] 抓取顺序 ({color}): {list(msg.data)}")
    return list(msg.data)

def init_arm_trajectory(robot_sdk: RobotSDK):
    """
    初始化机器人手臂的默认关节轨迹。

    本函数定义一条预设的手臂姿态轨迹，并调用底层控制接口执行。
    主要用于机器人启动或任务开始前，将手臂移动到一个标准初始姿态，
    以保证后续动作的稳定性和可重复性。

    参数：
        robot_sdk (RobotSDK): 机器人 SDK 实例，用于调用底层手臂控制接口。

    返回：
        None: 无显式返回值。如果控制失败，将在控制台输出错误提示。
    """
    target_poses = [
        [0.5, [21.573113, 8.605678, -20.807334, -47.349897, -9.777089, 12.313784, -9.395159, 24.291423, -8.738348, 11.655110, -46.388633, -15.214493, -1.107452, 7.228860]],
    ]

    times = [pose[0] for pose in target_poses]

    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]

    if not robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")

def get_tag_info_fast(target_id, max_retries=5):
    """快速重试获取标签信息，避免不必要的函数调用"""
    for _ in range(max_retries):
        result = subscribe_and_print_tag_info(target_tag_id=target_id)
        if result is not None:  # 根据实际情况调整空值判断
            return result
    return None

def get_stable_tag_position(target_id, scan_count=5, max_deviation=0.03):
    """
    多次扫描获取稳定的标签位置，过滤突变值

    参数:
        target_id: 目标标签ID
        scan_count: 扫描次数
        max_deviation: 最大允许偏差(米)

    返回:
        稳定的标签信息或None
    """
    positions = []
    valid_tag_info = None

    print(f"开始多次扫描标签 {target_id}，扫描 {scan_count} 次...")

    # 收集多次扫描结果
    for i in range(scan_count):
        tag_info = get_tag_info_fast(target_id, max_retries=3)
        if tag_info and tag_info[0] == target_id:
            pos = tag_info[1]
            positions.append([pos.x, pos.y, pos.z])
            valid_tag_info = tag_info
            print(f"扫描 {i+1}: 位置 ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        else:
            print(f"扫描 {i+1}: 未检测到标签")
        time.sleep(0.1)  # 扫描间隔

    if len(positions) < 3:
        print(f"有效扫描次数不足 ({len(positions)}/{scan_count})，无法获取稳定位置")
        return None

    # 计算平均位置
    positions = np.array(positions)
    mean_pos = np.mean(positions, axis=0)

    # 过滤突变值
    valid_positions = []
    for pos in positions:
        deviation = np.linalg.norm(pos - mean_pos)
        if deviation <= max_deviation:
            valid_positions.append(pos)
        else:
            print(f"过滤突变值: 偏差 {deviation:.3f}m > {max_deviation}m")

    if len(valid_positions) < 2:
        print("过滤后有效位置不足，使用原始检测结果")
        return valid_tag_info

    # 使用过滤后的位置计算最终稳定位置
    stable_pos = np.mean(valid_positions, axis=0)
    print(f"稳定位置: ({stable_pos[0]:.3f}, {stable_pos[1]:.3f}, {stable_pos[2]:.3f})")

    # 更新标签位置信息
    if valid_tag_info:
        valid_tag_info[1].x = stable_pos[0]
        valid_tag_info[1].y = stable_pos[1]
        valid_tag_info[1].z = stable_pos[2]
        return valid_tag_info

    return None

def arm_grab_pose_adjust(
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None,  # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
        enable_backward:bool = True,
        palce_position:Tuple[List, List] = None,
        kuavo_observation = None,
        task_type: str = "grab"
    ):
    def wait_for_navigation_near_completion():
        """改进的导航等待函数"""
        check_count = 0

        print("开始监控导航进度...")

        while True:
            try:
                current_pos = robot_sdk.state.robot_position()
                check_count += 1

                if current_pos is None:
                    print(f"位置获取失败 (尝试 {check_count})")
                    time.sleep(0.5)
                    continue

                print(f"当前位置: x={current_pos[0]:.2f}, y={current_pos[1]:.2f}")

                # 更宽松的触发条件
                if task_type == "grab":
                    # 检查是否接近任何合理的抓取位置
                    if current_pos[0] > 0.0:  # x坐标小于0.82就开始准备
                        print(f"接近抓取区域，x={current_pos[0]:.2f}，开始手臂预备")
                        return True

            except Exception as e:
                print(f"位置检测异常: {e}")

            time.sleep(0.3)  # 增加检查间隔


    # 等待导航接近完成
    wait_for_navigation_near_completion()


    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    target_poses = [
            [0.5,[-29.360422, -0.147433, -13.071436, -128.160133, -52.235542, -62.056332, 28.656031, 25.250437, -8.497068, 10.516729, -51.354687, -13.904824, -3.787784, 9.604658]],
            #[0.8,[-60.117336, 6.013391, -23.468164, -69.605215, -61.357893, -32.750915, 23.040583, 25.227598, -8.846246, 11.515202, -49.609585, -12.808242, -2.620409, 7.260419]],
            #[0.8,[-59.590915, 6.998922, -30.107941, -78.505948, -49.496376, -22.384231, 22.921844, 29.204862, -11.393852, 11.814985, -59.628385, -12.045088, -3.155328, 11.528772]],
    ]
    execute_joint_trajectory(robot_sdk,target_poses)

    return True


keyframes_all = {
        'middle': [-4.26494824435037, 0.874271517666988, -14.119617225832183, -73.46126165500165,
                  -90.92541037955736, 29.06975269443269, -8.283718221141488,
                  -54.173578825581615, -10.469553624840724, 73.52678446742146,
                  -41.20022586773412, -80.93620017082335, 38.555+10, 1.3987558967047071],

        'up': [-7.3957393953519945, 0.874290289201475, -14.119509821813628, -73.92013403959594,
               -90.94673315966298, 13.616895203842105, -9.44225186173218,
               -53.536663446752144, -8.89574710141445, 70.29045957983794,
               -41.19860284459875, -79.01294887824291, 38.555+10, 1.3773472622566927],

        'back': [-14.194362211188611, 0.9398908383763606, -12.545870043641484, -71.55959863694784,
                 -90.94690663334282, 12.436554463292586, -9.245465683064413,
                 -53.11540875158718, -6.076305469795286, 71.73452472397915,
                 -41.17835515624331, -79.01273820599327, 38.555+10, 1.377123120861737],

        'front': [2.747863397655428, 0.9398109807607391, -13.420137508283885, -79.60295692735333,
                  -90.94652569798765, 27.430520321057724, -11.605989298146508,
                  -53.08733732110353, -9.442200138691232, 71.69077242784748,
                  -41.17831360403326, -79.01258633535025, 38.555+10, 1.3769182218549667],

        'down': [-6.880190479048735, 0.9179348776108853, -13.310743704454518, -80.65221590379672,
                 -89.81036490047654, 24.96053507625993, -11.25625376655546,
                 -53.04886052871996, -9.048794404252337, 70.05155422618016,
                 -41.17830494735717, -79.01263448977447, 38.555+10, 1.377138174006322]
    }

def generate_extreme_figure8_trajectory(start_time:float, time_duration:float):
    """
    极限8字形轨迹 - 大幅度8字形运动
    """
    left_arm_fixed = [-58.02816392733586, 7.190858224049483, 78.40085603082575,
                      -45.28597675935387, 57.57132879036218, 0.787122339649496, -8.349344787619705]

    initial_pose = [-4.289052440887513, 20.501807230356203, -34.970919137053414,
                   -112.14799702160373, -54.96992440842371, 9, 25.135477645690695]

    target_poses = []
    current_time = start_time

    print("生成极限8字形扫描轨迹...")

    # 起始点
    target_poses.append([current_time, initial_pose + left_arm_fixed])
    current_time += time_duration

    # 大幅度8字形轨迹 - 10个点形成连续运动
    points = 10
    for i in range(points):
        t = 2 * math.pi * i / points

        pose = initial_pose.copy()

        # 大幅度8字形参数方程
        # 第一个大圆
        x1 = 15 * math.sin(t)
        y1 = 12 * math.cos(t)

        # 第二个大圆（相位偏移）
        x2 = 15 * math.sin(t + math.pi)
        y2 = 12 * math.cos(t + math.pi)

        # 合成8字形
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2

        # 应用到关节 - 大幅度
        pose[0] = initial_pose[0] + x  # r_arm_pitch - 大幅度上下运动
        pose[1] = initial_pose[1] + y  # r_arm_roll - 大幅度左右运动

        # 手腕大幅度随动
        pose[4] = initial_pose[4] + 40 * math.sin(t)        # r_hand_yaw 大幅度旋转
        pose[5] = initial_pose[5] + 35 * math.cos(t)        # r_hand_pitch 大幅度调整
        pose[6] = initial_pose[6] + 25 * math.sin(2*t)      # r_hand_roll 大幅度调整

        # 额外添加肘部和肩部调整
        pose[2] = initial_pose[2] + 10 * math.sin(t/2)      # r_arm_yaw 调整
        pose[3] = initial_pose[3] + 8 * math.cos(t/2)       # r_forearm 调整

        target_poses.append([current_time, pose + left_arm_fixed])
        current_time += time_duration

    print(f"生成完成，共 {len(target_poses)} 个轨迹点")
    return target_poses


def generate_full_coverage_trajectory(start_time:float, time_duration:float):
    """
    全覆盖轨迹 - 专门针对手腕夹取区域的死角设计
    通过极大幅度的手腕旋转确保所有角度都被覆盖
    """
    left_arm_fixed = [-58.02816392733586, 7.190858224049483, 78.40085603082575,
                      -45.28597675935387, 57.57132879036218, 0.787122339649496, -8.349344787619705]

    initial_pose = [-4.289052440887513, 20.501807230356203, -34.970919137053414,
                   -112.14799702160373, -54.96992440842371, 9, 25.135477645690695]

    target_poses = []
    current_time = start_time

    print("生成全覆盖扫描轨迹...")

    # 起始点
    target_poses.append([current_time, initial_pose + left_arm_fixed])
    current_time += time_duration

    # 专门针对手腕死角的几个关键位置

    # 位置1: 极限顺时针手腕旋转
    pose1 = initial_pose.copy()
    pose1[4] += 50   # r_hand_yaw 极限顺时针
    pose1[5] += 20   # r_hand_pitch
    pose1[6] += 15   # r_hand_roll
    target_poses.append([current_time, pose1 + left_arm_fixed])
    current_time += time_duration

    # 位置2: 极限逆时针手腕旋转
    pose2 = initial_pose.copy()
    pose2[4] -= 55   # r_hand_yaw 极限逆时针
    pose2[5] -= 18   # r_hand_pitch
    pose2[6] -= 20   # r_hand_roll
    target_poses.append([current_time, pose2 + left_arm_fixed])
    current_time += time_duration

    # 位置3: 极限向上倾斜
    pose3 = initial_pose.copy()
    pose3[0] += 20   # r_arm_pitch 极限前倾
    pose3[5] += 45   # r_hand_pitch 极限向上
    pose3[6] += 30   # r_hand_roll
    target_poses.append([current_time, pose3 + left_arm_fixed])
    current_time += time_duration

    # 位置4: 极限向下倾斜
    pose4 = initial_pose.copy()
    pose4[0] -= 16   # r_arm_pitch 极限后仰
    pose4[5] -= 40   # r_hand_pitch 极限向下
    pose4[6] -= 25   # r_hand_roll
    target_poses.append([current_time, pose4 + left_arm_fixed])
    current_time += time_duration

    # 位置5: 复合极限位置1 - 右上+旋转
    pose5 = initial_pose.copy()
    pose5[0] += 15   # r_arm_pitch
    pose5[1] += 18   # r_arm_roll
    pose5[4] += 35   # r_hand_yaw
    pose5[5] += 30   # r_hand_pitch
    pose5[6] += 35   # r_hand_roll
    target_poses.append([current_time, pose5 + left_arm_fixed])
    current_time += time_duration

    # 位置6: 复合极限位置2 - 左下+反向旋转
    pose6 = initial_pose.copy()
    pose6[0] -= 12   # r_arm_pitch
    pose6[1] -= 20   # r_arm_roll
    pose6[4] -= 45   # r_hand_yaw
    pose6[5] -= 32   # r_hand_pitch
    pose6[6] -= 30   # r_hand_roll
    target_poses.append([current_time, pose6 + left_arm_fixed])
    current_time += time_duration

    # 位置7: 手腕翻滚位置1
    pose7 = initial_pose.copy()
    pose7[6] += 40   # r_hand_roll 极限翻滚
    pose7[5] += 25   # r_hand_pitch
    target_poses.append([current_time, pose7 + left_arm_fixed])
    current_time += time_duration

    # 位置8: 手腕翻滚位置2
    pose8 = initial_pose.copy()
    pose8[6] -= 42   # r_hand_roll 极限反向翻滚
    pose8[5] -= 20   # r_hand_pitch
    target_poses.append([current_time, pose8 + left_arm_fixed])
    current_time += time_duration

    # 回到初始位置
    target_poses.append([current_time, initial_pose + left_arm_fixed])

    print(f"生成完成，共 {len(target_poses)} 个轨迹点")
    return target_poses


def generate_zigzag_from_keyframes(start_time=0.0, time_duration=0.4):
    """
    基于实际采集的关键帧生成Z字型轨迹
    使用真实采集的关节点数据
    """

    # 定义采集的关键帧 (前7个是左手关节，后7个是右手关节)
    keyframes = keyframes_all

    target_poses = []
    current_time = start_time

    print("基于采集关键帧生成Z字型扫描轨迹...")

    # Z字型路径顺序: 中 -> 上 -> 后 -> 前 -> 下 -> 中
    zigzag_sequence = ['middle', 'up', 'back', 'front', 'down', 'middle']

    # 添加起始点
    target_poses.append([current_time, keyframes[zigzag_sequence[0]]])
    current_time += time_duration

    # 生成Z字型轨迹
    for i in range(1, len(zigzag_sequence)):
        current_frame = zigzag_sequence[i-1]
        next_frame = zigzag_sequence[i]

        # 在两个关键帧之间插值生成平滑过渡
        interpolation_points = 3
        for j in range(1, interpolation_points + 1):
            t = j / (interpolation_points + 1)

            # 线性插值
            interpolated_pose = []
            for k in range(14):  # 14个关节
                start_val = keyframes[current_frame][k]
                end_val = keyframes[next_frame][k]
                interpolated_val = start_val + (end_val - start_val) * t
                interpolated_pose.append(interpolated_val)

            target_poses.append([current_time, interpolated_pose])
            current_time += time_duration

        # 添加关键帧点
        target_poses.append([current_time, keyframes[next_frame]])
        current_time += time_duration

    print(f"Z字型轨迹生成完成，共 {len(target_poses)} 个轨迹点")
    return target_poses


def generate_optimized_zigzag_from_keyframes(start_time=0.0, time_duration=0.1):
    """
    优化版本的Z字型轨迹，基于实际采集的关键帧
    添加额外的扫描点和微调
    """

    # 使用采集的关键帧
    keyframes =keyframes_all

    target_poses = []
    current_time = start_time

    print("生成优化Z字型扫描轨迹...")

    # 更复杂的Z字型路径，增加覆盖范围
    zigzag_sequence = ['middle', 'up', 'back', 'middle', 'front', 'down', 'middle']

    # 起始点
    target_poses.append([current_time, keyframes['middle']])
    current_time += time_duration

    # 生成轨迹
    for i in range(1, len(zigzag_sequence)):
        current_frame = zigzag_sequence[i-1]
        next_frame = zigzag_sequence[i]

        # 插值点数量根据距离调整
        current_pose = keyframes[current_frame]
        next_pose = keyframes[next_frame]

        # 计算关节角度变化总量
        total_change = sum(abs(next_pose[j] - current_pose[j]) for j in range(14))
        interpolation_points = max(2, min(5, int(total_change / 5)))  # 根据变化量调整插值点

        print(f"从 {current_frame} 到 {next_frame}: {interpolation_points} 个插值点")

        # 插值生成平滑轨迹
        for j in range(1, interpolation_points + 1):
            t = j / (interpolation_points + 1)

            interpolated_pose = []
            for k in range(14):
                start_val = current_pose[k]
                end_val = next_pose[k]
                interpolated_val = start_val + (end_val - start_val) * t
                interpolated_pose.append(interpolated_val)

            target_poses.append([current_time, interpolated_pose])
            current_time += time_duration

        # 添加目标关键帧
        target_poses.append([current_time, next_pose])
        current_time += time_duration

    # 添加额外的扫描点，提高覆盖度
    extra_points = [
        # 混合位置1: 上+前
        [-5.0, 0.9, -13.5, -76.0, -90.9, 20.0, -10.0,
         -53.5, -9.0, 71.0, -41.19, -79.5, 37.5, 1.38],

        # 混合位置2: 后+下
        [-10.0, 0.93, -13.0, -76.0, -90.3, 18.0, -10.5,
         -53.3, -7.5, 71.0, -41.18, -79.0, 37.0, 1.38]
    ]

    for extra_pose in extra_points:
        target_poses.append([current_time, extra_pose])
        current_time += time_duration

    # 回到起始点
    target_poses.append([current_time, keyframes['middle']])

    print(f"优化Z字型轨迹生成完成，共 {len(target_poses)} 个轨迹点")
    return target_poses


def generate_large_scale_scanning_trajectory(start_time=0.0, time_duration=0.3):
    """
    大幅度扫描轨迹 - 基于采集的关键帧进行扩展
    """

    # 使用middle帧作为基准
    base_frame = [-4.26494824435037, 0.874271517666988, -14.119617225832183, -73.46126165500165,
                  -90.92541037955736, 29.06975269443269, -8.283718221141488,
                  -54.173578825581615, -10.469553624840724, 73.52678446742146,
                  -41.20022586773412, -80.93620017082335, 38.555664016449036+10, 1.3987558967047071]

    target_poses = []
    current_time = start_time

    print("生成大幅度扫描轨迹...")

    # 起始点
    target_poses.append([current_time, base_frame])
    current_time += time_duration

    # 基于基准帧生成大幅度变化
    large_movements = [
        # 大幅向右上
        [8.0, 8.0, -16.0, -70.0, -85.0, 35.0, -5.0,
         -54.0, -10.0, 73.0, -41.0, -80.0, 38.0, 1.4],

        # 大幅向左下
        [-12.0, -6.0, -12.0, -78.0, -95.0, 15.0, -12.0,
         -54.0, -10.0, 73.0, -41.0, -80.0, 38.0, 1.4],

        # 大幅向前
        [5.0, 1.0, -15.0, -82.0, -88.0, 32.0, -7.0,
         -54.0, -10.0, 73.0, -41.0, -80.0, 38.0, 1.4],

        # 大幅向后
        [-18.0, 1.0, -11.0, -68.0, -92.0, 10.0, -11.0,
         -54.0, -10.0, 73.0, -41.0, -80.0, 38.0, 1.4]
    ]

    for movement in large_movements:
        target_poses.append([current_time, movement])
        current_time += time_duration

    # 回到基准点
    target_poses.append([current_time, base_frame])

    print(f"大幅度扫描轨迹生成完成，共 {len(target_poses)} 个轨迹点")
    return target_poses


def print_trajectory_info(target_poses, trajectory_name):
    """
    打印轨迹详细信息
    """
    print(f"\n=== {trajectory_name} 轨迹详情 ===")
    print(f"轨迹点数: {len(target_poses)}")
    print(f"总时长: {target_poses[-1][0]:.1f}秒")

    # 分析关节运动范围
    joints_data = np.array([pose[1] for pose in target_poses])
    print("\n关节运动范围分析:")

    joint_names = [
        "左肩俯仰", "左肩滚动", "左上臂偏航", "左肘俯仰", "左腕偏航", "左腕俯仰", "左腕滚动",
        "右肩俯仰", "右肩滚动", "右上臂偏航", "右肘俯仰", "右腕偏航", "右腕俯仰", "右腕滚动"
    ]

    for i in range(14):
        min_val = np.min(joints_data[:, i])
        max_val = np.max(joints_data[:, i])
        range_val = max_val - min_val
        print(f"{joint_names[i]}: {min_val:.1f}° ~ {max_val:.1f}° (范围: {range_val:.1f}°)")



def generate_extended_zigzag_from_keyframes(start_time=0.0, time_duration=0.3):
    """
    扩展版本的Z字型轨迹，大幅增加扫描覆盖面
    基于实际采集的关键帧，添加更多扫描路径和角度变化
    """

    # 使用采集的关键帧
    keyframes = keyframes_all

    target_poses = []
    current_time = start_time

    print("生成扩展Z字型扫描轨迹 - 大幅增加覆盖面...")

    # 扩展的Z字型路径，增加更多覆盖方向
    zigzag_sequence = [
        'middle', 'up', 'back', 'middle', 'front', 'down', 'middle',
        'up', 'front', 'down', 'back', 'middle'  # 增加反向路径
    ]

    # 起始点
    target_poses.append([current_time, keyframes['middle']])
    current_time += time_duration

    # 生成主轨迹
    for i in range(1, len(zigzag_sequence)):
        current_frame = zigzag_sequence[i-1]
        next_frame = zigzag_sequence[i]

        # 插值点数量根据距离调整
        current_pose = keyframes[current_frame]
        next_pose = keyframes[next_frame]

        # 计算关节角度变化总量
        total_change = sum(abs(next_pose[j] - current_pose[j]) for j in range(14))
        interpolation_points = max(2, min(4, int(total_change / 4)))

        print(f"从 {current_frame} 到 {next_frame}: {interpolation_points} 个插值点")

        # 插值生成平滑轨迹
        for j in range(1, interpolation_points + 1):
            t = j / (interpolation_points + 1)

            interpolated_pose = []
            for k in range(14):
                start_val = current_pose[k]
                end_val = next_pose[k]
                interpolated_val = start_val + (end_val - start_val) * t
                interpolated_pose.append(interpolated_val)

            target_poses.append([current_time, interpolated_pose])
            current_time += time_duration

        # 添加目标关键帧
        target_poses.append([current_time, next_pose])
        current_time += time_duration

    # 增加更多扩展扫描点，大幅提高覆盖度
    extended_points = [
        # 对角线方向1: 右上+前
        [-2.0, 1.2, -14.5, -75.0, -89.0, 25.0, -9.5,
         -53.8, -9.5, 72.0, -41.19, -79.8, 37.8, 1.39],

        # 对角线方向2: 左上+后
        [-12.0, 0.8, -13.0, -72.0, -92.0, 15.0, -10.0,
         -53.4, -7.0, 71.5, -41.18, -79.2, 37.2, 1.38],

        # 对角线方向3: 右下+前
        [1.0, 1.0, -14.0, -78.0, -88.0, 30.0, -10.5,
         -53.6, -9.8, 71.8, -41.19, -79.6, 37.6, 1.39],

        # 对角线方向4: 左下+后
        [-10.0, 0.9, -13.2, -74.0, -91.0, 18.0, -10.8,
         -53.3, -8.0, 71.3, -41.18, -79.1, 37.1, 1.38],

        # 极限位置1: 极右上
        [5.0, 2.0, -15.5, -70.0, -87.0, 35.0, -7.0,
         -54.0, -10.0, 73.0, -41.2, -80.5, 38.5, 1.4],

        # 极限位置2: 极左下
        [-16.0, -1.0, -12.0, -76.0, -94.0, 10.0, -12.0,
         -53.1, -6.0, 71.0, -41.17, -78.5, 36.5, 1.37],

        # 混合位置: 中上+微前
        [-6.0, 1.0, -14.2, -74.5, -90.5, 22.0, -9.0,
         -53.7, -9.2, 71.9, -41.19, -79.7, 37.7, 1.39],

        # 混合位置: 中下+微后
        [-8.0, 0.9, -13.8, -77.0, -91.5, 20.0, -10.2,
         -53.4, -8.5, 71.4, -41.18, -79.3, 37.3, 1.38]
    ]

    print("添加扩展扫描点...")
    for i, extra_pose in enumerate(extended_points):
        target_poses.append([current_time, extra_pose])
        current_time += time_duration
        print(f"添加扩展点 {i+1}")

    # 添加螺旋式扫描点，进一步提高覆盖
    spiral_points = generate_spiral_points(keyframes['middle'], points=6)
    print("添加螺旋扫描点...")
    for i, spiral_pose in enumerate(spiral_points):
        target_poses.append([current_time, spiral_pose])
        current_time += time_duration
        print(f"添加螺旋点 {i+1}")

    # 回到起始点
    target_poses.append([current_time, keyframes['middle']])

    print(f"扩展Z字型轨迹生成完成，共 {len(target_poses)} 个轨迹点")
    print("覆盖范围: 全方位扫描，包括对角线、极限位置和螺旋路径")

    return target_poses


def generate_spiral_points(base_pose, points=6, radius=3.0):
    """
    生成螺旋式扫描点，围绕基准点进行螺旋运动
    """
    spiral_poses = []

    for i in range(points):
        angle = 2 * math.pi * i / points
        # 计算螺旋偏移
        spiral_radius = radius * (i + 1) / points

        # 在水平和垂直方向应用螺旋偏移
        x_offset = spiral_radius * math.cos(angle)
        y_offset = spiral_radius * math.sin(angle)

        # 创建新的姿势
        new_pose = base_pose.copy()

        # 主要调整左手关节（前7个）来实现螺旋运动
        # 肩部关节调整
        new_pose[0] = base_pose[0] + y_offset * 0.6  # 左肩俯仰 - 上下
        new_pose[1] = base_pose[1] + x_offset * 0.4  # 左肩滚动 - 左右

        # 肘部和上臂配合
        new_pose[2] = base_pose[2] + x_offset * 0.2  # 左上臂偏航
        new_pose[3] = base_pose[3] + y_offset * 0.3  # 左肘俯仰

        # 手腕微调保持稳定
        new_pose[4] = base_pose[4] - x_offset * 0.1  # 左腕偏航
        new_pose[5] = base_pose[5] - y_offset * 0.2  # 左腕俯仰
        new_pose[6] = base_pose[6] + (x_offset + y_offset) * 0.05  # 左腕滚动

        spiral_poses.append(new_pose)

    return spiral_poses


def generate_comprehensive_scan_trajectory(start_time=0.0, time_duration=0.3):
    """
    全面扫描轨迹 - 结合多种扫描模式
    包括Z字型、螺旋型、对角线型等多种路径
    """

    # 使用采集的关键帧
    keyframes = keyframes_all

    target_poses = []
    current_time = start_time

    print("生成全面扫描轨迹 - 多种扫描模式组合...")

    # 起始点
    target_poses.append([current_time, keyframes['middle']])
    current_time += time_duration

    # 第一阶段: 基础Z字型
    print("=== 第一阶段: 基础Z字型扫描 ===")
    phase1_sequence = ['middle', 'up', 'back', 'front', 'down', 'middle']
    target_poses.extend(generate_sequence_trajectory(phase1_sequence, keyframes, current_time, time_duration))
    current_time = target_poses[-1][0] + time_duration

    # 第二阶段: 扩展对角线
    print("=== 第二阶段: 对角线扫描 ===")
    diagonal_points = generate_diagonal_points(keyframes, 8)
    for i, diagonal_pose in enumerate(diagonal_points):
        target_poses.append([current_time, diagonal_pose])
        current_time += time_duration
        print(f"添加对角点 {i+1}")

    # 第三阶段: 螺旋扫描
    print("=== 第三阶段: 螺旋扫描 ===")
    spiral_points = generate_spiral_points(keyframes['middle'], points=8, radius=4.0)
    for i, spiral_pose in enumerate(spiral_points):
        target_poses.append([current_time, spiral_pose])
        current_time += time_duration
        print(f"添加螺旋点 {i+1}")

    # 第四阶段: 反向Z字型
    print("=== 第四阶段: 反向Z字型扫描 ===")
    phase4_sequence = ['middle', 'down', 'front', 'back', 'up', 'middle']
    target_poses.extend(generate_sequence_trajectory(phase4_sequence, keyframes, current_time, time_duration))
    current_time = target_poses[-1][0] + time_duration

    # 最终回到起始点
    target_poses.append([current_time, keyframes['middle']])

    print(f"全面扫描轨迹生成完成，共 {len(target_poses)} 个轨迹点")
    print("覆盖模式: Z字型 + 对角线 + 螺旋 + 反向Z字型")

    return target_poses


def generate_sequence_trajectory(sequence, keyframes, start_time, time_duration):
    """
    生成序列轨迹的辅助函数
    """
    poses = []
    current_time = start_time

    for i in range(1, len(sequence)):
        current_frame = sequence[i-1]
        next_frame = sequence[i]

        current_pose = keyframes[current_frame]
        next_pose = keyframes[next_frame]

        # 插值点
        interpolation_points = 2
        for j in range(1, interpolation_points + 1):
            t = j / (interpolation_points + 1)
            interpolated_pose = []
            for k in range(14):
                start_val = current_pose[k]
                end_val = next_pose[k]
                interpolated_val = start_val + (end_val - start_val) * t
                interpolated_pose.append(interpolated_val)
            poses.append([current_time, interpolated_pose])
            current_time += time_duration

        poses.append([current_time, next_pose])
        current_time += time_duration

    return poses


def generate_diagonal_points(keyframes, num_points=6):
    """
    生成对角线扫描点
    """
    diagonal_poses = []

    # 定义四个对角方向
    directions = [
        ('up', 'front'),    # 右上
        ('up', 'back'),     # 左上
        ('down', 'front'),  # 右下
        ('down', 'back')    # 左下
    ]

    for dir1, dir2 in directions:
        pose1 = keyframes[dir1]
        pose2 = keyframes[dir2]

        # 在对角线方向上生成多个点
        for i in range(num_points):
            t = (i + 1) / (num_points + 1)

            diagonal_pose = []
            for j in range(14):
                # 对角插值
                val1 = pose1[j]
                val2 = pose2[j]
                diagonal_val = val1 + (val2 - val1) * t
                diagonal_pose.append(diagonal_val)

            diagonal_poses.append(diagonal_pose)

    return diagonal_poses



def arm_for_detector_code_multiMode(robot_sdk: RobotSDK, start_time:float, time_duration:float, mode:int):

    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    start_time = start_time
    time_duration = time_duration

    if mode == 1:
        target_poses = generate_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "基于关键帧的Z字型"
    elif mode == 2:
        target_poses = generate_optimized_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "优化Z字型"
    elif mode == 3:
        target_poses = generate_extended_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "扩展Z字型"
    elif mode == 4:
        target_poses = generate_comprehensive_scan_trajectory(start_time, time_duration)
        trajectory_name = "全面扫描"
    else:
        target_poses = generate_extended_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "扩展Z字型"

    print(f"轨迹详情: {len(target_poses)}个点，总时长: {target_poses[-1][0]:.1f}秒")
    # 打印关节变化范围
    print("\n关节运动范围:")
    joints_data = [pose[1][:7] for pose in target_poses]  # 只取右臂关节
    joints_data = np.array(joints_data)
    for i in range(7):
        min_val = np.min(joints_data[:, i])
        max_val = np.max(joints_data[:, i])
        range_val = max_val - min_val
        print(f"关节{i}: {min_val:.1f}° ~ {max_val:.1f}° (范围: {range_val:.1f}°)")

    total_length = len(target_poses)
    # 执行轨迹
    if execute_joint_trajectory(robot_sdk, target_poses):
        # 计算总轨迹时间并等待完成
        total_time = target_poses[-1][0] - start_time + 0.2  # 额外留秒缓冲
        print(f"轨迹执行中，预计需要 {total_time:.1f} 秒...")
        time.sleep(total_time)
        print("扫描完成!")
    else:
        print("轨迹执行失败!")

    return True

def split_trajectory_intelligently(trajectory, max_points=80, max_duration=4.0):
    """智能分段：同时考虑点数和时间"""
    segments = []
    current_segment = []

    for point in trajectory:
        current_segment.append(point)

        # 检查分段条件
        time_condition = (point[0] - current_segment[0][0] >= max_duration)
        points_condition = (len(current_segment) >= max_points)

        if time_condition or points_condition:
            segments.append(current_segment)
            current_segment = [point]  # 新段以当前点开始

    # 添加最后一段
    if current_segment:
        segments.append(current_segment)

    return segments

def arm_for_detector_code_multiMode_multiSegment(robot_sdk: RobotSDK, start_time:float, time_duration:float, mode:int, max_points:int, max_duration:float, stop_event):
    '''
    必需参数：
        robot_sdk (RobotSDK): 机器人控制SDK实例，提供底层硬件控制接口

        start_time (float): 轨迹执行的起始时间戳，用于时间同步

        time_duration (float): 期望的轨迹总执行时间

    模式选择：
    mode (int): 轨迹生成算法选择，支持4种预设轨迹模式：
        1: 基础Z字型扫描轨迹

        2: 优化后的Z字型轨迹（更平滑）

        3: 扩展Z字型轨迹（覆盖范围更大）

        4: 全面扫描轨迹（最完整的检测覆盖）

    轨迹分段参数：
        max_points (int): 每段轨迹的最大点数限制，避免单次传输数据过多

        max_duration (float): 每段轨迹的最大持续时间，确保实时性和安全性

    安全控制：
        stop_event: 线程事件对象，用于接收外部停止信号，实现安全中断

    '''

    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    start_time = start_time
    time_duration = time_duration

    if mode == 1:
        target_poses = generate_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "基于关键帧的Z字型"
    elif mode == 2:
        target_poses = generate_optimized_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "优化Z字型"
    elif mode == 3:
        target_poses = generate_extended_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "扩展Z字型"
    elif mode == 4:
        target_poses = generate_comprehensive_scan_trajectory(start_time, time_duration)
        trajectory_name = "全面扫描"
    else:
        target_poses = generate_extended_zigzag_from_keyframes(start_time, time_duration)
        trajectory_name = "扩展Z字型"

    print(f"轨迹详情: {len(target_poses)}个点，总时长: {target_poses[-1][0]:.1f}秒")
    # 打印关节变化范围
    print("\n关节运动范围:")
    joints_data = [pose[1][:7] for pose in target_poses]  # 只取右臂关节
    joints_data = np.array(joints_data)
    for i in range(7):
        min_val = np.min(joints_data[:, i])
        max_val = np.max(joints_data[:, i])
        range_val = max_val - min_val
        print(f"关节{i}: {min_val:.1f}° ~ {max_val:.1f}° (范围: {range_val:.1f}°)")


    segments = split_trajectory_intelligently(target_poses, max_points=60, max_duration=4.0)

    print(f"轨迹拆分为 {len(segments)} 段")

    # 打印各段信息
    for i, segment in enumerate(segments):
        duration = segment[-1][0] - segment[0][0]
        print(f"段 {i+1}: {len(segment)}点, {duration:.1f}秒")

    # 分段执行
    for i, segment in enumerate(segments):
        # 检查停止信号
        if stop_event.is_set():
            print("收到停止信号，终止轨迹执行")
            return False

        print(f"\n=== 执行第 {i+1}/{len(segments)} 段 ===")

        if execute_joint_trajectory(robot_sdk, segment):
            segment_duration = segment[-1][0] - segment[0][0] + 0.1

            # 等待本段完成，但期间检查停止信号
            segment_start_time = time.time()
            while time.time() - segment_start_time < segment_duration:
                time.sleep(0.05)  # 短暂睡眠
                if stop_event.is_set():
                    print("收到停止信号，立即终止")
                    return False

        else:
            print(f"✗ 第 {i+1} 段轨迹执行失败!")
            return False

    print("轨迹执行完成")

    return True

def arm_for_detector_code(robot_sdk: RobotSDK):

    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    start_time = 0.8
    time_duration = 0.4

    target_poses = generate_large_scale_scanning_trajectory(start_time=start_time, time_duration=time_duration)

    print(f"轨迹详情: {len(target_poses)}个点，总时长: {target_poses[-1][0]:.1f}秒")
    # 打印关节变化范围
    print("\n关节运动范围:")
    joints_data = [pose[1][:7] for pose in target_poses]  # 只取右臂关节
    joints_data = np.array(joints_data)
    for i in range(7):
        min_val = np.min(joints_data[:, i])
        max_val = np.max(joints_data[:, i])
        range_val = max_val - min_val
        print(f"关节{i}: {min_val:.1f}° ~ {max_val:.1f}° (范围: {range_val:.1f}°)")

    # 执行轨迹
    if execute_joint_trajectory(robot_sdk, target_poses):
        # 计算总轨迹时间并等待完成
        total_time = target_poses[-1][0] - start_time + 0.2  # 额外留秒缓冲
        print(f"轨迹执行中，预计需要 {total_time:.1f} 秒...")
        time.sleep(total_time)
        print("扫描完成!")
    else:
        print("轨迹执行失败!")

    return True

def arm_place_pose_adjust(
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        tag: Tag = None,  # 可选的目标标签，用于获取位置和姿态信息
        arm_wrench: Tuple[List, List] = None,  # 可选的手臂扭矩数据，分别存放左臂和右臂的扭矩
        enable_backward:bool = True,
        palce_position:Tuple[List, List] = None,
        kuavo_observation = None,
        task_type: str = "place"
    ):

    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    target_poses = [
        [0.5,[-55.94858079892865, 11.847661356046677, -59.1096426277084, -64.08723682515728, -19.72022572676684, -8.202121533739984, -20.115263527957676,
                27.047374428164584, -11.406112946986038, 11.485358748703637, -56.491676386860256, -13.801707640205462, -1.090192867404783, 8.369241324429169]],
        [0.8,[-58.98050409807267, -12.534475294258618, -67.54631403402183, -17.813915836411784, -59.49755270480513, 19.461068839622815, -24.524301346229002,
                25.500536212327944, -7.994605846140557, 12.113954126616326, -51.104789637359964, -11.3372125973276, -3.6256630188787886, 7.54137061053416]],
        [1.1,[-56.032079760328266, -10.26299731183272, -63.05054309117076, -15.809181042588943, -50.15786382975109, -7.583794710486194, -27.906737495740078,
               24.703313302082087, -7.70169820310629, 12.249448409425739, -49.07978604226111, -11.744624012484252, -2.908910763404843, 6.466916894875008]],
        [1.3,[-56.14388067814826, -10.302724231699866, -62.923698126952004, -16.181655080033618, -49.86720549779491, -9.029820922958445, -27.759823330253422,
               24.5898632204952, -7.718481518867492, 12.225287051030929, -48.762837899467435, -11.986386943106142, -2.829542433395776, 6.420255707589033]],
    ]
    execute_joint_trajectory(robot_sdk,target_poses)

    return True

def arm_grab_prepare_pose(
        robot_sdk: RobotSDK,
        target_id: int,
    ):
    print(f"arm_grab_prepare_pose: {target_id}")
    target_pose = [-56.14388067814826, -10.302724231699866, -62.923698126952004, -16.181655080033618, -49.86720549779491, -9.029820922958445, -27.759823330253422,
               24.5898632204952, -7.718481518867492, 12.225287051030929, -48.762837899467435, -11.986386943106142, -2.829542433395776, 6.420255707589033]

    for key in ['second_row', 'third_row', 'fourth_row', 'fifth_row', 'sixth_row']:
        cfg = config.pick.arm_pre_grab_pose[key]
        if target_id in cfg['ids']:
            print(f"找到目标ID: {target_id} 在 {cfg['debug_row_name']} \n 预抓取位姿： {cfg['pre_target_poses']}")

            # 转换为角度单位
            target_pose = [np.rad2deg(angle) for angle in cfg['pre_target_poses']]
            break

    print("target_pose", target_pose)

    robot_arm = KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()
    target_poses = [
        [1, target_pose],
    ]
    execute_joint_trajectory(robot_sdk, target_poses)
    time.sleep(1)

    return True

def subscribe_18bit_barcode_info():
    """
    订阅'/reelid_detector/reelid'话题并打印出检测到的码ID信息。

    返回:
        tuple: 如果找到目标标签，返回(barcode_id)，否则返回None
    """
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('barcode_id_info_subscriber', anonymous=True)
    try:
        msg = rospy.wait_for_message('/reelid_detector/reelid', String, timeout=3.0)
        barcode_id = msg.data
        print(" 扫描获取得到条形码结果:", barcode_id)
        return barcode_id
    except rospy.ROSException:
        print("Timeout while waiting for tag id message")
    return None

def subscribe_palletInHand_info():
    """
    订阅'/ldp_measure/laser_distance_data'话题并获取料盘是否抓取在手中的信息。

    返回:
        bool: 如果料盘抓取在手中，返回True，否则返回False
    """
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('pallet_info_subscriber', anonymous=True)
    try:
        msg = rospy.wait_for_message('/ldp_measure/laser_distance_data', Int32, timeout=2.0)
        distance_range = [5, 250]
        pallet_detected = False
        if msg.data >= distance_range[0] and msg.data <= distance_range[1]:
            pallet_detected = True
        else:
            pallet_detected = False

        print(" 料盘是否抓取在手中: ", pallet_detected)
        return pallet_detected
    except rospy.ROSException:
        print("Timeout while waiting for tag message")
    return False

def subscribe_palletInHand_info_byD435():
    """
    订阅'/pallet/filtered_distance'话题并获取料盘是否抓取在手中的信息。

    返回:
        bool: 如果料盘抓取在手中，返回True，否则返回False
    """
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('pallet_info_subscriber', anonymous=True)
    try:
        msg = rospy.wait_for_message('/pallet/filtered_distance', Float32, timeout=2.0)
        print(" 料盘距离信息！！ ", msg.data)
        distance_range = [0.15, 0.70]
        pallet_detected = False
        if msg.data >= distance_range[0] and msg.data <= distance_range[1]:
            pallet_detected = True
        else:
            pallet_detected = False

        print(" 料盘是否抓取在手中: ", pallet_detected)
        return pallet_detected
    except rospy.ROSException:
        print("Timeout while waiting for tag message")
    return False

def publish_music_trigger(trigger_num:int):
    if rospy.get_node_uri() is None:  # 如果节点未初始化
        rospy.init_node('music_alarm_publisher', anonymous=True)
    try:

        trigger_pub = rospy.Publisher('/music_trigger', Int8, queue_size=10)
        time.sleep(2)

        msg = Int8()
        msg.data = trigger_num
        trigger_pub.publish(msg)
        rospy.sleep(0.1)

        return True
    except rospy.ROSException:
        print("Timeout while publishing message")
    return False

if __name__ == '__main__':

    rospy.init_node('navigation_client')

    # Start navigation to (1.0, 2.0) with orientation 0.0 radians
    success = start_navigation(1.0, 2.0, 0.0)
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")

    # Cancel navigation
    canceled = stop_navigation()
    rospy.loginfo(f"Navigation canceled: {'successfully' if canceled else 'failed'}")
