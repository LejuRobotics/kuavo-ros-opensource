import time
from typing import Any, Tuple, List
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.utils import normalize_angle
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
import math

class EventNavigationToPose(BaseEvent):
    """
    机器人导航到指定位姿的事件类。

    负责处理机器人从当前位置移动到目标位姿的完整流程，包括目标设置、
    移动控制、状态检查等功能。
    """
    def __init__(self,
                 robot_sdk: RobotSDK,
                 timeout,
                 yaw_threshold,
                 pos_threshold,
                 control_mode,
                 ):
        """
        初始化走到指定位置事件。

        参数：
            robot_sdk (RobotSDK): 机器人SDK实例，用于控制机器人运动和获取状态
            timeout (float): 事件超时时间，单位秒
            yaw_threshold (float): 偏航角允许的误差阈值，单位弧度
            pos_threshold (float): 位置允许的误差阈值，单位米
            control_mode (str): 控制模式（如"cmd_vel"速度控制、"cmd_pos_world"世界坐标控制）
        """
        super().__init__(
            event_name="EventNavigationToPose",
        )
        self.robot_sdk = robot_sdk  # 使用封装的RobotSDK类

        ## members
        self.target: Pose = None  # 目标位置
        self.robot_pose_when_target_set: Pose = None  # 记录设置目标时机器人的位姿
        self.target_executed = False  # 标记目标位置未执行

        ## params
        self.timeout = timeout  # 事件超时时间
        self.yaw_threshold = yaw_threshold  # 偏航角阈值，单位弧度
        self.pos_threshold = pos_threshold  # 位置阈值，单位米
        self.control_mode = control_mode  # 控制模式，默认为相对位置控制

    def reset(self):
        """
        重置事件状态。
        """
        self.target = None  # 目标位置
        self.robot_pose_when_target_set = None  # 记录设置目标时机器人的位姿
        self.target_executed = False  # 标记目标位置未执行

    def close(self):
        """
        关闭事件。
        """
        self.logger.info(f"🔵 navigation---------------------------------1------事件关闭啦")
        super().close()
        self.reset()
        # 控制原地站立
        time.sleep(0.1)
        if self.control_mode == "cmd_vel":
            self.robot_sdk.control.walk(
                linear_x=0.0,  # 不前进
                linear_y=0.0,  # 不侧移
                angular_z=0.0  # 不转动
            )
        self.logger.info(f"🔵 navigation---------------------------------1------事件关闭啦")
        # self.robot_sdk.control.stance()

    def utils_enable_base_pitch_limit(self, enable: bool):
        """
        启用或禁用base_link的俯仰角限制。

        参数：
            enable (bool): 是否启用俯仰角限制。
        """
        self.robot_sdk.control.enable_base_pitch_limit(enable)
        self.logger.info(f"🔵 base_link俯仰角限制已{'启用' if enable else '禁用'}")

    def step(self):
        """
        执行事件的每一步操作。
        """
        cmd_vel = self.robot_sdk.navigation.get_cmd_vel()
        self.robot_sdk.control.walk(
                linear_x=cmd_vel.linear.x,
                linear_y=cmd_vel.linear.y,
                angular_z=cmd_vel.angular.z
            )
        time.sleep(0.1)  # 控制频率

        return self.get_status()

    def move_pose_yaw(self, target_x: float, target_y: float, target_z: float, target_yaw: float):
        """
        执行事件的每一步操作。
        """
        self.robot_sdk.control.control_command_pose_world(target_x, target_y, target_z, target_yaw)
        time.sleep(2)  # 控制频率

        return self.get_status()

    def set_target_pose(self, target_x: float, target_y: float, target_yaw: float):
        """
        通过x、y坐标和yaw角设置导航目标

        参数：
            target_x (float): 目标位置x坐标（世界坐标系）
            target_y (float): 目标位置y坐标（世界坐标系）
            target_yaw (float): 目标朝向yaw角（弧度，世界坐标系）

        返回：
            bool: 目标设置成功返回True，否则返回False
        """
        res = self.robot_sdk.navigation.set_target_pose(target_x, target_y, target_yaw)
        return res

    def set_target_pose_april(self, target_x: float, target_y: float, target_yaw: float):
        """
        通过x、y坐标和yaw角设置导航目标

        参数：
            target_x (float): 目标位置x坐标（世界坐标系）
            target_y (float): 目标位置y坐标（世界坐标系）
            target_yaw (float): 目标朝向yaw角（弧度，世界坐标系）

        返回：
            bool: 目标设置成功返回True，否则返回False
        """
        target_yaw = math.degrees(target_yaw)
        res = self.robot_sdk.navigation.set_target_pose(target_x, target_y, target_yaw)
        return res

    def set_target_pose_tag(self, pose: Pose):
        """
        通过Pose对象设置导航目标（适用于从标签坐标系转换的位姿）

        参数：
            pose (Pose): 目标位姿对象（包含位置和姿态信息）

        返回：
            bool: 目标设置成功返回True，否则返回False
        """
        # 从Pose对象提取位置信息
        target_x = pose.position.x
        target_y = pose.position.y

        # 从Pose对象中提取姿态四元数并转换为yaw角
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        _, _, target_yaw = euler_from_quaternion(quat)  # 转换为弧度
        target_yaw = math.degrees(target_yaw)

        # 调用机器人SDK设置目标位姿
        res = self.robot_sdk.navigation.set_target_pose(target_x, target_y, target_yaw)

        return res

    def _check_target_valid(self, target: Pose):
        """
        检查目标位姿是否有效（可扩展为检查是否在安全区域等）

        参数：
            target (Pose): 待检查的目标位姿

        返回：
            bool: 目标有效返回True，否则返回False
        """
        # 目前简单返回True，可根据实际需求添加有效性检查逻辑
        return True

    def _check_failed(self):
        """
        检查事件是否失败（如超时、碰撞等）

        返回：
            bool: 事件失败返回True，否则返回False
        """
        # 目前未实现具体失败检查逻辑，可根据需求扩展
        return False

    def _check_success(self):
        """
        检查导航目标是否已成功到达

        返回：
            bool: 目标到达返回True，否则返回False
        """
        # 通过SDK检查目标是否到达
        if self.robot_sdk.navigation.check_target_reached():
            return True

        return False

    def _check_position_2d(self, target_x, target_y) -> bool:
        """
        检查机器人当前位置是否在目标位置的阈值范围内。

        参数：
            target_x (float): 目标位置的x坐标。
            target_y (float): 目标位置的y坐标。

        返回：
            bool: 如果在阈值范围内返回True，否则返回False。
        """
        # 待补充
        return True


class SingleStepNav:
    """
    单步导航类
    """
    def __init__(self, robot_sdk: RobotSDK):
        """
        初始化单步导航

        参数：
            robot_sdk (RobotSDK): 机器人SDK实例
        """
        self.robot_sdk = robot_sdk

        # 参数设置
        self.torso_spd_threshold = 0.02  # m/s
        self.step_dt = 0.4  # 迈一步的时间间隔
        self.pub_wait_time = 0.1  # 发布foot pose target trajectories后等待的时间
        self.foot_bias = 0.1  # 双脚间距/2
        self.max_delta_pose = np.array([0.15, 0.05, 0.1, 60])  # [x, y, z, yaw_deg]

        # 步态类型
        self.FOUR_DOF = 0
        self.ONE_DOF = 1

        # 状态变量
        self.is_custom_gait_published = False
        self.is_custom_gait = False

        # 服务客户端
        self.get_current_gait_client = None
        # self._init_service_clients()

        print("SingleStepNav initialized")

    # def _init_service_clients(self):
    #     """
    #     初始化服务客户端
    #     """
    #     try:
    #         import rospy
    #         from std_srvs.srv import SetBool

    #         self.get_current_gait_client = rospy.ServiceProxy("humanoid_get_current_gait", SetBool)
    #         print("Service clients initialized successfully")

    #     except Exception as e:
    #         print(f"Failed to initialize service clients: {e}")
    #         self.get_current_gait_client = None