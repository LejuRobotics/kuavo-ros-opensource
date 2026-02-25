import time
import rospy
from typing import Any, Tuple, List
import numpy as np
from std_msgs.msg import Bool

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.utils import normalize_angle
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcFrame)


class EventWalkToPose(BaseEvent):
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
            timeout (float): 事件超时时间，单位秒。
            yaw_threshold (float): 偏航角阈值，单位弧度。
            pos_threshold (float): 位置阈值，单位米。
            control_mode (str): 控制模式。
        """
        super().__init__(
            event_name="EventWalkToPose",
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
        self.x_direction_locked = False  # 是否已经锁定方向
        self.locked_x_direction = 0      # 锁定的方向：1为正向，-1为负向，0为未锁定
        self.min_x_velocity = 0.01       # 最小x方向速度

        self.filtered_vel_x = 0.0
        self.filtered_vel_y = 0.0
        self.filtered_vel_yaw = 0.0

        # cmd_pos_world 模式相关（延迟初始化，只在使用该模式时才创建订阅器）
        self.cmd_pos_world_reached = False  # 是否到达目标标志
        self._cmd_pose_world_flag_sub = None  # 订阅器（延迟创建）

    def _cmd_pose_world_flag_callback(self, msg):
        """cmd_pose_world_flag 话题的回调函数"""
        self.cmd_pos_world_reached = msg.data
        if msg.data:
            print("✅ 收到到达信号: cmd_pose_world_flag = True")
        else:
            print("🚀 收到开始执行信号: cmd_pose_world_flag = False")

    def _setup_cmd_pos_world_subscriber(self):
        """设置 cmd_pose_world_flag 话题的订阅器"""
        self._cmd_pose_world_flag_sub = rospy.Subscriber(
            "/cmd_pose_world_flag",
            Bool,
            self._cmd_pose_world_flag_callback,  # 使用类方法
            queue_size=1
        )

    def open(self):
        """
        开始走到指定位置事件。
        """
        super().open()
        # 重置状态，确保每次open都能正确执行轨迹
        self.reset()

        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)

    def reset(self):
        """
        重置事件状态。
        """
        self.target = None  # 目标位置
        self.robot_pose_when_target_set = None  # 记录设置目标时机器人的位姿
        self.target_executed = False  # 标记目标位置未执行

        self.control_mode = 'cmd_pos_world'

        # 重置 cmd_pos_world 模式相关标志
        self.cmd_pos_world_reached = False

        # 重置x轴速度死区状态
        if hasattr(self, 'last_vel_x_sign'):
            self.last_vel_x_sign = 0

    def close(self):
        """
        关闭事件。
        """
        super().close()
        self.reset()
        # 控制原地站立
        # time.sleep(3)
        if self.control_mode == "cmd_vel":
            self.robot_sdk.control.walk(
                linear_x=0.0,  # 不前进
                linear_y=0.0,  # 不侧移
                angular_z=0.0  # 只转动
            )
        # self.robot_sdk.control.stance()
        # self.robot_sdk.control.walk(linear_x=0, linear_y=0, angular_z=0)
        self.filtered_vel_x = 0.0
        self.filtered_vel_y = 0.0
        self.filtered_vel_yaw = 0.0

    def set_control_mode(self, control_mode: str):
        """
        设置控制模式。

        参数：
            control_mode (str): 控制模式，支持 'cmd_pos_world', 'cmd_pos', 'cmd_vel'。
        """
        valid_modes = ['cmd_pos_world', 'cmd_pos', 'cmd_vel']
        if control_mode not in valid_modes:
            raise ValueError(f"无效的控制模式: {control_mode}，支持的模式有: {valid_modes}")
        self.control_mode = control_mode
        self.logger.info(f"�� 控制模式已设置为: {self.control_mode}")

    def utils_enable_base_pitch_limit(self, enable: bool):
        """
        启用或禁用base_link的俯仰角限制。

        参数：
            enable (bool): 是否启用俯仰角限制。
        """
        # self.robot_sdk.control.enable_base_pitch_limit(enable)
        # self.logger.info(f"�� base_link俯仰角限制已{'启用' if enable else '禁用'}")

    def stop(self):
        """
        停止事件。
        """
        self.robot_sdk.control.walk(
            linear_x=0.0,  # 不前进
            linear_y=0.0,  # 不侧移
            angular_z=0.0  # 只转动
        )

    def change_control_mode(self, control_mode: str):
        """
        切换控制模式。
        """

        # if control_mode == "BaseArm":
        #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        # elif control_mode == "BaseOnly":
        #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseOnly)
        # else:
        #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)

        print(f"切换控制模式为: {control_mode}")

    def _cmd_vel_compute_linear_velocity(self, dx: float, dy: float) -> Tuple[float, float]:
        """
        计算纯线性速度（用于cmd_vel模式）
        只计算线性速度，不管角度

        参数：
            dx: x方向位置误差（米），在机器人坐标系下
            dy: y方向位置误差（米），在机器人坐标系下

        返回：
            Tuple[float, float]: (linear_x, linear_y) 线性速度命令
        """
        # 控制参数
        kp_pos_x = 0.5  # x方向位置比例系数
        kp_pos_y = 1.0  # y方向位置比例系数
        max_vel_x = 0.3  # 最大x方向速度
        max_vel_y = 0.3  # 最大y方向速度
        filter_alpha = 0.15  # 低通滤波系数
        deadzone = 0.01  # 死区：距离小于此值时不输出速度（米）

        # 初始化滤波器状态
        if not hasattr(self, 'filtered_linear_x'):
            self.filtered_linear_x = 0.0
            self.filtered_linear_y = 0.0

        # 分别判断 x 方向死区
        if abs(dx) < deadzone:
            vel_x = 0.0
            self.filtered_linear_x = 0.0  # 立即归零
        else:
            vel_x = kp_pos_x * dx
            vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)
            self.filtered_linear_x = filter_alpha * vel_x + (1 - filter_alpha) * self.filtered_linear_x

        # 分别判断 y 方向死区
        if abs(dy) < deadzone:
            vel_y = 0.0
            self.filtered_linear_y = 0.0  # 立即归零
        else:
            vel_y = kp_pos_y * dy
            vel_y = np.clip(vel_y, -max_vel_y, max_vel_y)
            self.filtered_linear_y = filter_alpha * vel_y + (1 - filter_alpha) * self.filtered_linear_y

        return self.filtered_linear_x, self.filtered_linear_y

    def _cmd_vel_compute_angular_velocity(self, dyaw: float) -> float:
        """
        计算纯角速度（用于cmd_vel模式）
        只计算角速度，不管位置

        参数：
            dyaw: 角度误差（弧度），已经过 normalize_angle 处理

        返回：
            float: angular_z 角速度命令
        """
        # 控制参数
        kp_yaw = 1.0  # 角度比例系数
        max_vel_yaw = 0.399  # 最大转动速度（rad/s）
        filter_alpha = 0.2  # 低通滤波系数（角度可以快一点）
        deadzone = np.deg2rad(0.5)  # 死区：角度小于此值时不输出速度（约0.5°）

        # 如果角度误差很小，直接返回0并重置滤波状态
        if abs(dyaw) < deadzone:
            self.filtered_angular_z = 0.0
            return 0.0

        # 计算期望角速度
        vel_yaw = kp_yaw * dyaw
        vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)

        # 低通滤波（避免抖动）
        if not hasattr(self, 'filtered_angular_z'):
            self.filtered_angular_z = 0.0

        self.filtered_angular_z = filter_alpha * vel_yaw + (1 - filter_alpha) * self.filtered_angular_z

        return self.filtered_angular_z

    def step(self, walk_time: float = 3.0):
        """
        执行事件的每一步操作。
        """
        if self.control_mode == "cmd_pos_world":
            # 使用世界坐标控制模式（发送一次命令，等待到达信号）

            # 延迟初始化：第一次使用时才创建订阅器
            if self._cmd_pose_world_flag_sub is None:
                self._setup_cmd_pos_world_subscriber()

            # 第一次调用时发送命令
            if not self.target_executed:
                # # 发送命令前重置标志位，避免被 latched 的旧值影响
                # self.cmd_pos_world_reached = False

                # 发送位置命令（只发送一次）
                target_x = self.target.pos[0]
                target_y = self.target.pos[1]
                target_z = 0.0
                target_yaw = self.target.get_euler(degrees=False)[2]

                self.robot_sdk.control.control_command_pose_world(target_x, target_y, target_z, target_yaw)
                print(f"📤 cmd_pos_world 发送一次: 目标=[{target_x:.3f}, {target_y:.3f}, {self.target.get_euler(degrees=True)[2]:.1f}°]")
                self.target_executed = True

            # 使用基类的 get_status() 来判断超时和成功
            return self.get_status()

        elif self.control_mode == "cmd_pos":
            # 使用相对位置控制模
            if not self.target_executed:
            # if True:
                self.robot_sdk.control.control_command_pose(self.target.pos[0], self.target.pos[1], self.target.pos[2],
                                                            self.target.get_euler()[2])
                self.target_executed = True

        elif self.control_mode == "cmd_vel":
            # 简化的cmd_vel控制：根据set_target时确定的模式执行控制

            # 1. 将目标位姿转换到odom坐标系
            if Frame.BASE == self.target.frame:
                transform_init_to_world = Transform3D(
                    trans_pose=self.robot_pose_when_target_set,
                    source_frame=Frame.BASE,
                    target_frame=Frame.ODOM
                )
                target_in_odom = transform_init_to_world.apply_to_pose(self.target)
            else:
                target_in_odom = self.target

            # 2. 获取当前机器人位姿（2D简化）
            robot_pose = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            robot_euler = robot_pose.get_euler(degrees=False)
            robot_pose_2d = Pose.from_euler(
                pos=robot_pose.pos,
                euler=[0, 0, robot_euler[2]],
                frame=Frame.ODOM,
                degrees=False
            )

            # 3. 目标位姿2D简化
            target_euler = target_in_odom.get_euler(degrees=False)
            target_in_odom_2d = Pose.from_euler(
                pos=target_in_odom.pos,
                euler=[0, 0, target_euler[2]],
                frame=Frame.ODOM,
                degrees=False
            )

            # 4. 计算目标在机器人基坐标系下的位姿
            transform_base_to_odom = Transform3D(
                trans_pose=robot_pose_2d,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_base = transform_base_to_odom.apply_to_pose_inverse(target_in_odom_2d)

            # 5. 提取误差值
            dx = target_in_base.pos[0]  # x方向位置误差
            dy = target_in_base.pos[1]  # y方向位置误差
            dyaw = normalize_angle(target_in_base.get_euler(degrees=False)[2])  # 角度误差

            # 降低打印频率（每2秒打印一次）
            if not hasattr(self, '_last_debug_print_time'):
                self._last_debug_print_time = 0
            current_time = time.time()
            if current_time - self._last_debug_print_time > 0.3:
                print(f"dx: {dx:.4f}, dy: {dy:.4f}, dyaw: {np.rad2deg(dyaw):.2f}°")
                self._last_debug_print_time = current_time

            # 6. 根据预先确定的模式执行控制（带权重分配）
            if not hasattr(self, '_cmd_vel_mode'):
                self._cmd_vel_mode = "mixed"  # 默认混合模式（向后兼容）

            # 定义模式权重
            MAIN_WEIGHT = 0.8      # 主要控制方向的权重
            ASSIST_WEIGHT = 0.2    # 辅助方向的权重（保持一定的耦合）

            linear_x = 0.0
            linear_y = 0.0
            angular_z = 0.0

            if self._cmd_vel_mode == "angular_only":
                # 纯旋转模式：主要输出角速度，辅助线性速度
                angular_z = MAIN_WEIGHT * self._cmd_vel_compute_angular_velocity(dyaw)
                lin_x, lin_y = self._cmd_vel_compute_linear_velocity(dx, dy)
                linear_x = ASSIST_WEIGHT * lin_x
                linear_y = ASSIST_WEIGHT * lin_y
            elif self._cmd_vel_mode == "linear_only":
                # 纯线性模式：主要输出线性速度，辅助角速度
                linear_x, linear_y = self._cmd_vel_compute_linear_velocity(dx, dy)
                # angular_z = ASSIST_WEIGHT * self._cmd_vel_compute_angular_velocity(dyaw)
            elif self._cmd_vel_mode == "mixed":
                # 混合模式：同时输出线性和角速度（全权重）
                linear_x, linear_y = self._cmd_vel_compute_linear_velocity(dx, dy)
                angular_z = self._cmd_vel_compute_angular_velocity(dyaw)
            # 如果是 "none" 模式，所有速度保持为0

            # 7. 获取当前机器人yaw角，将机器人坐标系速度转换到世界坐标系
            robot_pose_data = self.robot_sdk.tools.get_link_pose(link_name="base_link", reference_frame=Frame.ODOM)
            if robot_pose_data is not None:
                robot_pose = Pose(
                    pos=robot_pose_data.position,
                    quat=robot_pose_data.orientation,
                    frame=Frame.ODOM,
                )
                robot_yaw = robot_pose.get_euler(degrees=False)[2]

                # 坐标系转换：机器人系 -> 世界系
                cos_yaw = np.cos(robot_yaw)
                sin_yaw = np.sin(robot_yaw)
                linear_x_world = linear_x * cos_yaw - linear_y * sin_yaw
                linear_y_world = linear_x * sin_yaw + linear_y * cos_yaw

                # 调试：打印转换信息（降低频率）
                if not hasattr(self, '_last_transform_print_time'):
                    self._last_transform_print_time = 0
                current_time = time.time()
                if current_time - self._last_transform_print_time > 1.0:
                    self.logger.debug(f"[CMD_VEL坐标转换] yaw={np.rad2deg(robot_yaw):.1f}° | "
                                    f"机器人系: vx={linear_x:.3f}, vy={linear_y:.3f} -> "
                                    f"世界系: vx={linear_x_world:.3f}, vy={linear_y_world:.3f}")
                    self._last_transform_print_time = current_time
            else:
                # 获取位姿失败，使用原始速度（可能不准确）
                self.logger.warning("[CMD_VEL] 无法获取机器人位姿，使用未转换的速度")
                linear_x_world = linear_x
                linear_y_world = linear_y

            # 8. 发送世界坐标系的速度命令
            self.robot_sdk.control.walk(
                linear_x=linear_x_world,
                linear_y=linear_y_world,
                angular_z=angular_z
            )

        return self.get_status()

    def plan_velocity_sequence_to_target(self,
                                         dt: float = 0.05,
                                         v_max: float = 0.8,
                                         accel_time: float = 1.0,
                                         decel_time: float = 1.0) -> List[tuple]:
        """
        生成到当前 target 的直线路径速度序列（机体系cmd_vel），采用梯形速度曲线：线性加速-匀速-线性减速。
        返回值为 [(vx, vy, wz), ...]，其中wz恒为0。

        参数：
            dt (float): 采样周期，单位秒
            v_max (float): 最高线速度，单位m/s
            accel_time (float): 加速时间（从0到v_max），单位秒
            decel_time (float): 减速时间（从v_max到0），单位秒

        返回：
            List[tuple]: 速度三元组序列
        """
        if self.target is None:
            return []

        # 将目标位姿转换到 ODOM 坐标系
        if Frame.BASE == self.target.frame:
            transform_init_to_world = Transform3D(
                trans_pose=self.robot_pose_when_target_set,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_odom = transform_init_to_world.apply_to_pose(self.target)
        else:
            target_in_odom = self.target

        # 当前真实位姿（仅获取初始yaw用于系转换）
        robot_pose_now = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )
        yaw = robot_pose_now.get_euler(degrees=False)[2]

        # 规划直线段（ODOM）
        start_xy = np.array(robot_pose_now.pos[:2], dtype=float)
        goal_xy = np.array(target_in_odom.pos[:2], dtype=float)
        delta = goal_xy - start_xy
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return []

        direction_world = delta / distance  # 世界坐标下单位方向

        # 梯形速度曲线参数
        accel_time = max(1e-6, float(accel_time))
        decel_time = max(1e-6, float(decel_time))
        v_max = max(0.0, float(v_max))

        # 若距离不足以跑到v_max，退化为三角形曲线，计算可达峰值速度
        v_peak_limit = 2.0 * distance / (accel_time + decel_time)
        v_peak = min(v_max, v_peak_limit)

        # 计算巡航时长（可能为0）
        t_acc = accel_time
        t_dec = decel_time
        d_acc = 0.5 * v_peak * t_acc
        d_dec = 0.5 * v_peak * t_dec
        d_cruise = max(0.0, distance - d_acc - d_dec)
        t_cruise = d_cruise / v_peak if v_peak > 0 else 0.0
        t_total = t_acc + t_cruise + t_dec

        # 采样生成速度序列（机体系）
        c, s = np.cos(yaw), np.sin(yaw)
        R_world_to_body = np.array([[ c,  s],
                                    [-s,  c]])

        seq: List[tuple] = []
        t = 0.0
        while t < t_total + 1e-9:
            if t < t_acc:
                speed = v_peak * (t / t_acc)
            elif t < t_acc + t_cruise:
                speed = v_peak
            else:
                t_into_dec = t - (t_acc + t_cruise)
                speed = v_peak * max(0.0, 1.0 - t_into_dec / t_dec)

            v_world = direction_world * speed
            v_body = R_world_to_body @ v_world
            vx, vy = float(v_body[0]), float(v_body[1])

            seq.append((vx, vy, 0.0))
            t += dt

        return seq

    def plan_velocity_sequence_to_target_power4(self,
                                                dt: float = 0.01,
                                                v_max: float = 0.8,
                                                accel_time: float = 1.0,
                                                decel_time: float = 1.0) -> List[tuple]:
        """
        使用 power4_ultra_smooth 的曲线生成到当前 target 的直线路径速度序列（机体系cmd_vel）。
        返回值为 [(vx, vy, wz), ...]，其中wz恒为0。输入输出与 plan_velocity_sequence_to_target 一致。
        """
        if self.target is None:
            return []

        # 目标位姿 -> ODOM
        if Frame.BASE == self.target.frame:
            transform_init_to_world = Transform3D(
                trans_pose=self.robot_pose_when_target_set,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_odom = transform_init_to_world.apply_to_pose(self.target)
        else:
            target_in_odom = self.target

        # 当前姿态与直线方向
        robot_pose_now = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )
        yaw = robot_pose_now.get_euler(degrees=False)[2]
        start_xy = np.array(robot_pose_now.pos[:2], dtype=float)
        goal_xy = np.array(target_in_odom.pos[:2], dtype=float)
        delta = goal_xy - start_xy
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return []

        direction_world = delta / distance

        # 峰值速度限制（与原方法一致）
        accel_time = max(1e-6, float(accel_time))
        decel_time = max(1e-6, float(decel_time))
        v_max = max(0.0, float(v_max))
        v_peak_limit = 2.0 * distance / (accel_time + decel_time)
        v_peak = min(v_max, v_peak_limit)

        # 计算巡航与总时长
        d_acc = 0.5 * v_peak * accel_time
        d_dec = 0.5 * v_peak * decel_time
        d_cruise = max(0.0, distance - d_acc - d_dec)
        t_cruise = d_cruise / v_peak if v_peak > 0 else 0.0
        t_total = accel_time + t_cruise + decel_time

        # world->body 旋转
        c, s = np.cos(yaw), np.sin(yaw)
        R_world_to_body = np.array([[ c,  s],
                                    [-s,  c]])

        # 生成曲线速度序列（沿标量速度，再投影到body系x/y）
        seq: List[tuple] = []
        t = 0.0
        while t < t_total + 1e-9:
            if t < accel_time:
                progress = t / accel_time
                # 凸加速：1 - (1-progress)^2
                speed_ratio = 1.0 - (1.0 - progress)**2
                speed = v_peak * speed_ratio
            elif t < accel_time + t_cruise:
                speed = v_peak
            else:
                t_into_dec = t - (accel_time + t_cruise)
                progress = t_into_dec / decel_time
                # 使用 power4_ultra_smooth.py 中当前实现的 decel 曲线：
                # 三段式（0.6, 0.8）并在末段凹向x轴（1 - (1-p)^alpha）
                if progress < 0.6:
                    curve_val = progress**4
                elif progress < 0.8:
                    blend_p = (progress - 0.6) / 0.2
                    concave_val = progress**4
                    rem = (progress - 0.6) / 0.4
                    convex_val = 1.0 - (1.0 - rem)**3
                    at60 = 0.6**4
                    target_val = at60 + (1.0 - at60) * convex_val
                    smooth_blend = 6 * blend_p**5 - 15 * blend_p**4 + 10 * blend_p**3
                    curve_val = concave_val * (1 - smooth_blend) + target_val * smooth_blend
                else:
                    rem = (progress - 0.8) / 0.2
                    alpha = 1.8
                    convex_decel = 1.0 - (1.0 - rem)**alpha
                    # 与80%处的目标值对齐
                    blend_p80 = 1.0
                    smooth_blend_80 = 6 * blend_p80**5 - 15 * blend_p80**4 + 10 * blend_p80**3
                    concave_80 = 0.8**4
                    rem80 = 0.5
                    convex_val_80 = 1.0 - (1.0 - rem80)**3
                    at60 = 0.6**4
                    target_80 = concave_80 * (1 - smooth_blend_80) + (at60 + (1.0 - at60) * convex_val_80) * smooth_blend_80
                    curve_val = target_80 + (1.0 - target_80) * convex_decel

                speed_ratio = max(0.0, 1.0 - curve_val)
                speed = v_peak * speed_ratio

            v_world = direction_world * speed
            v_body = R_world_to_body @ v_world
            vx, vy = float(v_body[0]), float(v_body[1])
            seq.append((vx, vy, 0.0))
            t += dt

        return seq

    def plan_yaw_velocity_sequence_by_delta(self,
                                            yaw_delta: float,
                                            dt: float = 0.02,
                                            w_max: float = 1.0,
                                            accel_time: float = 0.5,
                                            decel_time: float = 0.5) -> List[tuple]:
        """
        仅根据给定的旋转角增量 yaw_delta（弧度，正为逆时针）生成角速度序列（梯形/三角曲线）。

        返回：[(vx, vy, wz), ...]，其中 vx=vy=0，仅下发 wz。
        """
        total_angle = float(yaw_delta)
        if abs(total_angle) < 1e-6:
            return []

        sign = 1.0 if total_angle >= 0.0 else -1.0
        distance = abs(total_angle)

        accel_time = max(1e-6, float(accel_time))
        decel_time = max(1e-6, float(decel_time))
        w_max = max(0.0, float(w_max))

        # 若角度不足以跑到 w_max，退化为三角速度曲线，计算可达峰值角速度
        w_peak_limit = 2.0 * distance / (accel_time + decel_time)
        w_peak = min(w_max, w_peak_limit)

        t_acc = accel_time
        t_dec = decel_time
        d_acc = 0.5 * w_peak * t_acc
        d_dec = 0.5 * w_peak * t_dec
        d_cruise = max(0.0, distance - d_acc - d_dec)
        t_cruise = d_cruise / w_peak if w_peak > 0 else 0.0
        t_total = t_acc + t_cruise + t_dec

        seq: List[tuple] = []
        t = 0.0
        while t < t_total + 1e-9:
            if t < t_acc:
                w = w_peak * (t / t_acc)
            elif t < t_acc + t_cruise:
                w = w_peak
            else:
                t_into_dec = t - (t_acc + t_cruise)
                w = w_peak * max(0.0, 1.0 - t_into_dec / t_dec)

            seq.append((0.0, 0.0, float(sign * w)))
            t += dt

        return seq

    def plan_yaw_velocity_sequence_to_yaw(self,
                                          target_yaw: float,
                                          dt: float = 0.02,
                                          w_max: float = 1.0,
                                          accel_time: float = 0.5,
                                          decel_time: float = 0.5) -> List[tuple]:
        """
        根据目标绝对 yaw（ODOM系，弧度）生成角速度序列。内部读取当前 yaw 并计算最短角差。
        返回：[(0,0,wz), ...]
        """
        robot_pose_now = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )
        cur_yaw = robot_pose_now.get_euler(degrees=False)[2]
        yaw_delta = normalize_angle(float(target_yaw) - float(cur_yaw))
        return self.plan_yaw_velocity_sequence_by_delta(
            yaw_delta=yaw_delta,
            dt=dt,
            w_max=w_max,
            accel_time=accel_time,
            decel_time=decel_time,
        )

    def plan_combined_velocity_sequence(self,
                                       yaw_delta: float,
                                       distance: float,
                                       dt: float = 0.02,
                                       v_max: float = 0.8,
                                       w_max: float = 1.0,
                                       v_accel_time: float = 1.0,
                                       v_decel_time: float = 1.0,
                                       w_accel_time: float = 0.5,
                                       w_decel_time: float = 0.5) -> List[tuple]:
        """
        同时控制yaw角和线速度，生成组合速度序列。

        参数：
            yaw_delta (float): 旋转角增量（弧度，正为逆时针）
            distance (float): 直线距离（米，正为前进）
            dt (float): 采样周期，单位秒
            v_max (float): 最高线速度，单位m/s
            w_max (float): 最高角速度，单位rad/s
            v_accel_time (float): 线速度加速时间，单位秒
            v_decel_time (float): 线速度减速时间，单位秒
            w_accel_time (float): 角速度加速时间，单位秒
            w_decel_time (float): 角速度减速时间，单位秒

        返回：
            List[tuple]: [(vx, vy, wz), ...] 速度序列
        """
        # 生成线速度序列（仅x方向）
        v_seq = self._plan_linear_velocity_sequence(
            distance=distance,
            dt=dt,
            v_max=v_max,
            accel_time=v_accel_time,
            decel_time=v_decel_time
        )

        # 生成角速度序列
        w_seq = self.plan_yaw_velocity_sequence_by_delta(
            yaw_delta=yaw_delta,
            dt=dt,
            w_max=w_max,
            accel_time=w_accel_time,
            decel_time=w_decel_time
        )

        # 合并序列，取较长序列的长度
        max_len = max(len(v_seq), len(w_seq))
        combined_seq = []

        for i in range(max_len):
            vx = v_seq[i][0] if i < len(v_seq) else 0.0
            vy = 0.0  # 只沿x方向移动
            wz = w_seq[i][2] if i < len(w_seq) else 0.0
            combined_seq.append((vx, vy, wz))

        return combined_seq

    def _plan_linear_velocity_sequence(self,
                                      distance: float,
                                      dt: float = 0.02,
                                      v_max: float = 0.8,
                                      accel_time: float = 1.0,
                                      decel_time: float = 1.0) -> List[tuple]:
        """
        生成纯线速度序列（仅x方向）。

        参数：
            distance (float): 直线距离（米，正为前进）
            dt (float): 采样周期，单位秒
            v_max (float): 最高线速度，单位m/s
            accel_time (float): 加速时间，单位秒
            decel_time (float): 减速时间，单位秒

        返回：
            List[tuple]: [(vx, 0, 0), ...] 线速度序列
        """
        total_distance = abs(distance)
        if total_distance < 1e-6:
            return []

        sign = 1.0 if distance >= 0.0 else -1.0

        accel_time = max(1e-6, float(accel_time))
        decel_time = max(1e-6, float(decel_time))
        v_max = max(0.0, float(v_max))

        # 若距离不足以跑到v_max，退化为三角形曲线
        v_peak_limit = 2.0 * total_distance / (accel_time + decel_time)
        v_peak = min(v_max, v_peak_limit)

        t_acc = accel_time
        t_dec = decel_time
        d_acc = 0.5 * v_peak * t_acc
        d_dec = 0.5 * v_peak * t_dec
        d_cruise = max(0.0, total_distance - d_acc - d_dec)
        t_cruise = d_cruise / v_peak if v_peak > 0 else 0.0
        t_total = t_acc + t_cruise + t_dec

        seq: List[tuple] = []
        t = 0.0
        while t < t_total + 1e-9:
            if t < t_acc:
                speed = v_peak * (t / t_acc)
            elif t < t_acc + t_cruise:
                speed = v_peak
            else:
                t_into_dec = t - (t_acc + t_cruise)
                speed = v_peak * max(0.0, 1.0 - t_into_dec / t_dec)

            seq.append((float(sign * speed), 0.0, 0.0))
            t += dt

        return seq

    def execute_velocity_sequence(self, velocity_sequence: List[tuple], dt: float = 0.05):
        """
        顺序下发速度序列到底盘。

        参数：
            velocity_sequence (List[tuple]): [(vx, vy, wz), ...]
            dt (float): 下发周期
        """
        if not velocity_sequence:
            return
        # x轴死区限制，避免小抖动。与cmd_vel控制保持一致的默认阈值
        if not hasattr(self, 'x_velocity_deadzone'):
            self.x_velocity_deadzone = 0.02
        # 正负方向限制：尽量与cmd_vel运行时保持一致
        # 优先使用已有的方向锁定；若不存在，则根据序列中首个非零vx推断
        x_dir = None
        if hasattr(self, 'x_direction_locked') and getattr(self, 'x_direction_locked') and hasattr(self, 'locked_x_direction'):
            x_dir = 1 if getattr(self, 'locked_x_direction') > 0 else -1
        else:
            for _vx, _vy, _wz in velocity_sequence:
                if abs(_vx) >= self.x_velocity_deadzone:
                    x_dir = 1 if _vx > 0 else -1
                    break

        for vx, vy, wz in velocity_sequence:
            # 应用x轴死区与正负方向限制
            if x_dir is None:
                # 未确定方向时，单纯应用死区
                if abs(vx) < self.x_velocity_deadzone:
                    vx = 0.01 if vx >= 0 else -0.01
            else:
                # 已锁定方向：禁止反向，且保证最小幅值
                if x_dir > 0:
                    # 只允许正向
                    if vx < 0:
                        vx = 0.01
                    elif abs(vx) < self.x_velocity_deadzone:
                        vx = 0.01
                else:
                    # 只允许负向
                    if vx > 0:
                        vx = -0.01
                    elif abs(vx) < self.x_velocity_deadzone:
                        vx = -0.01
            if abs(vy) <= 0.02:
                vx = 0.0
            self.robot_sdk.control.walk(
                linear_x=vx,
                linear_y=vy,
                angular_z=wz
            )
            time.sleep(dt)

    def plan_and_execute_velocity_to_target(self,
                                            dt: float = 0.05,
                                            max_duration: float = 10.0) -> bool:
        """
        先规划速度序列，再按序执行。

        返回：
            bool: 是否已规划并执行（序列可能为空则返回 False）
        """
        seq = self.plan_velocity_sequence_to_target(dt=0.01, v_max=0.3, accel_time=0.3, decel_time=0.3)
        print("t(s), vx(m/s), vy(m/s), wz(rad/s)")
        for i, (vx, vy, wz) in enumerate(seq):
            print(f"{i*dt:.4f}, {vx:.4f}, {vy:.4f}, {wz:.4f}")
        if not seq:
            return False
        self.execute_velocity_sequence(seq, dt=dt)
        # 结束后同时输出当前与目标位置（odom）
        cur_pos = self.robot_sdk.state.robot_position()
        cur_quat = self.robot_sdk.state.robot_orientation()
        cur_yaw = Pose(pos=cur_pos, quat=cur_quat, frame=Frame.ODOM).get_euler(degrees=False)[2]
        # 目标统一转ODOM
        if Frame.BASE == self.target.frame:
            transform_init_to_world = Transform3D(
                trans_pose=self.robot_pose_when_target_set,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_odom = transform_init_to_world.apply_to_pose(self.target)
        else:
            target_in_odom = self.target
        tyaw = target_in_odom.get_euler(degrees=False)[2]
        print(
            f"[final] current@odom pos=({cur_pos[0]:.3f}, {cur_pos[1]:.3f}, {cur_pos[2]:.3f}), yaw={cur_yaw:.3f} | "
            f"target@odom pos=({target_in_odom.pos[0]:.3f}, {target_in_odom.pos[1]:.3f}, {target_in_odom.pos[2]:.3f}), yaw={tyaw:.3f}"
        )
        return True

    def set_target(self, target: Any, *args, **kwargs):
        """
        设置事件的目标。

        参数：
            target (Any): 目标。
            *args: 额外的参数。
            **kwargs: 额外的关键字参数。

        返回：
            bool: 如果目标设置成功返回True，否则返回False。
        """
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseOnly)
        res = super().set_target(target, *args, **kwargs)
        self.x_direction_locked = False
        self.locked_x_direction = 0
        if res:
            # 为了应对相对位置控制的情况，记录设置目标时机器人的位姿
            self.robot_pose_when_target_set = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            self.target = target  # 设置目标位置
            self.target_executed = False  # 标记目标位置未执行

            # 为cmd_pos_world模式重置到达标志
            self.cmd_pos_world_reached = False

            # 对于cmd_vel模式，在设置目标时就决定控制模式
            if self.control_mode == "cmd_vel":
                self._determine_cmd_vel_control_mode()
        return res

    def _determine_cmd_vel_control_mode(self):
        """
        根据目标确定cmd_vel的控制模式（纯旋转/纯线性/混合）
        只在set_target时调用一次
        """
        # 将目标位姿转换到odom坐标系
        if Frame.BASE == self.target.frame:
            transform_init_to_world = Transform3D(
                trans_pose=self.robot_pose_when_target_set,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_odom = transform_init_to_world.apply_to_pose(self.target)
        else:
            target_in_odom = self.target

        # 获取当前机器人位姿
        robot_pose = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )

        # 计算初始误差
        robot_euler = robot_pose.get_euler(degrees=False)
        robot_pose_2d = Pose.from_euler(
            pos=robot_pose.pos,
            euler=[0, 0, robot_euler[2]],
            frame=Frame.ODOM,
            degrees=False
        )

        target_euler = target_in_odom.get_euler(degrees=False)
        target_in_odom_2d = Pose.from_euler(
            pos=target_in_odom.pos,
            euler=[0, 0, target_euler[2]],
            frame=Frame.ODOM,
            degrees=False
        )

        transform_base_to_odom = Transform3D(
            trans_pose=robot_pose_2d,
            source_frame=Frame.BASE,
            target_frame=Frame.ODOM
        )
        target_in_base = transform_base_to_odom.apply_to_pose_inverse(target_in_odom_2d)

        # 计算初始误差
        dx = target_in_base.pos[0]
        dy = target_in_base.pos[1]
        dyaw = normalize_angle(target_in_base.get_euler(degrees=False)[2])
        distance = np.linalg.norm([dx, dy])

        # 根据初始误差决定控制模式
        has_pos_error = distance > self.pos_threshold
        has_yaw_error = abs(dyaw) > self.yaw_threshold

        if has_pos_error and has_yaw_error:
            self._cmd_vel_mode = "mixed"
            self.logger.info(f"🔄➡️ [CMD_VEL] 混合控制模式: 初始distance={distance:.3f}m, dyaw={np.rad2deg(dyaw):.1f}°")
        elif has_yaw_error:
            self._cmd_vel_mode = "angular_only"
            self.logger.info(f"🔄 [CMD_VEL] 纯旋转模式: 初始dyaw={np.rad2deg(dyaw):.1f}°")
            self.pos_threshold = 0.35
        elif has_pos_error:
            self._cmd_vel_mode = "linear_only"
            self.logger.info(f"➡️ [CMD_VEL] 纯线性模式: 初始distance={distance:.3f}m")
            self.yaw_threshold = np.deg2rad(5)  # 放宽到10°，避免运动过程中的角度漂移导致无法到达
        else:
            self._cmd_vel_mode = "none"
            self.logger.info("✅ [CMD_VEL] 已在目标位置")

    def _check_target_valid(self, target: Pose):
        """
        检查目标位置是否有效。

        参数：
            target (Pose): 目标位置。

        返回：
            bool: 如果目标有效返回True，否则返回False。
        """
        if self.target is not None:
            if self.target.frame != target.frame:
                print(f"事件运行途中不能更换target坐标系（从 {self.target.frame} 到 {target.frame}）")
                return False

        if not isinstance(target, Pose):
            print("目标位置必须是Pose对象")
            return False

        if target.frame not in [Frame.ODOM, Frame.BASE]:
            print("目标位姿的坐标系必须是'base_link'或'odom'")
            return False

        if self.control_mode == 'cmd_pos':
            if target.frame != Frame.BASE:
                print("使用'cmd_pos'相对位置控制模式时，目标位姿的坐标系必须是'base_link'")
                return False

        if self.control_mode == 'cmd_vel':
            if target.frame not in [Frame.ODOM, Frame.BASE]:
                print("使用'cmd_vel'速度控制模式时，目标位姿的坐标系必须是'odom' 或'base_link'")
                return False

        if self.control_mode == 'cmd_pose_world':
            if target.frame != Frame.ODOM:
                print("使用'cmd_pose_world'世界坐标控制模式时，目标位姿的坐标系必须是'odom'")
                return False

        return True

    def _check_failed(self):
        """
        检查事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。
        """
        # 无失败状态
        return False

    def _check_success(self):
        """
        检查事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。
        """
        if self.control_mode == "cmd_pos_world":
            # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseOnly)
            # cmd_pos_world 模式：通过订阅 /cmd_pose_world_flag 话题判断是否到达
            # if self.cmd_pos_world_reached:
            #     self.logger.info(f'✅ cmd_pos_world 成功到达目标位置!')
            #     return True
            # return False

            # 将目标位姿转换到odom坐标系
            if self.target is None:
                return False

            if Frame.BASE == self.target.frame:
                transform_init_to_world = Transform3D(
                    trans_pose=self.robot_pose_when_target_set,
                    source_frame=Frame.BASE,
                    target_frame=Frame.ODOM
                )
                target_in_odom = transform_init_to_world.apply_to_pose(self.target)
            else:
                target_in_odom = self.target

            target_yaw = target_in_odom.get_euler(degrees=False)[2]  # 获取目标偏航角

            # === check_yaw ===
            robot_pose = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation()
            )
            robot_yaw = robot_pose.get_euler(degrees=False)[2]  # 获取机器人的偏航角
            yaw_diff = normalize_angle(target_yaw - robot_yaw)
            yaw_reached = abs(yaw_diff) <= self.yaw_threshold

            robot_pos = self.robot_sdk.state.robot_position()
            pos_diff = np.linalg.norm(np.array(robot_pos[:2]) - np.array(target_in_odom.pos[:2]))
            pos_reached = pos_diff <= self.pos_threshold

            # print(
            # f'目标未到达: {target_in_odom.pos}, 偏航角未到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')

            if yaw_reached and pos_reached:
                print(
                    f'目标位置已到达: {target_in_odom.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
                return True

            # return False

        elif self.control_mode == "cmd_pos":
            # 相对位置控制模式
            transform_init_to_world = Transform3D(
                trans_pose=self.robot_pose_when_target_set,
                source_frame=Frame.BASE,  # 源坐标系为base_link
                target_frame=Frame.ODOM  # 目标坐标系为odom
            )
            target_pose = transform_init_to_world.apply_to_pose(self.target)
            yaw_reached, yaw_diff = self._check_yaw(target_pose.get_euler()[2])
            pos_reached, pos_diff = self._check_position_2d(target_pose.pos[0], target_pose.pos[1])

            if yaw_reached and pos_reached:
                self.logger.info(
                    f'目标位置已到达: {self.target.pos}, 偏航角已到达: {target_pose.get_euler()[2]:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
                return True

        elif self.control_mode == "cmd_vel":
            # 速度控制模式，检查是否在目标位置附近
            if Frame.BASE == self.target.frame:
                transform_init_to_world = Transform3D(
                    trans_pose=self.robot_pose_when_target_set,
                    source_frame=Frame.BASE,  # 源坐标系为base_link
                    target_frame=Frame.ODOM  # 目标坐标系为odom
                )
                target_in_odom = transform_init_to_world.apply_to_pose(self.target)
            else:
                target_in_odom = self.target

            target_yaw = target_in_odom.get_euler(degrees=False)[2]  # 获取目标偏航角
            yaw_reached, yaw_diff = self._check_yaw(target_yaw)

            pos_reached, pos_diff = self._check_position_2d(target_in_odom.pos[0], target_in_odom.pos[1])

            if yaw_reached and pos_reached:
                self.logger.info(f'目标位置已到达: {target_in_odom.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
                return True

        return False

    def _check_yaw(self, target_yaw: float) -> bool:
        """
        检查机器人当前偏航角是否在目标偏航角的阈值范围内。

        参数：
            target_yaw (float): 目标偏航角。

        返回：
            bool: 如果在阈值范围内返回True，否则返回False。
        """
        robot_pose_data = self.robot_sdk.tools.get_link_pose(link_name="base_link_lb", reference_frame=Frame.ODOM)
        if robot_pose_data is None:
            self.logger.warning("无法获取 base_link 在 odom 下的位姿，跳过偏航角检查")
            return False, float('inf')

        robot_pose = Pose(
            pos=robot_pose_data.position,
            quat=robot_pose_data.orientation,
            frame=Frame.ODOM,
        )

        robot_yaw = robot_pose.get_euler(degrees=False)[2]  # 获取机器人的偏航角
        yaw_diff = normalize_angle(target_yaw - robot_yaw)

        # 时间控制打印，每0.2秒打印一次
        current_time = time.time()
        if not hasattr(self, '_last_yaw_print_time'):
            self._last_yaw_print_time = 0.0

        if current_time - self._last_yaw_print_time >= 0.2:
            self.logger.info(f"偏航角差: {yaw_diff:.4f}, 阈值: {self.yaw_threshold:.4f}")
            self._last_yaw_print_time = current_time

        return abs(yaw_diff) <= self.yaw_threshold, yaw_diff

    def _check_position_2d(self, target_x, target_y) -> bool:
        """
        检查机器人当前位置是否在目标位置的阈值范围内。

        参数：
            target_x (float): 目标位置的x坐标。
            target_y (float): 目标位置的y坐标。

        返回：
            bool: 如果在阈值范围内返回True，否则返回False。
        """
        robot_pose = self.robot_sdk.tools.get_link_pose(link_name="base_link_lb", reference_frame=Frame.ODOM)
        if robot_pose is None:
            self.logger.warning("无法获取 base_link 在 odom 下的位置，跳过位置检查")
            return False, float('inf')

        robot_pos = np.array(robot_pose.position)
        pos_diff = np.linalg.norm(robot_pos[:2] - np.array([target_x, target_y]))

        # 时间控制打印，每0.2秒打印一次
        current_time = time.time()
        if not hasattr(self, '_last_pos_print_time'):
            self._last_pos_print_time = 0.0

        if current_time - self._last_pos_print_time >= 0.2:
            self.logger.info(f"位置差: {pos_diff:.4f}, 阈值: {self.pos_threshold:.4f}")
            self._last_pos_print_time = current_time

        return pos_diff <= self.pos_threshold, pos_diff

