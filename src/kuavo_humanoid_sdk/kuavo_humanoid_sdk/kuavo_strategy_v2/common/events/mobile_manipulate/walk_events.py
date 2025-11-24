import time
from typing import Any, Tuple, List
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.utils import normalize_angle
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK


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

    def reset(self):
        """
        重置事件状态。
        """
        self.target = None  # 目标位置
        self.robot_pose_when_target_set = None  # 记录设置目标时机器人的位姿
        self.target_executed = False  # 标记目标位置未执行

        self.control_mode = 'cmd_pos_world'

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
        self.robot_sdk.control.stance()
        # self.robot_sdk.control.walk(linear_x=0, linear_y=0, angular_z=0)

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
        self.logger.info(f"🔵 控制模式已设置为: {self.control_mode}")

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
        if self.control_mode == "cmd_pos_world":
            # 使用世界坐标控制模式
            # if not self.target_executed:
            if True:  # 这个可以连续发
                self.robot_sdk.control.control_command_pose_world(
                    target_pose_x=self.target.pos[0],
                    target_pose_y=self.target.pos[1],
                    # target_pose_z=self.target.pos[2],
                    target_pose_z=0.0,
                    target_pose_yaw=self.target.get_euler(degrees=False)[2]
                )
                time.sleep(0.2)
                self.target_executed = True

        elif self.control_mode == "cmd_pos":
            # 使用相对位置控制模
            if not self.target_executed:
            # if True:
                self.robot_sdk.control.control_command_pose(self.target.pos[0], self.target.pos[1], self.target.pos[2],
                                                            self.target.get_euler()[2])
                self.target_executed = True

        elif self.control_mode == "cmd_vel":
            self.kp_pos = 0.5  # 前进速度比例系数
            self.kp_yaw = 0.5  # 转动速度比例系数
            self.max_vel_x = 0.5  # 最大前进速度
            self.max_vel_yaw = 0.4  # 最大转动速度

            if Frame.BASE == self.target.frame:
                transform_init_to_world = Transform3D(
                    trans_pose=self.robot_pose_when_target_set,
                    source_frame=Frame.BASE,  # 源坐标系为base_link
                    target_frame=Frame.ODOM  # 目标坐标系为odom
                )
                target_in_odom = transform_init_to_world.apply_to_pose(self.target)
            else:
                target_in_odom = self.target

            # raise NotImplementedError("cmd_vel控制模式尚未实现")
            # 如果是cmd_vel，则需要外部每次调用step()，来实现闭环控制
            # 逻辑是，先原地转到目标朝向，然后边走边调整朝向（确保朝向和机器人与目标间连线的朝向align）

            # 1. 获取当前世界系下位姿
            robot_pose = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            # 2. 目标位姿，默认只能是世界系
            assert Frame.ODOM == target_in_odom.frame, "目标位姿必须是世界坐标系（odom）"
            # 算目标朝向
            # angle_diff = robot_pose.angle_yaw(self.target)
            # 目标朝向是机器人与目标位置连线的朝向
            # compute target in frame of base

            euler = robot_pose.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0
            robot_pose_2d = Pose.from_euler(
                pos=robot_pose.pos,  # 只取x, y坐标
                euler= euler,  # 只取x, y朝向
                frame=Frame.ODOM,  # 使用base_link坐标系
                degrees=False
            )

            euler = target_in_odom.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0

            target_in_odom_2d = Pose.from_euler(
                pos=target_in_odom.pos,  # 只取x, y坐标
                euler=euler,  # 只取x, y朝向
                frame=Frame.ODOM,  # 使用base_link坐标系
                degrees=False
            )

            transform_basa_to_odom = Transform3D(
                trans_pose=robot_pose_2d,
                source_frame=Frame.BASE,  # 源坐标系为base_link
                target_frame=Frame.ODOM  # 目标坐标系为odom
            )
            target_in_base = transform_basa_to_odom.apply_to_pose_inverse(target_in_odom_2d)
            print(f"目标在base_link坐标系下的位置：{target_in_base}")
            print(f"base in odom pose: {robot_pose_2d}")
            print(f"target in odom pose: {target_in_odom_2d}")


            angle_diff_line = np.arctan2(
                target_in_base.pos[1],
                target_in_base.pos[0]
            )
            angle_diff_line = normalize_angle(angle_diff_line)  # 归一化角度
            angle_diff_frame = target_in_base.get_euler(degrees=False)[2]  ## 两个坐标系间的角度差
            dis_diff = np.linalg.norm(target_in_base.pos[:2])

            print(f"当前坐标系间角度差：{np.rad2deg(angle_diff_frame):.2f}°，距离差：{dis_diff:.2f}米， 连线角度差：{np.rad2deg(angle_diff_line):.2f}°")
            # 如果朝向大于某个值，先转不走
            max_yaw_to_walk = np.deg2rad(10)  # 超过这个值就不走只转
            max_dis_to_rotate = self.pos_threshold  # 小于这个距离就转到angle_diff_frame

            # 1. if dis too small， then use holonomic fine tune

            if dis_diff < max_dis_to_rotate:
                vel_yaw = self.kp_yaw * angle_diff_frame
                vel_yaw = np.clip(vel_yaw, -self.max_vel_yaw, self.max_vel_yaw)  # 限制转速
                print(f"转向target frame朝向，转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )

            elif dis_diff < (max_dis_to_rotate + 0.1) or (abs(target_in_base.pos[1]) < 0.1 and abs(angle_diff_frame) < np.deg2rad(10)):
                # 如果距离小于阈值，使用holonomic控制
                x_diff = target_in_base.pos[0]
                y_diff = target_in_base.pos[1]
                vel_x = self.kp_pos * x_diff
                vel_x = np.clip(vel_x, -self.max_vel_x, self.max_vel_x)
                vel_y = self.kp_pos * y_diff
                vel_y = np.clip(vel_y, -self.max_vel_x, self.max_vel_x)
                print(f'holonomic控制，前进速度：{vel_x:.2f} m/s, 侧移速度：{vel_y:.2f} m/s')
                self.robot_sdk.control.walk(
                    linear_x=vel_x,  # 前进
                    linear_y=vel_y,  # 侧移
                    angular_z=0.0  # 不转动
                )

            elif abs(angle_diff_line) > max_yaw_to_walk:
                vel_yaw = self.kp_yaw * angle_diff_line
                vel_yaw = np.clip(vel_yaw, -self.max_vel_yaw, self.max_vel_yaw)  # 限制转速
                print(f"dis_diff {dis_diff}; 转向连线方向，转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )
            elif dis_diff >= max_dis_to_rotate:
                # 如果连线朝向小于某个值，开始前进
                # dis_sign = (abs(angle_diff_line) > np.pi)

                vel_x = self.kp_pos * dis_diff
                vel_x = np.clip(vel_x, -self.max_vel_x, self.max_vel_x)  # 限制前进速度

                vel_yaw = self.kp_yaw * angle_diff_line
                vel_yaw = np.clip(vel_yaw, -self.max_vel_yaw, self.max_vel_yaw)  # 限制转速
                print(f"dis_diff {dis_diff}, 前进速度：{vel_x:.2f} m/s, 转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=vel_x,  # 前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 不转动
                )

            time.sleep(0.1)  # 控制频率
            # time.sleep(0.05)


        elif self.control_mode == "cmd_vel_good":

            # FIXME: 改成config设置的时间
            self.kp_pos = 0.5  # 前进速度比例系数
            self.kp_yaw = 0.5  # 转动速度比例系数
            self.max_vel_x = 0.5  # 最大前进速度
            self.max_vel_yaw = 0.4  # 最大转动速度


            if Frame.BASE == self.target.frame:
                transform_init_to_world = Transform3D(
                    trans_pose=self.robot_pose_when_target_set,
                    source_frame=Frame.BASE,  # 源坐标系为base_link
                    target_frame=Frame.ODOM  # 目标坐标系为odom
                )
                target_in_odom = transform_init_to_world.apply_to_pose(self.target)
            else:
                target_in_odom = self.target

            # raise NotImplementedError("cmd_vel控制模式尚未实现")
            # 如果是cmd_vel，则需要外部每次调用step()，来实现闭环控制
            # 逻辑是，先原地转到目标朝向，然后边走边调整朝向（确保朝向和机器人与目标间连线的朝向align）

            # 1. 获取当前世界系下位姿
            robot_pose = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            # 2. 目标位姿，默认只能是世界系
            assert Frame.ODOM == target_in_odom.frame, "目标位姿必须是世界坐标系（odom）"
            # 算目标朝向
            # angle_diff = robot_pose.angle_yaw(self.target)
            # 目标朝向是机器人与目标位置连线的朝向
            # compute target in frame of base

            euler = robot_pose.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0
            robot_pose_2d = Pose.from_euler(
                pos=robot_pose.pos,  # 只取x, y坐标
                euler= euler,  # 只取x, y朝向
                frame=Frame.ODOM,  # 使用base_link坐标系
                degrees=False
            )

            euler = target_in_odom.get_euler(degrees=False)
            euler[0] = 0.0
            euler[1] = 0.0

            target_in_odom_2d = Pose.from_euler(
                pos=target_in_odom.pos,  # 只取x, y坐标
                euler=euler,  # 只取x, y朝向
                frame=Frame.ODOM,  # 使用base_link坐标系
                degrees=False
            )

            transform_basa_to_odom = Transform3D(
                trans_pose=robot_pose_2d,
                source_frame=Frame.BASE,  # 源坐标系为base_link
                target_frame=Frame.ODOM  # 目标坐标系为odom
            )
            target_in_base = transform_basa_to_odom.apply_to_pose_inverse(target_in_odom_2d)
            print(f"目标在base_link坐标系下的位置：{target_in_base}")
            print(f"base in odom pose: {robot_pose_2d}")
            print(f"target in odom pose: {target_in_odom_2d}")


            angle_diff_line = np.arctan2(
                target_in_base.pos[1],
                target_in_base.pos[0]
            )
            angle_diff_line = normalize_angle(angle_diff_line)  # 归一化角度
            angle_diff_frame = target_in_base.get_euler(degrees=False)[2]  ## 两个坐标系间的角度差
            dis_diff = np.linalg.norm(target_in_base.pos[:2])

            print(f"当前坐标系间角度差：{np.rad2deg(angle_diff_frame):.2f}°，距离差：{dis_diff:.2f}米， 连线角度差：{np.rad2deg(angle_diff_line):.2f}°")
            # ======= 1. 初始定向阶段 ========
            if not self.initial_orient_complete:
                # 只转不走
                if abs(angle_diff_line) < 0.3:
                    self.robot_sdk.control.walk(0, 0, 0)
                    self.logger.info("🔵 初始定向阶段完成，转到目标连线方向")
                    self.initial_orient_complete = True

                # ==== 正在进行初始定向 =====
                min_vel_yaw = 0.2
                max_vel_yaw = 0.5

                kp_yaw = 0.8

                vel_yaw = kp_yaw * angle_diff_line
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)

                if abs(vel_yaw) < min_vel_yaw:
                    vel_yaw = np.sign(vel_yaw) * min_vel_yaw

                # 原地旋转
                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )

            # ======= 2. 进入目标位置附近阶段 ========
            # distance_threshold = 0.1  # 进入目标位置附近的距离阈值

            # ======= 3. 前进阶段（既不在初始定向阶段，也不在终局转向阶段） ========
            elif not self.walk_enter_pos_threshold:
                if dis_diff < np.max([0, self.pos_threshold - 0.05]):
                    # 停止机器人
                    self.robot_sdk.control.walk(0, 0, 0)
                    self.logger.info("🔵 进入目标位置附近，停止机器人")

                    # 开始调整朝向至目标frame朝向
                    # 一旦进入到这个状态，就不再出来，直到转向调整完毕
                    self.walk_enter_pos_threshold = True

                max_lin_vel = 0.5
                max_vel_yaw = 0.5
                kp_pos = 0.3 # 前进速度比例系数
                kp_yaw = 0.8  # 转动速度比例系数

                lin_vel = kp_pos * dis_diff
                lin_vel = np.clip(lin_vel, -max_lin_vel, max_lin_vel)  # 限制前进速度
                vel_x = lin_vel * np.cos(angle_diff_line)  # 前进速度
                vel_y = lin_vel * np.sin(angle_diff_line)  # 侧移速度

                vel_yaw = kp_yaw * angle_diff_line
                ## FIXME: 优化反向移动的逻辑
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)

                # 如果连线朝向小于某个值，开始前进
                if abs(angle_diff_line) < 0.2:
                    self.robot_sdk.control.walk(
                        linear_x=vel_x,  # 前进
                        linear_y=vel_y,  # 侧移
                        angular_z=vel_yaw  # 转动
                    )
                    self.logger.info(f"🔵 前进阶段，前进速度：{vel_x:.2f} m/s, 侧移速度：{vel_y:.2f} m/s, 转动速度：{vel_yaw:.2f} rad/s")
                else:
                    self.robot_sdk.control.walk(
                        linear_x=0.0,  # 不前进
                        linear_y=0.0,  # 不侧移
                        angular_z=vel_yaw  # 只转动
                    )
                    self.logger.info(f"🔵 前进阶段，转向连线方向，转动速度：{vel_yaw:.2f} rad/s")

            else:
                # 一旦进入到这个状态，就不再出来，直到转向调整完毕
                # 开始只转向
                # FIXME：添加角度差滤波
                min_vel_yaw = 0.05
                max_vel_yaw = 0.5
                kp_yaw = 0.8 * (1 - np.exp(-2*abs(angle_diff_frame)))  # 非线性增益

                if abs(angle_diff_frame) < np.max([0, self.yaw_threshold - np.deg2rad(2)]):  # ~ 0.5°，认为已经到达目标朝向
                    # 不再转动
                    self.robot_sdk.control.walk(0, 0, 0)
                    self.logger.info("🔵 已经到达目标朝向，停止转动")

                vel_yaw = kp_yaw * angle_diff_frame
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)
                if abs(vel_yaw) < min_vel_yaw:
                    vel_yaw = np.sign(vel_yaw) * min_vel_yaw

                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )

        if self.control_mode == "cmd_vel_loop":
            if self.target_executed:
                return self.get_status()
            self.target_executed = True

            start_time = time.time()

            # Get only x, y coordinates for 2D movement
            target_x, target_y = self.target.pos[0], self.target.pos[1]

            print(f"Moving toward target at ({target_x:.2f}, {target_y:.2f})")

            # 新增: 初始定向阶段 - 如果目标在后方，先转向
            initial_orient_complete = False
            initial_orient_timeout = 15.0  # 初始定向的超时时间
            initial_orient_start = time.time()

            while not initial_orient_complete and time.time() - initial_orient_start < initial_orient_timeout:
                # 获取当前位置和朝向
                current_odom = self.robot_sdk.state.odometry
                current_x, current_y = current_odom.position[0], current_odom.position[1]

                # 计算到目标的方向
                dx = target_x - current_x
                dy = target_y - current_y
                desired_angle = np.arctan2(dy, dx)

                # 获取当前朝向
                qx, qy, qz, qw = current_odom.orientation
                current_yaw = np.arctan2(2 * (qw * qz + qx * qy), qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2)

                # 计算角度差（确保在-π到π范围内）
                angle_diff = np.arctan2(np.sin(desired_angle - current_yaw),
                                        np.cos(desired_angle - current_yaw))

                print(f"初始定向: 当前角度差: {np.degrees(angle_diff):.1f}°")

                # 如果角度差很小，则初始定向完成
                if abs(angle_diff) < 0.3:  # 约17度
                    initial_orient_complete = True
                    self.robot_sdk.control.walk(0, 0, 0)  # 停止旋转
                    print("初始定向完成，开始移动")
                    break

                # 计算旋转速度 - 使用自适应旋转速度
                min_angular_speed = 0.2
                max_angular_speed = 0.5
                adaptive_gain = 0.8

                angular_speed = np.clip(
                    adaptive_gain * angle_diff,
                    -max_angular_speed,
                    max_angular_speed
                )

                # 确保最小旋转速度
                if abs(angular_speed) < min_angular_speed:
                    angular_speed = np.sign(angular_speed) * min_angular_speed

                # 原地旋转
                self.robot_sdk.control.walk(0, 0, angular_speed)
                time.sleep(0.1)

            # 如果初始定向超时，打印警告但继续执行
            if not initial_orient_complete:
                print("警告: 初始定向超时，目标可能在机器人后方，继续尝试移动")

            # 主移动循环
            while time.time() - start_time < 50:
                # Get current robot position
                current_odom = self.robot_sdk.state.odometry
                current_x, current_y = current_odom.position[0], current_odom.position[1]

                # Calculate distance to target
                dx = target_x - current_x
                dy = target_y - current_y
                distance = np.sqrt(dx ** 2 + dy ** 2)

                # If we're close enough, stop and return success
                if distance < 0.15:
                    # Stop the robot
                    self.robot_sdk.control.walk(0, 0, 0)
                    print(f"Target reached! Current position: ({current_x:.2f}, {current_y:.2f})")

                    target_orientation = self.target.quat
                    # 新增朝向调整逻辑
                    if target_orientation is not None:
                        print("开始调整目标朝向...")
                        print(f"Moving toward target_orientation at {target_orientation}")
                        start_orient_time = time.time()
                        orient_timeout = 30

                        # 优化四元数到yaw的转换公式
                        qx_t, qy_t, qz_t, qw_t = target_orientation
                        # 使用更精确的yaw角计算方式
                        target_yaw = np.arctan2(2 * (qw_t * qz_t + qx_t * qy_t),
                                                qw_t ** 2 + qx_t ** 2 - qy_t ** 2 - qz_t ** 2)

                        # 添加低通滤波器参数
                        prev_angle_diff = 0.0
                        filter_alpha = 0.2  # 滤波系数

                        while time.time() - start_orient_time < orient_timeout:
                            current_q = self.robot_sdk.state.odometry.orientation
                            # 使用相同的优化公式计算当前yaw
                            current_yaw = np.arctan2(2 * (current_q[3] * current_q[2] + current_q[0] * current_q[1]),
                                                     current_q[3] ** 2 + current_q[0] ** 2 - current_q[1] ** 2 -
                                                     current_q[2] ** 2)

                            angle_diff = target_yaw - current_yaw
                            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

                            # 添加角度差滤波
                            filtered_angle_diff = filter_alpha * angle_diff + (1 - filter_alpha) * prev_angle_diff
                            prev_angle_diff = filtered_angle_diff

                            # 添加自适应控制参数
                            min_angular_speed = 0.05  # 最小角速度
                            max_angular_speed = 0.5  # 最大角速度
                            adaptive_gain = 0.8 * (1 - np.exp(-2 * abs(filtered_angle_diff)))  # 非线性增益

                            if abs(filtered_angle_diff) < 0.0087:  # ~0.5度
                                self.robot_sdk.control.walk(0, 0, 0)
                                print(f"朝向精确调整完成，最终角度差: {np.degrees(filtered_angle_diff):.2f}°")
                                return self.get_status()

                            # 计算带死区和速度限制的角速度
                            angular_speed = np.clip(
                                adaptive_gain * filtered_angle_diff,
                                -max_angular_speed,
                                max_angular_speed
                            )
                            if abs(angular_speed) < min_angular_speed:
                                angular_speed = np.sign(angular_speed) * min_angular_speed

                            self.robot_sdk.control.walk(0, 0, angular_speed)
                            time.sleep(0.05)  # 缩短控制周期

                    return self.get_status()

                # 新增：初始化 linear_speed
                max_linear_speed = 0.5
                linear_speed = min(0.3 * distance, max_linear_speed)  # 提前计算

                # 改进后的角度差计算
                desired_angle = np.arctan2(dy, dx)

                # 转换四元数到yaw时使用更健壮的公式
                qx, qy, qz, qw = current_odom.orientation
                current_yaw = np.arctan2(2 * (qw * qz + qx * qy), qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2)

                # 改进后的角度差计算（处理2π周期问题）
                angle_diff = np.arctan2(np.sin(desired_angle - current_yaw),
                                        np.cos(desired_angle - current_yaw))

                # 新增：确保 angular_speed 在条件判断前被正确计算
                angular_speed = 0.8 * angle_diff  # 基础角速度

                # 反向移动逻辑优化（现在可以安全使用 linear_speed）
                if abs(angle_diff) > np.pi / 2:
                    angle_diff = angle_diff - np.sign(angle_diff) * np.pi
                    linear_speed *= -1
                    angular_speed *= 1.2  # 反向时增加旋转速度

                # 新增：限制最大角速度
                max_angular_speed = 0.5
                angular_speed = np.clip(angular_speed, -max_angular_speed, max_angular_speed)

                # 修改后的运动控制逻辑
                if abs(angle_diff) < 0.2:
                    body_linear_x = linear_speed * np.cos(angle_diff)
                    body_linear_y = linear_speed * np.sin(angle_diff)
                    self.robot_sdk.control.walk(body_linear_x, body_linear_y, angular_speed)
                else:
                    self.robot_sdk.control.walk(0, 0, angular_speed)

                print(f"Current: ({current_x:.2f}, {current_y:.2f}), Distance: {distance:.2f}m, "
                      f"Angle diff: {np.degrees(angle_diff):.1f}°")

                # Sleep a bit to avoid excessive CPU usage
                time.sleep(0.1)

            # If we get here, timeout occurred
            self.robot_sdk.control.walk(0, 0, 0)  # Stop the robot
            print("Timeout reached without getting to target")

        # time.sleep(0.05)
        return self.get_status()

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
        res = super().set_target(target, *args, **kwargs)
        if res:
            # 为了应对相对位置控制的情况，记录设置目标时机器人的位姿
            self.robot_pose_when_target_set = Pose(
                pos=self.robot_sdk.state.robot_position(),
                quat=self.robot_sdk.state.robot_orientation(),
                frame=Frame.ODOM
            )
            self.target = target  # 设置目标位置
            self.target_executed = False  # 标记目标位置未执行

            if self.control_mode == "cmd_vel":
                self.initial_orient_complete = False  # 标记初始定向阶段
                self.walk_enter_pos_threshold = False

        return res

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
            target_yaw = self.target.get_euler(degrees=False)[2]  # 获取目标偏航角
            yaw_reached, yaw_diff = self._check_yaw(target_yaw)

            pos_reached, pos_diff = self._check_position_2d(self.target.pos[0], self.target.pos[1])

            if yaw_reached and pos_reached:
                self.logger.info(
                    f'目标位置已到达: {self.target.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
                return True

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
        robot_pose = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation()
        )
        robot_yaw = robot_pose.get_euler(degrees=False)[2]  # 获取机器人的偏航角
        yaw_diff = normalize_angle(target_yaw - robot_yaw)
        self.logger.info(f"偏航角差: {yaw_diff}, 阈值: {self.yaw_threshold}")
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
        robot_pos = self.robot_sdk.state.robot_position()
        pos_diff = np.linalg.norm(np.array(robot_pos[:2]) - np.array([target_x, target_y]))
        self.logger.info(f"位置差: {pos_diff}, 阈值: {self.pos_threshold}")
        return pos_diff <= self.pos_threshold, pos_diff

