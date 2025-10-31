from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.utils.utils import normalize_angle
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoArmCtrlMode,
    KuavoManipulationMpcFrame)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcControlFlow

import threading
from concurrent.futures import ThreadPoolExecutor, Future
from typing import List, Tuple
import time
import numpy as np
import copy


def transform_pose_from_tag_to_world(tag: Tag, pose: Pose) -> Pose:
    """
    将tag坐标系下的位姿转换到世界坐标系下。

    参数：
        tag (Tag): Tag对象，包含位姿信息。
        pose (Pose): 需要转换的位姿。

    返回：
        Pose: 转换后的Pose对象。
    """
    # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
    transform_tag_to_world = Transform3D(
        trans_pose=tag.pose,
        source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
        target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
    )
    stand_pose_in_world = transform_tag_to_world.apply_to_pose(
        pose  # 将站立位置转换到世界坐标系
    )
    return stand_pose_in_world


class HeadAPI:
    """
    头部控制API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)

    def _move_head_traj(self,
                        head_traj: List[Tuple[float, float]] = [],  # 头部目标点列表，格式为[(yaw, pitch), ...
                        ):
        for pair in head_traj:
            yaw, pitch = pair
            self.robot_sdk.control.move_head_to_pitch_yaw(
                pitch, yaw, timeout=5.0)
            time.sleep(0.7)

    def move_head_traj(self,
                       head_traj: List[Tuple[float, float]] = [],  # 头部目标点列表，格式为[(yaw, pitch), ...
                       asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
                       ) -> Future:
        if asynchronous:
            fut = self._pool.submit(self._move_head_traj, head_traj)
            return fut  # 外部拿到 Future

        else:
            self._move_head_traj(head_traj)
            return None


class ArmAPI:
    """
    根据手臂和躯干控制接口，封装手臂控制的API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)
        self.robot_sdk.control.set_external_control_arm_mode()

    def _move_eef_traj_kmpc(self,
                            left_traj: List[List[float]],  # 末端6d位姿的轨迹，带时间戳
                            right_traj: List[List[float]],  # 末端6d位姿的轨迹，带时间戳
                            total_time: float,  # 轨迹总时间，单位秒
                            control_base: bool,  # 是否连带base一起控制
                            direct_to_wbc: bool,  # 指令是否经过全身MPC的优化再到WBC
                            frame: str

                            ):

        # 切成外部控制模式
        self.robot_sdk.control.set_external_control_arm_mode()
        if control_base:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        else:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        if direct_to_wbc:
            self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.DirectToWbc)

        num_points = min(len(left_traj), len(right_traj))
        time_per_point = total_time / (num_points - 1) if num_points > 1 else total_time
        for i in range(num_points):
            self.robot_sdk.control.control_robot_end_effector_pose(
                left_pose=left_traj[i],
                right_pose=right_traj[i],
                frame=frame,
            )
            if i < num_points - 1:  # 最后一个点不需要延时
                time.sleep(time_per_point)

        # 运动结束后，切回默认模式
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

    def move_eef_traj_kmpc(
            self,
            left_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            right_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            control_base: bool = False,  # 是否连带base一起控制
            direct_to_wbc: bool = True,  # 指令是否经过全身MPC的优化再到WBC
            total_time: float = 5.0,  # 轨迹总时间，单位秒
            frame: str = KuavoManipulationMpcFrame.WorldFrame
            # 指令位置所在的坐标系： 'base_link'： 在机器人base_link坐标系下； 'foot_print': 'base_link' 在地面的投影; 'world': 世界系
    ):
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_eef_traj_kmpc,
                                    left_traj, right_traj, total_time, control_base, direct_to_wbc, frame)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_eef_traj_kmpc(
                left_traj, right_traj, total_time, control_base, direct_to_wbc, frame
            )

            return None

    def get_eef_pose_world(self):
        target_frame = Frame.ODOM

        left_pose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_l7_end_effector",
            reference_frame=target_frame
        )
        right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
            link_name="zarm_r7_end_effector",
            reference_frame=target_frame
        )

        current_left_pose = Pose(
            pos=left_pose.position,
            quat=left_pose.orientation,
            frame=target_frame
        )
        current_right_pose = Pose(
            pos=right_pose.position,
            quat=right_pose.orientation,
            frame=target_frame
        )

        return current_left_pose, current_right_pose

    def get_current_transform(self, source_frame: Frame, target_frame: Frame) -> Transform3D:
        """
        将tf的变换转换为Transform3D对象。

        参数：
            source_frame (Frame): 源坐标系。
            target_frame (Frame): 目标坐标系。

        返回：
            Transform3D: 转换后的Transform3D对象。
        """
        tf_pose = self.robot_sdk.tools.get_tf_transform(target_frame, source_frame)

        source_to_target_pose = Pose(
            pos=tf_pose.position,
            quat=tf_pose.orientation,
            frame=target_frame
        )

        transform_source_to_target = Transform3D(
            trans_pose=source_to_target_pose,
            source_frame=source_frame,  # 源坐标系为Tag坐标系
            target_frame=target_frame  # 目标坐标系为里程计坐标系
        )

        return transform_source_to_target


class TorsoAPI:
    """
    根据手臂和躯干控制接口，封装躯干控制的API
    """

    def __init__(self, robot_sdk: RobotSDK):
        self.robot_sdk = robot_sdk
        self._pool = ThreadPoolExecutor(max_workers=2)

        # self._goal_lock = threading.Lock()
        # self._goal_cv   = threading.Condition(self._goal_lock)
        # self._current_goal = None  # 当前目标位姿

        # 共享状态
        self._target_lock = threading.Lock()
        self._current_target: Pose = None

    def _check_success_walk(self,
                            target_in_odom,
                            yaw_threshold,
                            pos_threshold
                            ):
        target_yaw = target_in_odom.get_euler(degrees=False)[2]  # 获取目标偏航角

        # === check_yaw ===
        robot_pose = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation()
        )
        robot_yaw = robot_pose.get_euler(degrees=False)[2]  # 获取机器人的偏航角
        yaw_diff = normalize_angle(target_yaw - robot_yaw)
        yaw_reached = abs(yaw_diff) <= yaw_threshold

        robot_pos = self.robot_sdk.state.robot_position()
        pos_diff = np.linalg.norm(np.array(robot_pos[:2]) - np.array(target_in_odom.pos[:2]))
        pos_reached = pos_diff <= pos_threshold

        # print(
        # f'目标未到达: {target_in_odom.pos}, 偏航角未到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')

        if yaw_reached and pos_reached:
            print(
                f'目标位置已到达: {target_in_odom.pos}, 偏航角已到达: {target_yaw:.2f} rad, diff: {yaw_diff:.2f} rad | {pos_diff}')
            return True

        return False

    def _walk_to_pose_by_vel(self,
                             # target: Pose,
                             pos_threshold=0.05,
                             kp_pos=0.5,
                             kp_yaw=0.5,
                             max_vel_x=0.4,
                             max_vel_yaw=0.6,
                             timeout=60
                             ):
        """
        躯干行走到某个点，通过速度控制
        """
        robot_pose_when_start = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )

        # 2) 取目标（如果外部刚更新，这里能立刻看到）
        is_target_new = True

        tic = time.time()

        while time.time() - tic < timeout:
            # 0. 获取并处理target

            with self._target_lock:
                if self._current_target is not None:
                    target = copy.deepcopy(self._current_target)
                    is_target_new = True  # 目标更新了

            with self._target_lock:
                self._current_target = None  # 取走目标

            # assert target is not None, "目标位姿不能为空"

            if is_target_new:
                if target.frame not in [Frame.ODOM, Frame.BASE]:
                    print("使用'cmd_vel'速度控制模式时，目标位姿的坐标系必须是'odom' 或'base_link'")
                    return False

                if Frame.BASE == target.frame:
                    transform_init_to_world = Transform3D(
                        trans_pose=robot_pose_when_start,
                        source_frame=Frame.BASE,  # 源坐标系为base_link
                        target_frame=Frame.ODOM  # 目标坐标系为odom
                    )
                    target_in_odom = transform_init_to_world.apply_to_pose(target)
                else:
                    target_in_odom = target
                is_target_new = False  # 只处理一次

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
                euler=euler,  # 只取x, y朝向
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
            # print(f"目标在base_link坐标系下的位置：{target_in_base}")
            # print(f"base in odom pose: {robot_pose_2d}")
            # print(f"target in odom pose: {target_in_odom_2d}")

            angle_diff_line = np.arctan2(
                target_in_base.pos[1],
                target_in_base.pos[0]
            )
            angle_diff_line = normalize_angle(angle_diff_line)  # 归一化角度
            angle_diff_frame = target_in_base.get_euler(degrees=False)[2]  ## 两个坐标系间的角度差
            dis_diff = np.linalg.norm(target_in_base.pos[:2])

            # print(
            #     f"当前坐标系间角度差：{np.rad2deg(angle_diff_frame):.2f}°，距离差：{dis_diff:.2f}米， 连线角度差：{np.rad2deg(angle_diff_line):.2f}°")
            # 如果朝向大于某个值，先转不走
            max_yaw_to_walk = np.deg2rad(10)  # 超过这个值就不走只转
            max_dis_to_rotate = pos_threshold  # 小于这个距离就转到angle_diff_frame

            # 1. if dis too small， then use holonomic fine tune

            if dis_diff < max_dis_to_rotate:
                vel_yaw = kp_yaw * angle_diff_frame
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                # print(f"转向target frame朝向，转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )

            elif dis_diff < (max_dis_to_rotate + 0.1) or (
                    abs(target_in_base.pos[1]) < 0.1 and abs(angle_diff_frame) < np.deg2rad(10)):
                # 如果距离小于阈值，使用holonomic控制
                x_diff = target_in_base.pos[0]
                y_diff = target_in_base.pos[1]
                vel_x = kp_pos * x_diff
                vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)
                vel_y = kp_pos * y_diff
                vel_y = np.clip(vel_y, -max_vel_x, max_vel_x)
                # print(f'holonomic控制，前进速度：{vel_x:.2f} m/s, 侧移速度：{vel_y:.2f} m/s')
                self.robot_sdk.control.walk(
                    linear_x=vel_x,  # 前进
                    linear_y=vel_y,  # 侧移
                    angular_z=0.0  # 不转动
                )

            elif abs(angle_diff_line) > max_yaw_to_walk:
                vel_yaw = kp_yaw * angle_diff_line
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                # print(f"dis_diff {dis_diff}; 转向连线方向，转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=0.0,  # 不前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 只转动
                )
            elif dis_diff >= max_dis_to_rotate:
                # 如果连线朝向小于某个值，开始前进
                # dis_sign = (abs(angle_diff_line) > np.pi)

                vel_x = kp_pos * dis_diff
                vel_x = np.clip(vel_x, -max_vel_x, max_vel_x)  # 限制前进速度

                vel_yaw = kp_yaw * angle_diff_line
                vel_yaw = np.clip(vel_yaw, -max_vel_yaw, max_vel_yaw)  # 限制转速
                # print(f"dis_diff {dis_diff}, 前进速度：{vel_x:.2f} m/s, 转动速度：{vel_yaw:.2f} rad/s")
                self.robot_sdk.control.walk(
                    linear_x=vel_x,  # 前进
                    linear_y=0.0,  # 不侧移
                    angular_z=vel_yaw  # 不转动
                )

            # time.sleep(0.1)  # 控制频率
            time.sleep(0.05)

            success = self._check_success_walk(
                target_in_odom, yaw_threshold=np.deg2rad(5), pos_threshold=0.1)
            if success:
                self.stop_walk()
                break
        return None

    def update_walk_goal(self, new_goal: Pose):
        """线程安全：更新当前目标并唤醒控制线程；返回新的版本号。"""
        print(f'接收到新的行走目标：{new_goal}')
        with self._target_lock:
            self._current_target = new_goal

    def walk_to_pose_by_vel(self,
                            # target: Pose,
                            pos_threshold=0.1,
                            kp_pos=0.5,
                            kp_yaw=0.5,
                            max_vel_x=0.5,
                            max_vel_yaw=0.4,
                            asynchronous: bool = True):
        if asynchronous:
            # 多线程，异步

            fut = self._pool.submit(self._walk_to_pose_by_vel, pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw)

            return fut

        else:
            self._walk_to_pose_by_vel(pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw)

            return None

    def stop_walk(self):
        for _ in range(10):
            self.robot_sdk.control.walk(0.0, 0.0, 0.0)
