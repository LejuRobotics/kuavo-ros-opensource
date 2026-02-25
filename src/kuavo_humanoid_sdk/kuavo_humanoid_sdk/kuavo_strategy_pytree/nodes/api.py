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
import math
import rospy
from std_msgs.msg import Float32


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
                            back_default: bool,  # 是否返回默认模式
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
        if back_default:
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
            self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

    def _move_joint_traj(self,
                        joint_traj: List[List[float]],  # 关节角度轨迹，每个元素是14维关节角度列表
                        total_time: float,  # 轨迹总时间，单位秒
                        ):
        """
        执行关节轨迹（私有方法，内部调用）

        参数：
            joint_traj (List[List[float]]): 关节角度轨迹，每个元素是14维关节角度列表
            total_time (float): 轨迹总时间，单位秒
        """
        # 切换到外部控制模式
        # self.robot_sdk.control.set_external_control_arm_mode()
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        # self.robot_sdk.control.set_manipulation_mpc_control_flow(
        #     KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

        num_points = len(joint_traj)
        time_per_point = total_time / (num_points - 1) if num_points > 1 else total_time

        for i, joint_pos in enumerate(joint_traj):
            self.robot_sdk.control.control_arm_joint_positions(
                joint_positions=joint_pos
            )
            if i < num_points - 1:  # 最后一个点不需要延时
                time.sleep(time_per_point)

        # # 运动结束后，切回默认模式
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        # self.robot_sdk.control.set_manipulation_mpc_control_flow(
        #     KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)

    def move_joint_traj(self,
                       joint_traj: List[List[float]],  # 关节角度轨迹，每个元素是14维关节角度列表
                       asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
                       total_time: float = 5.0,  # 轨迹总时间，单位秒
                       ):
        """
        执行关节轨迹控制接口

        参数：
            joint_traj (List[List[float]]): 关节角度轨迹，每个元素是14维关节角度列表
            asynchronous (bool): 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            total_time (float): 轨迹总时间，单位秒

        返回：
            Future: 如果 asynchronous=True，返回 Future 对象；否则返回 None
        """
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_joint_traj,
                                    joint_traj, total_time)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_joint_traj(joint_traj, total_time)
            return None

    def move_eef_traj_kmpc(
            self,
            left_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            right_traj: List[List[float]],  # 末端6d位姿的轨迹，不带时间戳
            asynchronous: bool = False,  # 布尔值，指定运动命令是否为异步。默认值为 false，表示函数会阻塞
            control_base: bool = False,  # 是否连带base一起控制
            direct_to_wbc: bool = True,  # 指令是否经过全身MPC的优化再到WBC
            total_time: float = 5.0,  # 轨迹总时间，单位秒
            back_default: bool = True,  # 是否返回默认模式
            frame: str = KuavoManipulationMpcFrame.WorldFrame
            # 指令位置所在的坐标系： 'base_link'： 在机器人base_link坐标系下； 'foot_print': 'base_link' 在地面的投影; 'world': 世界系
    ):
        if asynchronous:
            # 多线程
            fut = self._pool.submit(self._move_eef_traj_kmpc,
                                    left_traj, right_traj, total_time, control_base, direct_to_wbc, back_default, frame)
            return fut  # 外部拿到 Future

        else:
            # 本函数阻塞
            self._move_eef_traj_kmpc(
                left_traj, right_traj, total_time, control_base, direct_to_wbc, back_default, frame
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
        
        # 提前创建躯干到达时间订阅者，避免临时创建的时间损耗
        self._torso_reach_time_received = None
        self._torso_reach_time_event = threading.Event()
        self._torso_reach_time_lock = threading.Lock()
        
        def torso_reach_time_callback(msg):
            with self._torso_reach_time_lock:
                self._torso_reach_time_received = msg.data
                self._torso_reach_time_event.set()
        
        self._torso_reach_time_sub = rospy.Subscriber('/lb_torso_pose_reach_time', Float32, torso_reach_time_callback)

    def _move_wheel_lower_joint(self,
                            joint_traj: list,
                            total_time: float):
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)

        # 直接发布单组关节角度
        self.robot_sdk.control.control_wheel_lower_joint(joint_traj)


    def move_wheel_lower_joint(self,
                            joint_traj: list,
                            asynchronous: bool = False,
                            total_time: float = 5.0):
        if asynchronous:
            return self._pool.submit(self._move_wheel_lower_joint, joint_traj, total_time)
        else:
            self._move_wheel_lower_joint(joint_traj, total_time)
            return None

    @staticmethod
    def _calculate_time_with_jerk_limited(distance, max_vel, max_acc, max_jerk):
        """
        计算 jerk-limited 轨迹规划的时间

        参数:
            distance: 位移距离 (m 或 rad)
            max_vel: 最大速度 (m/s 或 rad/s)
            max_acc: 最大加速度 (m/s² 或 rad/s²)
            max_jerk: 最大加加速度 (m/s³ 或 rad/s³)

        返回:
            total_time: 总时间 (s)
        """
        distance = abs(distance)
        if distance < 1e-6:
            return 0.0

        # 计算达到最大加速度所需的时间
        t_acc_to_max_acc = max_acc / max_jerk

        # 计算在最大加速度下达到最大速度所需的时间
        t_acc_to_max_vel = max_vel / max_acc

        # 判断是否能够达到最大速度
        if t_acc_to_max_vel <= t_acc_to_max_acc:
            # 情况1: 无法达到最大加速度，直接达到最大速度
            # 根据 v(t) = J*t²/2，要达到 max_vel 需要：t = sqrt(2*max_vel/max_jerk)
            t1 = math.sqrt(2 * max_vel / max_jerk)
            s1 = (max_jerk * t1**3) / 6

            s2 = distance - 2 * s1
            if s2 > 0:
                t2 = s2 / max_vel
                return t1 + t2 + t1
            else:
                # 无法达到最大速度，全程S曲线
                t_half = (distance * 3 / max_jerk) ** (1/3)
                return 2 * t_half
        else:
            # 情况2: 能够达到最大加速度和最大速度
            t1_1 = t_acc_to_max_acc
            s1_1 = (max_jerk * t1_1**3) / 6

            t1_2 = (max_vel - max_jerk * t1_1**2 / 2) / max_acc
            s1_2 = max_jerk * t1_1**2 * t1_2 / 2 + max_acc * t1_2**2 / 2

            t1 = t1_1 + t1_2
            s1 = s1_1 + s1_2

            s2 = distance - 2 * s1
            if s2 > 0:
                t2 = s2 / max_vel
                return t1 + t2 + t1
            else:
                # 无法达到最大速度，使用迭代方法求解
                return TorsoAPI._calculate_time_short_distance(distance, max_vel, max_acc, max_jerk)

    @staticmethod
    def _calculate_time_short_distance(distance, max_vel, max_acc, max_jerk):
        """短距离情况下的时间计算（无法达到最大速度）"""
        t_total = 0.1
        for _ in range(100):
            t_half = t_total / 2

            if t_half <= max_acc / max_jerk:
                s_half = (max_jerk * t_half**3) / 6
            else:
                t1 = max_acc / max_jerk
                s1 = (max_jerk * t1**3) / 6
                t2 = t_half - t1
                v1 = max_jerk * t1**2 / 2
                s2 = v1 * t2 + max_acc * t2**2 / 2
                s_half = s1 + s2

            s_total = 2 * s_half

            if abs(s_total - distance) < 0.001:
                return t_total

            if s_total < distance:
                t_total *= 1.1
            else:
                t_total *= 0.9

        return t_total

    @staticmethod
    def _calculate_torso_move_time(current_pose, target_pose, auto_calculate=True):
        """
        根据当前位姿和目标位姿计算所需的移动时间

        参数:
            current_pose: 当前位姿 [x, y, z, roll, pitch, yaw]
            target_pose: 目标位姿 [x, y, z, roll, pitch, yaw]
            auto_calculate: 是否自动计算时间

        返回:
            total_time: 总时间 (s)
        """
        if not auto_calculate:
            return None

        # 从 task.info 读取的配置（对应 torsoPose_move）
        config = {
            'max_vel': {
                'x': 0.9,      # m/s
                'z': 0.9,      # m/s
                'yaw': 2.08,   # rad/s
                'pitch': 2.08  # rad/s
            },
            'max_acc': {
                'x': 9.0,      # m/s²
                'z': 9.0,      # m/s²
                'yaw': 10.4,   # rad/s²
                'pitch': 10.4  # rad/s²
            },
            'max_jerk': {
                'x': 15.6,     # m/s³
                'z': 15.6,     # m/s³
                'yaw': 40.6,   # rad/s³
                'pitch': 40.6  # rad/s³
            }
        }

        # 计算各自由度的位移
        displacements = {
            'x': abs(target_pose[0] - current_pose[0]),
            'z': abs(target_pose[2] - current_pose[2]),
            'yaw': abs(normalize_angle(target_pose[5] - current_pose[5])),
            'pitch': abs(normalize_angle(target_pose[4] - current_pose[4]))
        }

        # 计算各自由度所需时间，取最大值
        max_time = 0.0
        for dof in ['x', 'z', 'yaw', 'pitch']:
            distance = displacements[dof]
            if distance > 1e-6:  # 忽略极小位移
                max_vel = config['max_vel'][dof]
                max_acc = config['max_acc'][dof]
                max_jerk = config['max_jerk'][dof]

                dof_time = TorsoAPI._calculate_time_with_jerk_limited(
                    distance, max_vel, max_acc, max_jerk
                )
                max_time = max(max_time, dof_time)

        # 确保最小时间为 0.1 秒
        return max(max_time, 0.1)

    @staticmethod
    def resample_and_execute_traj(traj, total_time, publish_fn, publish_rate=100.0):
        """
        对轨迹进行重采样并以固定频率下发

        参数：
            traj: 轨迹数据，每个元素是一个位置列表
            total_time: 总执行时间（秒）
            publish_fn: 下发函数，接受一个位置列表作为参数
            publish_rate: 下发频率（Hz），默认100Hz
        """
        num_points_orig = len(traj)
        if num_points_orig < 1:
            return

        if num_points_orig == 1:
            publish_fn(traj[0])
            return

        # 根据总时间和固定频率计算需要的点数
        num_points_target = int(total_time * publish_rate) + 1
        dt = 1.0 / publish_rate

        # 对轨迹进行线性插值重采样
        traj_np = np.array(traj)
        t_orig = np.linspace(0, 1, num_points_orig)
        t_target = np.linspace(0, 1, num_points_target)

        resampled_traj = []
        for t in t_target:
            idx = np.searchsorted(t_orig, t)
            if idx == 0:
                resampled_traj.append(traj_np[0].tolist())
            elif idx >= num_points_orig:
                resampled_traj.append(traj_np[-1].tolist())
            else:
                t0, t1 = t_orig[idx - 1], t_orig[idx]
                alpha = (t - t0) / (t1 - t0)
                interpolated = (1 - alpha) * traj_np[idx - 1] + alpha * traj_np[idx]
                resampled_traj.append(interpolated.tolist())

        # 以固定频率下发
        for i, pos in enumerate(resampled_traj):
            publish_fn(pos)
            if i < len(resampled_traj) - 1:
                time.sleep(dt)


    def _move_torso_pose(self,
                         desir_torso_pose: Pose,
                         total_time: float = None):
        """
        执行躯干位姿控制，从当前位姿插值到目标位姿

        参数：
            desir_torso_pose: 目标躯干位姿
            total_time: 执行时间（秒），如果为 None 或 <= 0，则自动计算
        """
        if desir_torso_pose is None:
            raise ValueError("desir_torso_pose must not be None")

        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)

        # 获取目标位姿 [x, y, z, roll, pitch, yaw]
        target_x, target_y, target_z = desir_torso_pose.pos.tolist()
        target_roll, target_pitch, target_yaw = desir_torso_pose.get_euler(degrees=False).tolist()
        target_pose = [target_x, target_y, target_z, target_roll, target_pitch, target_yaw]

        # 获取当前躯干位姿（通过 tf 树获取 base_link 的位置，会被映射为 waist_yaw_link）
        try:
            current_tf_pose = self.robot_sdk.tools.get_link_pose(
                link_name="base_link",
                reference_frame="base_link_lb"
            )
            if current_tf_pose is not None:
                # 打印原始 tf 位姿
                print(f"[TorsoAPI] base_link(->waist_yaw_link) tf position: {current_tf_pose.position}")
                print(f"[TorsoAPI] base_link(->waist_yaw_link) tf orientation (quat): {current_tf_pose.orientation}")

                # 将四元数转换为欧拉角
                current_pose_obj = Pose(
                    pos=current_tf_pose.position,
                    quat=current_tf_pose.orientation,
                    frame=Frame.BASE
                )
                current_euler = current_pose_obj.get_euler(degrees=False).tolist()
                print(f"[TorsoAPI] base_link euler (rad): {current_euler}")

                current_pose = [
                    current_tf_pose.position[0],
                    current_tf_pose.position[1],
                    current_tf_pose.position[2],
                    current_euler[0],
                    current_euler[1],
                    current_euler[2]
                ]
            else:
                # 如果获取失败，使用默认零位姿
                print("[Warning] 获取当前躯干位姿失败，使用默认零位姿")
                current_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
        except Exception as e:
            print(f"[Warning] 获取当前躯干位姿异常: {e}，使用默认零位姿")
            current_pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]

        print(f"[TorsoAPI] 当前位姿: {current_pose}")
        print(f"[TorsoAPI] 目标位姿: {target_pose}")

        # 如果 total_time 未指定或 <= 0，则自动计算
        if total_time is None or total_time <= 0:
            calculated_time = self._calculate_torso_move_time(current_pose, target_pose, auto_calculate=True)
            total_time = calculated_time
            print(f"[TorsoAPI] 自动计算执行时间: {total_time:.3f}s")
        else:
            print(f"[TorsoAPI] 使用指定执行时间: {total_time:.3f}s")

        # calculated_time = self._calculate_torso_move_time(current_pose, target_pose, auto_calculate=True)
        # total_time = calculated_time
        # print(f"[TorsoAPI] 自动计算执行时间: {total_time:.3f}s")

        # 生成从当前位姿到目标位姿的轨迹
        traj = [current_pose, target_pose]

        # 定义下发函数
        def publish_torso_pose(pose):
            x, y, z, roll, pitch, yaw = pose
            self.robot_sdk.control.control_torso_pose(x, y, z, roll, pitch, yaw)

        # 使用通用函数进行重采样和下发
        import time as time_module
        t_before_traj = time_module.time()
        print(f"[TorsoAPI] 开始执行轨迹，total_time={total_time:.3f}s")
        self.resample_and_execute_traj(traj, total_time, publish_torso_pose)
        t_after_traj = time_module.time()
        traj_exec_time = t_after_traj - t_before_traj
        print(f"[TorsoAPI] 轨迹执行完成，实际耗时: {traj_exec_time:.3f}s (预期: {total_time:.3f}s)")

    def _move_torso_pose_with_time(self,
                                   desir_torso_pose: Pose,
                                   total_time: float):
        """
        执行躯干位姿控制，以 100Hz 频率持续下发目标位姿
        注意：这个方法现在主要用于轨迹控制，单次调用请使用 move_torso_pose
        """
        print(f"[TorsoAPI] 开始执行躯干位姿控制")
        
        # 重置事件和接收值，准备接收新消息
        with self._torso_reach_time_lock:
            self._torso_reach_time_received = None
            self._torso_reach_time_event.clear()
        
        # 在主线程中调用 control_torso_pose（这个方法可能在主线程或子线程中被调用）
        self.robot_sdk.control.control_torso_pose(
            desir_torso_pose.pos[0], 
            desir_torso_pose.pos[1], 
            desir_torso_pose.pos[2], 
            desir_torso_pose.get_euler(degrees=False)[0], 
            desir_torso_pose.get_euler(degrees=False)[1], 
            desir_torso_pose.get_euler(degrees=False)[2]
        )

        # 等待接收到消息（最多等待10秒）
        if self._torso_reach_time_event.wait(timeout=10.0):
            with self._torso_reach_time_lock:
                reach_time_received = self._torso_reach_time_received
            
            # 等待接收到的到达时间
            if reach_time_received is not None and reach_time_received > 0:
                time.sleep(reach_time_received)
            print(f"[TorsoAPI] 等待接收到的到达时间: {reach_time_received:.3f}s")
        else:
            print("[Warning] 未能在10秒内接收到 /lb_torso_pose_reach_time 消息")

        return True

    def move_torso_pose(self,
                        desir_torso_pose: Pose,
                        asynchronous: bool = False,
                        total_time: float = None):
        """
        执行躯干位姿控制，以 100Hz 频率持续下发目标位姿

        参数：
            desir_torso_pose: 目标躯干位姿
            asynchronous: 是否异步执行
            total_time: 执行时间（秒），如果为 None 或 <= 0，则根据位移自动计算
        """
        if asynchronous:
            return self._pool.submit(
                self._move_torso_pose,
                desir_torso_pose,
                total_time,
            )

        self._move_torso_pose(desir_torso_pose, total_time)
        return True


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

    def _walk_to_pose_by_pose_world(self,
                                    pos_threshold=0.05,
                                    timeout=60):
        """
        躯干行走到某个点，通过世界位置控制
        """
        # 获取目标位姿
        target = None
        with self._target_lock:
            if self._current_target is not None:
                target = copy.deepcopy(self._current_target)
                self._current_target = None  # 取走目标

        if target is None:
            print("_walk_to_pose_by_pose_world: 没有目标位姿")
            return False

        # 发送位置命令（只发送一次）
        target_x = target.pos[0]
        target_y = target.pos[1]
        target_z = 0.0
        target_yaw = target.get_euler(degrees=False)[2]

        self.robot_sdk.control.control_command_pose_world(target_x, target_y, target_z, target_yaw)
        print(f"📤 cmd_pos_world 发送一次: 目标=[{target_x:.3f}, {target_y:.3f}, {target.get_euler(degrees=True)[2]:.1f}°]")

        # 等待到达目标（使用轮询方式）
        tic = time.time()
        while time.time() - tic < timeout:
            success = self._check_success_walk(
                target,
                yaw_threshold=np.deg2rad(5),
                pos_threshold=pos_threshold
            )
            if success:
                return True
            time.sleep(0.1)

        print(f"❌ cmd_pos_world 超时 {timeout}s，未到达目标")
        return False

    def _walk_to_pose_by_pose(self,
                              pos_threshold=0.05,
                              timeout=60):
        """
        躯干行走到某个点，通过相对位置控制
        """
        # 获取目标位姿
        target = None
        with self._target_lock:
            if self._current_target is not None:
                target = copy.deepcopy(self._current_target)
                self._current_target = None  # 取走目标

        if target is None:
            print("_walk_to_pose_by_pose: 没有目标位姿")
            return False

        robot_pose_when_start = Pose(
            pos=self.robot_sdk.state.robot_position(),
            quat=self.robot_sdk.state.robot_orientation(),
            frame=Frame.ODOM
        )

        # 如果目标是base_link坐标系，需要转换到世界坐标系
        if target.frame == Frame.BASE:
            transform_base_to_world = Transform3D(
                trans_pose=robot_pose_when_start,
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            target_in_world = transform_base_to_world.apply_to_pose(target)
        else:
            target_in_world = target

        # 发送相对位置命令
        self.robot_sdk.control.control_command_pose(
            target.pos[0], target.pos[1], target.pos[2],
            target.get_euler(degrees=False)[2]
        )
        print(f"📤 cmd_pos 发送相对位移: Δx={target.pos[0]:.3f}, Δy={target.pos[1]:.3f}, yaw={target.get_euler(degrees=True)[2]:.1f}°")

        # 等待到达目标
        tic = time.time()
        while time.time() - tic < timeout:
            success = self._check_success_walk(
                target_in_world,
                yaw_threshold=np.deg2rad(5),
                pos_threshold=pos_threshold
            )
            if success:
                return True
            time.sleep(0.1)

        print(f"❌ cmd_pos 超时 {timeout}s，未到达目标")
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

    def walk_to_pose(self,
                            # target: Pose,
                            pos_threshold=0.1,
                            kp_pos=0.5,
                            kp_yaw=0.5,
                            max_vel_x=0.5,
                            max_vel_yaw=0.4,
                            timeout=60,
                            walk_mode='cmd_vel',
                            asynchronous: bool = True):
        if asynchronous:
            # 多线程，异步
            if walk_mode == 'cmd_vel':
                fut = self._pool.submit(self._walk_to_pose_by_vel, pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, timeout)
            elif walk_mode == 'cmd_pos_world':
                fut = self._pool.submit(self._walk_to_pose_by_pose_world, pos_threshold, timeout)
            elif walk_mode == 'cmd_pos':
                fut = self._pool.submit(self._walk_to_pose_by_pose, pos_threshold, timeout)
            else:
                raise ValueError(f"无效的行走模式: {walk_mode}")

            return fut

        else:
            if walk_mode == 'cmd_vel':
                self._walk_to_pose_by_vel(pos_threshold, kp_pos, kp_yaw, max_vel_x, max_vel_yaw, timeout)
            elif walk_mode == 'cmd_pos_world':
                self._walk_to_pose_by_pose_world(pos_threshold, timeout)
            elif walk_mode == 'cmd_pos':
                self._walk_to_pose_by_pose(pos_threshold, timeout)
            else:
                raise ValueError(f"无效的行走模式: {walk_mode}")

            return None

    def stop_walk(self):
        for _ in range(10):
            self.robot_sdk.control.walk(0.0, 0.0, 0.0)
