import time
from typing import Any, Tuple, List

from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcControlFlow

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import BaseEvent, EventStatus
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoPose,
    KuavoManipulationMpcCtrlMode,
    KuavoManipulationMpcFrame)

import numpy as np
from scipy.spatial.transform import Rotation as R
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import ArmAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.utils import (
    interpolate_joint_positions_bezier,
    calculate_elbow_y,
    get_elbow_position,
    interpolate_joint_positions_cubic_spline
)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.ik_library import IKAnalytical

class EventArmMoveKeyPoint(BaseEvent):
    def __init__(self,
                 robot_sdk: RobotSDK,
                 timeout,
                 arm_control_mode: str,
                 pos_threshold: float,
                 angle_threshold: float
                 ):
        """
        初始化手臂控制事件。

        参数：
            robot_sdk (RobotSDK): 机器人SDK实例。
            timeout (float): 事件超时时间。
            arm_control_mode (str): 手臂控制模式。
            pos_threshold (float): 位置误差阈值，单位米。
            angle_threshold (float): 角度误差阈值，单位弧度。
        """
        super().__init__("EventArmMoveKeyPoint")

        self.robot_sdk = robot_sdk
        # 创建ArmAPI实例
        self.arm_api = ArmAPI(robot_sdk)

        # members
        self.target: List
        self.target_wrench: List = None
        self.current_pose_id = 0
        self.pre_pose_id = -1
        self.current_left_pose: Pose = None  # 目标位置
        self.current_right_pose: Pose = None  # 目标位置

        # params
        self.timeout = timeout  # 事件超时时间
        self.sub_goal_wait_time = 2.0  # 等待这么多时间再判断成功与否
        self.arm_control_mode = arm_control_mode  # 手臂控制模式
        self.pos_threshold = pos_threshold  # 位置误差阈值，单位米
        self.angle_threshold = angle_threshold  # 角度误差阈值，单位弧度（10度）

        # 安全范围参数（世界系）
        self.min_height = 0.3
        self.max_height = 2.2
        self.max_reach = 0.85
        self.min_reach = 0.15

    def reset(self):
        """
        重置事件状态。
        """
        self.current_pose_id = 0
        self.pre_pose_id = -1
        self.current_left_pose: Pose = None  # 目标位置
        self.current_right_pose: Pose = None  # 目标位置
        self.target = None

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

    def open(self):
        """
        开始走到指定位置事件。
        """
        super().open()
        # 重置状态，确保每次open都能正确执行轨迹
        self.reset()

        self.robot_sdk.control.arm_reset()
        self.robot_sdk.control.set_external_control_arm_mode()
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        # if self.arm_control_mode == "manipulation_mpc":
        #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        # elif self.arm_control_mode == "fixed_base":
        #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        # else:
        #     self.robot_sdk.control.set_fixed_arm_mode()

    def open_without_manipulation_mpc_mode_change(self):
        """
        开始走到指定位置事件。
        """
        super().open()
        # 重置状态，确保每次open都能正确执行轨迹
        self.reset()

    def close(self):
        """
        关闭事件并重置状态。
        """
        super().close()
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        # self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)
        self.reset()

    def close_without_manipulation_mpc_mode_change(self):
        """
        关闭事件并重置状态。
        """
        super().close()
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        # self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)
        self.reset()


    def interpolate_poses(self, start_pose, end_pose, max_speed=0.2, time_step=0.01):
        """
        基于速度控制的姿态插值
        位置使用三次样条插值，姿态使用SLERP，总时间由距离和最大速度决定

        参数：
            start_pose: 起始姿态（KuavoPose或Pose）
            end_pose: 目标姿态（KuavoPose或Pose）
            max_speed: 最大移动速度 (m/s)，默认0.2m/s
            time_step: 时间步长 (s)，默认0.05s（20Hz控制频率）

        返回：
            List[KuavoPose]: 插值后的姿态列表（含时间戳）
            List[float]: 每个姿态对应的时间戳（相对于起始时刻）
        """
        import numpy as np
        from scipy.interpolate import CubicSpline

        # 提取位置和姿态
        start_pos = np.array(start_pose.position, dtype=np.float64)
        end_pos = np.array(end_pose.position, dtype=np.float64)
        start_quat = np.array(start_pose.orientation, dtype=np.float64)
        end_quat = np.array(end_pose.orientation, dtype=np.float64)

        # 1. 计算位置距离和总时间
        pos_diff = end_pos - start_pos
        distance = np.linalg.norm(pos_diff)  # 直线距离

        if distance < 1e-6:  # 距离过近，无需运动
            return [KuavoPose(position=tuple(start_pos), orientation=start_quat.tolist())], [0.0]

        # 总时间由距离和最大速度决定（确保不超过速度限制）
        total_time = distance / max_speed
        # 生成时间戳（从0到total_time，步长time_step）
        timestamps = np.arange(0.0, total_time + time_step, time_step)
        num_points = len(timestamps)

        # 2. 位置三次样条插值（确保速度连续）
        # 构造控制点（加入起始和终止速度约束，这里设为0，确保平滑启停）
        t_control = [0.0, total_time/3, 2*total_time/3, total_time]
        # 中间点使用线性插值，确保轨迹通过起点和终点
        pos_control = np.array([
            start_pos,
            start_pos + pos_diff/3,
            start_pos + 2*pos_diff/3,
            end_pos
        ])
        # 对x, y, z分别做三次样条
        cs_x = CubicSpline(t_control, pos_control[:, 0], bc_type=((1, 0.0), (1, 0.0)))  # 始末速度为0
        cs_y = CubicSpline(t_control, pos_control[:, 1], bc_type=((1, 0.0), (1, 0.0)))
        cs_z = CubicSpline(t_control, pos_control[:, 2], bc_type=((1, 0.0), (1, 0.0)))

        # 3. 四元数SLERP插值（保持姿态平滑）
        # 修正四元数方向（取最短路径）
        if np.dot(start_quat, end_quat) < 0:
            end_quat = -end_quat

        # 4. 生成插值轨迹
        interp_poses = []
        for t in timestamps:
            # 位置插值（三次样条）
            pos = np.array([cs_x(t), cs_y(t), cs_z(t)])
            pos = tuple(pos)

            # 姿态插值（SLERP）
            t_norm = t / total_time  # 归一化到[0,1]
            cos_half_theta = np.dot(start_quat, end_quat)
            cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)

            if abs(cos_half_theta) >= 1.0:
                quat = start_quat.copy()
            else:
                half_theta = np.arccos(cos_half_theta)
                sin_half_theta = np.sqrt(1.0 - cos_half_theta**2)

                if abs(sin_half_theta) < 1e-6:
                    # 角度过小时用线性插值
                    quat = start_quat * (1 - t_norm) + end_quat * t_norm
                else:
                    # SLERP公式
                    ratio_a = np.sin((1 - t_norm) * half_theta) / sin_half_theta
                    ratio_b = np.sin(t_norm * half_theta) / sin_half_theta
                    quat = start_quat * ratio_a + end_quat * ratio_b

                quat /= np.linalg.norm(quat)  # 归一化

            interp_poses.append(KuavoPose(
                position=pos,
                orientation=quat.tolist()
            ))

        return interp_poses, timestamps

    def interpolate_cartesian_and_solve_ik(
        self,
        start_left_pose: KuavoPose,
        start_right_pose: KuavoPose,
        end_left_pose: KuavoPose,
        end_right_pose: KuavoPose,
        num_points: int = None,
        max_step: float = 0.01,
        min_points: int = 10,
        max_points: int = 100,
        model_type: str = '60',
        keep_left_joint: bool = False,
        keep_right_joint: bool = False,
        start_left_joint: List[float] = None,
        start_right_joint: List[float] = None
    ) -> List[List[float]]:
        """
        笛卡尔空间插值

        参数:
            start_left_pose: 起始左臂末端位姿
            start_right_pose: 起始右臂末端位姿
            end_left_pose: 目标左臂末端位姿
            end_right_pose: 目标右臂末端位姿
            num_points: 插值点数量，如果为None则自动计算
            max_step: 自动计算时，每个插值步允许的最大位置变化（米）
            min_points: 自动计算时的最少插值点数量
            max_points: 自动计算时的最多插值点数量
            model_type: 模型类型
            keep_left_joint: 是否保持左手关节角不变（直接使用start_left_joint）
            keep_right_joint: 是否保持右手关节角不变（直接使用start_right_joint）
            start_left_joint: 左手起始关节角（当keep_left_joint=True时使用）
            start_right_joint: 右手起始关节角（当keep_right_joint=True时使用）

        返回:
            List[List[float]]
        """
        start_left_pos = np.array(start_left_pose.position)
        start_right_pos = np.array(start_right_pose.position)
        end_left_pos = np.array(end_left_pose.position)
        end_right_pos = np.array(end_right_pose.position)

        start_left_quat = np.array(start_left_pose.orientation)
        start_right_quat = np.array(start_right_pose.orientation)
        end_left_quat = np.array(end_left_pose.orientation)
        end_right_quat = np.array(end_right_pose.orientation)

        # 确保四元数方向一致
        if np.dot(start_left_quat, end_left_quat) < 0:
            end_left_quat = -end_left_quat
        if np.dot(start_right_quat, end_right_quat) < 0:
            end_right_quat = -end_right_quat

        # 插值点数量
        if num_points is None:
            left_pos_delta = np.linalg.norm(end_left_pos - start_left_pos)
            right_pos_delta = np.linalg.norm(end_right_pos - start_right_pos)
            max_pos_delta = max(left_pos_delta, right_pos_delta)
            num_points = int(np.ceil(max_pos_delta / max_step)) + 1
            num_points = max(min_points, min(num_points, max_points))

        # 生成插值参数t
        t_values = np.linspace(0, 1, num_points)

        joint_trajectory = []

        for t in t_values:
            # 插值
            left_pos = start_left_pos * (1 - t) + end_left_pos * t
            right_pos = start_right_pos * (1 - t) + end_right_pos * t

            # SLERP插值
            def quat_slerp(q0, q1, t_val):
                """四元数球面线性插值"""
                cos_half_theta = np.dot(q0, q1)
                cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)

                if abs(cos_half_theta) >= 1.0:
                    return q0

                half_theta = np.arccos(cos_half_theta)
                sin_half_theta = np.sqrt(1.0 - cos_half_theta**2)

                if abs(sin_half_theta) < 1e-6:
                    interp_quat = q0 * (1 - t_val) + q1 * t_val
                else:
                    ratio_a = np.sin((1 - t_val) * half_theta) / sin_half_theta
                    ratio_b = np.sin(t_val * half_theta) / sin_half_theta
                    interp_quat = q0 * ratio_a + q1 * ratio_b

                return interp_quat / np.linalg.norm(interp_quat)

            left_quat = quat_slerp(start_left_quat, end_left_quat, t)
            right_quat = quat_slerp(start_right_quat, end_right_quat, t)

            left_quat = left_quat / np.linalg.norm(left_quat)
            right_quat = right_quat / np.linalg.norm(right_quat)

            # IK
            try:
                # 如果左手需要保持关节角不变，直接使用起始关节角
                if keep_left_joint and start_left_joint is not None:
                    left_joint_angles = np.array(start_left_joint)
                else:
                    left_joint_angles = IKAnalytical.compute(
                        eef_pos=left_pos.tolist(),
                        eef_quat_xyzw=left_quat.tolist(),
                        eef_frame='zarm_l7_link',
                        model_type=model_type,
                        limit=True
                    )

                # 如果右手需要保持关节角不变，直接使用起始关节角
                if keep_right_joint and start_right_joint is not None:
                    right_joint_angles = np.array(start_right_joint)
                else:
                    right_joint_angles = IKAnalytical.compute(
                        eef_pos=right_pos.tolist(),
                        eef_quat_xyzw=right_quat.tolist(),
                        eef_frame='zarm_r7_link',
                        model_type=model_type,
                        limit=True
                    )

                # 组合成14维数组
                joint_angles = np.concatenate([left_joint_angles, right_joint_angles])

                # # 镜像处理（确保双臂对称）
                # left_joint_pose = joint_angles[:7]
                # right_joint_pose = joint_angles[7:]

                # if left_joint_pose[1] > 0:
                #     # 镜像左->右
                #     right_joint_pose = np.array([
                #         left_joint_pose[0],
                #         -left_joint_pose[1],
                #         -left_joint_pose[2],
                #         left_joint_pose[3],
                #         -left_joint_pose[4],
                #         -left_joint_pose[5],
                #         left_joint_pose[6]
                #     ])
                #     # self.logger.debug(f"镜像左->右 (t={t:.3f})")
                # else:
                #     # 镜像右->左
                #     left_joint_pose = np.array([
                #         right_joint_pose[0],
                #         -right_joint_pose[1],
                #         -right_joint_pose[2],
                #         right_joint_pose[3],
                #         -right_joint_pose[4],
                #         -right_joint_pose[5],
                #         right_joint_pose[6]
                #     ])
                #     # self.logger.debug(f"镜像右->左 (t={t:.3f})")

                # # 重新组合14维关节角度数组
                # joint_angles = np.concatenate([left_joint_pose, right_joint_pose]).tolist()
                joint_trajectory.append(joint_angles)

            except Exception as e:
                self.logger.warning(f"笛卡尔空间插值点IK求解失败 (t={t:.3f}): {e}，跳过该点")
                # 如果IK失败，尝试使用上一个成功的关节角度（如果存在）
                if len(joint_trajectory) > 0:
                    joint_trajectory.append(joint_trajectory[-1])
                else:
                    # 如果第一个点就失败，使用起始关节角度
                    self.logger.error(f"第一个插值点IK求解失败，无法生成轨迹")
                    return []

        return joint_trajectory

    def util_set_arm_dof_to_work(self):
        """
        设置手臂的dof位置为双手抬起手肘弯曲姿态，用于检测零点。
        """
        joint_position_default = [
                    -0.45, 0.05, -0.27, -1.88, -0.00, 0.22, 0.53,
                    -0.45, -0.05, 0.27, -1.88, 0.00, -0.22, 0.53
                ]
        ratio = 0
        while True:
            ratio += 0.1
            ratio = np.min([ratio, 1.0])
            ratio = np.max([ratio, 0.0])
            joint_position = np.asarray(joint_position_default) * ratio
            joint_position = list(joint_position)
            self.robot_sdk.control.control_arm_joint_positions(
                joint_positions=joint_position # 手臂站立位置的关节角度，单位弧度
            )
            time.sleep(0.3)

    def util_set_arm_dof_to_zero(self):
        """
        手臂从工作姿态回到零点姿态。
        """
        joint_position_default = [
                    -0.45, 0.05, -0.27, -1.88, -0.00, 0.22, 0.53,
                    -0.45, -0.05, 0.27, -1.88, 0.00, -0.22, 0.53
                ]
        ratio = 1
        while True:
            ratio -= 0.1
            ratio = np.min([ratio, 1.0])
            ratio = np.max([ratio, 0.0])
            joint_position = np.asarray(joint_position_default) * ratio
            joint_position = list(joint_position)
            self.robot_sdk.control.control_arm_joint_positions(
                joint_positions=joint_position # 手臂站立位置的关节角度，单位弧度
            )
            time.sleep(0.3)

    def get_arm_pose_world(self, mode="tf"):
        """
        获取手臂在世界坐标系中的姿态。

        参数：
            mode (str): 获取模式，默认为"tf"。

        返回：
            Pose: 手臂的世界坐标系姿态。
        """
        from time import time
        t_start = time()

        assert mode in ["fk", "tf"], self.logger.error("mode must be 'fk' or 'tf'")

        if mode == "fk":
            # 用前向运动学计算手臂末端位置
            t_before_state = time()
            arm_state = self.robot_sdk.state.arm_joint_state()
            t_after_state = time()
            get_state_time = t_after_state - t_before_state

            t_before_fk = time()
            left_pose, right_pose = self.robot_sdk.arm.arm_fk(arm_state.position)
            t_after_fk = time()
            arm_fk_time = t_after_fk - t_before_fk

            t_before_transform = time()
            transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
            t_after_transform = time()
            get_transform_time = t_after_transform - t_before_transform

            t_before_apply = time()
            self.current_left_pose = transform_base_to_world.apply_to_pose(
                Pose(
                pos=left_pose.position,
                quat=left_pose.orientation,
                frame=Frame.BASE
            ))
            self.current_right_pose = transform_base_to_world.apply_to_pose(
                Pose(
                pos=right_pose.position,
                quat=right_pose.orientation,
                frame=Frame.BASE
            ))
            t_after_apply = time()
            apply_transform_time = t_after_apply - t_before_apply

            t_end = time()
            total_time = t_end - t_start
            self.logger.info(f"[性能分析] get_arm_pose_world (fk模式) 总耗时: {total_time:.3f}s")
            self.logger.info(f"  - arm_joint_state(): {get_state_time:.3f}s")
            self.logger.info(f"  - arm_fk(): {arm_fk_time:.3f}s")
            self.logger.info(f"  - get_current_transform(): {get_transform_time:.3f}s")
            self.logger.info(f"  - apply_to_pose(): {apply_transform_time:.3f}s")

        elif mode == "tf":
            # ============ 另外一种获取末端位置方式 (gazebo) =================
            # 通过TF获取手臂末端位置
            t_before_left_link = time()
            left_pose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_l7_end_effector",
                reference_frame=Frame.ODOM
            )
            t_after_left_link = time()
            get_left_link_time = t_after_left_link - t_before_left_link

            t_before_right_link = time()
            right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_r7_end_effector",
                reference_frame=Frame.ODOM
            )
            t_after_right_link = time()
            get_right_link_time = t_after_right_link - t_before_right_link

            t_before_create_pose = time()
            self.current_left_pose = Pose(
                pos=left_pose.position,
                quat=left_pose.orientation,
                frame=Frame.ODOM
            )
            self.current_right_pose = Pose(
                pos=right_pose.position,
                quat=right_pose.orientation,
                frame=Frame.ODOM
            )
            t_after_create_pose = time()
            create_pose_time = t_after_create_pose - t_before_create_pose

            t_end = time()
            total_time = t_end - t_start
            self.logger.info(f"[性能分析] get_arm_pose_world (tf模式) 总耗时: {total_time:.3f}s")
            self.logger.info(f"  - get_link_pose (左臂): {get_left_link_time:.3f}s")
            self.logger.info(f"  - get_link_pose (右臂): {get_right_link_time:.3f}s")
            self.logger.info(f"  - 创建 Pose 对象: {create_pose_time:.3f}s")
            self.current_right_pose = Pose(
                pos=right_pose.position,
                quat=right_pose.orientation,
                frame=Frame.ODOM
            )

    def step(self):
        """
        执行事件的每一步操作。
        使用joint控制方式，将末端位姿目标转换为关节角度轨迹并执行（参考case_test_arm.py的实现）。
        """
        if self.target is None:
            self.logger.error("arm_event.target is None, cannot step!")
            return EventStatus.FAILED

        status = self.get_status()
        if status != EventStatus.RUNNING:
            return status

        # 检查是否已经执行完成（使用左右手关键点数量的最大值）
        if self.target is not None:
            max_keypoints = max(len(self.target[0]), len(self.target[1])) if len(self.target) == 2 else len(self.target[0])
            if self.current_pose_id >= max_keypoints:
                return status

        # 只在第一次执行时生成并下发完整轨迹（确保状态已重置）
        if self.current_pose_id == 0 and self.pre_pose_id == -1:
            # 第一次执行关键点时，使用FK获取当前手臂位置
            left_target_list, right_target_list = self.target
            num_keypoints_left = len(left_target_list)
            num_keypoints_right = len(right_target_list)
            num_keypoints = max(num_keypoints_left, num_keypoints_right)  # 使用较大的数量

            arm_joint_state = self.robot_sdk.state.arm_joint_state()

            # 检查关节状态是否有效（不全为0）
            start_joint_positions = np.array(arm_joint_state.position, dtype=float) if arm_joint_state is not None else np.zeros(14)
            is_joint_state_valid = not np.allclose(start_joint_positions, 0)

            if not is_joint_state_valid:
                start_joint_positions = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
                self.logger.warning("⚠️ arm_joint_state无效，使用默认关节状态")
            else:
                self.logger.info(f"🔵 arm_joint_state有效，使用关节状态: {start_joint_positions}")

            # 获取坐标系变换（ODOM到BASE）
            transform_odom_to_base = self.get_current_transform(
                source_frame=Frame.ODOM,
                target_frame=Frame.BASE
            )

            # FK获取当前末端位置
            if is_joint_state_valid:
                try:
                    model_type = '60'

                    # l1_link在base坐标系中的位置（根据model_type）
                    l1_link_map = {
                        '45': [-0.017, 0.293, 0.424],
                        '46': [-0.0174999, 0.2927, 0.445],
                        '60': [-0.018, 0.255, 0.192],
                    }
                    l1_link_in_base = np.array(l1_link_map[model_type])

                    # 提取左右臂关节角度
                    left_joint_angles = start_joint_positions[:7]
                    right_joint_angles = start_joint_positions[7:]

                    # 左臂FK
                    X_07_left = IKAnalytical.fk_leftarm(left_joint_angles)
                    left_pos_in_base = X_07_left[:3, 3] + l1_link_in_base
                    left_rot_matrix = X_07_left[:3, :3]
                    left_quat_xyzw = R.from_matrix(left_rot_matrix).as_quat()  # [x, y, z, w]

                    # 右臂FK
                    right_joint_angles_mirrored = np.array([
                        right_joint_angles[0],
                        -right_joint_angles[1],
                        -right_joint_angles[2],
                        right_joint_angles[3],
                        -right_joint_angles[4],
                        -right_joint_angles[5],
                        right_joint_angles[6]
                    ])
                    # FK
                    X_07_right_mirrored = IKAnalytical.fk_leftarm(right_joint_angles_mirrored)
                    # 右臂l1_link在base坐标系中的位置（y坐标取反）
                    l1_link_in_base_right = np.array([l1_link_in_base[0], -l1_link_in_base[1], l1_link_in_base[2]])
                    right_pos_in_base_mirrored = X_07_right_mirrored[:3, 3] + l1_link_in_base
                    right_pos_in_base = np.array([
                        right_pos_in_base_mirrored[0],
                        -right_pos_in_base_mirrored[1],
                        right_pos_in_base_mirrored[2]
                    ])
                    right_euler_mirrored = R.from_matrix(X_07_right_mirrored[:3, :3]).as_euler('xyz')
                    right_euler = np.array([-right_euler_mirrored[0], right_euler_mirrored[1], -right_euler_mirrored[2]])
                    right_quat_xyzw = R.from_euler('xyz', right_euler).as_quat()  # [x, y, z, w]

                    current_left_pose_base = Pose(
                        pos=tuple(left_pos_in_base),
                        quat=left_quat_xyzw.tolist(),
                        frame=Frame.BASE
                    )
                    current_right_pose_base = Pose(
                        pos=tuple(right_pos_in_base),
                        quat=right_quat_xyzw.tolist(),
                        frame=Frame.BASE
                    )
                    self.logger.info("✅ fk获取当前位置")
                except Exception as e:
                    self.logger.warning(f"⚠️ FK计算失败，回退: {e}")
                    import traceback
                    self.logger.debug(traceback.format_exc())
                    try:
                        left_pose_fk, right_pose_fk = self.robot_sdk.arm.arm_fk(start_joint_positions)
                        current_left_pose_base = Pose(
                            pos=left_pose_fk.position,
                            quat=left_pose_fk.orientation,
                            frame=Frame.BASE
                        )
                        current_right_pose_base = Pose(
                            pos=right_pose_fk.position,
                            quat=right_pose_fk.orientation,
                            frame=Frame.BASE
                        )
                        self.logger.info("✅ fk获取当前位置")
                    except Exception as e2:
                        self.logger.warning(f"⚠️ fk失败，回退到TF模式: {e2}")
                        # 如果都失败，回退到TF模式
                        self.get_arm_pose_world(mode="tf")
                        current_left_pose_base = transform_odom_to_base.apply_to_pose(self.current_left_pose)
                        current_right_pose_base = transform_odom_to_base.apply_to_pose(self.current_right_pose)
            else:
                # 如果关节状态无效，使用TF模式
                self.get_arm_pose_world(mode="tf")
                current_left_pose_base = transform_odom_to_base.apply_to_pose(self.current_left_pose)
                current_right_pose_base = transform_odom_to_base.apply_to_pose(self.current_right_pose)

            current_left_kuavo_pose = KuavoPose(
                position=list(current_left_pose_base.pos),
                orientation=list(current_left_pose_base.quat)
            )
            current_right_kuavo_pose = KuavoPose(
                position=list(current_right_pose_base.pos),
                orientation=list(current_right_pose_base.quat)
            )

            # 生成关节轨迹
            joint_traj = []
            model_type = '60'

            # 第一个轨迹点直接使用当前关节状态
            if is_joint_state_valid:
                joint_traj.append(start_joint_positions.tolist())

            # 遍历所有关键点，在笛卡尔空间进行插值
            for keypoint_idx in range(num_keypoints):
                # 处理左手目标位置
                if keypoint_idx < num_keypoints_left:
                    # 左手还有关键点，使用目标位置
                    target_left_pose_obj = left_target_list[keypoint_idx]
                    if target_left_pose_obj.frame == Frame.BASE:
                        left_pose_in_base = target_left_pose_obj
                    else:
                        left_pose_in_base = transform_odom_to_base.apply_to_pose(target_left_pose_obj)
                    left_target_kuavo_pose = KuavoPose(
                        position=list(left_pose_in_base.pos),
                        orientation=list(left_pose_in_base.quat)
                    )
                else:
                    # 左手关键点用完了，使用当前位姿（保持不动）
                    left_target_kuavo_pose = current_left_kuavo_pose
                    self.logger.info(f"🔵 左手关键点已用完（共{num_keypoints_left}个），第{keypoint_idx + 1}个关键点使用当前关节角保持不动")

                # 处理右手目标位置
                if keypoint_idx < num_keypoints_right:
                    # 右手还有关键点，使用目标位置
                    target_right_pose_obj = right_target_list[keypoint_idx]
                    if target_right_pose_obj.frame == Frame.BASE:
                        right_pose_in_base = target_right_pose_obj
                    else:
                        right_pose_in_base = transform_odom_to_base.apply_to_pose(target_right_pose_obj)
                    right_target_kuavo_pose = KuavoPose(
                        position=list(right_pose_in_base.pos),
                        orientation=list(right_pose_in_base.quat)
                    )
                else:
                    # 右手关键点用完了，使用当前位姿（保持不动）
                    right_target_kuavo_pose = current_right_kuavo_pose
                    self.logger.info(f"🔵 右手关键点已用完（共{num_keypoints_right}个），第{keypoint_idx + 1}个关键点使用当前关节角保持不动")

                # 在笛卡尔空间进行插值，然后对每个插值点进行IK求解
                try:
                    # 判断哪只手的关键点用完了，需要保持关节角不变
                    keep_left_joint = (keypoint_idx >= num_keypoints_left)
                    keep_right_joint = (keypoint_idx >= num_keypoints_right)

                    # 获取当前关节角（用于保持不动的手）
                    if keep_left_joint or keep_right_joint:
                        if len(joint_traj) > 0:
                            last_joint_angles = np.array(joint_traj[-1])
                            current_left_joint = last_joint_angles[:7].tolist()
                            current_right_joint = last_joint_angles[7:].tolist()
                        elif is_joint_state_valid:
                            current_left_joint = start_joint_positions[:7].tolist()
                            current_right_joint = start_joint_positions[7:].tolist()
                        else:
                            current_left_joint = None
                            current_right_joint = None
                    else:
                        current_left_joint = None
                        current_right_joint = None

                    segment_trajectory = self.interpolate_cartesian_and_solve_ik(
                        start_left_pose=current_left_kuavo_pose,
                        start_right_pose=current_right_kuavo_pose,
                        end_left_pose=left_target_kuavo_pose,
                        end_right_pose=right_target_kuavo_pose,
                        num_points=None,  # 自动计算
                        max_step=0.01,  # 每个插值步最大5cm
                        min_points=50,
                        max_points=500,
                        model_type=model_type,
                        keep_left_joint=keep_left_joint,
                        keep_right_joint=keep_right_joint,
                        start_left_joint=current_left_joint,
                        start_right_joint=current_right_joint
                    )

                    if len(segment_trajectory) == 0:
                        self.logger.error(f"❌ 关键点{keypoint_idx + 1} 笛卡尔空间插值轨迹生成失败，跳过")
                        break

                    # 连接轨迹，确保段之间连续
                    if keypoint_idx == 0:
                        # 跳过第一个点
                        if is_joint_state_valid and len(segment_trajectory) > 1:
                            points_to_append = segment_trajectory[1:]
                        else:
                            points_to_append = segment_trajectory
                    else:
                        # 使用上一段最后一个点的关节角度作为起始点
                        if len(joint_traj) > 0:
                            last_joint_angles = np.array(joint_traj[-1])
                            if len(segment_trajectory) > 1:
                                segment_trajectory[0] = last_joint_angles.tolist()
                                points_to_append = segment_trajectory[1:]
                            else:
                                segment_trajectory[0] = last_joint_angles.tolist()
                                points_to_append = segment_trajectory
                        else:
                            points_to_append = segment_trajectory[1:] if len(segment_trajectory) > 1 else []

                    if len(points_to_append) > 0:
                        joint_traj.extend(points_to_append)

                    # 更新当前末端位姿
                    if len(joint_traj) > 0:
                        last_joint_angles = np.array(joint_traj[-1])
                        try:
                            # FK
                            left_joint_angles = last_joint_angles[:7]
                            right_joint_angles = last_joint_angles[7:]

                            l1_link_map = {
                                '45': [-0.017, 0.293, 0.424],
                                '46': [-0.0174999, 0.2927, 0.445],
                                '60': [-0.018, 0.255, 0.192],
                            }
                            l1_link_in_base = np.array(l1_link_map[model_type])

                            # 左臂FK
                            X_07_left = IKAnalytical.fk_leftarm(left_joint_angles)
                            left_pos_in_base = X_07_left[:3, 3] + l1_link_in_base
                            left_rot_matrix = X_07_left[:3, :3]
                            left_quat_xyzw = R.from_matrix(left_rot_matrix).as_quat()

                            # 右臂FK
                            right_joint_angles_mirrored = np.array([
                                right_joint_angles[0],
                                -right_joint_angles[1],
                                -right_joint_angles[2],
                                right_joint_angles[3],
                                -right_joint_angles[4],
                                -right_joint_angles[5],
                                right_joint_angles[6]
                            ])
                            X_07_right_mirrored = IKAnalytical.fk_leftarm(right_joint_angles_mirrored)
                            right_pos_in_base_mirrored = X_07_right_mirrored[:3, 3] + l1_link_in_base
                            right_pos_in_base = np.array([
                                right_pos_in_base_mirrored[0],
                                -right_pos_in_base_mirrored[1],
                                right_pos_in_base_mirrored[2]
                            ])
                            right_euler_mirrored = R.from_matrix(X_07_right_mirrored[:3, :3]).as_euler('xyz')
                            right_euler = np.array([-right_euler_mirrored[0], right_euler_mirrored[1], -right_euler_mirrored[2]])
                            right_quat_xyzw = R.from_euler('xyz', right_euler).as_quat()

                            # 更新当前末端位姿
                            current_left_kuavo_pose = KuavoPose(
                                position=left_pos_in_base.tolist(),
                                orientation=left_quat_xyzw.tolist()
                            )
                            current_right_kuavo_pose = KuavoPose(
                                position=right_pos_in_base.tolist(),
                                orientation=right_quat_xyzw.tolist()
                            )
                        except Exception as e:
                            self.logger.warning(f"⚠️ 使用FK更新当前位姿失败，使用目标位姿: {e}")
                            # 如果FK失败，使用目标位姿
                            current_left_kuavo_pose = left_target_kuavo_pose
                            current_right_kuavo_pose = right_target_kuavo_pose
                    else:
                        # 如果没有轨迹点，使用目标位姿
                        current_left_kuavo_pose = left_target_kuavo_pose
                        current_right_kuavo_pose = right_target_kuavo_pose

                except Exception as e:
                    self.logger.error(f"❌ 关键点{keypoint_idx + 1} 笛卡尔空间插值失败: {e}，跳过关节轨迹生成")
                    import traceback
                    self.logger.debug(traceback.format_exc())
                    break

            if len(joint_traj) == 0:
                self.logger.error("❌ 关节轨迹生成失败")
                return EventStatus.FAILED

            self.logger.info(f"🔵 [步骤5完成] 完整关节轨迹生成完成，总点数: {len(joint_traj)}")

            # 设置末端力（如果有）
            if self.target_wrench is not None:
                left_wrench_list, right_wrench_list = self.target_wrench
                # 使用最后一个关键点的力
                final_left_wrench = left_wrench_list[-1] if len(left_wrench_list) > 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                final_right_wrench = right_wrench_list[-1] if len(right_wrench_list) > 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.robot_sdk.arm.control_hand_wrench(
                    final_left_wrench,
                    final_right_wrench,
                )

            # 计算总时间（根据轨迹点数和速度估算）
            # 假设每个点间隔0.01秒（100Hz），可以根据需要调整
            total_time = len(joint_traj) * 0.03

            self.arm_api.move_joint_traj(
                joint_traj=joint_traj,
                asynchronous=False,
                total_time=total_time
            )

            # self.get_arm_pose_world()
            # actual_left_pos = self.current_left_pose.pos
            # actual_right_pos = self.current_right_pose.pos
            # final_target_left = left_target_list[-1].pos
            # final_target_right = right_target_list[-1].pos
            # self.logger.info(f"🔵 执行后实际位置(ODOM): left={actual_left_pos}, right={actual_right_pos}")
            # self.logger.info(f"🔵 最终目标位置(ODOM): left={final_target_left}, right={final_target_right}")
            # pos_err_left = np.linalg.norm(np.array(actual_left_pos) - np.array(final_target_left))
            # pos_err_right = np.linalg.norm(np.array(actual_right_pos) - np.array(final_target_right))
            # self.logger.info(f"🔵 位置误差: left={pos_err_left:.4f}m, right={pos_err_right:.4f}m")

            # 标记所有关键点已完成
            self.pre_pose_id = num_keypoints - 1
            self.current_pose_id = num_keypoints

        return status

    def arm_reset(self):
        """
        重置手臂到初始状态。
        """
        self.robot_sdk.control.arm_reset()
        time.sleep(3.0)
        self.logger.info("🔵 手臂已复位")

    def set_target(self, target: Any, *args, **kwargs):
        """
        设置事件的目标。

        参数：
            target (Any): 目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标设置成功返回True，否则返回False。
        """
        from time import time
        t_start = time()

        # 直接检查整个 [left_target_list, right_target_list] 结构
        target_wrench = kwargs.get('arm_wrench', None)

        left_target_world = []
        right_target_world = []

        t_before_loop = time()
        transform_base_to_world = None  # 缓存变换，避免重复获取
        get_transform_time = 0.0
        apply_transform_time = 0.0

        # 分别处理左右手臂的坐标系转换（支持左右手关键点数量不一致）
        left_target_list = target[0]
        right_target_list = target[1]
        max_keypoints = max(len(left_target_list), len(right_target_list))

        for keypoint_idx in range(max_keypoints):
            # 处理左臂坐标系转换
            if keypoint_idx < len(left_target_list):
                left_key_pose = left_target_list[keypoint_idx]
                if left_key_pose.frame == Frame.BASE:
                    t_before_get_tf = time()
                    transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
                    t_after_get_tf = time()
                    get_transform_time += (t_after_get_tf - t_before_get_tf)
                    t_before_apply = time()
                    left_target_world_pose = transform_base_to_world.apply_to_pose(left_key_pose)
                    t_after_apply = time()
                    apply_transform_time += (t_after_apply - t_before_apply)
                elif left_key_pose.frame == Frame.ODOM:
                    left_target_world_pose = left_key_pose
                elif left_key_pose.frame == Frame.TAG:
                    tag = kwargs.get('tag', None)
                    assert tag is not None, "Tag must be provided when target frame is TAG"
                    transform_tag_to_world = Transform3D(
                        trans_pose=tag.pose,
                        source_frame=Frame.TAG,
                        target_frame=Frame.ODOM
                    )
                    left_target_world_pose = transform_tag_to_world.apply_to_pose(left_key_pose)
                else:
                    raise ValueError(f"Unsupported frame for left arm: {left_key_pose.frame}")
                left_target_world.append(left_target_world_pose)
            # 如果左手关键点用完了，不添加（在step中会用当前关节角补全）

            # 处理右臂坐标系转换
            if keypoint_idx < len(right_target_list):
                right_key_pose = right_target_list[keypoint_idx]
                if right_key_pose.frame == Frame.BASE:
                    t_before_get_tf = time()
                    transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
                    t_after_get_tf = time()
                    get_transform_time += (t_after_get_tf - t_before_get_tf)
                    t_before_apply = time()
                    right_target_world_pose = transform_base_to_world.apply_to_pose(right_key_pose)
                    t_after_apply = time()
                    apply_transform_time += (t_after_apply - t_before_apply)
                elif right_key_pose.frame == Frame.ODOM:
                    right_target_world_pose = right_key_pose
                elif right_key_pose.frame == Frame.TAG:
                    tag = kwargs.get('tag', None)
                    assert tag is not None, "Tag must be provided when target frame is TAG"
                    transform_tag_to_world = Transform3D(
                        trans_pose=tag.pose,
                        source_frame=Frame.TAG,
                        target_frame=Frame.ODOM
                    )
                    right_target_world_pose = transform_tag_to_world.apply_to_pose(right_key_pose)
                else:
                    raise ValueError(f"Unsupported frame for right arm: {right_key_pose.frame}")
                right_target_world.append(right_target_world_pose)
            # 如果右手关键点用完了，不添加（在step中会用当前关节角补全）

        t_after_loop = time()
        loop_time = t_after_loop - t_before_loop

        t_before_super = time()
        res = super().set_target(target=[left_target_world, right_target_world], *args, **kwargs)
        t_after_super = time()
        super_set_target_time = t_after_super - t_before_super

        t_before_print = time()
        print(f"🔵 设置目标: {left_target_world}, {right_target_world}")
        t_after_print = time()
        print_time = t_after_print - t_before_print

        t_before_get_pose = time()
        if res:
            # 为了应对相对位置控制的情况，记录设置目标时机器人的位姿
            self.target_wrench = target_wrench
            self.get_arm_pose_world()
        t_after_get_pose = time()
        get_arm_pose_time = t_after_get_pose - t_before_get_pose

        t_end = time()
        total_time = t_end - t_start

        # 输出性能分析
        self.logger.info(f"[性能分析] EventArmMoveKeyPoint.set_target 总耗时: {total_time:.3f}s")
        self.logger.info(f"  - 坐标系转换循环: {loop_time:.3f}s")
        self.logger.info(f"    - get_current_transform (TF查询): {get_transform_time:.3f}s")
        self.logger.info(f"    - apply_to_pose (坐标变换计算): {apply_transform_time:.3f}s")
        self.logger.info(f"  - super().set_target(): {super_set_target_time:.3f}s")
        self.logger.info(f"  - print(): {print_time:.3f}s")
        self.logger.info(f"  - get_arm_pose_world(): {get_arm_pose_time:.3f}s")
        other_time = total_time - loop_time - super_set_target_time - print_time - get_arm_pose_time
        self.logger.info(f"  - 其他操作: {other_time:.3f}s")

        return True

    def _check_failed(self):
        """
        检查事件是否失败。

        返回：
            bool: 如果事件失败返回True，否则返回False。
        """
        return False

    def _check_success(self):
        """
        检查事件是否成功。

        返回：
            bool: 如果事件成功返回True，否则返回False。
        """
        # 使用左右手关键点数量的最大值来判断是否完成
        if self.target is not None and len(self.target) == 2:
            max_keypoints = max(len(self.target[0]), len(self.target[1]))
            if self.current_pose_id >= max_keypoints:
                self.status = EventStatus.SUCCESS
                print("✅运动执行到位")
                return True
        elif self.target is not None:
            if self.current_pose_id >= len(self.target[0]):
                self.status = EventStatus.SUCCESS
                print("✅运动执行到位")
                return True

        return False

    def _check_target_valid(self, target, *args, **kwargs) -> bool:
        """
        检查目标是否有效。

        参数：
            target: 目标。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果目标有效返回True，否则返回False。
        """
        # 如果是 [list, list]，递归检查
        target_wrench = kwargs.get('arm_wrench', None)

        if isinstance(target, (list, tuple)) and len(target) == 2:
            left_list, right_list = target

            if not (isinstance(left_list, list) and isinstance(right_list, list)):
                print("❌目标不是两个列表")
                return False

            if target_wrench is not None:
                if not isinstance(target_wrench, (list, tuple)):
                    # print(target_wrench)
                    print("❌目标力矩必须是列表的列表")
                    return False
                left_wrench, right_wrench = target_wrench
                if len(left_wrench) != len(left_list):
                    print("❌左手力矩列表长度与关键点列表长度不匹配")
                    return False
                if len(right_wrench) != len(right_list):
                    print("❌右手力矩列表长度与关键点列表长度不匹配")
                    return False

            # 检查每个关键点的有效性和可达性
            for i, (left_pose, right_pose) in enumerate(zip(left_list, right_list)):
                if not isinstance(left_pose, Pose) or not isinstance(right_pose, Pose):
                    print(f"关键点{i + 1}不是Pose对象: {type(left_pose)}, {type(right_pose)}")
                    return False

                # 检查坐标系有效性
                if left_pose.frame not in [Frame.ODOM, Frame.BASE, Frame.TAG]:
                    print(f"关键点{i + 1}左手臂坐标系无效: {left_pose.frame}")
                    return False
                if right_pose.frame not in [Frame.ODOM, Frame.BASE, Frame.TAG]:
                    print(f"关键点{i + 1}右手臂坐标系无效: {right_pose.frame}")
                    return False

                # 检查位置有效性和可达性
                if not self._check_arm_pose_validity(left_pose, "左手臂", i + 1, tag=kwargs.get('tag', None)):
                    return False
                if not self._check_arm_pose_validity(right_pose, "右手臂", i + 1, tag=kwargs.get('tag', None)):
                    return False

            return True
        # 单个 Pose 检查
        if not isinstance(target, Pose):
            print("❌目标位置必须是Pose对象")
            return False
        if target.frame not in [Frame.ODOM, Frame.BASE]:
            print("❌目标位姿的坐标系必须是'base_link'或'odom'")
            return False
        return True