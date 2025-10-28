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
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

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
        self.max_height = 1.8
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
        # 原有代码保持不变

        # self.robot_sdk.control.arm_reset()
        if self.arm_control_mode == "manipulation_mpc":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
        elif self.arm_control_mode == "fixed_base":
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        else:
            self.robot_sdk.control.set_fixed_arm_mode()

    def close(self):
        """
        关闭事件并重置状态。
        """
        super().close()
        self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.NoControl)
        self.robot_sdk.control.set_manipulation_mpc_control_flow(KuavoManipulationMpcControlFlow.ThroughFullBodyMpc)
        self.reset()

    def interpolate_poses(self, start_pose, end_pose, num_points=20):
        """
        在两个笛卡尔空间姿态之间进行三次样条插值。

        参数：
            start_pose: 起始KuavoPose或Pose。
            end_pose: 终点KuavoPose或Pose。
            num_points (int): 插值点数量。

        返回：
            List[KuavoPose]: 插值后的KuavoPose列表。
        """
        # 提取位置
        start_pos = np.array(start_pose.position)
        end_pos = np.array(end_pose.position)

        # 提取四元数
        start_quat = np.array(start_pose.orientation)
        end_quat = np.array(end_pose.orientation)

        # 确保四元数方向一致（避免绕远路）
        if np.dot(start_quat, end_quat) < 0:
            end_quat = -end_quat

        # 生成参数t
        t = np.linspace(0, 1, num_points)

        # 位置插值 - 使用三次样条
        # 为了进行三次样条插值，我们需要在t, x, y, z上分别拟合样条

        # 四元数插值 - 球面线性插值 (SLERP)
        interp_poses = []
        for i in range(num_points):
            # 位置插值
            pos = start_pos * (1 - t[i]) + end_pos * t[i]
            pos = (pos[0], pos[1], pos[2])

            # 四元数球面插值
            # 计算四元数之间的夹角
            cos_half_theta = np.dot(start_quat, end_quat)
            cos_half_theta = np.clip(cos_half_theta, -1.0, 1.0)  # 确保在有效范围内

            if abs(cos_half_theta) >= 1.0:
                # 如果四元数几乎相同，直接使用起始四元数
                quat = start_quat
            else:
                half_theta = np.arccos(cos_half_theta)
                sin_half_theta = np.sqrt(1.0 - cos_half_theta * cos_half_theta)

                # 如果夹角足够大，使用SLERP插值
                if abs(sin_half_theta) < 0.001:
                    # 夹角太小，使用线性插值
                    quat = start_quat * (1 - t[i]) + end_quat * t[i]
                    quat = quat / np.linalg.norm(quat)  # 归一化
                else:
                    # SLERP公式
                    ratio_a = np.sin((1 - t[i]) * half_theta) / sin_half_theta
                    ratio_b = np.sin(t[i] * half_theta) / sin_half_theta
                    quat = start_quat * ratio_a + end_quat * ratio_b
                    quat = quat / np.linalg.norm(quat)  # 归一化

            # 创建新的KuavoPose
            interp_poses.append(KuavoPose(
                position=pos,
                orientation=quat.tolist()
            ))

        return interp_poses

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
        assert mode in ["fk", "tf"], self.logger.error("mode must be 'fk' or 'tf'")

        if mode == "fk":
            # 用前向运动学计算手臂末端位置
            arm_state = self.robot_sdk.state.arm_joint_state()
            left_pose, right_pose = self.robot_sdk.arm.arm_fk(arm_state.position)


            transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)

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

        elif mode == "tf":
            # ============ 另外一种获取末端位置方式 (gazebo) =================
            # 通过TF获取手臂末端位置
            left_pose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_l7_end_effector",
                reference_frame=Frame.ODOM
            )
            right_pose: KuavoPose = self.robot_sdk.tools.get_link_pose(
                link_name="zarm_r7_end_effector",
                reference_frame=Frame.ODOM
            )

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

    def step(self):
        """
        执行事件的每一步操作。
        """
        if self.target is None:
            self.logger.error("arm_event.target is None, cannot step!")
            return EventStatus.FAILED

        status = self.get_status()
        if status != EventStatus.RUNNING:
            return status

        left_target_list, right_target_list = self.target
        current_left_target_pose = left_target_list[self.current_pose_id]
        current_right_target_pose = right_target_list[self.current_pose_id]
        # self.get_arm_pose_world()
        current_meet = self.check_current_point_meet(current_left_target_pose, current_right_target_pose)
        if not current_meet and self.current_pose_id != self.pre_pose_id:
            if self.target_wrench is not None:
                left_wrench_list, right_wrench_list = self.target_wrench
                current_left_wrench = left_wrench_list[self.current_pose_id]
                current_right_wrench = right_wrench_list[self.current_pose_id]
                self.logger.info(f"🔵 开始施加末端力 {current_left_wrench}, {current_right_wrench}")
                self.robot_sdk.arm.control_hand_wrench(
                    current_left_wrench,
                    current_right_wrench,
                )
                time.sleep(1.0)

            self.logger.info(f"🔵 开始执行关键点{self.current_pose_id + 1}")
            self.pre_pose_id = self.current_pose_id

            # 将 Pose 转换为 KuavoPose 类型
            left_start_kuavo_pose = KuavoPose(
                position=self.current_left_pose.pos,
                orientation=self.current_left_pose.quat.tolist()
            )
            left_target_kuavo_pose = KuavoPose(
                position=current_left_target_pose.pos,
                orientation=current_left_target_pose.quat.tolist()
            )
            right_start_kuavo_pose = KuavoPose(
                position=self.current_right_pose.pos,
                orientation=self.current_right_pose.quat.tolist()
            )
            right_target_kuavo_pose = KuavoPose(
                position=current_right_target_pose.pos,
                orientation=current_right_target_pose.quat.tolist()
            )

            left_arm_traj = self.interpolate_poses(left_start_kuavo_pose, left_target_kuavo_pose)
            right_arm_traj = self.interpolate_poses(right_start_kuavo_pose, right_target_kuavo_pose)

            total_time = 2.0
            num_points = min(len(left_arm_traj), len(right_arm_traj))
            time_per_point = total_time / (num_points - 1) if num_points > 1 else total_time

            for i in range(num_points):
                self.robot_sdk.control.control_robot_end_effector_pose(
                    left_pose=left_arm_traj[i],
                    right_pose=right_arm_traj[i],
                    frame=KuavoManipulationMpcFrame.WorldFrame,
                )
                if i < num_points - 1:  # 最后一个点不需要延时
                    time.sleep(time_per_point)

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

        # 直接检查整个 [left_target_list, right_target_list] 结构
        target_wrench = kwargs.get('arm_wrench', None)

        left_target_world = []
        right_target_world = []
        for left_key_pose, right_key_pose in zip(target[0], target[1]):
            if Frame.BASE == left_key_pose.frame:

                transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
                self.logger.info(f'此刻base在odom下的位置：{transform_base_to_world.trans_pose}')
                left_target_world.append(transform_base_to_world.apply_to_pose(left_key_pose))
                right_target_world.append(transform_base_to_world.apply_to_pose(right_key_pose))

            elif Frame.ODOM == left_key_pose.frame:
                left_target_world.append(left_key_pose)
                right_target_world.append(right_key_pose)

            elif Frame.TAG == left_key_pose.frame:
                tag = kwargs.get('tag', None)
                assert tag is not None, "Tag must be provided when target frame is TAG"
                transform_source_to_target = Transform3D(
                    trans_pose=tag.pose,
                    source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
                    target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
                )

                left_target_world.append(transform_source_to_target.apply_to_pose(left_key_pose))
                right_target_world.append(transform_source_to_target.apply_to_pose(right_key_pose))

        res = super().set_target(target=[left_target_world, right_target_world], *args, **kwargs)

        if res:
            # 为了应对相对位置控制的情况，记录设置目标时机器人的位姿
            self.target_wrench = target_wrench
            self.get_arm_pose_world()

        return True

    def check_current_point_meet(self, current_left_target_pose: Pose, current_right_target_pose: Pose):
        """
        检查当前点是否满足目标条件。

        参数：
            current_left_target_pose (Pose): 当前左手目标姿态。
            current_right_target_pose (Pose): 当前右手目标姿态。

        返回：
            bool: 如果满足条件返回True，否则返回False。
        """

        self.get_arm_pose_world()

        current_id = self.current_pose_id

        # print('🔵 当前左手点:', self.current_left_pose)
        # print('🔵 当前右手点:', self.current_right_pose)

        # 统一计算误差
        pos_err_left = current_left_target_pose.position_l2_norm(self.current_left_pose)
        angle_err_left = current_left_target_pose.angle(self.current_left_pose)
        pos_err_right = current_right_target_pose.position_l2_norm(self.current_right_pose)
        angle_err_right = current_right_target_pose.angle(self.current_right_pose)

        print(f"手臂误差：{pos_err_left:.4f} | {np.rad2deg(angle_err_left):.4f} | {pos_err_right:.4f} | {np.rad2deg(angle_err_right):.4f}")
        # print(f"当前左手臂位置误差: {pos_err_left:.4f} 米，角度误差: {np.rad2deg(angle_err_left):.4f} 度")
        # print(f"当前右手臂位置误差: {pos_err_right:.4f} 米，角度误差: {np.rad2deg(angle_err_right):.4f} 度")

        if pos_err_left > self.pos_threshold:
            # print(f"当前左手臂位置误差: {pos_err_left:.4f} 米，超过阈值0.15米")
            return False

        if angle_err_left > self.angle_threshold:
            # print(f"当前左手臂角度误差: {np.rad2deg(angle_err_left):.4f} 度，超过阈值10度")
            return False

        if pos_err_right > self.pos_threshold:
            # print(f"当前右手臂位置误差: {pos_err_right:.4f} 米，超过阈值0.15米")
            return False

        if angle_err_right > self.angle_threshold:
            # print(f"当前右手臂角度误差: {np.rad2deg(angle_err_right):.4f} 度，超过阈值8度")
            return False

        time.sleep(self.sub_goal_wait_time)

        # # 再计算一次，等待了之后可能会更准
        self.get_arm_pose_world()
        pos_err_left = current_left_target_pose.position_l2_norm(self.current_left_pose)
        angle_err_left = current_left_target_pose.angle(self.current_left_pose)
        pos_err_right = current_right_target_pose.position_l2_norm(self.current_right_pose)
        angle_err_right = current_right_target_pose.angle(self.current_right_pose)

        # 当前位置执行到位
        print(
            f"✅ 关键点{self.current_pose_id + 1}执行到位 | 左手误差: {pos_err_left:.4f}m, {np.rad2deg(angle_err_left):.2f}°")
        print(
            f"✅ 关键点{self.current_pose_id + 1}执行到位 | 右手误差: {pos_err_right:.4f}m, {np.rad2deg(angle_err_right):.2f}°")

        transform_base_to_world = self.get_current_transform(source_frame=Frame.BASE, target_frame=Frame.ODOM)
        print(f'此刻base在odom下的位置：{transform_base_to_world.trans_pose}')
        current_id += 1
        self.current_pose_id = current_id

        return True

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

    def _check_arm_pose_validity(self, pose: Pose, arm_name: str, point_id: int, *args, **kwargs) -> bool:
        """
        检查手臂姿态的有效性。

        参数：
            pose (Pose): 位姿。
            arm_name (str): 手臂名称。
            point_id (int): 点ID。
            `*args`: 额外的参数。
            `**kwargs`: 额外的关键字参数。

        返回：
            bool: 如果姿态有效返回True，否则返回False。
        """
        try:
            # 提前计算所有需要的转换

            # 1. 根据输入坐标系，计算odom坐标系下的位置
            if pose.frame == Frame.BASE:
                # 转换到odom坐标系
                transform_base_to_odom = self.get_current_transform(
                    source_frame=Frame.BASE,
                    target_frame=Frame.ODOM
                )
                pose_in_odom = transform_base_to_odom.apply_to_pose(pose)
                pose_in_base = pose  # 已经在base_link坐标系下
            elif pose.frame == Frame.ODOM:
                # 已经在odom坐标系下
                pose_in_odom = pose
                # 转换到base_link坐标系
                transform_odom_to_base = self.get_current_transform(
                    source_frame=Frame.ODOM,
                    target_frame=Frame.BASE
                )
                pose_in_base = transform_odom_to_base.apply_to_pose(pose)
            elif pose.frame == Frame.TAG:
                # 对于TAG坐标系，使用self.tag转换到odom坐标系
                tag: Tag = kwargs.get('tag', None)
                assert tag is not None, "Tag must be provided when target frame is TAG"
                transform_tag_to_odom = Transform3D(
                    trans_pose=tag.pose,
                    source_frame=Frame.TAG,
                    target_frame=Frame.ODOM
                )
                pose_in_odom = transform_tag_to_odom.apply_to_pose(pose)

                # 再转换到base_link坐标系
                transform_odom_to_base = self.get_current_transform(
                    source_frame=Frame.ODOM,
                    target_frame=Frame.BASE
                )
                pose_in_base = transform_odom_to_base.apply_to_pose(pose_in_odom)
            else:
                print(f"❌ 关键点{point_id} {arm_name}坐标系不支持: {pose.frame}")
                return False

            # 2. 检查高度范围（在odom坐标系下）
            height = pose_in_odom.pos[2]
            if height < self.min_height or height > self.max_height:
                print(
                    f"❌ 关键点{point_id} {arm_name}高度 {height:.2f}m 超出工作范围({self.min_height}m-{self.max_height}m)")
                return False

            # 3. 检查末端pose相对于肩关节的位置
            if arm_name == "左手臂":
                shoulder_joint_name = "zarm_l1_link"  # 左臂肩关节
            else:
                shoulder_joint_name = "zarm_r1_link"  # 右臂肩关节

            try:
                # 获取肩关节在base_link坐标系下的位置
                shoulder_pos = self.robot_sdk.tools.get_link_position(shoulder_joint_name)
                if shoulder_pos is not None:
                    # 计算末端pose相对于肩关节的位置向量（只在x,y平面上）
                    shoulder_to_end_xy = np.array([pose_in_base.pos[0] - shoulder_pos[0],
                                                   pose_in_base.pos[1] - shoulder_pos[1]])

                    # 计算相对于肩关节的水平距离（x,y平面）
                    shoulder_distance_xy = np.linalg.norm(shoulder_to_end_xy)

                    # 检查最小距离
                    if shoulder_distance_xy < self.min_reach:
                        self.logger.error(
                            f"❌ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 小于最小可达距离({self.min_reach}m)")
                        return False

                    # 检查最大距离
                    if shoulder_distance_xy > self.max_reach:
                        self.logger.error(
                            f"❌ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 超出最大可达距离({self.max_reach}m)")
                        return False

                    self.logger.info(f"✅ 关键点{point_id} {arm_name}相对于肩关节水平距离 {shoulder_distance_xy:.2f}m 在允许范围内")
                else:
                    self.logger.warn(f"⚠️ 无法获取{arm_name}肩关节位置，跳过肩关节距离检查")
            except Exception as e:
                self.logger.warn(f"⚠️ 获取{arm_name}肩关节位置出错，跳过肩关节距离检查: {e}")

            self.logger.info(f"✅ 关键点{point_id} {arm_name}位置检查通过 - 高度(odom):{height:.2f}m")
            return True

        except Exception as e:
            self.logger.error(f"❌ 关键点{point_id} {arm_name}位置检查出错: {e}")
            return False

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
        if self.current_pose_id >= len(self.target[0]):
            self.status = EventStatus.SUCCESS
            print("✅运动执行到位")
            return True

        return False

