from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import transform_pose_from_tag_to_world, ArmAPI, TorsoAPI, \
    HeadAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.utils import generate_full_bezier_trajectory, \
    interpolate_joint_positions_bezier, calculate_elbow_y, get_elbow_position

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from typing import List
import time
import numpy as np


# ---------------- 白板和config管理 -------------------


# class NodeHead(Behaviour):
#     def __init__(self,
#                  name,
#                  head_api: HeadAPI
#                  ):
#         super(NodeHead, self).__init__(name)
#
#     def setup(self):
#         self.logger.debug(f"NodeHead::setup {self.name}")
#
#     def initialise(self):
#         self.logger.debug(f"NodeHead::initialise {self.name}")
#
#     def update(self):
#         self.logger.debug(f"NodeHead::update {self.name}")
#         return Status.SUCCESS
#
#     def terminate(self, new_status):
#         self.logger.debug(f"NodeHead::terminate {self.name} to {new_status}")

# --------------- 转换节点 -----------------
class NodeTagToArmGoal(Behaviour):
    def __init__(self,
                 name,
                 arm_api: ArmAPI,
                 tag_id: int,
                 left_arm_relative_keypoints,
                 right_arm_relative_keypoints,
                 control_type: str = 'eef_world',  # joint 是关节轨迹，eef_world/eef_base是末端轨迹
                 enable_joint_mirroring: bool = True,  # 是否启用关节镜像处理
                 ):
        assert control_type in ['eef_world', 'eef_base', 'joint'], "control_type must be 'eef_world' or 'eef_base' or 'joint'"
        super(NodeTagToArmGoal, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)

        # 只读
        for k in [f'latest_tag_{tag_id}', f'latest_tag_{tag_id}_version']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        # 读写
        for k in ['left_arm_eef_traj', 'right_arm_eef_traj',
                  'left_arm_joint_traj', 'right_arm_joint_traj']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        self.arm_api = arm_api
        self.tag_id = tag_id
        self.tag_version = -1
        self.control_type = control_type
        self.enable_joint_mirroring = enable_joint_mirroring

        self.left_arm_relative_keypoints = left_arm_relative_keypoints
        self.right_arm_relative_keypoints = right_arm_relative_keypoints

    def initialise(self):
        self.logger.debug(f"NodeTagToArmGoal::initialise {self.name}")
        print(f'===== Initializing NodeTagToArmGoal for tag {self.tag_id}')
        self.tag_version = -1

    def update(self):
        self.logger.debug(f"NodeTagToArmGoal::update {self.name}")
        latest_tag = getattr(self.bb, f"latest_tag_{self.tag_id}", None)
        tag_version = getattr(self.bb, f"latest_tag_{self.tag_id}_version", None)
        print(f'===== latest_tag version {tag_version} found, current version {self.tag_version}')
        if latest_tag is None or tag_version is None:
            return Status.RUNNING

        elif tag_version <= self.tag_version:
            return Status.RUNNING

        # TODO： 在这里把tag转换成手臂轨迹
        left_targets = []
        right_targets = []

        print('===== Generating arm trajectory based on tag and keypoints')
        for left_key_pose, right_key_pose in zip(self.left_arm_relative_keypoints, self.right_arm_relative_keypoints):
            assert left_key_pose.frame in [Frame.ODOM, Frame.BASE, Frame.TAG], \
                self.logger.error(
                    "在全局控制模式下，left_key_pose.frame must be Frame.ODOM, Frame.BASE or Frame.TAG")
            assert right_key_pose.frame in [Frame.ODOM, Frame.BASE, Frame.TAG], \
                self.logger.error(
                    "在全局控制模式下，right_key_pose.frame must be Frame.ODOM, Frame.BASE or Frame.TAG")
            if Frame.BASE == left_key_pose.frame:

                transform_base_to_world = self.arm_api.get_current_transform(source_frame=Frame.BASE,
                                                                             target_frame=Frame.ODOM)
                left_targets.append(transform_base_to_world.apply_to_pose(left_key_pose))
                right_targets.append(transform_base_to_world.apply_to_pose(right_key_pose))

            elif Frame.ODOM == left_key_pose.frame:
                left_targets.append(left_key_pose)
                right_targets.append(right_key_pose)

            elif Frame.TAG == left_key_pose.frame:
                tag = latest_tag
                transform_source_to_target = Transform3D(
                    trans_pose=tag.pose,
                    source_frame=Frame.TAG,  # 源坐标系为Tag坐标系
                    target_frame=Frame.ODOM  # 目标坐标系为里程计坐标系
                )

                left_targets.append(transform_source_to_target.apply_to_pose(left_key_pose))
                right_targets.append(transform_source_to_target.apply_to_pose(right_key_pose))

        # ===== 2. 根据控制模式生成对应轨迹 =====
        if self.control_type in ['eef_world', 'eef_base']:
            left_eef_pose_world, right_eef_pose_world = self.arm_api.get_eef_pose_world()
            left_bezier_trajectory, right_bezier_trajectory = generate_full_bezier_trajectory(
                current_left_pose=left_eef_pose_world,
                current_right_pose=right_eef_pose_world,
                left_keypoints_list=left_targets,
                right_keypoints_list=right_targets,
            )

            self.bb.left_arm_eef_traj = left_bezier_trajectory
            self.bb.right_arm_eef_traj = right_bezier_trajectory
            # 清空旧的关节轨迹，避免被其他节点误用
            self.bb.left_arm_joint_traj = None
            self.bb.right_arm_joint_traj = None

        elif self.control_type == 'joint':
            robot_sdk = self.arm_api.robot_sdk
            start_joint_positions = np.array(robot_sdk.state.arm_joint_state().position, dtype=float)
            current_joints = start_joint_positions.copy()

            transform_odom_to_base = self.arm_api.get_current_transform(
                source_frame=Frame.ODOM,
                target_frame=Frame.BASE
            )

            left_joint_traj = []
            right_joint_traj = []

            for idx, (left_pose_world, right_pose_world) in enumerate(zip(left_targets, right_targets)):
                if left_pose_world.frame == Frame.BASE:
                    left_pose_in_base = left_pose_world
                    right_pose_in_base = right_pose_world
                else:
                    left_pose_in_base = transform_odom_to_base.apply_to_pose(left_pose_world)
                    right_pose_in_base = transform_odom_to_base.apply_to_pose(right_pose_world)

                left_target_kuavo_pose = KuavoPose(
                    position=list(left_pose_in_base.pos),
                    orientation=list(left_pose_in_base.quat)
                )
                right_target_kuavo_pose = KuavoPose(
                    position=list(right_pose_in_base.pos),
                    orientation=list(right_pose_in_base.quat)
                )

                left_elbow_y = calculate_elbow_y(left_target_kuavo_pose.position[1], True)
                right_elbow_y = calculate_elbow_y(right_target_kuavo_pose.position[1], False)

                left_elbow = get_elbow_position(
                    robot_sdk, "zarm_l4_link", left_elbow_y, True, logger=self.logger)
                right_elbow = get_elbow_position(
                    robot_sdk, "zarm_r4_link", right_elbow_y, False, logger=self.logger)

                target_joint_positions = None
                for retry in range(5):
                    if retry > 0:
                        left_elbow[1] -= 0.02
                        right_elbow[1] += 0.02

                    target_joint_positions = robot_sdk.arm.arm_ik(
                        left_pose=left_target_kuavo_pose,
                        right_pose=right_target_kuavo_pose,
                        left_elbow_pos_xyz=left_elbow,
                        right_elbow_pos_xyz=right_elbow,
                        arm_q0=current_joints.tolist()
                    )
                    if target_joint_positions is not None:
                        break

                if target_joint_positions is None:
                    self.logger.error(f"❌ 关键点 {idx + 1} 逆解失败，跳过关节轨迹生成")
                    left_joint_traj.clear()
                    right_joint_traj.clear()
                    break

                target_joint_positions = np.array(target_joint_positions, dtype=float)

                # === 镜像处理逻辑 ===
                if self.enable_joint_mirroring:
                    left_joint_pose = target_joint_positions[:7]
                    right_joint_pose = target_joint_positions[7:]
                    
                    self.logger.debug(f"IK求解结果 - 左臂: {left_joint_pose}")
                    self.logger.debug(f"IK求解结果 - 右臂: {right_joint_pose}")

                    if(left_joint_pose[1] > 0):
                        self.logger.debug("镜像左->右")
                        right_joint_pose = (left_joint_pose[0], -left_joint_pose[1],
                                        -left_joint_pose[2], left_joint_pose[3],
                                        -left_joint_pose[4], -left_joint_pose[5], left_joint_pose[6])
                    else:
                        self.logger.debug("镜像右->左")
                        left_joint_pose = (right_joint_pose[0], -right_joint_pose[1],
                                        -right_joint_pose[2], right_joint_pose[3],
                                        -right_joint_pose[4], -right_joint_pose[5], right_joint_pose[6])
                    
                    # 重新组合14维关节角度数组
                    target_joint_positions = np.array(list(left_joint_pose) + list(right_joint_pose))
                    self.logger.debug(f"镜像后关节角度: {target_joint_positions}")
                else:
                    self.logger.debug("跳过关节镜像处理 - enable_joint_mirroring=False")
                    self.logger.debug(f"使用原始IK求解结果: {target_joint_positions}")

                segment_trajectory = interpolate_joint_positions_bezier(
                    current_joints.tolist(),
                    target_joint_positions.tolist(),
                    num_points=20
                )

                points_to_append = segment_trajectory if idx == 0 else segment_trajectory[1:]
                for joint_point in points_to_append:
                    left_joint_traj.append(joint_point[:7])
                    right_joint_traj.append(joint_point[7:])

                current_joints = target_joint_positions

            if left_joint_traj and right_joint_traj:
                self.bb.left_arm_joint_traj = left_joint_traj
                self.bb.right_arm_joint_traj = right_joint_traj
            # 清空旧的末端轨迹
            self.bb.left_arm_eef_traj = None
            self.bb.right_arm_eef_traj = None

        self.tag_version = getattr(self.bb, f"latest_tag_{self.tag_id}_version", 0)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeTagToArmGoal::terminate {self.name} to {new_status}")


class NodeTagToNavGoal(Behaviour):
    def __init__(self,
                 name,
                 tag_id,
                 stand_in_tag_pos,
                 stand_in_tag_euler,
                 ):
        super(NodeTagToNavGoal, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)
        # 只读
        for k in [f'latest_tag_{tag_id}', f'latest_tag_{tag_id}_version']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        #  读写
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        # TODO: 支持从白板读取config
        self.tag_id = tag_id
        self.tag_version = 0
        self.stand_in_tag_pos = stand_in_tag_pos
        self.stand_in_tag_euler = stand_in_tag_euler

    def initialise(self):
        self.logger.debug(f"NodeTagToNavGoal::initialise {self.name}")
        self.tag_version = -1

    def update(self):
        self.logger.debug(f"NodeTagToNavGoal::update {self.name}")
        latest_tag = getattr(self.bb, f"latest_tag_{self.tag_id}", None)
        tag_version = getattr(self.bb, f"latest_tag_{self.tag_id}_version", None)
        print(f'===== latest_tag version {tag_version} found')
        if latest_tag is None or tag_version is None:
            return Status.RUNNING

        elif tag_version <= self.tag_version:
            return Status.RUNNING

        stand_pose_in_tag = Pose.from_euler(
            pos=self.stand_in_tag_pos,  # 站立位置在目标标签中的位置猜测，单位米
            euler=self.stand_in_tag_euler,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
            frame=Frame.TAG,  # 使用标签坐标系
            degrees=False
        )

        stand_pose_in_world = transform_pose_from_tag_to_world(latest_tag, stand_pose_in_tag)
        self.bb.walk_goal = stand_pose_in_world
        self.bb.is_walk_goal_new = True
        print(f'===== setting walk goal to tag {self.tag_id} at {stand_pose_in_world}')
        self.tag_version = tag_version
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeTagToNavGoal::terminate {self.name} to {new_status}")


# ---------------- 条件节点 -------------------
class NodeWaitForBlackboard(py_trees.behaviour.Behaviour):
    def __init__(self, key, name=None,
                 timeout: float = None):
        super().__init__(name or f"WaitFor({key})")
        self.key = key
        self.timeout = timeout
        self.start_t = None
        self.bb = py_trees.blackboard.Client(name=f"{self.name}/reader")
        self.bb.register_key(key=self.key, access=py_trees.common.Access.READ)

    def initialise(self):
        self.start_t = time.time()

    def update(self):
        # 读不到键会抛 KeyError：当作“还没准备好”
        try:
            val = getattr(self.bb, self.key)
            if val is not None:
                return Status.SUCCESS
        except KeyError:
            pass

        # 超时就失败；否则继续等待
        if self.timeout is not None and (time.time() - self.start_t) > self.timeout:
            return Status.FAILURE
        return Status.RUNNING


# ---------------- 工具节点 -------------------

class NodeFuntion(py_trees.behaviour.Behaviour):
    """
    把一个函数快速包装成一个行为节点
    """

    def __init__(self, fn, name=None):
        super().__init__(name or fn.__name__)
        self.fn = fn

    def update(self):
        # 执行函数，返回值转换成 py_trees 的 Status
        result = self.fn()
        if result is True:
            return Status.SUCCESS
        elif result is False:
            return Status.FAILURE
        else:
            # 如果函数不返回布尔，可以自己定义约定
            return Status.RUNNING


class NodeDelay(py_trees.behaviour.Behaviour):
    """
    简单的延时节点，用于在行为树中插入可配置的等待时间
    """

    def __init__(self, duration: float, name: str = None):
        super().__init__(name or f"Delay({duration:.2f}s)")
        self.duration = max(0.0, duration)
        self.start_t = None

    def initialise(self):
        self.start_t = None

    def update(self):
        if self.start_t is None:
            self.start_t = time.time()
            return Status.RUNNING

        elapsed = time.time() - self.start_t
        if elapsed >= self.duration:
            return Status.SUCCESS

        time.sleep(0.01)
        return Status.RUNNING


# ------------------ 动作节点 -------------------

class NodePercep(Behaviour):
    def __init__(self, name,
                 robot_sdk: RobotSDK,
                 tag_ids: List,
                 ):
        super(NodePercep, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)

        for k in [f'latest_tag_{tag_id}' for tag_id in tag_ids] + [f'latest_tag_{tag_id}_version' for tag_id in
                                                                   tag_ids]:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.robot_sdk = robot_sdk
        self.tag_ids = tag_ids

    def initialise(self):
        self.logger.debug(f"NodePercep::initialise {self.name}")
        for tag_id in self.tag_ids:
            setattr(self.bb, f"latest_tag_{tag_id}_version", 0)

    def update(self):
        self.logger.debug(f"NodePercep::update {self.name}")
        for tag_id in self.tag_ids:
            target_data = self.robot_sdk.vision.get_data_by_id_from_odom(tag_id)
            if target_data is not None:
                tag_pose = target_data["poses"][0]  # 获取第一个tag的位姿
                latest_tag = Tag(
                    id=tag_id,
                    pose=Pose(
                        pos=(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z),
                        quat=(tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z,
                              tag_pose.orientation.w),
                        frame=Frame.ODOM  # 假设感知到的tag位姿在odom坐标系下
                    )
                )
                # print(f'===== updating tag {tag_id}')
                setattr(self.bb, f"latest_tag_{tag_id}", latest_tag)
                current_version = getattr(self.bb, f"latest_tag_{tag_id}_version", 0)
                # print(f'===== tag {tag_id} updated to version {current_version + 1}')
                setattr(self.bb, f"latest_tag_{tag_id}_version", current_version + 1)
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodePercep::terminate {self.name} to {new_status}")


class NodeWalk(Behaviour):
    """
    支持cmd_vel,cmd_pos,cmd_pos_world三种行走模式
    """

    def __init__(self, name,
                 torso_api: TorsoAPI,
                 walk_mode: str = 'cmd_vel'
                 ):
        assert walk_mode in ['cmd_vel', 'cmd_pos', 'cmd_pos_world'], "walk_mode must be 'cmd_vel', 'cmd_pos' or 'cmd_pos_world'"
        super(NodeWalk, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        self.torso_api = torso_api
        self.walk_mode = walk_mode

    def initialise(self):
        self.logger.debug(f"NodeWalk::initialise {self.name}")

        target_pose = getattr(self.bb, "walk_goal", None)
        is_walk_goal_new = getattr(self.bb, "is_walk_goal_new", True)
        if target_pose is None:
            self.logger.error(f"NodeWalk::update {self.name} - No target_pose found on blackboard")
            return Status.FAILURE

        if is_walk_goal_new:
            self.logger.info("New walk goal detected, updating torso_api")
            self.bb.is_walk_goal_new = False
            self.torso_api.update_walk_goal(target_pose)

        self.fut = self.torso_api.walk_to_pose(
            pos_threshold=0.1,
            kp_pos=0.5,
            kp_yaw=0.5,
            max_vel_x=0.4,
            max_vel_yaw=0.4,
            walk_mode=self.walk_mode,
            asynchronous=True
        )
        return None

    def update(self):
        self.logger.debug(f"NodeWalk::update {self.name}")
        target_pose = getattr(self.bb, "walk_goal", None)
        is_walk_goal_new = getattr(self.bb, "is_walk_goal_new", True)
        if target_pose is None:
            self.logger.error(f"NodeWalk::update {self.name} - No target_pose found on blackboard")
            return Status.FAILURE

        if is_walk_goal_new:
            self.bb.is_walk_goal_new = False
            self.torso_api.update_walk_goal(target_pose)

        if not self.fut.done():
            time.sleep(0.01)
            return Status.RUNNING

        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeWalk::terminate {self.name} to {new_status}")
        self.torso_api.stop_walk()
        self.bb.is_walk_goal_new = True
        self.bb.walk_goal = None


class NodeArm(Behaviour):
    def __init__(self,
                 name,
                 arm_api: ArmAPI,
                 control_type: str = 'eef_world',
                 control_base: bool = False,
                 direct_to_wbc: bool = False,
                 back_default: bool = True,
                 ):
        super(NodeArm, self).__init__(name)
        assert control_type in ['eef_world', 'eef_base', 'joint'], "control_type must be 'eef_world' or 'eef_base' or 'joint'"
        self.control_type = control_type
        self.control_base = control_base
        self.direct_to_wbc = direct_to_wbc
        self.back_default = back_default
        self.bb = py_trees.blackboard.Client(name=name)
        # 从白板拿到手臂目标轨迹
        traj_keys = ['left_arm_eef_traj', 'right_arm_eef_traj'] \
            if self.control_type in ['eef_world', 'eef_base'] else ['left_arm_joint_traj', 'right_arm_joint_traj']
        for k in traj_keys:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.arm_api = arm_api

    def initialise(self):
        self.logger.debug(f"NodeArm::initialise {self.name}")
        if self.control_type in ['eef_world', 'eef_base']:
            left_traj = getattr(self.bb, "left_arm_eef_traj", None)
            right_traj = getattr(self.bb, "right_arm_eef_traj", None)
            if left_traj is None or right_traj is None:
                self.logger.error(f"NodeArm::initialise {self.name} - No eef traj on blackboard")
                self.fut = None
                return Status.FAILURE

            self.fut = self.arm_api.move_eef_traj_kmpc(
                left_traj=left_traj,
                right_traj=right_traj,
                asynchronous=True,
                control_base=self.control_base,
                direct_to_wbc=self.direct_to_wbc,
                total_time=5.0,
                back_default=self.back_default,
                frame=KuavoManipulationMpcFrame.WorldFrame if self.control_type == 'eef_world' else KuavoManipulationMpcFrame.LocalFrame,
            )
        else:
            left_joint_traj = getattr(self.bb, "left_arm_joint_traj", None)
            right_joint_traj = getattr(self.bb, "right_arm_joint_traj", None)
            if not left_joint_traj or not right_joint_traj:
                self.logger.error(f"NodeArm::initialise {self.name} - No joint traj on blackboard")
                self.fut = None
                return Status.FAILURE

            if len(left_joint_traj) != len(right_joint_traj):
                self.logger.error(
                    f"NodeArm::initialise {self.name} - Joint traj length mismatch "
                    f"{len(left_joint_traj)} vs {len(right_joint_traj)}")
                self.fut = None
                return Status.FAILURE

            joint_traj = [
                left_point + right_point
                for left_point, right_point in zip(left_joint_traj, right_joint_traj)
            ]

            self.fut = self.arm_api.move_joint_traj(
                joint_traj=joint_traj,
                asynchronous=True,
                total_time=5.0,
            )

    def update(self):
        self.logger.debug(f"NodeArm::update {self.name}")

        if self.fut is None:
            return Status.FAILURE

        if not self.fut.done():
            time.sleep(0.01)
            return Status.RUNNING

        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeArm::terminate {self.name} to {new_status}")


class NodeTorsoPose(Behaviour):
    """专门处理躯干位姿控制的节点"""
    def __init__(
            self,
            name: str,
            torso_api: TorsoAPI,
            target_pose: Pose,
            control_base: bool = False,
    ):
        super().__init__(name)
        self.torso_api = torso_api
        self.target_pose = target_pose
        self.control_base = control_base
        self.future = None

    def initialise(self):
        self.logger.debug(f"NodeTorsoPose::initialise {self.name}")
        
        self.future = self.torso_api.move_torso_pose(
            desir_torso_pose=self.target_pose,
            asynchronous=True,
            control_base=self.control_base,
        )

    def update(self):
        if self.future is None:
            self.logger.error(f"NodeTorsoPose::update {self.name} - future is None in asynchronous mode")
            return Status.FAILURE

        if self.future.done():
            try:
                self.future.result()
            except Exception as exc:
                self.logger.error(f"NodeTorsoPose::update {self.name} failed: {exc}")
                return Status.FAILURE
            return Status.SUCCESS

        time.sleep(0.01)
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodeTorsoPose::terminate {self.name} to {new_status}")


class NodeTorsoJoint(Behaviour):
    """专门处理躯干关节控制的节点"""
    def __init__(
            self,
            name: str,
            torso_api: TorsoAPI,
            joint_trajectory: list,
            total_time: float = 5.0,
    ):
        super().__init__(name)
        self.torso_api = torso_api
        self.joint_trajectory = joint_trajectory
        self.total_time = total_time
        self.future = None

    def initialise(self):
        self.logger.debug(f"NodeTorsoJoint::initialise {self.name}")
        
        self.future = self.torso_api.move_wheel_lower_joint(
            joint_traj=self.joint_trajectory,
            asynchronous=True,
            total_time=self.total_time,
        )

    def update(self):
        if self.future is None:
            self.logger.error(f"NodeTorsoJoint::update {self.name} - future is None in asynchronous mode")
            return Status.FAILURE

        if self.future.done():
            try:
                self.future.result()
            except Exception as exc:
                self.logger.error(f"NodeTorsoJoint::update {self.name} failed: {exc}")
                return Status.FAILURE
            return Status.SUCCESS

        time.sleep(0.01)
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodeTorsoJoint::terminate {self.name} to {new_status}")
