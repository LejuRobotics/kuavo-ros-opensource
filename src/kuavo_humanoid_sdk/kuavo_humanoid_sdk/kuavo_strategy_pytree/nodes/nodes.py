from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import transform_pose_from_tag_to_world, ArmAPI, TorsoAPI, \
    HeadAPI
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.utils import generate_full_bezier_trajectory

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from typing import List
import time
import rospy
from kuavo_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcFrame
import numpy as np


# ---------------- 白板和config管理 -------------------


class NodeHead(Behaviour):
    """
    头部控制节点，用于头部搜索扫描
    每次转到一个位置后，会让并行的识别节点有机会检查是否识别到tag
    """
    def __init__(self,
                 name,
                 head_api: HeadAPI,
                 head_search_yaws: List[float],
                 head_search_pitchs: List[float],
                 tag_id: int = None,  # 用于检查是否识别到tag
                 check_interval: float = 0.1  # 转头后等待的时间，给视觉识别时间
                 ):
        super(NodeHead, self).__init__(name)
        self.head_api = head_api
        self.head_search_yaws = head_search_yaws
        self.head_search_pitchs = head_search_pitchs
        self.tag_id = tag_id
        self.check_interval = check_interval
        
        self.head_traj = []
        self.current_index = 0
        self.last_move_time = 0
        
        # 如果需要检查tag，注册黑板访问
        if self.tag_id is not None:
            self.bb = py_trees.blackboard.Client(name=self.name)
            self.bb.register_key(key=f'latest_tag_{tag_id}', access=py_trees.common.Access.READ)

    def initialise(self):
        self.logger.debug(f"NodeHead::initialise {self.name}")
        
        # 生成头部搜索轨迹
        import numpy as np
        self.head_traj = []
        for pitch in self.head_search_pitchs:
            for yaw in self.head_search_yaws:
                self.head_traj.append((np.deg2rad(yaw), np.deg2rad(pitch)))
        
        self.current_index = 0
        self.last_move_time = 0

    def update(self):
        if len(self.head_traj) == 0:
            return Status.FAILURE
        
        current_time = time.time()
        
        # 如果刚转完头，等待一段时间让视觉识别
        if self.last_move_time > 0 and (current_time - self.last_move_time) < self.check_interval:
            time.sleep(0.01)
            return Status.RUNNING
        
        # 检查是否已经识别到tag
        if self.tag_id is not None:
            try:
                tag = getattr(self.bb, f"latest_tag_{self.tag_id}", None)
                if tag is not None:
                    return Status.SUCCESS
            except KeyError:
                pass
        
        # 如果所有位置都转完了
        if self.current_index >= len(self.head_traj):
            # 检查是否识别到tag
            if self.tag_id is not None:
                try:
                    tag = getattr(self.bb, f"latest_tag_{self.tag_id}", None)
                    if tag is None:
                        self.logger.error(f"tag {self.tag_id} NOT detected!")
                        return Status.FAILURE
                    else:
                        return Status.SUCCESS
                except KeyError:
                    self.logger.error(f"tag {self.tag_id} NOT detected!")
                    return Status.FAILURE
        
        # 转到下一个位置
        yaw, pitch = self.head_traj[self.current_index]
        
        try:
            self.head_api.robot_sdk.control.control_head(yaw, pitch)
            self.last_move_time = time.time()
            self.current_index += 1
        except Exception as e:
            self.logger.error(f"NodeHead - Error moving head: {e}")
            return Status.FAILURE
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodeHead::terminate {self.name} to {new_status}")

# --------------- 转换节点 -----------------
class NodeTagToArmGoal(Behaviour):
    def __init__(self,
                 name,
                 arm_api: ArmAPI,
                 tag_id: int,
                 left_arm_relative_keypoints,
                 right_arm_relative_keypoints,
                 ):
        super(NodeTagToArmGoal, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)

        # 只读
        for k in [f'latest_tag_{tag_id}', f'latest_tag_{tag_id}_version']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        # 读写
        for k in ['left_arm_eef_traj', 'right_arm_eef_traj']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        self.arm_api = arm_api
        self.tag_id = tag_id
        self.tag_version = -1

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

        # ===== 2. 根据关键点生成完整的贝塞尔轨迹 =====
        left_eef_pose_world, right_eef_pose_world = self.arm_api.get_eef_pose_world()
        left_bezier_trajectory, right_bezier_trajectory = generate_full_bezier_trajectory(
            current_left_pose=left_eef_pose_world,
            current_right_pose=right_eef_pose_world,
            left_keypoints_list=left_targets,
            right_keypoints_list=right_targets,
        )

        self.bb.left_arm_eef_traj = left_bezier_trajectory
        self.bb.right_arm_eef_traj = right_bezier_trajectory

        self.tag_version = getattr(self.bb, f"latest_tag_{self.tag_id}_version", 0)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeTagToArmGoal::terminate {self.name} to {new_status}")


class NodeDirectToArmGoal(Behaviour):
    """
    直接将目标位姿转换为手臂轨迹并写入黑板
    生成frame下的手臂插值轨迹
    支持LocalFrame和WorldFrame
    """
    def __init__(self,
                 name,
                 arm_api: ArmAPI,
                 left_arm_poses: List[Pose],
                 right_arm_poses: List[Pose],
                 frame: str = None):
        super(NodeDirectToArmGoal, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)
        for k in ['left_arm_eef_traj', 'right_arm_eef_traj']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
        
        self.arm_api = arm_api
        self.left_arm_poses = left_arm_poses
        self.right_arm_poses = right_arm_poses
        self.frame = frame

    def initialise(self):
        self.logger.debug(f"NodeDirectToArmGoal::initialise {self.name}")

    def update(self):
        
        # 获取当前手臂位置
        left_eef_pose_world, right_eef_pose_world = self.arm_api.get_eef_pose_world()
        
        # 如果使用LocalFrame，需要将当前手臂位姿转换到转换到base在地面的投影点坐标系
        if self.frame == KuavoManipulationMpcFrame.LocalFrame:
            transform_base_to_world = self.arm_api.get_current_transform(
                source_frame=Frame.BASE,
                target_frame=Frame.ODOM
            )
            current_left_pose = transform_base_to_world.apply_to_pose_inverse(left_eef_pose_world)
            current_right_pose = transform_base_to_world.apply_to_pose_inverse(right_eef_pose_world)
            
            # Local坐标系：x,y使用BASE坐标系，z使用ODOM坐标系的绝对高度
            current_left_pose.pos = np.array([current_left_pose.pos[0], current_left_pose.pos[1], left_eef_pose_world.pos[2]])
            current_right_pose.pos = np.array([current_right_pose.pos[0], current_right_pose.pos[1], right_eef_pose_world.pos[2]])
        else:
            current_left_pose = left_eef_pose_world
            current_right_pose = right_eef_pose_world
        
        # 生成轨迹并写入黑板
        left_traj, right_traj = generate_full_bezier_trajectory(
            current_left_pose=current_left_pose,
            current_right_pose=current_right_pose,
            left_keypoints_list=self.left_arm_poses,
            right_keypoints_list=self.right_arm_poses,
        )
        self.bb.left_arm_eef_traj = left_traj
        self.bb.right_arm_eef_traj = right_traj
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeDirectToArmGoal::terminate {self.name} to {new_status}")


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
        
        # 创建ROS发布器，用于发布识别到的tag数据
        self.tag_publisher = rospy.Publisher('/detected_tags', AprilTagDetectionArray, queue_size=10)

    def initialise(self):
        self.logger.debug(f"NodePercep::initialise {self.name}")
        for tag_id in self.tag_ids:
            setattr(self.bb, f"latest_tag_{tag_id}_version", 0)

    def update(self):
        self.logger.debug(f"NodePercep::update {self.name}")
        
        # 创建AprilTagDetectionArray消息
        tag_array_msg = AprilTagDetectionArray()
        tag_array_msg.header = Header()
        tag_array_msg.header.stamp = rospy.Time.now()
        tag_array_msg.header.frame_id = "odom"
        
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
                
                # 构造AprilTagDetection消息
                detection = AprilTagDetection()
                detection.id = [tag_id]
                detection.size = [0.0]  # 如果有尺寸信息可以填入
                
                # 构造PoseWithCovarianceStamped
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = tag_array_msg.header
                pose_msg.pose.pose.position.x = tag_pose.position.x
                pose_msg.pose.pose.position.y = tag_pose.position.y
                pose_msg.pose.pose.position.z = tag_pose.position.z
                pose_msg.pose.pose.orientation.x = tag_pose.orientation.x
                pose_msg.pose.pose.orientation.y = tag_pose.orientation.y
                pose_msg.pose.pose.orientation.z = tag_pose.orientation.z
                pose_msg.pose.pose.orientation.w = tag_pose.orientation.w
                
                detection.pose = pose_msg
                tag_array_msg.detections.append(detection)
        
        # 发布tag数据（即使为空也发布，表示当前没有检测到）
        if len(tag_array_msg.detections) > 0:
            self.tag_publisher.publish(tag_array_msg)
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodePercep::terminate {self.name} to {new_status}")


class NodeWalk(Behaviour):
    """
    支持cmd_vel和cmd_pose_world行走模式
    """

    def __init__(self, name,
                 torso_api: TorsoAPI,
                 control_mode: str = "cmd_vel",
                 pos_threshold: float = 0.1,
                 backward_mode: bool = False
                 ):
        super(NodeWalk, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        self.torso_api = torso_api
        self.control_mode = control_mode
        self.pos_threshold = pos_threshold
        self.backward_mode = backward_mode
        self.target_executed = False

    def initialise(self):
        self.logger.debug(f"NodeWalk::initialise {self.name}")

        target_pose = getattr(self.bb, "walk_goal", None)
        is_walk_goal_new = getattr(self.bb, "is_walk_goal_new", True)
        if target_pose is None:
            self.logger.error(f"NodeWalk::initialise {self.name} - No target_pose found on blackboard")
            return Status.FAILURE

        if self.control_mode == "cmd_vel":
            if is_walk_goal_new:
                self.logger.info("New walk goal detected, updating torso_api")
                self.bb.is_walk_goal_new = False
                self.torso_api.update_walk_goal(target_pose, backward_mode=self.backward_mode)

            self.fut = self.torso_api.walk_to_pose_by_vel(
                pos_threshold=self.pos_threshold,
                kp_pos=1.2,
                kp_yaw=1.0,
                max_vel_x=0.4,
                max_vel_yaw=1.0,
                backward_mode=self.backward_mode,
                asynchronous=True
            )
        elif self.control_mode == "cmd_pose_world":
            if is_walk_goal_new:
                self.logger.info("New walk goal detected, updating torso_api")
                self.bb.is_walk_goal_new = False
                self.torso_api.update_walk_goal(target_pose, backward_mode=self.backward_mode)

            self.fut = self.torso_api.walk_to_pose_by_world(
                pos_threshold=self.pos_threshold,
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

        if self.control_mode == "cmd_vel":
            if is_walk_goal_new:
                self.bb.is_walk_goal_new = False
                self.torso_api.update_walk_goal(target_pose, backward_mode=self.backward_mode)

            if not self.fut.done():
                time.sleep(0.01)
                return Status.RUNNING

            return Status.SUCCESS

        elif self.control_mode == "cmd_pose_world":
            if is_walk_goal_new:
                self.bb.is_walk_goal_new = False
                self.torso_api.update_walk_goal(target_pose, backward_mode=self.backward_mode)

            if not self.fut.done():
                time.sleep(0.01)
                return Status.RUNNING

            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"NodeWalk::terminate {self.name} to {new_status}")
        if self.control_mode == "cmd_vel":
            self.torso_api.stop_walk()

        self.bb.is_walk_goal_new = True
        self.bb.walk_goal = None


class NodeArm(Behaviour):
    def __init__(self, name, arm_api: ArmAPI, control_base: bool = False, total_time: float = 2.0, frame: str = None):
        super(NodeArm, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # 从白板拿到手臂目标轨迹
        for k in ['left_arm_eef_traj', 'right_arm_eef_traj']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.arm_api = arm_api
        self.control_base = control_base
        self.total_time = total_time
        self.frame = frame

    def initialise(self):
        self.logger.debug(f"NodeArm::initialise {self.name}")
        left_traj = getattr(self.bb, "left_arm_eef_traj", None)
        right_traj = getattr(self.bb, "right_arm_eef_traj", None)
        if left_traj is None or right_traj is None:
            self.logger.error(f"NodeArm::update {self.name} - No arm trajectory found on blackboard")
            return Status.FAILURE

        self.fut = self.arm_api.move_eef_traj_kmpc(
            left_traj=left_traj,
            right_traj=right_traj,
            asynchronous=True,
            control_base=self.control_base,
            direct_to_wbc=True,
            total_time=self.total_time,
            frame=self.frame,
        )

    def update(self):
        self.logger.debug(f"NodeArm::update {self.name}")

        if not self.fut.done():
            time.sleep(0.01)
            return Status.RUNNING

        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeArm::terminate {self.name} to {new_status}")


class NodeWaist(Behaviour):
    """
    腰部控制节点，用于控制机器人转腰
    """
    def __init__(self, name, robot_sdk: RobotSDK, waist_pos: float, 
                 angle_threshold: float = 3.0, waist_dof: int = 1):
        super(NodeWaist, self).__init__(name)
        
        self.robot_sdk = robot_sdk
        self.waist_pos = waist_pos  # 腰部目标角度（度数）
        self.angle_threshold = angle_threshold  # 角度误差阈值（度数）
        self.waist_dof = waist_dof  # 腰部自由度
        self.control_executed = False

    def initialise(self):
        self.logger.debug(f"NodeWaist::initialise {self.name}")
        print(f'===== Initializing waist control to position: {self.waist_pos}°')
        self.control_executed = False

    def update(self):
        self.logger.debug(f"NodeWaist::update {self.name}")

        if self.control_executed:
            # 检查角度反馈
            import numpy as np
            waist_state = self.robot_sdk.state.waist_joint_state(waist_dof=self.waist_dof)
            current_angle = np.rad2deg(waist_state.position[0])
            angle_error = abs(self.waist_pos - current_angle)
            print(f'===== Waist: target={self.waist_pos:.1f}°, current={current_angle:.1f}°, error={angle_error:.1f}°')
            
            if angle_error < self.angle_threshold:
                return Status.SUCCESS
            else:
                # 继续发送控制指令
                self.robot_sdk.control.control_waist_pos([self.waist_pos])
                return Status.RUNNING
        
        # 第一次调用，直接执行转腰
        return self._execute_waist_control()

    def _execute_waist_control(self):
        """执行腰部控制"""
        print(f'===== Executing waist control to position: {self.waist_pos}°')
        # 控制腰部到指定角度
        result = self.robot_sdk.control.control_waist_pos([self.waist_pos])
        self.control_executed = True

        if result:
            print(f'===== Waist control successful: {self.waist_pos}°')
            return Status.RUNNING  # 改为RUNNING，继续检查反馈
        else:
            print(f'===== Waist control failed: {self.waist_pos}°')
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"NodeWaist::terminate {self.name} to {new_status}")

class NodeWalkWithDistanceMonitor(NodeWalk):
    """
    带距离监控的行走节点
    在行走过程中监控已行走的距离，当达到指定距离时在黑板上设置触发标志
    """
    
    def __init__(self, trigger_distance: float = 0.2, **kwargs):
        """
        Args:
            trigger_distance: 触发转腰的行走距离（米）
            **kwargs: 其他参数透传给父类NodeWalk
        """
        super(NodeWalkWithDistanceMonitor, self).__init__(**kwargs)
        self.trigger_distance = trigger_distance
        self.start_position = None
        self.distance_triggered = False
        
        # 注册黑板键用于触发标志
        self.bb.register_key(key='walk_distance_trigger_reached', access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.logger.debug(f"NodeWalkWithDistanceMonitor::initialise {self.name}")
        
        # 重置触发状态和黑板标志（确保每轮都能重新触发）
        self.distance_triggered = False
        self.bb.walk_distance_trigger_reached = None
        
        # 记录起始位置
        import numpy as np
        robot_pos = self.torso_api.robot_sdk.state.robot_position()
        self.start_position = np.array(robot_pos[:2])  # 只记录x, y坐标
        
        print(f'===== 开始监控行走距离，起始位置: {self.start_position}, 触发距离: {self.trigger_distance}m')
        
        # 调用父类的初始化
        return super(NodeWalkWithDistanceMonitor, self).initialise()

    def update(self):
        self.logger.debug(f"NodeWalkWithDistanceMonitor::update {self.name}")
        
        # 计算已行走距离
        import numpy as np
        current_pos = self.torso_api.robot_sdk.state.robot_position()
        current_position = np.array(current_pos[:2])
        distance_traveled = np.linalg.norm(current_position - self.start_position)
        
        # 如果距离达到触发阈值且尚未触发，设置黑板标志
        if distance_traveled >= self.trigger_distance and not self.distance_triggered:
            print(f'===== 已行走 {distance_traveled:.3f}m，达到触发距离 {self.trigger_distance}m，触发转腰动作')
            self.bb.walk_distance_trigger_reached = True
            self.distance_triggered = True
        
        # 调用父类的update方法继续行走
        return super(NodeWalkWithDistanceMonitor, self).update()

    def terminate(self, new_status):
        self.logger.debug(f"NodeWalkWithDistanceMonitor::terminate {self.name} to {new_status}")
        super(NodeWalkWithDistanceMonitor, self).terminate(new_status)
