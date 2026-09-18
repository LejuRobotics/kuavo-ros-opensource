from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import (
    transform_pose_from_tag_to_world,
    ArmAPI,
    TorsoAPI,
    HeadAPI,
    TimedCmdAPI,
    ChassisApi,
)
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Tag, Frame, Transform3D
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame, KuavoManipulationMpcCtrlMode
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.utils import generate_full_bezier_trajectory, \
    interpolate_joint_positions_bezier, calculate_elbow_y, get_elbow_position
from kuavo_humanoid_sdk.interfaces.data_types import KuavoIKParams

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from typing import List, Optional
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from kuavo_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from kuavo_msgs.srv import setContactForceInterpParams, setContactForceInterpParamsRequest
from geometry_msgs.msg import PoseWithCovarianceStamped, Wrench, WrenchStamped
from std_msgs.msg import Header
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcFrame
import numpy as np
import sys


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
        
        # 确保为新 tag_id 注册 BlackBoard 访问权限（支持动态更新 tag_id）
        if self.tag_id is not None:
            # 如果 self.bb 不存在，创建它
            if not hasattr(self, 'bb') or self.bb is None:
                self.bb = py_trees.blackboard.Client(name=self.name)
            try:
                self.bb.register_key(key=f'latest_tag_{self.tag_id}', access=py_trees.common.Access.READ)
            except Exception:
                # 如果已经注册过，忽略错误
                pass
        
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
                 control_type: str = 'eef_world',  # joint 是关节轨迹，eef_world/eef_base是末端轨迹
                 enable_joint_mirroring: bool = True,  # 是否启用关节镜像处理
                 enable_high_position_accuracy: bool = False,  # 是否启用高位置精度
                 traj_point_num: int = 20,  # 轨迹点数
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
        self.traj_point_num = traj_point_num

        self.left_arm_relative_keypoints = left_arm_relative_keypoints
        self.right_arm_relative_keypoints = right_arm_relative_keypoints
        self.enable_high_position_accuracy = enable_high_position_accuracy

    def initialise(self):
        self.logger.debug(f"NodeTagToArmGoal::initialise {self.name}")
        print(f'===== Initializing NodeTagToArmGoal for tag {self.tag_id}')
        # 确保为新 tag_id 注册 BlackBoard 访问权限（支持动态更新 tag_id）
        for k in [f'latest_tag_{self.tag_id}', f'latest_tag_{self.tag_id}_version']:
            try:
                self.bb.register_key(key=k, access=py_trees.common.Access.READ)
            except Exception:
                # 如果已经注册过，忽略错误
                pass
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
                custom_param = None
                for retry in range(5):
                    if retry > 0:
                        left_elbow[1] -= 0.02
                        right_elbow[1] += 0.02

                    if self.enable_high_position_accuracy:
                        custom_param = KuavoIKParams(
                                                    major_optimality_tol=1e-3,
                                                    major_feasibility_tol=1e-3,
                                                    minor_feasibility_tol=3e-3,
                                                    major_iterations_limit=100,
                                                    oritation_constraint_tol=1e-3,
                                                    pos_constraint_tol=1e-3,
                                                    pos_cost_weight=10.0,
                                                    constraint_mode=6
                                                    )
                    else:
                        custom_param = KuavoIKParams(
                                                    major_optimality_tol=9e-3,
                                                    major_feasibility_tol=9e-3,
                                                    minor_feasibility_tol=9e-3,
                                                    major_iterations_limit=50,
                                                    oritation_constraint_tol=19e-3,
                                                    pos_constraint_tol=9e-3,
                                                    pos_cost_weight=10.0,
                                                    constraint_mode=0
                                                    )
                    target_joint_positions = robot_sdk.arm.arm_ik(
                        left_pose=left_target_kuavo_pose,
                        right_pose=right_target_kuavo_pose,
                        left_elbow_pos_xyz=left_elbow,
                        right_elbow_pos_xyz=right_elbow,
                        arm_q0=current_joints.tolist(),
                        params=custom_param
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
                    num_points=self.traj_point_num
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
        # 确保为新 tag_id 注册 BlackBoard 访问权限（支持动态更新 tag_id）
        for k in [f'latest_tag_{self.tag_id}', f'latest_tag_{self.tag_id}_version']:
            try:
                self.bb.register_key(key=k, access=py_trees.common.Access.READ)
            except Exception:
                # 如果已经注册过，忽略错误
                pass
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
                 timeout: float = None,
                 exit_on_timeout: bool = False,
                 timeout_message: str = None,
                 on_timeout_callable=None):
        """
        on_timeout_callable: 超时且 exit_on_timeout 时，在打印错误并退出前调用的可调用对象（如打印已识别数据）。
        """
        super().__init__(name or f"WaitFor({key})")
        self.key = key
        self.timeout = timeout
        self.exit_on_timeout = exit_on_timeout
        self.timeout_message = timeout_message or f"等待黑板键 {key} 超时"
        self.on_timeout_callable = on_timeout_callable
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
            if self.exit_on_timeout:
                if callable(self.on_timeout_callable):
                    try:
                        self.on_timeout_callable()
                    except Exception as e:
                        rospy.logwarn(f"on_timeout_callable 执行异常: {e}")
                rospy.logerr(self.timeout_message)
                sys.exit(1)
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


class NodeTagToBase(py_trees.behaviour.Behaviour):
    """
    Tag 到 Base 的转换节点：从黑板读取 tag，将左右手关键点（TAG/BASE 混合）变换到 BASE 系并写入 arm_ee_local_keypoints。
    输入参数：robot_sdk, tag_id, left_poses, right_poses, total_time。
    """

    def __init__(
        self,
        name: str,
        robot_sdk,
        tag_id: int,
        left_poses: list,
        right_poses: list,
        total_time: float,
    ):
        super().__init__(name)
        self.robot_sdk = robot_sdk
        self.tag_id = tag_id
        self.left_poses = left_poses
        self.right_poses = right_poses
        self.total_time = total_time

    def _convert_tag_keypoints_to_base(self) -> bool:
        """从黑板读取 tag，将左右手关键点从 TAG 系变换到 BASE 系并写入黑板。"""
        from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.funcs import set_arm_ee_local_keypoints_from_poses
        bb = py_trees.blackboard.Client(name=self.name)
        bb.register_key(key=f"latest_tag_{self.tag_id}", access=py_trees.common.Access.READ)
        tag = getattr(bb, f"latest_tag_{self.tag_id}", None)
        if tag is None or not hasattr(tag, "pose"):
            rospy.logerr(f"NodeTagToBase: 黑板无 latest_tag_{self.tag_id}")
            return False

        tag_pose_odom = tag.pose
        if getattr(tag_pose_odom, "frame", None) != Frame.ODOM:
            tag_pose_odom = Pose(
                pos=tuple(np.array(tag_pose_odom.pos).tolist()),
                quat=tuple(np.array(tag_pose_odom.quat).tolist()),
                frame=Frame.ODOM,
            )
        transform_tag_to_odom = Transform3D(
            trans_pose=tag_pose_odom,
            source_frame=Frame.TAG,
            target_frame=Frame.ODOM,
        )

        arm_api = ArmAPI(self.robot_sdk)
        transform_base_to_odom = arm_api.get_current_transform(
            source_frame=Frame.BASE,
            target_frame=Frame.ODOM,
        )

        def to_base(pose):
            if pose.frame == Frame.TAG:
                pose_in_odom = transform_tag_to_odom.apply_to_pose(pose)
                pose_in_base = transform_base_to_odom.apply_to_pose_inverse(pose_in_odom)
                return pose_in_base
            return Pose(pos=tuple(pose.pos), quat=tuple(pose.quat), frame=Frame.BASE)

        left_poses_base = [to_base(p) for p in self.left_poses]
        right_poses_base = [to_base(p) for p in self.right_poses]

        if left_poses_base:
            rospy.loginfo(f"[TAG->BASE] 左臂第1点 -> BASE: pos={left_poses_base[0].pos}")
        if right_poses_base:
            rospy.loginfo(f"[TAG->BASE] 右臂第1点 -> BASE: pos={right_poses_base[0].pos}")

        return set_arm_ee_local_keypoints_from_poses(
            left_poses_base, right_poses_base, total_time=self.total_time
        )

    def update(self):
        ok = self._convert_tag_keypoints_to_base()
        return Status.SUCCESS if ok else Status.FAILURE


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
                 tag_stable_count: int = 10,
                 tag_pos_std_threshold: float = 0.05,
                 ):
        super(NodePercep, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)

        for k in [f'latest_tag_{tag_id}' for tag_id in tag_ids] + [f'latest_tag_{tag_id}_version' for tag_id in
                                                                   tag_ids]:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.robot_sdk = robot_sdk
        self.tag_ids = tag_ids
        self.tag_stable_count = tag_stable_count  # 连续 N 帧位置方差小于阈值才写入黑板
        self.tag_pos_std_threshold = tag_pos_std_threshold  # 位置标准差阈值(m)，x/y/z 均低于此认为数据稳定
        # 每个 tag 最近若干帧的位置缓冲，用于判定“连续误差较小”后再写黑板
        self._tag_pos_buffers = {tid: [] for tid in tag_ids}
        # 每个 tag 最近若干帧的姿态（四元数）缓冲，用于稳定姿态的平均
        self._tag_quat_buffers = {tid: [] for tid in tag_ids}

        # 创建ROS发布器，用于发布识别到的tag数据
        self.tag_publisher = rospy.Publisher('/detected_tags', AprilTagDetectionArray, queue_size=10)

    def initialise(self):
        self.logger.debug(f"NodePercep::initialise {self.name}")
        for tag_id in self.tag_ids:
            # 确保权限已注册（支持动态更新 tag_id）
            for k in [f'latest_tag_{tag_id}', f'latest_tag_{tag_id}_version']:
                try:
                    # 尝试注册权限（如果已经注册过，py_trees 会忽略）
                    self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
                    self.bb.register_key(key=k, access=py_trees.common.Access.READ)
                except Exception:
                    # 如果注册失败，继续执行（可能已经注册过了）
                    pass
            # 初始化黑板中的 key，确保其他 client 访问时不会因为 key 不存在而抛异常
            setattr(self.bb, f"latest_tag_{tag_id}_version", 0)
            self._tag_pos_buffers[tag_id] = []
            self._tag_quat_buffers[tag_id] = []

    def _is_tag_pos_stable(self, buf: list) -> bool:
        """连续多帧位置误差较小：x/y/z 标准差均小于阈值则视为稳定。"""
        if len(buf) < self.tag_stable_count:
            print(f"需{self.tag_stable_count}帧数据: {len(buf)}")
            return False
        else:
            print(f"第{len(buf)}帧数据稳定")
        arr = np.array(buf[-self.tag_stable_count:])
        std_x = float(np.std(arr[:, 0]))
        std_y = float(np.std(arr[:, 1]))
        std_z = float(np.std(arr[:, 2]))
        return (std_x <= self.tag_pos_std_threshold and
                std_y <= self.tag_pos_std_threshold and
                std_z <= self.tag_pos_std_threshold)

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
                pos = (tag_pose.position.x, tag_pose.position.y, tag_pose.position.z)
                quat = (tag_pose.orientation.x, tag_pose.orientation.y,
                        tag_pose.orientation.z, tag_pose.orientation.w)
                self._tag_pos_buffers[tag_id].append(pos)
                self._tag_quat_buffers[tag_id].append(quat)
                if len(self._tag_pos_buffers[tag_id]) > self.tag_stable_count:
                    self._tag_pos_buffers[tag_id].pop(0)
                if len(self._tag_quat_buffers[tag_id]) > self.tag_stable_count:
                    self._tag_quat_buffers[tag_id].pop(0)

                # 从视觉数据获取 tag 尺寸（单位：米），有则传入
                tag_size = None
                if target_data.get("sizes") and len(target_data["sizes"]) > 0:
                    tag_size = float(target_data["sizes"][0])
                # 仅当连续 tag_stable_count 帧位置方差较小时才写入黑板，避免头部转动时抖动导致错误写入
                if self._is_tag_pos_stable(self._tag_pos_buffers[tag_id]):
                    # 使用缓冲区内稳定帧的平均位置和平均姿态作为写入黑板的位姿
                    pos_arr = np.array(self._tag_pos_buffers[tag_id][-self.tag_stable_count:])
                    print(f"tag_id: {tag_id} ,pos_arr: \n{np.round(pos_arr, 2)}")
                    mean_pos = np.mean(pos_arr, axis=0)
                    avg_pos = (float(mean_pos[0]), float(mean_pos[1]), float(mean_pos[2]))

                    quat_arr = np.array(self._tag_quat_buffers[tag_id][-self.tag_stable_count:])
                    euler_deg = np.rad2deg(np.array([R.from_quat(q).as_euler('xyz') for q in quat_arr]))
                    print(f"tag_id: {tag_id} ,euler_arr(度) roll,pitch,yaw: \n{np.round(euler_deg, 1)}")
                    mean_quat = np.mean(quat_arr, axis=0)
                    norm = float(np.linalg.norm(mean_quat))
                    if norm > 1e-6:
                        mean_quat /= norm
                        avg_quat = (float(mean_quat[0]), float(mean_quat[1]),
                                    float(mean_quat[2]), float(mean_quat[3]))
                    else:
                        # 若数值异常，则退回到最后一帧的姿态
                        last_quat = self._tag_quat_buffers[tag_id][-1]
                        avg_quat = (float(last_quat[0]), float(last_quat[1]),
                                    float(last_quat[2]), float(last_quat[3]))

                    stable_tag = Tag(
                        id=tag_id,
                        pose=Pose(
                            pos=avg_pos,
                            quat=avg_quat,
                            frame=Frame.ODOM  # 假设感知到的tag位姿在odom坐标系下
                        ),
                        size=tag_size,
                    )
                    setattr(self.bb, f"latest_tag_{tag_id}", stable_tag)
                    current_version = getattr(self.bb, f"latest_tag_{tag_id}_version", 0)
                    setattr(self.bb, f"latest_tag_{tag_id}_version", current_version + 1)
                    self._tag_pos_buffers[tag_id].clear()
                    self._tag_quat_buffers[tag_id].clear()


                # 构造AprilTagDetection消息
                detection = AprilTagDetection()
                detection.id = [tag_id]
                detection.size = [tag_size if tag_size is not None else 0.0]
                
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
            else:
                # 本帧未检测到该 tag，清空缓冲，要求重新积累连续稳定帧
                self._tag_pos_buffers[tag_id] = []
                self._tag_quat_buffers[tag_id] = []
        
        # 发布tag数据（即使为空也发布，表示当前没有检测到）
        if len(tag_array_msg.detections) > 0:
            self.tag_publisher.publish(tag_array_msg)
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodePercep::terminate {self.name} to {new_status}")


class NodeWheelWalk(Behaviour):
    """
    支持cmd_vel,cmd_pos,cmd_pos_world三种行走模式
    
    Args:
        name: 节点名称
        torso_api: TorsoAPI 实例
        walk_mode: 行走模式，可选 'cmd_vel', 'cmd_pos', 'cmd_pos_world'
        mpc_ctrl_mode: MPC 控制模式，默认为 BaseOnly（仅控制底盘）
            - BaseOnly: 仅控制底盘
            - ArmOnly: 仅控制手臂
            - BaseArm: 同时控制底盘和手臂
            - NoControl: 无控制
    """

    def __init__(self, name,
                 torso_api: TorsoAPI,
                 walk_mode: str = 'cmd_vel',
                 mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.BaseOnly
                 ):
        assert walk_mode in ['cmd_vel', 'cmd_pos', 'cmd_pos_world'], "walk_mode must be 'cmd_vel', 'cmd_pos' or 'cmd_pos_world'"
        super(NodeWheelWalk, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.name)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        for k in ['walk_goal', 'is_walk_goal_new']:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)

        self.torso_api = torso_api
        self.walk_mode = walk_mode
        self.mpc_ctrl_mode = mpc_ctrl_mode

    def initialise(self):
        self.logger.debug(f"NodeWheelWalk::initialise {self.name}")

        # 设置 MPC 控制模式
        self.torso_api.robot_sdk.control.set_manipulation_mpc_mode(self.mpc_ctrl_mode)
        self.logger.info(f"NodeWheelWalk::initialise - Set MPC control mode to {self.mpc_ctrl_mode.name}")

        target_pose = getattr(self.bb, "walk_goal", None)
        is_walk_goal_new = getattr(self.bb, "is_walk_goal_new", True)
        if target_pose is None:
            self.logger.error(f"NodeWheelWalk::initialise {self.name} - No target_pose found on blackboard")
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
        self.logger.debug(f"NodeWheelWalk::update {self.name}")
        target_pose = getattr(self.bb, "walk_goal", None)
        is_walk_goal_new = getattr(self.bb, "is_walk_goal_new", True)
        if target_pose is None:
            self.logger.error(f"NodeWheelWalk::update {self.name} - No target_pose found on blackboard")
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
                kp_pos=1.0,
                kp_yaw=0.6,
                max_vel_x=0.4,
                max_vel_yaw=0.5,
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
    def __init__(self, name, arm_api: ArmAPI, control_base: bool = False, total_time: float = 2.0, frame: str = None, arm_pos_threshold: float = 0.2, arm_angle_threshold: float = 0.35, arm_error_detect: bool = True):
        super(NodeArm, self).__init__(name)
        self.bb = py_trees.blackboard.Client(name=name)
        # 从白板拿到手臂目标轨迹
        for k in ['left_arm_eef_traj', 'right_arm_eef_traj']:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.arm_api = arm_api
        self.control_base = control_base
        self.total_time = total_time
        self.frame = frame
        self.arm_pos_threshold = arm_pos_threshold  # 位置误差阈值（米）
        self.arm_angle_threshold = arm_angle_threshold  # 角度误差阈值（弧度）
        self.arm_error_detect = arm_error_detect  # 是否开启手臂误差检测

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
            arm_pos_threshold=self.arm_pos_threshold,
            arm_angle_threshold=self.arm_angle_threshold,
            arm_error_detect=self.arm_error_detect,
        )

    def update(self):
        self.logger.debug(f"NodeArm::update {self.name}")

        if not self.fut.done():
            time.sleep(0.01)
            return Status.RUNNING

        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeArm::terminate {self.name} to {new_status}")

class NodeWheelArm(Behaviour):
    """
    手臂控制节点
    
    Args:
        name: 节点名称
        arm_api: ArmAPI 实例
        control_type: 控制类型，可选 'eef_world', 'eef_base', 'joint'
        direct_to_wbc: 指令是否直接到 WBC
        total_time: 轨迹执行总时间（秒）
        back_default: 是否在执行完成后恢复默认模式
        mpc_ctrl_mode: MPC 控制模式，默认为 ArmOnly（仅控制手臂）
            - ArmOnly: 仅控制手臂
            - BaseOnly: 仅控制底盘
            - BaseArm: 同时控制底盘和手臂
            - NoControl: 无控制
    """
    def __init__(self,
                 name,
                 arm_api: ArmAPI,
                 control_type: str = 'eef_world',
                 direct_to_wbc: bool = False,
                 total_time: float = 5.0,
                 back_default: bool = True,
                 mpc_ctrl_mode: KuavoManipulationMpcCtrlMode = KuavoManipulationMpcCtrlMode.ArmOnly,
                 ):
        super(NodeWheelArm, self).__init__(name)
        assert control_type in ['eef_world', 'eef_base', 'joint'], "control_type must be 'eef_world' or 'eef_base' or 'joint'"
        self.control_type = control_type
        self.direct_to_wbc = direct_to_wbc
        self.back_default = back_default
        self.mpc_ctrl_mode = mpc_ctrl_mode
        self.bb = py_trees.blackboard.Client(name=name)
        # 从白板拿到手臂目标轨迹
        traj_keys = ['left_arm_eef_traj', 'right_arm_eef_traj'] \
            if self.control_type in ['eef_world', 'eef_base'] else ['left_arm_joint_traj', 'right_arm_joint_traj']
        for k in traj_keys:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)

        self.arm_api = arm_api
        self.total_time = total_time

    def initialise(self):
        self.logger.debug(f"NodeWheelArm::initialise {self.name}")
        
        if self.control_type in ['eef_world', 'eef_base']:
            left_traj = getattr(self.bb, "left_arm_eef_traj", None)
            right_traj = getattr(self.bb, "right_arm_eef_traj", None)
            if left_traj is None or right_traj is None:
                self.logger.error(f"NodeWheelArm::initialise {self.name} - No eef traj on blackboard")
                self.fut = None
                return Status.FAILURE

            self.fut = self.arm_api.move_eef_traj_wheel_mpc(
                left_traj=left_traj,
                right_traj=right_traj,
                asynchronous=True,
                direct_to_wbc=self.direct_to_wbc,
                total_time=self.total_time,
                back_default=self.back_default,
                frame=KuavoManipulationMpcFrame.WorldFrame if self.control_type == 'eef_world' else KuavoManipulationMpcFrame.LocalFrame,
                mpc_ctrl_mode=self.mpc_ctrl_mode,
            )
        else:
            left_joint_traj = getattr(self.bb, "left_arm_joint_traj", None)
            right_joint_traj = getattr(self.bb, "right_arm_joint_traj", None)
            if not left_joint_traj or not right_joint_traj:
                self.logger.error(f"NodeWheelArm::initialise {self.name} - No joint traj on blackboard")
                self.fut = None
                return Status.FAILURE

            if len(left_joint_traj) != len(right_joint_traj):
                self.logger.error(
                    f"NodeWheelArm::initialise {self.name} - Joint traj length mismatch "
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
                total_time=self.total_time,
                mpc_ctrl_mode=self.mpc_ctrl_mode,
            )

    def update(self):
        self.logger.debug(f"NodeWheelArm::update {self.name}")

        if self.fut is None:
            return Status.FAILURE

        if not self.fut.done():
            time.sleep(0.01)
            return Status.RUNNING

        # Future 已结束：如果后台线程抛异常，这里必须暴露出来，否则会“半途失败但显示SUCCESS”
        exc = self.fut.exception()
        if exc is not None:
            self.logger.error(f"NodeWheelArm::update {self.name} - 异步任务异常: {exc}")
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"NodeWheelArm::terminate {self.name} to {new_status}")


class NodeWheelMoveTimedCmd(Behaviour):
    """
    通用定时命令节点（支持底盘、下肢、上肢等）
    
    通过 /mobile_manipulator_timed_single_cmd 服务逐点发送命令，
    直接执行关键点序列。臂类为左右单臂规划器，arm_ee_local/arm_ee_world/arm 的
    臂类关键点会在 API 层拆成左+右各发一次。
    
    根据 cmd_type 参数选择控制模式：
        - 'chassis_world': 底盘世界系（planner 0，3维）
        - 'chassis_local': 底盘局部系（planner 1，3维）
        - 'torso': 躯干（planner 2，4维）
        - 'leg': 下肢关节（planner 3，4 个关节）
        - 'arm_ee_world' / 'arm_ee_local': 双臂末端笛卡尔位姿，12 维 = 左 6 [x,y,z,yaw,pitch,roll] + 右 6，拆为 planner 4/5（世界系）或 6/7（局部系）
        - 'arm': 双臂关节空间，14 维 = 左臂 7 个关节角 + 右臂 7 个关节角，拆为 planner 8/9
    
    从黑板读取（根据 cmd_type 自动选择键名）：
        - chassis_world/local/torso/leg: 对应 keypoints、keypoint_times
        - arm_ee_world/arm_ee_local: arm_ee_*_keypoints, arm_ee_keypoint_times
        - arm: arm_joint_keypoints, arm_joint_keypoint_times
    
    Args:
        name: 节点名称
        timed_cmd_api: TimedCmdAPI 实例
        cmd_type: 命令类型，默认 'leg'
        time_per_point: 每个关键点的默认执行时间（秒），当黑板无时间列表时使用
        focus_ee: 仅当 ``cmd_type`` 为 ``arm_ee_world`` / ``arm_ee_local`` 时有效。
    """
    
    # 需在发令前设置 /mobile_manipulator_focus_ee 的臂类命令
    _ARM_CMD_TYPES_WITH_FOCUS = frozenset({"arm_ee_world", "arm_ee_local"})

    # 黑板键名映射
    BLACKBOARD_KEYS = {
        'chassis_world': {'keypoints': 'chassis_world_keypoints', 'times': 'chassis_world_keypoint_times'},
        'chassis_local': {'keypoints': 'chassis_local_keypoints', 'times': 'chassis_local_keypoint_times'},
        'torso': {'keypoints': 'torso_keypoints', 'times': 'torso_keypoint_times'},
        'leg': {'keypoints': 'leg_keypoints', 'times': 'leg_keypoint_times'},
        'arm_ee_world': {'keypoints': 'arm_ee_world_keypoints', 'times': 'arm_ee_keypoint_times'},
        'arm_ee_local': {'keypoints': 'arm_ee_local_keypoints', 'times': 'arm_ee_keypoint_times'},
        'arm': {'keypoints': 'arm_joint_keypoints', 'times': 'arm_joint_keypoint_times'},
    }
    
    # 显示名称映射
    DISPLAY_NAMES = {
        'chassis_world': '底盘世界系',
        'chassis_local': '底盘局部系',
        'torso': '躯干',
        'arm_ee_world': '双臂末端世界系',
        'arm_ee_local': '双臂末端局部系',
        'leg': '下肢',
        'arm': '上肢',
    }
    
    def __init__(self,
                 name,
                 timed_cmd_api: TimedCmdAPI = None,
                 cmd_type: str = 'leg',
                 time_per_point: float = 2.0,
                 focus_ee: bool = False,
                 # 向后兼容参数
                 joint_api: TimedCmdAPI = None,
                 joint_type: str = None,
                 ):
        super(NodeWheelMoveTimedCmd, self).__init__(name)
        
        # 向后兼容：支持旧参数名
        if joint_api is not None:
            timed_cmd_api = joint_api
        if joint_type is not None:
            cmd_type = joint_type
        
        # 验证必需参数
        if timed_cmd_api is None:
            raise ValueError("必须提供 timed_cmd_api 或 joint_api 参数")
        
        if cmd_type not in self.BLACKBOARD_KEYS:
            supported = ', '.join(self.BLACKBOARD_KEYS.keys())
            raise ValueError(f"无效的 cmd_type: {cmd_type}，支持: {supported}")
        
        self.timed_cmd_api = timed_cmd_api
        self.cmd_type = cmd_type
        self.time_per_point = time_per_point
        self.focus_ee = focus_ee

        # 向后兼容属性
        self.joint_api = timed_cmd_api
        self.joint_type = cmd_type
        
        # 获取黑板键名
        keys = self.BLACKBOARD_KEYS[cmd_type]
        self.keypoints_key = keys['keypoints']
        self.times_key = keys['times']
        # 实际执行时间写入黑板，供外部读取（键名如 arm_ee_actual_times）
        self.actual_times_key = self.times_key.replace('keypoint_times', 'actual_times')
        
        # 从黑板读取关键点列表和时间列表
        self.bb = py_trees.blackboard.Client(name=name)
        for k in [self.keypoints_key, self.times_key]:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)
        self.bb.register_key(key=self.actual_times_key, access=py_trees.common.Access.WRITE)
        
        self.current_idx = 0
        self.keypoints = []
        self.keypoint_times = []
        self.is_waiting = False
        self.wait_until = 0.0
        
        # 显示名称
        self.display_name = self.DISPLAY_NAMES.get(cmd_type, cmd_type)

    def initialise(self):
        self.logger.debug(f"NodeWheelMoveTimedCmd::initialise {self.name}")
        
        # 从黑板获取关键点
        self.keypoints = getattr(self.bb, self.keypoints_key, None)
        self.keypoint_times = getattr(self.bb, self.times_key, None)
        
        if self.keypoints is None:
            self.logger.error(f"NodeWheelMoveTimedCmd::initialise {self.name} - 黑板中无关键点数据 ({self.keypoints_key})")
            return
        
        # 如果没有时间列表，使用默认时间
        if self.keypoint_times is None:
            self.keypoint_times = [self.time_per_point] * len(self.keypoints)
            
        self.current_idx = 0
        self.is_waiting = False
        self.wait_until = 0.0
        setattr(self.bb, self.actual_times_key, [])

        if (self.cmd_type in self._ARM_CMD_TYPES_WITH_FOCUS):
            if not rospy.core.is_initialized():
                rospy.init_node("node_wheel_move_timed_cmd", anonymous=True, disable_signals=True)
            self.focus_ee = True
            self.timed_cmd_api.set_focus_ee(self.focus_ee)

        print(f"===== NodeWheelMoveTimedCmd({self.display_name}): 共 {len(self.keypoints)} 个关键点待执行")

    def update(self):
        self.logger.debug(f"NodeWheelMoveTimedCmd::update {self.name}")
        
        # 检查数据有效性
        if self.keypoints is None:
            return Status.FAILURE
            
        if len(self.keypoints) == 0:
            return Status.SUCCESS
            
        # 如果正在等待当前点执行完成
        if self.is_waiting:
            if time.time() >= self.wait_until:
                self.is_waiting = False
                self.current_idx += 1
            else:
                time.sleep(0.01)
                return Status.RUNNING
        
        # 检查是否已执行完所有关键点
        if self.current_idx >= len(self.keypoints):
            print(f"===== NodeWheelMoveTimedCmd({self.display_name}): 所有关键点执行完成")
            actual_list = getattr(self.bb, self.actual_times_key, [])
            if actual_list:
                print(f"===== 各关键点实际执行时间(s): {actual_list}")
            return Status.SUCCESS
        
        # 发送当前关键点
        cmd_vec = self.keypoints[self.current_idx]
        desire_time = self.keypoint_times[self.current_idx]
        
        print(f"===== NodeWheelMoveTimedCmd({self.display_name}): 执行第 {self.current_idx + 1}/{len(self.keypoints)} 个关键点, 期望时间={desire_time}s")
        print(f"cmd_vec: {cmd_vec}")
        success, actual_time = self.timed_cmd_api.send_timed_cmd(
            cmd_type=self.cmd_type,
            cmd_vec=cmd_vec,
            desire_time=desire_time
        )
        
        if not success:
            self.logger.error(f"NodeWheelMoveTimedCmd::update {self.name} - 关键点 {self.current_idx + 1} 执行失败")
            return Status.FAILURE
        
        # 记录并打印当前关键点实际执行时间
        actual_list = getattr(self.bb, self.actual_times_key, [])
        actual_list.append(actual_time)
        setattr(self.bb, self.actual_times_key, actual_list)
        print(f"===== 关键点 {self.current_idx + 1} 实际执行时间: {actual_time:.3f}s")
        
        # 设置等待时间
        self.is_waiting = True
        self.wait_until = time.time() + actual_time + 0.01
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"NodeWheelMoveTimedCmd::terminate {self.name} to {new_status}")


class NodeSetRuckigParams(Behaviour):
    """
    设置Ruckig规划器参数节点
    
    用于在行为树中设置不同规划器的运动参数（速度、加速度、急动度等）。
    臂类为左右单臂：4/5 左/右臂世界系，6/7 左/右臂局部系，8/9 左/右臂关节。
    
    Args:
        name: 节点名称
        timed_cmd_api: TimedCmdAPI 实例
        planner_index: 规划器索引
            - 0: 底盘世界系  1: 底盘局部系  2: 躯干  3: 下肢
            - 4: 左臂笛卡尔世界系  5: 右臂笛卡尔世界系
            - 6: 左臂笛卡尔局部系  7: 右臂笛卡尔局部系
            - 8: 左臂上肢关节  9: 右臂上肢关节
        is_sync: 是否同步模式
        velocity_max: 最大速度列表
        acceleration_max: 最大加速度列表
        jerk_max: 最大急动度列表
        velocity_min: 最小速度列表（可选）
        acceleration_min: 最小加速度列表（可选）
    """
    
    def __init__(self,
                 name,
                 timed_cmd_api: TimedCmdAPI,
                 planner_index: int,
                 is_sync: bool,
                 velocity_max: List[float],
                 acceleration_max: List[float],
                 jerk_max: List[float],
                 velocity_min: List[float] = None,
                 acceleration_min: List[float] = None):
        super(NodeSetRuckigParams, self).__init__(name)
        
        if timed_cmd_api is None:
            raise ValueError("必须提供 timed_cmd_api 参数")
        
        self.timed_cmd_api = timed_cmd_api
        self.planner_index = planner_index
        self.is_sync = is_sync
        self.velocity_max = velocity_max
        self.acceleration_max = acceleration_max
        self.jerk_max = jerk_max
        self.velocity_min = velocity_min
        self.acceleration_min = acceleration_min
        self.executed = False

    def initialise(self):
        self.logger.debug(f"NodeSetRuckigParams::initialise {self.name}")
        self.executed = False

    def update(self):
        self.logger.debug(f"NodeSetRuckigParams::update {self.name}")
        
        if self.executed:
            return Status.SUCCESS
        
        # 调用API，可选参数只在不为None时传递
        if self.velocity_min is not None and self.acceleration_min is not None:
            success = self.timed_cmd_api.set_ruckig_params(
                planner_index=self.planner_index,
                is_sync=self.is_sync,
                velocity_max=self.velocity_max,
                acceleration_max=self.acceleration_max,
                jerk_max=self.jerk_max,
                velocity_min=self.velocity_min,
                acceleration_min=self.acceleration_min
            )
        else:
            success = self.timed_cmd_api.set_ruckig_params(
                planner_index=self.planner_index,
                is_sync=self.is_sync,
                velocity_max=self.velocity_max,
                acceleration_max=self.acceleration_max,
                jerk_max=self.jerk_max
            )
        
        self.executed = True
        
        if success:
            print(f"===== NodeSetRuckigParams({self.name}): 规划器参数设置成功 (planner_index={self.planner_index})")
            return Status.SUCCESS
        else:
            self.logger.error(f"NodeSetRuckigParams::update {self.name} - 规划器参数设置失败")
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"NodeSetRuckigParams::terminate {self.name} to {new_status}")



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
            # print(f'===== Waist: target={self.waist_pos:.1f}°, current={current_angle:.1f}°, error={angle_error:.1f}°')
            
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


class NodeTorsoPose(Behaviour):
    """专门处理躯干位姿控制的节点"""
    def __init__(
            self,
            name: str,
            torso_api: TorsoAPI,
            target_pose: Pose,
            total_time: float = 5.0,
    ):
        super().__init__(name)
        self.torso_api = torso_api
        self.target_pose = target_pose
        self.total_time = total_time
        self.future = None

    def initialise(self):
        self.logger.debug(f"NodeTorsoPose::initialise {self.name}")
        
        self.future = self.torso_api.move_torso_pose(
            desir_torso_pose=self.target_pose,
            asynchronous=True,
            total_time=self.total_time,
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

class NodeWaitChassisReach(Behaviour):
    """等待底盘到达黑板 walk_goal 位姿（xy 与 yaw 在容差内）后再执行下一步。"""
    def __init__(self, name: str, robot_sdk: RobotSDK, pos_tol: float = 0.05, yaw_tol_deg: float = 5.0):
        super().__init__(name)
        self.robot_sdk = robot_sdk
        self.pos_tol = pos_tol
        self.yaw_tol_rad = np.deg2rad(yaw_tol_deg)
        self.bb = py_trees.blackboard.Client(name=f"{name}_bb")
        self.bb.register_key(key="walk_goal", access=py_trees.common.Access.READ)

    def _yaw_diff_rad(self, current: float, target: float) -> float:
        """yaw 角度差（弧度），归一化到 [-pi, pi]。"""
        d = current - target
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

    def update(self):
        walk_goal = getattr(self.bb, "walk_goal", None)
        if walk_goal is None:
            rospy.logwarn_throttle(1.0, "NodeWaitChassisReach: 黑板无 walk_goal")
            return Status.RUNNING
        cur = self.robot_sdk.tools.get_link_pose(link_name="base_link", reference_frame=Frame.ODOM)
        if cur is None:
            rospy.logwarn_throttle(1.0, "NodeWaitChassisReach: 获取底盘位姿失败")
            return Status.RUNNING
        cur_pose = Pose(pos=cur.position, quat=cur.orientation, frame=Frame.ODOM)
        cur_yaw = cur_pose.get_euler(degrees=False)[2]
        goal_yaw = walk_goal.get_euler(degrees=False)[2]
        dx = abs(float(walk_goal.pos[0]) - cur_pose.pos[0])
        dy = abs(float(walk_goal.pos[1]) - cur_pose.pos[1])
        dyaw = abs(self._yaw_diff_rad(cur_yaw, goal_yaw))
        if dx <= self.pos_tol and dy <= self.pos_tol and dyaw <= self.yaw_tol_rad:
            rospy.loginfo(f"✅ 底盘到位: dx={dx:.3f}, dy={dy:.3f}, dyaw={np.rad2deg(dyaw):.2f}°")
            return Status.SUCCESS
        rospy.logwarn_throttle(1.0, f"⚠️ 底盘未到位: erros_x={dx:.3f}, erros_y={dy:.3f}, erros_yaw={np.rad2deg(dyaw):.2f}° (容差 pos={self.pos_tol}, yaw={np.rad2deg(self.yaw_tol_rad):.2f}°)")
        return Status.RUNNING


# --------------- 底盘运动到 Tag 节点 ---------------


class NodeChassisMoveToTag(py_trees.composites.Sequence):
    """
    底盘运动到 Tag 节点：依次执行「设置目标 → 写入关键点 → 执行运动 → 等待到位」。

    执行流程：
        1. 从黑板 latest_tag_{tag_id} 读取 tag，计算目标位姿，写入 walk_goal
        2. 从 walk_goal 生成底盘关键点，写入 chassis_world_keypoints
        3. [可选] 执行 prepare_nodes（如设置手臂预关键点）
        4. 执行底盘运动；若指定 parallel_with，则与 parallel_with 并行执行
        5. 等待底盘到达 walk_goal

    Args:
        prepare_nodes: 底盘运动前执行的节点（如设置手臂预关键点）
        parallel_with: 与底盘运动并行的节点（如手臂预动作）
    """

    def __init__(
        self,
        name: str,
        robot_sdk: RobotSDK,
        timed_cmd_api: TimedCmdAPI,
        tag_id: int,
        stand_pos: tuple,
        stand_euler: tuple,
        chassis_time: float = 7.0,
        pos_tol: float = 0.06,
        yaw_tol_deg: float = 5.0,
        prepare_nodes=None,
        parallel_with=None,
    ):
        self.robot_sdk = robot_sdk
        self.tag_id = tag_id
        self.stand_pos = stand_pos
        self.stand_euler = stand_euler
        self.chassis_time = chassis_time

        chassis_api = ChassisApi(robot_sdk)

        # Step 1: 从 tag 设置 walk_goal
        set_walk_goal_fn = lambda: chassis_api.set_walk_goal_from_tag(tag_id, stand_pos, stand_euler)
        step_set_goal = NodeFuntion(name="set_walk_goal", fn=set_walk_goal_fn)

        # Step 2: 从 walk_goal 设置底盘关键点
        set_keypoints_fn = lambda: chassis_api.set_chassis_keypoints_from_walk_goal(chassis_time)
        step_set_keypoints = NodeFuntion(name="set_chassis_keypoints", fn=set_keypoints_fn)

        # Step 3: 底盘运动
        step_chassis_move = NodeWheelMoveTimedCmd(
            name="chassis_move",
            timed_cmd_api=timed_cmd_api,
            cmd_type="chassis_world",
        )

        # Step 4: 等待底盘到位
        step_wait_reach = NodeWaitChassisReach(
            name="wait_chassis_reach",
            robot_sdk=robot_sdk,
            pos_tol=pos_tol,
            yaw_tol_deg=yaw_tol_deg,
        )

        # 组装子节点：1→2→[prepare]→3→4；step3 可与 parallel_with 并行
        children = [step_set_goal, step_set_keypoints]

        if prepare_nodes:
            prep = prepare_nodes if isinstance(prepare_nodes, list) else [prepare_nodes]
            children.extend(prep)

        if parallel_with is not None:
            move_parallel = py_trees.composites.Parallel(
                name="chassis_and_parallel",
                policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
            )
            move_parallel.add_children([step_chassis_move, parallel_with])
            children.append(move_parallel)
        else:
            children.append(step_chassis_move)

        children.append(step_wait_reach)
        super().__init__(name=name, memory=True, children=children)


class NodeArmForce(Behaviour):
    """
    为左右手末端施加/撤销“托箱”期望力和外力的节点。
    - 发布 /desired_ee_force/left 与 /desired_ee_force/right (WrenchStamped)
    - 发布 /external_wrench/left_hand 与 /external_wrench/right_hand (Wrench)
    左右手各施加 box_weight_kg * 9.8 / 2 的力，方向为末端坐标系 -Z 方向。
    """

    def __init__(
        self,
        name: str,
        box_weight_kg: float,
        enable: bool = True,
        transition_time: Optional[float] = None,
        interpolation_speed: Optional[float] = None,
    ):
        super().__init__(name)
        self.box_weight_kg = max(0.0, float(box_weight_kg))
        self.enable = enable
        self.transition_time = transition_time
        self.interpolation_speed = interpolation_speed
        self._interp_done = False

        # 期望末端力（接入 DesiredForceManager）
        self.pub_desired_left = rospy.Publisher(
            "/desired_ee_force/left", WrenchStamped, queue_size=10
        )
        self.pub_desired_right = rospy.Publisher(
            "/desired_ee_force/right", WrenchStamped, queue_size=10
        )

        # 仿真中的附加外力（MuJoCo/Gazebo）
        self.pub_ext_left = rospy.Publisher(
            "/external_wrench/left_hand", Wrench, queue_size=10
        )
        self.pub_ext_right = rospy.Publisher(
            "/external_wrench/right_hand", Wrench, queue_size=10
        )

    def update(self):
        if not self._interp_done:
            self._interp_done = True
            t, s = self.transition_time, self.interpolation_speed
            if t is not None and s is not None and float(t) > 0 and float(s) > 0:
                try:
                    rospy.wait_for_service("/set_contact_force_params", timeout=2.0)
                    req = setContactForceInterpParamsRequest()
                    req.transition_time = float(t)
                    req.interpolation_speed = float(s)
                    rospy.ServiceProxy("/set_contact_force_params", setContactForceInterpParams)(req)
                except (rospy.ROSException, rospy.ServiceException):
                    pass
        # 左右手各施加箱子重量的一半
        force_n = 0.5 * self.box_weight_kg * 9.8
        if not self.enable:
            force_n = 0.0

        # WrenchStamped: 供 DesiredForceManager 使用
        now = rospy.Time.now()
        for hand, pub in [("left_hand", self.pub_desired_left), ("right_hand", self.pub_desired_right)]:
            msg = WrenchStamped()
            msg.header.stamp = now
            # 使用 hand 名称作为 frame_id，便于管理
            msg.header.frame_id = hand
            msg.wrench.force.x = 0.0
            msg.wrench.force.y = 0.0
            msg.wrench.force.z = -force_n  # 末端 -Z 方向
            msg.wrench.torque.x = 0.0
            msg.wrench.torque.y = 0.0
            msg.wrench.torque.z = 0.0
            pub.publish(msg)

        # Wrench: 供仿真附加外力使用
        wrench = Wrench()
        wrench.force.x = 0.0
        wrench.force.y = 0.0
        wrench.force.z = -force_n
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = 0.0
        self.pub_ext_left.publish(wrench)
        self.pub_ext_right.publish(wrench)

        rospy.loginfo(
            f"[NodeArmForce] enable={self.enable}, box_weight_kg={self.box_weight_kg:.3f}, force_z={-force_n:.3f} N"
        )
        # 单次设置即可，返回 SUCCESS
        return Status.SUCCESS


# --------------- Tag 识别节点（合并 tag_source + log + validate）---------------


class NodeHeadSearchTag(py_trees.composites.Sequence):
    """
    Tag 识别节点：tag_source（虚假/头部扫描）+ log + validate。
    将原 tag_source_node、log_pick_tag_node、log_place_tag_node、validate_tag_euler_node 合并为一。
    config 需包含：common (tag_euler_error, tag_euler_error_validate_yaw, head_move_delay),
    fake_tags (enable, pick_pos/euler, place_pos/euler), pick (tag_id, tag_euler_world), place (tag_id, tag_euler_world)。
    tag_ids 顺序应与 config.pick/place 一致，通常为 [pick_tag_id, place_tag_id]。
    """

    @staticmethod
    def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
        """两角度差（度），考虑 360° 周期归一化。"""
        d = abs(a_deg - b_deg)
        return min(d, 360.0 - d)

    def _log_from_blackboard(self, tag_id: int) -> bool:
        """从黑板读取 tag 并在终端打印完整 tag 数据。"""
        bb = py_trees.blackboard.Client(name=f"tag_logger_{tag_id}")
        bb.register_key(key=f"latest_tag_{tag_id}", access=py_trees.common.Access.READ)
        tag = getattr(bb, f"latest_tag_{tag_id}", None)
        if tag is None or not hasattr(tag, "pose"):
            rospy.logwarn(f"⚠️ 无法在黑板上读取 latest_tag_{tag_id}")
            sys.exit(1)
        pos = tag.pose.pos if hasattr(tag.pose, "pos") else tag.pose
        quat = tag.pose.quat if hasattr(tag.pose, "quat") else None
        print(f"========== 识别到的 Tag (tag_id={tag_id}) 数据 ==========")
        print(f"  tag.id:    {getattr(tag, 'id', 'N/A')}")
        print(f"  tag.size:  {getattr(tag, 'size', 'N/A')}")
        print(f"  位置 pos:  [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        if quat is not None:
            print(f"  姿态 quat: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
            if hasattr(tag.pose, "get_euler"):
                euler = tag.pose.get_euler(degrees=True)
                print(f"  姿态 euler(度): [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]")
        return True

    def _validate_euler(
        self,
        tag_id: int,
        expected_euler_deg: tuple,
        threshold_deg: float,
        validate_yaw: bool = False,
    ) -> bool:
        """审查识别结果与预期 tag_euler_world：必查 roll+pitch，可选 yaw；总误差 >= 阈值则报错退出。"""
        bb = py_trees.blackboard.Client(name=f"tag_euler_validate_{tag_id}")
        bb.register_key(key=f"latest_tag_{tag_id}", access=py_trees.common.Access.READ)
        tag = getattr(bb, f"latest_tag_{tag_id}", None)
        if tag is None or not hasattr(tag, "pose") or not hasattr(tag.pose, "get_euler"):
            rospy.logerr(f"❌ 审查 tag {tag_id}：黑板上无有效 latest_tag_{tag_id} 或 pose 无 get_euler")
            sys.exit(1)
        recv = tag.pose.get_euler(degrees=True)
        exp = expected_euler_deg
        total_err = self._angle_diff_deg(recv[0], exp[0]) + self._angle_diff_deg(recv[1], exp[1])
        if validate_yaw:
            total_err += self._angle_diff_deg(recv[2], exp[2])
        if total_err >= threshold_deg:
            axes = "roll+pitch+yaw" if validate_yaw else "roll+pitch"
            recv_str = f"[{recv[0]:.2f}, {recv[1]:.2f}, {recv[2]:.2f}]" if validate_yaw else f"[{recv[0]:.2f}, {recv[1]:.2f}]"
            exp_str = f"[{exp[0]:.2f}, {exp[1]:.2f}, {exp[2]:.2f}]" if validate_yaw else f"[{exp[0]:.2f}, {exp[1]:.2f}]"
            rospy.logerr(
                f"❌ Tag {tag_id} 欧拉角审查未通过：识别 {recv_str}°，预期 {exp_str}°，"
                f"{axes} 误差合计 {total_err:.2f}° >= 阈值 {threshold_deg}°，程序退出"
            )
            sys.exit(1)
        axes_str = "roll+pitch+yaw" if validate_yaw else "roll+pitch"
        rospy.loginfo(f"✅ Tag {tag_id} 欧拉角审查通过：{axes_str} 误差 {total_err:.2f}° < {threshold_deg}°")
        return True

    def _print_on_timeout(self) -> None:
        """超时退出前调用：从黑板读取并打印所有已识别到的 tag 数据。"""
        bb = py_trees.blackboard.Client(name="timeout_tag_printer")
        for tid in self.tag_ids:
            key = f"latest_tag_{tid}"
            try:
                bb.register_key(key=key, access=py_trees.common.Access.READ)
            except Exception:
                pass
            tag = getattr(bb, key, None)
            if tag is None or not hasattr(tag, "pose"):
                continue
            pos = tag.pose.pos if hasattr(tag.pose, "pos") else tag.pose
            quat = getattr(tag.pose, "quat", None)
            print("========== 超时前已识别到的 Tag 数据 ==========")
            print(f"  --- tag_id={tid} ---")
            print(f"    tag.id:    {getattr(tag, 'id', 'N/A')}")
            print(f"    tag.size:  {getattr(tag, 'size', 'N/A')}")
            print(f"    位置 pos:  [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
            if quat is not None:
                print(f"    姿态 quat: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
                if hasattr(tag.pose, "get_euler"):
                    euler = tag.pose.get_euler(degrees=True)
                    print(f"    姿态 euler(度): [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]")

    def _set_fake_to_blackboard(self) -> bool:
        """设置虚假 tag 到黑板。"""
        tag_configs = self._get_fake_tag_configs()
        for tid, pos, euler in tag_configs:
            tag = Tag(id=tid, pose=Pose.from_euler(pos=pos, euler=euler, frame=Frame.ODOM, degrees=True))
            bb = py_trees.blackboard.Client(name=f"set_fake_tag_{tid}")
            bb.register_key(f"latest_tag_{tid}", py_trees.common.Access.WRITE)
            bb.register_key(f"latest_tag_{tid}_version", py_trees.common.Access.WRITE)
            setattr(bb, f"latest_tag_{tid}", tag)
            setattr(bb, f"latest_tag_{tid}_version", 1)
        rospy.loginfo(f"✅ 虚假 tag 已设置: {[(c[0], c[1]) for c in tag_configs]}")
        return True

    def _get_fake_tag_configs(self) -> list:
        """获取虚假 tag 配置：[(tag_id, pos, euler), ...]。"""
        return [
            (self.tag_ids[0], self.config.fake_tags.pick_pos, self.config.fake_tags.pick_euler),
            (self.tag_ids[1], self.config.fake_tags.place_pos, self.config.fake_tags.place_euler),
        ]

    def _build_head_search_tree(self) -> Behaviour:
        """创建头部搜索子树，一次扫描可同时检测多个 tag。"""
        tag_ids = list(self.tag_ids)
        tag_timeout_msg = (
            f"❌ 相机无法识别 tag_id: {tag_ids}，超时 {self.tag_wait_timeout}s 后仍未检测到，程序退出"
        )
        percep = NodePercep(
            name="PERCEP_tags_" + "_".join(map(str, tag_ids)),
            robot_sdk=self.robot_sdk,
            tag_ids=tag_ids,
            tag_stable_count=self.tag_stable_count,
            tag_pos_std_threshold=self.tag_pos_std_threshold,
        )
        head_seq = py_trees.composites.Sequence(
            name="head_search_tags_" + "_".join(map(str, tag_ids)), memory=True
        )
        head_move_nodes = []
        for yaw in self.head_search_yaws:
            for pitch in self.head_search_pitchs:
                def make_head_node(y, p, rsdk):
                    return NodeFuntion(
                        name=f"head_move_yaw_{np.rad2deg(y):.0f}_pitch_{np.rad2deg(p):.0f}",
                        fn=lambda y=y, p=p: rsdk.control.control_head(y, p) or True,
                    )
                head_move_nodes.append(make_head_node(yaw, pitch, self.robot_sdk))
                head_move_nodes.append(NodeDelay(duration=self.head_move_delay, name="delay_after_head_move"))

        on_timeout_print_tags = self._print_on_timeout
        if len(tag_ids) == 1:
            wait_node = NodeWaitForBlackboard(
                key=f"latest_tag_{tag_ids[0]}",
                timeout=self.tag_wait_timeout,
                exit_on_timeout=True,
                timeout_message=tag_timeout_msg,
                on_timeout_callable=on_timeout_print_tags,
            )
        else:
            wait_node = py_trees.composites.Parallel(
                name="wait_all_tags",
                policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
            )
            for tid in tag_ids:
                wait_node.add_child(
                    NodeWaitForBlackboard(
                        key=f"latest_tag_{tid}",
                        timeout=self.tag_wait_timeout,
                        exit_on_timeout=True,
                        timeout_message=f"❌ 相机无法识别 tag_id: {tid}，超时 {self.tag_wait_timeout}s 后仍未检测到，程序退出",
                        on_timeout_callable=on_timeout_print_tags,
                    )
                )
        head_return_0_node = NodeFuntion(
            name="head_return_0",
            fn=lambda: self.robot_sdk.control.control_head(0.0, 0.0) or True,
        )
        head_seq.add_children(head_move_nodes + [wait_node, head_return_0_node])
        root = py_trees.composites.Parallel(
            name="head_search_parallel",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        )
        root.add_children([head_seq, percep])
        return root

    def _log_and_validate_all(self) -> bool:
        """打印所有 tag 并做欧拉角审查。"""
        for tid in self.tag_ids:
            self._log_from_blackboard(tid)
        validate_yaw = getattr(self.config.common, "tag_euler_error_validate_yaw", True)
        threshold = getattr(self.config.common, "tag_euler_error", 15.0)
        self._validate_euler(
            self.tag_ids[0], self.config.pick.tag_euler_world, threshold, validate_yaw=validate_yaw
        )
        self._validate_euler(
            self.tag_ids[1], self.config.place.tag_euler_world, threshold, validate_yaw=validate_yaw
        )
        return True

    def __init__(
        self,
        name: str = "tag_recognition",
        robot_sdk: RobotSDK = None,
        tag_ids: list = None,
        config=None,
        head_search_yaws=None,
        head_search_pitchs=None,
        head_move_delay: float = 0.5,
        tag_wait_timeout: float = 1.0,
        tag_stable_count: int = 10,
        tag_pos_std_threshold: float = 0.05,
    ):
        self.robot_sdk = robot_sdk
        self.tag_ids = list(tag_ids) if tag_ids else []
        self.config = config
        self.head_search_yaws = head_search_yaws if head_search_yaws is not None else np.deg2rad([-90, 90])
        self.head_search_pitchs = head_search_pitchs if head_search_pitchs is not None else np.deg2rad([10, 0, -10])
        self.head_move_delay = getattr(config.common, "head_move_delay", head_move_delay) if config else head_move_delay
        self.tag_wait_timeout = tag_wait_timeout
        self.tag_stable_count = tag_stable_count
        self.tag_pos_std_threshold = tag_pos_std_threshold

        # 1. tag source 子节点
        if getattr(config.fake_tags, "enable", False):
            tag_source_child = NodeFuntion(
                name="set_fake_tags",
                fn=self._set_fake_to_blackboard,
            )
        else:
            head_search_tree = self._build_head_search_tree()
            tag_source_child = py_trees.decorators.OneShot(
                name="head_search_once",
                child=head_search_tree,
                policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
            )

        # 2. log + validate 子节点
        log_validate_node = NodeFuntion(
            name="log_and_validate_tags",
            fn=self._log_and_validate_all,
        )

        children = [tag_source_child, log_validate_node]
        super().__init__(name=name, memory=True, children=children)
