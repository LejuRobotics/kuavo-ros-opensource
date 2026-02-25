from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from py_trees.common import Status
import tf.transformations as tft

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint, EventNavigationToPose)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation
from kuavo_humanoid_sdk import KuavoRobot, KuavoSDK, KuavoRobotState
import py_trees

from py_trees.behaviour import Behaviour
import math

import time

from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy import navigation_approach_target_with_perception_loop
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy_slam_optimize import start_navigation


class MoveToTargetslam(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params):
        super(MoveToTargetslam, self).__init__(name)
        self.params = params
        self.move_mode = self.params.get('move_mode')
        self.started = False
        self.target_pose = None
        self.robot_type = None
        self.walk_opened = False
        # 运行期状态

        self._imu_sub     = None
        self._latest_yaw  = 0.0
        # 延迟初始化，避免启动时创建（可能很慢）
        self.robot = None
        self.robot_sdk = None
        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)


        self.global_blackboard = self.attach_blackboard_client()
        self.global_blackboard.register_key(key="TargetPoseOdom", access=py_trees.common.Access.READ)
        # self.global_blackboard.register_key(key="SLAMNavDest", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Common_robot_type", access=py_trees.common.Access.READ)

        self.global_blackboard.register_key(key="Walk_WalkEventTimeout", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Walk_WalkEventYawThreshold", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Walk_WalkEventPosThreshold", access=py_trees.common.Access.READ)
        self.mode = self.params.get('mode', 'relative_pose')

        # 延迟初始化，避免启动时创建（可能很慢）
        self.robot_state = None

        # 初始化事件（延迟初始化，因为需要 robot_sdk）
        # 注意：walk_event 和 navigation_event 会在需要时延迟创建
        self.walk_event = None
        self.navigation_event = None

        # aruco-navigation events（延迟初始化）
        self.aruco_navigation_event = None
        self.aruco_percep_event = None
        self.aruco_odom = None

        # SLAM坐标相关（延迟初始化）
        self.slam_current_pose = None
        rospy.Subscriber("/move_base/amcl_pose", PoseWithCovarianceStamped, self._slam_pose_callback)

    def _ensure_robot_initialized(self):
        """延迟初始化机器人对象，避免启动时创建（可能很慢）"""
        if self.robot is None:
            self.robot = KuavoRobot()
        if self.robot_sdk is None:
            self.robot_sdk = RobotSDK()
        if self.robot_state is None:
            self.robot_state = KuavoRobotState()

    def _ensure_walk_event_initialized(self):
        """延迟初始化行走事件"""
        if self.walk_event is None:
            walk_timeout = int(getattr(self.global_blackboard, "Walk_WalkEventTimeout", 60))
            walk_yaw_threshold = float(getattr(self.global_blackboard, "Walk_WalkEventYawThreshold", 0.1))
            walk_pos_threshold = float(getattr(self.global_blackboard, "Walk_WalkEventPosThreshold", 0.05))
            self._ensure_robot_initialized()
            self.walk_event = EventWalkToPose(
                robot_sdk=self.robot_sdk,
                timeout=walk_timeout,
                yaw_threshold=walk_yaw_threshold,
                pos_threshold=walk_pos_threshold,
                control_mode='cmd_pos_world'
            )

    def _ensure_navigation_event_initialized(self):
        """延迟初始化导航事件"""
        if self.navigation_event is None:
            walk_timeout = int(self.params.get('Walk_WalkEventTimeout', 60))
            self._ensure_robot_initialized()
            self.navigation_event = EventNavigationToPose(
                robot_sdk=self.robot_sdk,
                timeout=walk_timeout,
                yaw_threshold=0.1,
                pos_threshold=0.3,
                control_mode='cmd_pos_world'
            )

    def _ensure_aruco_events_initialized(self):
        """延迟初始化aruco导航事件"""
        if self.aruco_navigation_event is None:
            robot_sdk = RobotSDK()
            self.aruco_navigation_event = EventNavigationToPose(
                robot_sdk=robot_sdk,
                timeout=1000,
                yaw_threshold=0.1,
                pos_threshold=0.3,
                control_mode="cmd_pos_world"
            )
            self.aruco_percep_event = EventPercep(
                robot_sdk=robot_sdk,
                half_fov=60,
                timeout=np.inf
            )
            rospy.Subscriber("aruco_pose1", PoseStamped, lambda msg: self.__setattr__('aruco_odom', msg))

    def _slam_pose_callback(self, msg):
        """SLAM坐标回调函数"""
        self.slam_current_pose = msg.pose.pose

    def initialise(self):
        """初始化节点"""
        self.started = False
        self.target_pose = None
        self.feedback_message = f"准备执行 {self.move_mode} 移动"
        # if not KuavoSDK.Init():
        #     self.feedback_message = f"KuavoSDK 初始化失败"
        #     print(self.feedback_message)
        #     exit(1)
        print(f"In MoveToTarget, self.mode = {self.mode}")
        self._initiate_movement()

    def update(self):
        try:
            # 执行移动步骤
            return self._execute_movement()

        except Exception as e:
            rospy.logerr(f"[{self.name}] 移动失败: {e}")
            self.feedback_message = f"移动失败: {e}"
            # 确保资源被清理
            if self.walk_opened and self.walk_event is not None:
                self.walk_event.close()
            return Status.FAILURE

    def _initiate_movement(self):
        # 延迟初始化机器人对象
        self._ensure_robot_initialized()

        if self.mode == "aruco_nav":
            self._ensure_aruco_events_initialized()
            self._ensure_navigation_event_initialized()
            x_dist = float(self.params.get('x', 0))
            y_dist = float(self.params.get('y', 0))
            yaw = float(self.params.get('yaw', 0))
            print(f"aruco navigation, target pose is {x_dist, y_dist, yaw}")
            success = navigation_approach_target_with_perception_loop(
                navigation_event = self.aruco_navigation_event,
                percep_event = self.aruco_percep_event,
                posex=x_dist,
                posey=y_dist,
                posez=yaw,
                enable_percep_when_walking=True
            )
            if success:
                self.robot.stance()
                time.sleep(0.5)
                orientation = self.aruco_odom.pose.orientation
                position = self.aruco_odom.pose.position
                current_robot_pose = Pose([position.x, position.y, position.z],[orientation.x, orientation.y, orientation.z, orientation.w])
                yaw_diff = yaw - current_robot_pose.get_euler(degrees=True)[2]
                destination = [0, 0, 0, math.radians(yaw_diff)]
                self.logger.info(f"single step navigation, dist: {destination}")
                self.navigation_event.single_step_navigation(destination, step_type=1)
            return Status.RUNNING

        elif self.mode == "step_by_step":
            self.robot_type = self.global_blackboard.Common_robot_type

            # 初始化行走事件
            print(f"self.robot_type = {self.robot_type}")
            if self.robot_type == "kuavo_lb":
                self._ensure_walk_event_initialized()
                self.walk_event.open()
                yaw = float(self.params.get('yaw', 0))
                self.walk_event.set_control_mode('cmd_pos_world')
                current_pos = self.robot_sdk.state.robot_position()
                self.target_pose = Pose.from_euler(
                    pos=(current_pos[0], current_pos[1], 0),
                    euler=(0, 0, yaw),
                    frame=Frame.BASE,
                    degrees=True
                )
                self.walk_event.set_target(self.target_pose)
            else:
                self.robot.stance()
                time.sleep(0.5)
                yaw = float(self.params.get('yaw', 0))
                destination = [0, 0, 0, math.radians(yaw)]
                print(f"step by step, target yaw is {yaw}, destination = {destination}")
                self._ensure_navigation_event_initialized()
                self.navigation_event.single_step_navigation(destination, step_type=1)
                return Status.RUNNING
            """初始化移动"""
        elif self.mode == "relative_pose":
            print(f"In MoveToTarget, self.mode = {self.mode}, self.params = {self.params}")
            x_dist = float(self.params.get('x', 0))
            y_dist = float(self.params.get('y', 0))
            yaw = float(self.params.get('yaw', 0))

            # 创建相对位移目标
            self.target_pose = Pose.from_euler(
                pos=(x_dist, y_dist, 0),
                euler=(0, 0, yaw),
                frame=Frame.BASE,
                degrees=True
            )

            # 初始化行走事件
            self._ensure_walk_event_initialized()
            self.walk_event.open()
            self.walk_event.utils_enable_base_pitch_limit(True)  # 启用基座俯仰限制
            self.walk_event.set_control_mode('cmd_pos')
            self.walk_event.set_target(self.target_pose)


        elif self.mode == "absolute_pose":
            self.logger.info(f"In MoveToTarget, self.params = {self.params}")
            # x_dist = float(self.params.get('x', 0))
            # y_dist = float(self.params.get('y', 0))
            x_dist = self.robot_sdk.state.robot_position()[0]
            y_dist = self.robot_sdk.state.robot_position()[1]
            # yaw = float(self.params.get('yaw', 0))
            yaw = self.robot_sdk.state.robot_orientation()[2] + float(self.params.get('yaw',0))
            # 归一化yaw到[-180, 180]度范围
            yaw = ((yaw + 180) % 360) - 180

            # 创建相对位移目标
            self.target_pose = Pose.from_euler(
                pos=(x_dist, y_dist, 0),
                euler=(0, 0, yaw),
                frame=Frame.ODOM,
                degrees=True
            )

            # 初始化行走事件
            self._ensure_walk_event_initialized()
            self.walk_event.open()
            self.walk_event.utils_enable_base_pitch_limit(True)  # 启用基座俯仰限制
            if self.robot_type == "kuavo_lb":
                self.walk_event.set_control_mode('cmd_pos_world')
            else:
                self.walk_event.set_control_mode('cmd_vel')
            self.walk_event.set_target(self.target_pose)

            self.feedback_message = f"执行absolute pose: {x_dist:.2f}m, {y_dist:.2f}, {yaw:.2f}m"

        elif self.mode == "tag_target":
            self.target_pose = self.global_blackboard.TargetPoseOdom  # 目标位姿
            self.robot_type = self.global_blackboard.Common_robot_type
            self.leg_mode = self.params.get("leg_mode", "target")
            self.logger.info(f"self.TargetPoseOdom: {self.target_pose}")
            self.logger.info(f"self.robot_type: {self.robot_type}")

            # 初始化行走事件
            self._ensure_walk_event_initialized()
            self.walk_event.open()
            if self.robot_type == "kuavo_lb":
                self.walk_event.set_control_mode('cmd_pos_world')
                if self.leg_mode == "leg":
                    # 获取当前欧拉角（度数），只旋转yaw角180度，保持roll和pitch不变
                    print(f"self.target_pose or: {self.target_pose}")
                    current_euler_deg = self.target_pose.get_euler(degrees=True)
                    new_yaw_deg = current_euler_deg[2] - 180
                    # 保持原始坐标系不变
                    original_frame = self.target_pose.frame
                    self.target_pose = Pose.from_euler(
                        pos=[self.target_pose.pos[0], self.target_pose.pos[1], self.target_pose.pos[2]],
                        euler=[current_euler_deg[0], current_euler_deg[1], new_yaw_deg],  # 保持roll和pitch，只修改yaw
                        frame=original_frame,  # 保持原始坐标系
                        degrees=True
                    )
                    print(f"self.target_pose change: {self.target_pose}")
            else:
                self.walk_event.set_control_mode('cmd_vel')
            self.walk_event.set_target(self.target_pose)

            self.feedback_message = "执行绝对移动"

        elif self.mode == "slam":
            self.robot_type = self.global_blackboard.Common_robot_type
            # 从params读取x, y, yaw
            if 'x' in self.params and 'y' in self.params and 'yaw' in self.params:
                self.slam_target_pose = [
                    float(self.params.get('x', 0)),
                    float(self.params.get('y', 0)),
                    float(self.params.get('yaw', 0))
                ]
            elif 'x' in self.params and 'y' in self.params and 'theta' in self.params:
                # 兼容theta参数名
                self.slam_target_pose = [
                    float(self.params.get('x', 0)),
                    float(self.params.get('y', 0)),
                    float(self.params.get('theta', 0))
                ]
            else:
                self.logger.error(f"SLAM模式需要提供目标坐标（x/y/yaw参数）")
                return Status.FAILURE
            print(f"self.slam_target_pose: {self.slam_target_pose}")
            print(f"self.robot_type: {self.robot_type}")

            # 初始化行走事件
            if self.robot_type == "kuavo_lb":
                # 等待SLAM坐标数据
                max_wait_time = 5.0
                wait_start = time.time()
                while self.slam_current_pose is None and (time.time() - wait_start) < max_wait_time:
                    rospy.sleep(0.1)

                if self.slam_current_pose is None:
                    self.logger.error(f"等待SLAM坐标超时")
                    return Status.FAILURE

                # 获取SLAM坐标系下的当前位置
                slam_current_pos = [
                    self.slam_current_pose.position.x,
                    self.slam_current_pose.position.y
                ]
                slam_current_quat = [
                    self.slam_current_pose.orientation.x,
                    self.slam_current_pose.orientation.y,
                    self.slam_current_pose.orientation.z,
                    self.slam_current_pose.orientation.w
                ]
                slam_current_yaw = R.from_quat(slam_current_quat).as_euler('xyz', degrees=True)[2]

                # 计算SLAM坐标系下的偏移量
                slam_offset_x = self.slam_target_pose[0] - slam_current_pos[0]
                slam_offset_y = self.slam_target_pose[1] - slam_current_pos[1]
                slam_offset_yaw = self.slam_target_pose[2] - slam_current_yaw

                # 归一化yaw偏移到[-180, 180]度范围
                slam_offset_yaw = ((slam_offset_yaw + 180) % 360) - 180

                print(f"SLAM当前位置: x={slam_current_pos[0]:.3f}, y={slam_current_pos[1]:.3f}, yaw={slam_current_yaw:.3f}")
                print(f"SLAM目标位置: x={self.slam_target_pose[0]:.3f}, y={self.slam_target_pose[1]:.3f}, yaw={self.slam_target_pose[2]:.3f}")
                print(f"SLAM偏移量: dx={slam_offset_x:.3f}, dy={slam_offset_y:.3f}, dyaw={slam_offset_yaw:.3f}")

                # 获取odom坐标系下的当前位置
                odom_current_pos = self.robot_sdk.state.robot_position()
                odom_current_quat = self.robot_sdk.state.robot_orientation()
                odom_current_yaw = R.from_quat([
                    odom_current_quat[0],
                    odom_current_quat[1],
                    odom_current_quat[2],
                    odom_current_quat[3]
                ]).as_euler('xyz', degrees=True)[2]

                print(f"ODOM当前位置: x={odom_current_pos[0]:.3f}, y={odom_current_pos[1]:.3f}, yaw={odom_current_yaw:.3f}")

                # 将SLAM偏移量转换到odom坐标系
                # 由于SLAM和odom都是世界坐标系，假设它们的方向一致，直接应用偏移量
                # 将偏移量从SLAM坐标系旋转到odom坐标系
                slam_to_odom_yaw_diff = odom_current_yaw - slam_current_yaw
                slam_to_odom_yaw_diff_rad = math.radians(slam_to_odom_yaw_diff)

                # 旋转偏移向量到odom坐标系
                cos_yaw = math.cos(slam_to_odom_yaw_diff_rad)
                sin_yaw = math.sin(slam_to_odom_yaw_diff_rad)
                odom_offset_x = slam_offset_x * cos_yaw - slam_offset_y * sin_yaw
                odom_offset_y = slam_offset_x * sin_yaw + slam_offset_y * cos_yaw
                odom_offset_yaw = slam_offset_yaw

                # 计算odom目标位置
                odom_target_x = odom_current_pos[0] + odom_offset_x
                odom_target_y = odom_current_pos[1] + odom_offset_y
                odom_target_yaw = odom_current_yaw + odom_offset_yaw
                # 归一化yaw到[-180, 180]度范围
                odom_target_yaw = ((odom_target_yaw + 180) % 360) - 180

                print(f"ODOM目标位置: x={odom_target_x:.3f}, y={odom_target_y:.3f}, yaw={odom_target_yaw:.3f}")

                # 创建odom目标位姿
                self.target_pose = Pose.from_euler(
                    pos=(odom_target_x, odom_target_y, 0),
                    euler=(0, 0, odom_target_yaw),
                    frame=Frame.ODOM,
                    degrees=True
                )

                # 初始化行走事件
                self._ensure_walk_event_initialized()
                self.walk_event.open()
                self.walk_event.utils_enable_base_pitch_limit(True)  # 启用基座俯仰限制
                self.walk_event.set_control_mode('cmd_pos_world')
                self.walk_event.set_target(self.target_pose)

                self.feedback_message = f"执行SLAM导航: odom目标({odom_target_x:.2f}m, {odom_target_y:.2f}m, {odom_target_yaw:.2f}°)"
            else:
                walk_status = start_navigation(self.slam_target_pose[0], self.slam_target_pose[1], self.slam_target_pose[2])
                if walk_status  == True:
                    self.feedback_message = "✅ 移动完成"
                    print(f"self.feedback_message: {self.feedback_message}")
                    self.slam_nav_status = Status.SUCCESS
                elif walk_status == False:
                # 移动失败
                    self.feedback_message = "❌ 移动失败"
                    print(f"self.feedback_message: {self.feedback_message}")
                    self.slam_nav_status = Status.FAILURE
        else:
            self.logger.error(f"未知的移动模式: {self.mode}")
            return Status.FAILURE

        self.started = True
        return Status.RUNNING

    def _execute_movement(self):
        # 延迟初始化机器人对象
        self._ensure_robot_initialized()

        if self.mode == 'slam':
            if self.robot_type == "kuavo_lb":
                # 使用walk_event执行移动
                self._ensure_walk_event_initialized()
                walk_status = self.walk_event.step()

                if walk_status == EventStatus.SUCCESS:
                    # 移动成功完成
                    self.walk_event.close()
                    self.feedback_message = "✅ SLAM导航完成"
                    self.robot.stance()
                    return Status.SUCCESS
                elif walk_status == EventStatus.FAILED:
                    # 移动失败
                    self.walk_event.close()
                    self.feedback_message = "❌ SLAM导航失败"
                    return Status.FAILURE
                elif walk_status == EventStatus.TIMEOUT:
                    # 移动超时
                    self.walk_event.close()
                    self.feedback_message = "⏰ SLAM导航超时"
                    return Status.FAILURE
                else:
                    # 移动仍在进行中
                    current_pos = self.robot_sdk.state.robot_position()
                    target_pos = self.target_pose.pos
                    distance = np.linalg.norm([current_pos[0] - target_pos[0], current_pos[1] - target_pos[1]])
                    self.feedback_message = f"SLAM导航中，剩余距离: {distance:.2f}m"
                    return Status.RUNNING
            else:
                self.robot.stance()
                time.sleep(1)
                return Status.SUCCESS
        if self.mode == 'step_by_step':
            if self.walk_event is not None:
                self.walk_event.close()
            self.robot.stance()
            time.sleep(1)
            return Status.SUCCESS
        if self.mode == 'aruco_nav':
            if self.walk_event is not None:
                self.walk_event.close()
            self.robot.stance()
            time.sleep(1)
            return Status.SUCCESS
        # 检查行走状态
        self._ensure_walk_event_initialized()
        walk_status = self.walk_event.step()

        if walk_status == EventStatus.SUCCESS:
            # 移动成功完成
            self.walk_event.close()
            self.feedback_message = "✅ 移动完成"
            self.robot.stance()
            return Status.SUCCESS

        elif walk_status == EventStatus.FAILED:
            # 移动失败
            self.walk_event.close()
            self.feedback_message = "❌ 移动失败"
            return Status.FAILURE

        elif walk_status == EventStatus.TIMEOUT:
            # 移动超时
            self.walk_event.close()
            self.feedback_message = "⏰ 移动超时"
            return Status.FAILURE

        # 移动仍在进行中，提供进度反馈
        if self.mode == "relative_dist":
            # 对于相对位移，计算进度
            current_pos = self.robot_sdk.state.robot_position()
            target_pos = self.target_pose.pos
            distance = np.linalg.norm([current_pos[0] - target_pos[0], current_pos[1] - target_pos[1]])
            self.feedback_message = f"位移中: distance = {distance:.2f}m 完成"

        elif self.mode == "absolute_target":
            # 对于绝对移动，计算距离进度
            current_pos = self.robot_sdk.state.robot_position()
            target_pos = self.target_pose.pos
            distance = np.linalg.norm([current_pos[0] - target_pos[0], current_pos[1] - target_pos[1]])
            self.feedback_message = f"移动中，剩余距离: {distance:.2f}m"

        return Status.RUNNING

    def terminate(self, new_status):
        """节点终止时的清理工作"""
        if self.started and self.walk_opened and self.walk_event is not None:
            self.walk_event.close()
            self.walk_opened = False
            self.logger.info(f"清理行走事件资源")