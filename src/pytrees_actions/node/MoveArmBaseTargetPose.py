from pickle import NONE
from time import sleep, time
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode

import sys
import os
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy import grab_box_and_backward, place_box_and_backward
from .utils import filter_tree_path
# 添加上级目录到路径以导入 shared_robot_sdk
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)
from shared_robot_sdk import get_shared_robot_sdk
from .performance_monitor import performance_monitor

class MoveArmBaseTargetPose(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        self.executed = False

        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.robot_sdk = None  # 延迟初始化

        self.params = params

        self.global_blackboard.register_key(key="TargetTag", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="ArmPoseAndWrench", access=py_trees.common.Access.READ)

        self.arm_event = None  # 延迟初始化
        self.success = False
        self.robot_sdk = RobotSDK()
        self.global_blackboard.register_key(key="Arm_ArmEventTimeout", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Arm_PoseThreshold", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Arm_AngleThreshold", access=py_trees.common.Access.READ)
        arm_timeout = int(getattr(self.global_blackboard, "Arm_ArmEventTimeout", 40))
        self.arm_control_mode = self.params.get('arm_control_mode', "manipulation_mpc")
        print(f"arm_control_mode = {self.arm_control_mode}")
        arm_pos_threshold = float(getattr(self.global_blackboard, "Arm_PoseThreshold", 0.21))
        arm_angle_threshold = float(getattr(self.global_blackboard, "Arm_AngleThreshold", 0.45))
        self.arm_event = EventArmMoveKeyPoint(
            robot_sdk=self.robot_sdk,
            timeout=arm_timeout,  # 手臂移动事件的超时时间，单位秒
            arm_control_mode=self.arm_control_mode,  # 手臂控制模式
            pos_threshold=arm_pos_threshold,  # 手臂位置阈值，单位米
            angle_threshold=arm_angle_threshold,  # 手臂角度阈值，单位弧度
        )

    def _ensure_robot_initialized(self):
        return

    @performance_monitor(method_name="initialise")
    def initialise(self):
        # 性能监控：记录各步骤耗时
        t_start = time()

        # self._ensure_robot_initialized()
        print(f"move arm base target pose initialise")
        """初始化节点"""
        self.executed = False
        self.success = False
        self.feedback_message = "准备移动手臂"

        t_before_logger1 = time()
        self.logger.info(f"开始移动手臂")
        t_after_logger1 = time()
        logger1_time = t_after_logger1 - t_before_logger1

        try:
            t_before_mpc = time()
            # if self.arm_control_mode == "manipulate_mpc":
            #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
            # elif self.arm_control_mode == "fixed_base":
            #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
            t_after_mpc = time()
            mpc_mode_time = t_after_mpc - t_before_mpc

            t_before_blackboard = time()
            self.logger.info(f"self.global_blackboard.TargetTag = {self.global_blackboard.TargetTag}")
            target_arm_pose_and_wrench = self.global_blackboard.ArmPoseAndWrench
            target_arm_pose = target_arm_pose_and_wrench[0]
            target_arm_wrench = target_arm_pose_and_wrench[1]
            t_after_blackboard = time()
            blackboard_time = t_after_blackboard - t_before_blackboard

            t_before_open = time()
            self.arm_event.open()  # 打开手臂事件
            t_after_open = time()
            arm_event_open_time = t_after_open - t_before_open

            t_before_logger2 = time()
            self.logger.info(f"target_arm_pose = {target_arm_pose}")
            self.logger.info(f"target_arm_wrench = {target_arm_wrench}")
            print(f"target arm pose = {target_arm_pose}")
            t_after_logger2 = time()
            logger2_time = t_after_logger2 - t_before_logger2

            t_before_set_target = time()
            if not self.arm_event.set_target(target_arm_pose, arm_wrench=target_arm_wrench, tag=self.global_blackboard.TargetTag):
                self.logger.error("❌ 设置手臂key point失败")
            else:
                self.feedback_message = "设置手臂key point success."
            t_after_set_target = time()
            set_target_time = t_after_set_target - t_before_set_target

            t_end = time()
            total_time = t_end - t_start

            # 输出性能分析
            self.logger.info(f"[性能分析] MoveArmBaseTargetPose.initialise 总耗时: {total_time:.3f}s")
            self.logger.info(f"  - logger.info (开始): {logger1_time:.3f}s")
            self.logger.info(f"  - set_manipulation_mpc_mode: {mpc_mode_time:.3f}s")
            self.logger.info(f"  - 读取 blackboard 数据: {blackboard_time:.3f}s")
            self.logger.info(f"  - arm_event.open(): {arm_event_open_time:.3f}s")
            self.logger.info(f"  - logger.info (目标): {logger2_time:.3f}s")
            self.logger.info(f"  - arm_event.set_target(): {set_target_time:.3f}s")
            self.logger.info(f"  - 其他操作: {total_time - logger1_time - mpc_mode_time - blackboard_time - arm_event_open_time - logger2_time - set_target_time:.3f}s")

        except Exception as e:
            rospy.logerr(f"[{self.name}] 设置手臂key point失败: {e}")
            self.feedback_message = f"设置手臂key point失败: {e}"


    @performance_monitor(method_name="update")
    def update(self):
        arm_status = self.arm_event.step()
        if arm_status == EventStatus.RUNNING:
            return Status.RUNNING
        elif arm_status == EventStatus.SUCCESS:
            self.arm_event.close()
            return Status.SUCCESS
        else:
            self.arm_event.close()
            return Status.FAILURE
