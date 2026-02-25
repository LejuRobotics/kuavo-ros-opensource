from pickle import NONE
from time import sleep
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

import sys
import os
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy import grab_box_and_backward, place_box_and_backward
from .utils import filter_tree_path
import ast
# 添加上级目录到路径以导入 shared_robot_sdk
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)
from shared_robot_sdk import get_shared_robot_sdk
from .performance_monitor import performance_monitor

# ==========================================================
# 4. 抓起箱子
# ==========================================================
class MoveArmBaseJointValue(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        print(f"label = {label}")
        self.executed = False
        self.label = label.split('/', -1)[-1]

        blackboard_namespace = namespace
        self.logger.info(f"MoveArmBaseTargetPose function, blackboard_namespace = {blackboard_namespace}")
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.robot_sdk = None  # 延迟初始化

        self.params = params
        self.logger.info(f"self.params = {self.params} ")
        self.logger.info(f"type(self.params.get('left_positions')) = {type(self.params.get('left_positions'))}")
        left_positions = self.params.get('left_positions', '[0.0]*7')
        right_positions = self.params.get('right_positions', '[0.0]*7')
        if type(left_positions) == str:
            left_positions = [(float(x.strip())) for x in left_positions.split(',') if x.strip()]
        if type(right_positions) == str:
            right_positions = [(float(x.strip())) for x in right_positions.split(',') if x.strip()]

        print(f"left_positions = {left_positions}, type = {type(left_positions)}")
        print(f"right_positions = {right_positions}, type = {type(right_positions)}")
        self.joint_positions = left_positions + right_positions
        self.logger.info(f"In MoveArmBaseTargetPose, self.joint_positions = {self.joint_positions}")
        self.success = False
        self.robot_sdk = RobotSDK()

    def _ensure_robot_initialized(self):
        """延迟初始化机器人对象"""
        if self.robot_sdk is None:
            self.robot_sdk = get_shared_robot_sdk()

    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点"""
        self._ensure_robot_initialized()
        times = [2]
        q_frames = [self.joint_positions]
        if not self.robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
            print("control_arm_joint_trajectory failed!")
            return False
        self.success = True

    @performance_monitor(method_name="update")
    def update(self):
        if self.success:
            return Status.SUCCESS
        else:
            return Status.FAILURE