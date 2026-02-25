from doctest import Example
from pickle import NONE
from time import sleep
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
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

from .utils import filter_tree_path
from .performance_monitor import performance_monitor


class MoveArmBaseJointTrajectories(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)

        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.robot_sdk = RobotSDK()

        # 获取预抓取配置信息
        self.global_blackboard.register_key(key="ArmJointTrajectories", access=py_trees.common.Access.READ)

        self.success = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点"""

        times = self.global_blackboard.ArmJointTrajectories["times"]
        q_frames = self.global_blackboard.ArmJointTrajectories["q_frames"]
        print(f"times = {times}")
        print(f"q_frames = {q_frames}")

        print("🚀🚀🚀运行关节轨迹运动🚀🚀🚀")
        if not self.robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
            print("control_arm_joint_trajectory failed!")
            return False

        # 等待关节运动完成，休眠一段时间
        self.success = True

    @performance_monitor(method_name="update")
    def update(self):
        if self.success:
            return Status.SUCCESS
        else:
            return Status.FAILURE