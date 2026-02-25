from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
import py_trees
import time
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from .performance_monitor import performance_monitor

class CtrlHeightPitch(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        self.robot_sdk = RobotSDK()
        self.executed = False
        self.params = params

        self.mode = self.params.get("mode", "manual")
        self.height = float(self.params.get("height", 0.0))
        self.pitch = float(self.params.get("pitch", 0.0))

        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.global_blackboard.register_key(key="HeightAndPitchValues", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Common_robot_type", access=py_trees.common.Access.READ)

    @performance_monitor(method_name="initialise")
    def initialise(self):
        # 检查是否是轮臂，如果是则跳过所有操作
        try:
            robot_type = getattr(self.global_blackboard, 'Common_robot_type', None)
            if robot_type == "kuavo_lb":
                print(f"CtrlHeightPitch: 轮臂模式，跳过高度和俯仰角控制")
                self.executed = True
                return
        except Exception as e:
            print(f"CtrlHeightPitch: 获取robot_type失败: {e}")

        print(f"CtrlHeightPitch function, mode = {self.mode}, height = {self.height}, pitch = {self.pitch}")
        if self.mode == "manual":
            self.robot_sdk.control.squat(self.height, self.pitch)
        elif self.mode == "from_board":
            height_pitch = self.global_blackboard.HeightAndPitchValues
            height = height_pitch.get("height", 0.0)
            pitch = height_pitch.get("pitch", 0.0)
            self.robot_sdk.control.squat(float(height), float(pitch))
        time.sleep(1)

    @performance_monitor(method_name="update")
    def update(self):
        # 如果是轮臂且已跳过操作，直接返回成功
        if self.executed:
            return Status.SUCCESS

        # 检查是否是轮臂
        try:
            robot_type = getattr(self.global_blackboard, 'Common_robot_type', None)
            if robot_type == "kuavo_lb":
                return Status.SUCCESS
        except Exception:
            pass

        return Status.SUCCESS
