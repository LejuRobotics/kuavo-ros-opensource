from time import sleep
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import py_trees
import numpy as np
from .utils import filter_tree_path
from .performance_monitor import performance_monitor

class CalcHeightPitch(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)

        self.params = params

        # 黑板客户端
        blackboard_namespace = namespace
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.global_blackboard.register_key(key="TargetSMTIDRow", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="HeightAndPitchValues", access=py_trees.common.Access.WRITE)

        # 默认参数
        self.height_and_pitch_values = {
            "height": 0,
            "pitch": 0
        }
        self.mode = self.params.get("mode", "smt_pick_squat_mode")
        self.initialise_success = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        if self.mode.startswith("smt"):
            self._initialise_smt()
        if self.mode.startswith('box'):
            self._initialise_box()

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if not self.initialise_success:
            return Status.FAILURE

        try:
            # 将 height 和 pitch 写入黑板
            self.global_blackboard.HeightAndPitchValues = self.height_and_pitch_values
            self.logger.info(f"成功写入 HeightAndPitchValues: {self.height_and_pitch_values}")
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f"写入 HeightAndPitchValues 时发生错误: {e}")
            return Status.FAILURE

    def _initialise_box(self):
        """初始化节点，计算 height 和 pitch 值"""
        ##TODO
        self.initialise_success = True

    def _initialise_smt(self):
        self.global_blackboard.register_key(key="TargetSMTIDRow", access=py_trees.common.Access.READ)
        """初始化节点，计算 height 和 pitch 值"""

        # 从黑板读取 TargetSMTIDRow
        target_row = int(self.global_blackboard.TargetSMTIDRow)
        if target_row is None:
            self.logger.error("TargetSMTIDRow 未在黑板中找到！")
            return

        # 映射 TargetSMTIDRow 到行名称
        row_mapping = { 2: "Second", 3: "Third", 4: "Fourth", 5: "Fifth", 6: "Sixth"}
        row_name = row_mapping[target_row]
        if row_name is None:
            self.logger.error(f"无效的 TargetSMTIDRow 值: {target_row}")
            return

        mode_mapping = {
            "smt_pick_squat_mode": "PickSquatMode",
            "smt_place_squat_mode": "PlaceSquatMode",
            "smt_pick_stand_mode": "PickStandMode",
            "smt_place_stand_mode": "PlaceStandMode"
        }
        # 动态加载 height 和 pitch 参数
        try:
            mode = mode_mapping[self.mode]  # 默认模式为 PickSquatMode
            key = f"SMTSquatStandControl_{row_name}Row{mode}"
            control_values = self._read_board_value(key)
            self.height_and_pitch_values = {
                "height": control_values.get("height", 0),
                "pitch": control_values.get("pitch", 0)
            }
            self.logger.info(f"成功加载 TargetSMTIDRow = {target_row} ({row_name}) 的 HeightAndPitchValues 参数: {self.height_and_pitch_values}")
            self.initialise_success = True
        except KeyError as e:
            self.logger.error(f"从 board.json 加载 HeightAndPitchValues 参数失败: {e}")
            self.initialise_success = False
            return

    def _read_board_value(self, key):
        """
        从 board.json 中读取指定键的值
        Args:
            key (str): 键名
        Returns:
            dict: 对应的值
        """
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        value = self.global_blackboard.get(key)
        if value is None:
            raise KeyError(f"Key '{key}' not found in board.json")
        # 假设值是以逗号分隔的字符串，解析为字典
        value_list = [float(x) for x in value.split(",")]
        return {
            "height": value_list[0],
            "pitch": value_list[1]
        }
