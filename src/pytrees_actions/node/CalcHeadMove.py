from py_trees.behaviour import Behaviour
from py_trees.common import Status
import py_trees
import numpy as np
from .utils import filter_tree_path
import sys
from .performance_monitor import performance_monitor

class CalcHeadMove(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super(CalcHeadMove, self).__init__(name)
        self.params = params
        self.label = label.split('/', -1)[-1]

        # 黑板客户端
        blackboard_namespace = namespace
        self.global_blackboard = self.attach_blackboard_client(name=name)

        # 注册黑板键
        self.global_blackboard.register_key(key="TargetSMTIDRow", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="HeadMoveValues", access=py_trees.common.Access.WRITE)
        self.mode = self.params.get("mode", "smt_pick_squat_mode")

        # 默认头部移动参数
        self.head_move_values = {
            "head_yaw": 0,
            "head_pitch": 0
        }
        self.initialise_success = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        print(f"self.mode = {self.mode}")
        sys.stdout.flush()
        if self.mode.startswith("box_"):
            return
        if self.mode.startswith("smt_"):
            self._initialise_smt()

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if not self.initialise_success:
            return Status.FAILURE

        try:
            # 将 yaw 和 pitch 转换为弧度
            yaw_rad = np.deg2rad(self.head_move_values["head_yaw"])
            pitch_rad = np.deg2rad(self.head_move_values["head_pitch"])

            # 将计算结果写入黑板
            self.global_blackboard.HeadMoveValues = [yaw_rad, pitch_rad]
            self.logger.info(f"成功计算头部移动值: yaw={yaw_rad:.2f}, pitch={pitch_rad:.2f}")
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f"计算头部移动值时发生错误: {e}")
            return Status.FAILURE

    def _initialise_smt(self):
        """初始化节点"""
        # 从黑板读取 TargetSMTIDRow
        target_row = int(self.global_blackboard.TargetSMTIDRow)
        if target_row is None:
            self.logger.error("TargetSMTIDRow 未在黑板中找到！")
            self.initialise_success = False
            return

        # 映射 TargetSMTIDRow 到行名称
        row_mapping = {
            2: "Second",
            3: "Third",
            4: "Fourth",
            5: "Fifth",
            6: "Sixth"
        }
        row_name = row_mapping[target_row]
        if row_name is None:
            self.logger.error(f"无效的 TargetSMTIDRow 值: {target_row}")
            self.initialise_success = False
            return

        # 从 board.json 动态加载头部移动参数
        try:
            mode_mapping = {
                "smt_pick_squat_mode": "PickSquatMode",
                "smt_place_squat_mode": "PlaceSquatMode"
            }
            mode = mode_mapping[self.mode]
            print(f"mode = {mode}")
            key = f"SMTMoveHeadControl_{row_name}Row{mode}"
            control_values = self._read_board_value(key)
            self.head_move_values = {
                "head_yaw": control_values.get("head_yaw", 0),
                "head_pitch": control_values.get("head_pitch", 0)
            }
            self.logger.info(f"成功加载 TargetSMTIDRow = {target_row} ({row_name}) 的头部移动参数: {self.head_move_values}")
        except KeyError as e:
            self.logger.error(f"从 board.json 加载头部移动参数失败: {e}")
            self.initialise_success = False
            return

        self.initialise_success = True

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
            "head_yaw": value_list[0],
            "head_pitch": value_list[1]
        }
