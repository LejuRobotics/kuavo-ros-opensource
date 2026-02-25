from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import numpy as np
import sys
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path
import sys
import os
# 添加上级目录到路径以导入 shared_robot_sdk
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)
from shared_robot_sdk import get_shared_robot_sdk
from .performance_monitor import performance_monitor

class MoveHead(Behaviour):
    def __init__(self, name: str, label:str, namespace: str, params):
        super(MoveHead, self).__init__(name)
        self.params = params
        self._finished = False
        self.robot_sdk = None  # 延迟初始化
        self.head_event = None  # 延迟初始化

        self.label = label.split('/', -1)[-1]
        self.count = int(self.params.get('count',0))
        self.temp = self.count
        blackboard_namespace = namespace
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.mode = self.params.get('mode', 'manual')  # 模式参数，默认为'search'
        self._ensure_robot_initialized()

    def _ensure_robot_initialized(self):
        """延迟初始化机器人对象"""
        if self.robot_sdk is None:
            self.robot_sdk = get_shared_robot_sdk()
        if self.head_event is None:
            head_timeout = int(self.params.get('head_event_timeout', 20000))
            self.head_event = EventHeadMoveKeyPoint(
                robot_sdk=self.robot_sdk,
                timeout=head_timeout,  # 头部移动事件的超时时间，单位秒
                dt=0.05
            )  # 可选的头部移动事件

    @performance_monitor(method_name="initialise")
    def initialise(self):
        # self._ensure_robot_initialized()
        self.head_search_yaw_pitch = []
        if self.mode == 'manual':
            yaws_str = self.params.get('head_search_yaws', "")
            head_search_yaws = [float(yaw) for yaw in yaws_str.split(',')]
            head_search_pitchs = self.params.get('head_search_pitchs', "")
            head_search_pitchs = [float(pitch) for pitch in head_search_pitchs.split(',')]
            # 校验两个列表长度是否一致（一一对应必须条件）
            if len(head_search_yaws) != len(head_search_pitchs):
                self.logger.error(f"yaws和pitchs长度不匹配：yaws={len(head_search_yaws)}, pitchs={len(head_search_pitchs)}")
                raise ValueError("head_search_yaws和head_search_pitchs长度必须一致（一一对应）")

            for yaw_deg, pitch_deg in zip(head_search_yaws, head_search_pitchs):
                # 转换为弧度（机器人控制常用单位）
                # 修复：先将字符串类型的角度转换为浮点数
                try:
                    # 确保输入值是数值类型
                    yaw_num = float(yaw_deg) if isinstance(yaw_deg, str) else yaw_deg
                    pitch_num = float(pitch_deg) if isinstance(pitch_deg, str) else pitch_deg

                    # 转换为弧度
                    yaw_rad = np.deg2rad(yaw_num)
                    pitch_rad = np.deg2rad(pitch_num)

                    # 添加到角度对列表（yaw与pitch按索引一一对应）
                    self.head_search_yaw_pitch.append((yaw_rad, pitch_rad))
                except ValueError as e:
                    self.logger.error(f"角度值转换失败: yaw={yaw_deg}, pitch={pitch_deg}, 错误: {str(e)}")
                    # 可以选择抛出异常或使用默认值
                    raise ValueError(f"无效的角度值: yaw={yaw_deg}, pitch={pitch_deg}")

        elif self.mode == 'from_board':
            self.global_blackboard.register_key(key="HeadMoveValues", access=py_trees.common.Access.READ)
            print(f"self.global_blackboard.HeadMoveValues = {self.global_blackboard.HeadMoveValues}")
            self.head_search_yaw_pitch = self.global_blackboard.HeadMoveValues
            if len(self.global_blackboard.HeadMoveValues) == 0:
                self.logger.error(f"head_search_yaw_pitch为空")
            yaw_rad = self.global_blackboard.HeadMoveValues[0]
            pitch_rad = self.global_blackboard.HeadMoveValues[1]
            self.head_search_yaw_pitch = []
            self.head_search_yaw_pitch.append((yaw_rad, pitch_rad))
        else:
            self.logger.error(f"未知的模式: {self.mode}")
            raise ValueError(f"未知的模式: {self.mode}")

        self.head_event.close()
        self.head_event.open()

        # 2. 再复位索引、下发目标
        self.head_event.cur_head_target_index = 0
        self.head_event.set_target(self.head_search_yaw_pitch)

    @performance_monitor(method_name="update")
    def update(self):
        print(f"len(self.head_search_yaw_pitch) = {len(self.head_search_yaw_pitch)}")
        head_status = self.head_event.step()
        self.logger.info(f"Head Move Event Status: {head_status}, Current Target Index: {self.head_event.cur_head_target_index}")
        # 只要事件内部索引走到头，就成功结束
        if self.head_event.cur_head_target_index >= len(self.head_search_yaw_pitch):
            self._finished = False
            self.count = self.temp
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
            self.logger.debug(f"MoveHead::terminate {self.name} to {new_status}")
