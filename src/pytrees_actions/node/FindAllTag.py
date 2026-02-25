from time import sleep, time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventPercep
from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation
import sys
import os
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

import json
from dataclasses import dataclass
from typing import Tuple, List

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path
# 添加上级目录到路径以导入 shared_robot_sdk
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)
from shared_robot_sdk import get_shared_robot_sdk
from .performance_monitor import performance_monitor

class FindAllTag(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super(FindAllTag, self).__init__(name)
        self.robot_sdk = None  # 延迟初始化
        self.params = params

        self.percep_event = None  # 延迟初始化
        self._seen_ids = {}
        # 可选的感知事件

        self.target: int = None  # 当前目标tag的ID
        self.latest_tag: Tag = None  # 儲存最近的tag位置
        self.is_new_tag = False

        # 解析超时参数（单位：秒）
        self.max_timeout = float(self.params.get('max_percep_time', 10.0))  # 最大超时时间
        self.min_time = float(self.params.get('min_percep_time', 0.0))  # 最小时间，防止偶发丢失
        self.start_time = None  # 开始时间，在 initialise 中设置
        self.logger.info(f"FindAllTag 超时设置: 最小时间={self.min_time}s, 最大超时={self.max_timeout}s")

        blackboard_namespace = namespace
        self.bb = py_trees.blackboard.Client(name=self.name, namespace = blackboard_namespace)
        self.bb.register_key(key="AllTag", access=py_trees.common.Access.WRITE)  # 可读写
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.global_blackboard.register_key(key="AllTagInfoOfBase", access=py_trees.common.Access.WRITE)

        self.Percep_Flag = False
        self._ensure_robot_initialized()
        self.robot_sdk.control.disable_head_tracking()

    def _ensure_robot_initialized(self):
        """延迟初始化机器人对象"""
        if self.robot_sdk is None:
            self.robot_sdk = get_shared_robot_sdk()
        if self.percep_event is None:
            head_half_fov = int(getattr(self.global_blackboard, "Head_PercepHalfFov", 100))
            head_move_timeout = int(getattr(self.global_blackboard, "Head_HeadEventTimeout", 1000))
            self.logger.info(f"In FindAllTag function, head_half_fov = {head_half_fov}")
            self.percep_event = EventPercep(
                robot_sdk=self.robot_sdk,
                half_fov=head_half_fov,  # 半视场角度，单位度
                timeout=head_move_timeout,  # 头部移动事件的超时时间，单位秒
            )

    def setup(self):
        self.logger.debug(f"FindAllTag::setup {self.name}")

    @performance_monitor(method_name="initialise")
    def initialise(self):
        # self._ensure_robot_initialized()
        self.percep_event.open()  # 在初始化后打开事件
        self.logger.debug(f"FindAllTag::initialise {self.name}")
        self.skip_stable = True  # 跳过稳定性校验
        self.bb.AllTag = None
        self._seen_ids = {}
        self.tag_info_odom = {}
        # 记录开始时间
        self.start_time = time()
        self.logger.info(f"FindAllTag 开始计时，最小时间: {self.min_time}s, 最大超时: {self.max_timeout}s")

    @performance_monitor(method_name="update")
    def update(self):
        # 确保已初始化（防止在 initialise 之前被调用）
        # self._ensure_robot_initialized()

        # 安全检查：如果 start_time 未初始化，则初始化它
        if self.start_time is None:
            self.start_time = time()
            self.logger.warning(f"FindAllTag::update {self.name} start_time 未初始化，已自动初始化")

        # 执行感知逻辑
        self._run_perception_logic()

        # 计算已用时间
        elapsed_time = time() - self.start_time

        # 检查是否找到 tag
        has_tags = len(self._seen_ids) > 0

        # 如果找到 tag 且已超过最小时间，返回成功（防止偶发丢失）
        if has_tags and elapsed_time >= self.min_time:
            self.percep_event.close()
            self.logger.info(f"FindAllTag::update {self.name} 找到 {len(self._seen_ids)} 个tag，耗时: {elapsed_time:.2f}s (>= 最小时间 {self.min_time}s)")
            return Status.SUCCESS

        # 如果超过最大超时时间，返回失败
        if elapsed_time >= self.max_timeout:
            self.percep_event.close()
            if has_tags:
                # 有 tag 但超时了（可能是最小时间设置太长）
                self.logger.warning(f"FindAllTag::update {self.name} 超时 ({elapsed_time:.2f}s >= {self.max_timeout}s)，但找到 {len(self._seen_ids)} 个tag")
                return Status.SUCCESS
            else:
                self.logger.error(f"FindAllTag::update {self.name} 超时 ({elapsed_time:.2f}s >= {self.max_timeout}s)，未找到任何tag")
                return Status.FAILURE

        # 如果找到 tag 但时间未达到最小时间，继续等待（防止偶发丢失）
        if has_tags:
            remaining_min_time = self.min_time - elapsed_time
            self.logger.info(f"FindAllTag 找到 {len(self._seen_ids)} 个tag，但时间未达到最小时间 ({elapsed_time:.2f}s < {self.min_time}s)，继续等待 {remaining_min_time:.2f}s")
        else:
            remaining_time = self.max_timeout - elapsed_time
            self.logger.info(f"FindAllTag 运行中，已用时: {elapsed_time:.2f}s，剩余时间: {remaining_time:.2f}s，当前找到 {len(self._seen_ids)} 个tag")

        return Status.RUNNING

    def _run_perception_logic(self):
        """感知核心逻辑（常量模式和非常量模式共用）"""
        target_data = self.robot_sdk.vision.apriltag_data_from_odom
        target_base_data = self.robot_sdk.vision.apriltag_data_from_base
        if target_data and target_data.id:
            for tid, pose in zip(target_data.id, target_data.pose):
                self._seen_ids[tid] = pose          # 累积

        target_base_data = self.robot_sdk.vision.apriltag_data_from_base
        if target_base_data and target_base_data.id:
            for tid, pose in zip(target_base_data.id, target_base_data.pose):
                self.tag_info_odom[tid] = pose          # 累积

        # 组装成与原始格式一致的对象
        from types import SimpleNamespace
        self.bb.AllTag = SimpleNamespace(id=list(self._seen_ids.keys()),
                                        pose=list(self._seen_ids.values()))

        self.global_blackboard.AllTagInfoOfBase = SimpleNamespace(id=list(self.tag_info_odom.keys()),
                                        pose=list(self.tag_info_odom.values()))

        self.logger.info(f"ALL_TAG is {self.bb.AllTag}")

