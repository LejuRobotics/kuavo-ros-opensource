from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventPercep
from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation

from dataclasses import dataclass

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .performance_monitor import performance_monitor

class BeginSMTWork(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: str):
        super(BeginSMTWork, self).__init__(name)

        self.params = params
        self.label = label.split('/', -1)[-1]
        self.success = False

        # 全局黑板获取，target id list
        self.global_blackboard = self.attach_blackboard_client()
        target_id_list = self.params.get("target_id_list")
        self.target_id_list = [int(id) for id in target_id_list.split(',')]
        self.target_length = len(self.target_id_list)

        # 局部黑板, 节点间通信，将当前Id 进行写入
        self.global_blackboard.register_key(key="TargetSMTID", access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(key="TargetSMTIDRow", access=py_trees.common.Access.WRITE)

        # 注册 SMTIdRowMapping_TagIDRow2 到 SMTIdRowMapping_TagIDRow6 的键
        self.row_mappings = {}
        for row in range(2, 7):  # Row2 到 Row6
            key = f"SMTIdRowMapping_TagIDRow{row}"
            self.global_blackboard.register_key(key=key, access=py_trees.common.Access.READ)
            self.row_mappings[row] = key

        self.current_target_id = None
        self.last_target_id = -100
        self.index = 0

    @performance_monitor(method_name="initialise")
    def initialise(self):
        # 判断当前索引，不能超过数组长度
        print(f"self.target_id_list = {self.target_id_list}, type(self.target_id_list) = {type(self.target_id_list)}")
        if self.index >= self.target_length:
            self.logger.info(f"index为 {self.index}, 长度超出, 从头开始!!!")
            self.index = 0

        self.current_target_id = self.target_id_list[self.index]
        self.smt_work_status = Status.RUNNING

        print(f"当前的抓取id 是{self.current_target_id}")

        # 判断传入 list 是否为空
        if len(self.target_id_list) == 0:
            self.success = False
            self.logger.info("传入的ID列表为空!!!")
            self.smt_work_status = Status.FAILURE
            return

        # 检查当前抓取的 ID 是否是旧的 ID
        if self.last_target_id == self.current_target_id:
            self.logger.info("当前抓取的ID是旧的ID, 请检查一下抓取ID设置!!!")

        # 判断 current_target_id 属于哪个 SMTIdRowMapping_TagIDRow 列表
        target_row = None
        for row, key in self.row_mappings.items():
            tag_id_list_str = getattr(self.global_blackboard, key, [])
            tag_id_list = [int(id) for id in tag_id_list_str.split(',')]
            if self.current_target_id in tag_id_list:
                target_row = row
                break

        if target_row is None:
            self.logger.info(f"当前抓取的ID {self.current_target_id} 不在任何映射列表中!!!")
            self.smt_work_status = Status.FAILURE
            self.success = False
            return

        # 将当前 ID 和行号写入黑板
        self.global_blackboard.TargetSMTID = self.current_target_id
        self.global_blackboard.TargetSMTIDRow = target_row
        print(f"target_id = {self.current_target_id}, index = {self.index}, row = {target_row}, type = {type(self.current_target_id)}")

        self.last_target_id = self.current_target_id
        self.index += 1
        self.success = True
        self.smt_work_status = Status.SUCCESS

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        return self.smt_work_status
