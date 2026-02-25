from py_trees.behaviour import Behaviour
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path
from .performance_monitor import performance_monitor

class SelectTagByID(Behaviour):
    def __init__(self, name:str, label:str, namespace: str, params):
        super(SelectTagByID, self).__init__(name)

        self.params = params
        self.success = False

        print(f"self.params.get('target_id') = {self.params.get('target_id')}")
        # 修复：检查 target_id 是整数还是可迭代对象
        target_id = self.params.get('target_id')
        target_id = [(float(x.strip())) for x in target_id.split(',') if x.strip()]
        if isinstance(target_id, int):
            # 如果是整数，转换为只包含这个整数的列表
            self.target_id_list = [target_id]
        else:
            # 如果是可迭代对象，按照原来的方式处理
            self.target_id_list = [int(id) for id in target_id]
        self.target: int = None  # 当前目标tag的ID
        self.latest_tag: Tag = None  # 儲存最近的tag位置
        self.is_new_tag = False

        blackboard_namespace = namespace
        print(f"SelectTagByID function, blackboard_namespace = {blackboard_namespace}")
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)

        self.global_blackboard = self.attach_blackboard_client()
        self.global_blackboard.register_key(key="TargetTagList", access=py_trees.common.Access.WRITE)  # 可读写
        self.global_blackboard.register_key(key="TagInROI", access=py_trees.common.Access.READ)  # 可读

    @performance_monitor(method_name="initialise")
    def initialise(self):
        self.logger.debug(f"SelectTagBaseID::initialise {self.name}")
        target_data_ROI=self.global_blackboard.TagInROI
        print(f"In selectTagBaseID, target_data = {target_data_ROI}")

        #target id must be in target_id_list
        target_data = []
        for target in target_data_ROI:
            if target['id'] in self.target_id_list:
                target_data.append(target)
        self.global_blackboard.TargetTagList = target_data

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if len(self.global_blackboard.TargetTagList) != 0:
            return Status.SUCCESS
        else:
            return Status.FAILURE