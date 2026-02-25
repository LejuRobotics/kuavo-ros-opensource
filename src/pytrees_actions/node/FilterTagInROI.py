from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame
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
from .performance_monitor import performance_monitor

class FilterTagInROI(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super(FilterTagInROI, self).__init__(name)
        self.executed = False
        self.success = False
        self.params = params

        self.label = label.split('/', -1)[-1]
        blackboard_namespace = namespace
        print(f"FilterTagInROI function, blackboard_namespace = {blackboard_namespace}")
        self.bb = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.bb.register_key(key="AllTag", access=py_trees.common.Access.READ)  # 可读写

        self.global_blackboard = py_trees.blackboard.Client(name=self.name)
        self.global_blackboard.register_key(key="TagInROI", access=py_trees.common.Access.WRITE)

        self.logger.info(f"filter tags in ROI ")

    @performance_monitor(method_name="initialise")
    def initialise(self):
        self.logger.debug(f"FilterTaginROI::initialise {self.name}")

        all_tag = self.bb.AllTag
        print(f"[FilterTagInROI] all_tag = {all_tag}")
        try:
            # 筛选出 X 坐标绝对值小于 3 的 Tag
            filtered_tags = []
            x_min = float(self.params.get('x_min', -4.0))
            x_max = float(self.params.get('x_max', 4.0))
            y_min = float(self.params.get('y_min', -4.0))
            y_max = float(self.params.get('y_max', 4.0))
            z_min = float(self.params.get('z_min', 0.0))
            z_max = float(self.params.get('z_max', 2.0))

            # 处理 SimpleNamespace 对象结构
            if hasattr(all_tag, 'pose') and hasattr(all_tag, 'id'):
                # 假设每个 pose 对应一个 id
                for i, tag_pose in enumerate(all_tag.pose):
                    # 创建符合原有代码预期的字典结构
                    tag = {
                        'id': all_tag.id[i] if i < len(all_tag.id) else None,
                        'pose': {
                            'position': {
                                'x': tag_pose.position.x,
                                'y': tag_pose.position.y,
                                'z': tag_pose.position.z
                            },
                            'orientation': {
                                'x': tag_pose.orientation.x,
                                'y': tag_pose.orientation.y,
                                'z': tag_pose.orientation.z,
                                'w': tag_pose.orientation.w
                            }
                        }
                    }

                    # 使用与原有代码相同的筛选逻辑
                    position = tag.get('pose', {}).get('position', {})
                    x = position.get('x', 0)
                    y = position.get('y', 0)
                    z = position.get('z', 0)
                    print(f"In SelectTagByID function, x = {x}, y = {y}, z = {z}")
                    print(f"x_min = {x_min}, x_max = {x_max}, y_min = {y_min}, y_max = {y_max}, z_min = {z_min}, z_max = {z_max}")
                    if x_min < x < x_max and y_min < y < y_max and z_min < z < z_max:
                        filtered_tags.append(tag)
            else:
                # 保持原有的遍历逻辑作为后备方案
                for tag in all_tag:
                    print(f"In FilterTagInROI function, tag is {tag}")
                    position = tag.get('pose', {}).get('position', {})
                    x = position.get('x', 0)
                    if x_min < x < x_max:
                        filtered_tags.append(tag)

            # 将筛选后的 Tag 写入黑板
            self.global_blackboard.TagInROI = filtered_tags
            print(f"[FilterTagInROI] filtered_tags = {filtered_tags}")
            self.feedback_message = f"筛选 Tag 成功: {len(filtered_tags)} 个 Tag"
            self.success=True

        except Exception as e:
            self.logger.info(f"[{self.name}] 筛选 Tag 失败: {e}")
            self.feedback_message = f"筛选 Tag 失败: {e}"
            self.success=False
            self.global_blackboard.TagInROI = json.dumps([], indent=2)  # 保留键但值为空列表

        self.executed = True

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if not self.executed:
            return Status.RUNNING

        if len(self.global_blackboard.TagInROI) > 0 and self.success:
            return Status.SUCCESS
        else:
            print(f"[FilterTagInROI] No tags found in ROI.")
            return Status.FAILURE