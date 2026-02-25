from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import numpy as np

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventPercep
from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation
from dataclasses import dataclass
from typing import Tuple, List

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path
from .performance_monitor import performance_monitor


class SelectTagBaseSortForDepalletize(Behaviour):
    def __init__(self, name: str, label: str, namespace: str, params):
        super(SelectTagBaseSortForDepalletize, self).__init__(name)
        self.robot_sdk = RobotSDK()

        # 可选的感知事件
        self.label = label.split('/', -1)[-1]
        self.params = params

        self.success = False
        self.target: int = None  # 当前目标tag的ID
        self.latest_tag: Tag = None  # 保存最近的tag位置
        self.is_new_tag = False
        blackboard_namespace = filter_tree_path(label)
        print(f"SelectTagBaseSortForDepalletize function, blackboard_namespace = {blackboard_namespace}")
        self.bb = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.bb.register_key(key="TargetTag", access=py_trees.common.Access.WRITE)  # 可读写
        self.bb.register_key(key="TagLeftOrRight", access=py_trees.common.Access.WRITE)  # 可读写
        self.bb.register_key(key="TagInROI", access=py_trees.common.Access.READ)  # 可读

    @performance_monitor(method_name="initialise")
    def initialise(self):
        self.logger.debug(f"SelectTagBaseSortForDepalletize::initialise {self.name}")
        target_data = self.bb.TagInROI

        if len(target_data) > 0:
            # Step 1: Group tags into heaps based on z-axis distance
            z_heaps = self._group_tags_by_distance(target_data, axis='z', threshold=0.05)

            # Step 2: Sort heaps by average z values (from largest to smallest)
            z_heaps.sort(key=self._calculate_heap_average_z, reverse=True)

            # Step 3: Sort each heap by x-axis distance (from largest to smallest)
            for heap in z_heaps:
                heap.sort(key=lambda tag: tag['pose']['position']['y'], reverse=True)

            # Step 4: Select the largest y value from the first heap
            if z_heaps:
                target_heap = z_heaps[0]
                target_tag = target_heap[0]  # The tag with the largest y in the first heap
                self._set_target_tag(target_tag)

                # Step 5: Determine TagLeftOrRight
                left_heap, right_heap = self._split_left_right_heaps(target_data)
                print(f"left_heap = {left_heap}, right_heap = {right_heap}")
                print(f"target_id = {target_tag['id']}")
                if target_tag['id'] in [tag['id'] for tag in left_heap]:
                    self.bb.TagLeftOrRight = "left"
                elif target_tag['id'] in [tag['id'] for tag in right_heap]:
                    self.bb.TagLeftOrRight = "right"

                self.success = True
            else:
                self.success = False
                self.logger.info("No valid heaps found.")
        else:
            self.success = False
            self.logger.info("No tags detected.")

    def _group_tags_by_distance(self, tags: List[dict], axis: str, threshold: float) -> List[List[dict]]:
        """
        Group tags into heaps based on the specified axis and distance threshold.
        Args:
            tags (List[dict]): List of tags to group.
            axis (str): Axis to group by ('x' or 'z').
            threshold (float): Maximum distance between tags in the same heap.
        Returns:
            List[List[dict]]: Grouped heaps of tags.
        """
        heaps = []
        current_heap = []

        for tag in sorted(tags, key=lambda t: t['pose']['position'][axis]):
            if not current_heap:
                current_heap.append(tag)
            else:
                last_tag = current_heap[-1]
                if abs(tag['pose']['position'][axis] - last_tag['pose']['position'][axis]) <= threshold:
                    current_heap.append(tag)
                else:
                    heaps.append(current_heap)
                    current_heap = [tag]

        if current_heap:
            heaps.append(current_heap)

        return heaps

    def _calculate_heap_average_z(self, heap: List[dict]) -> float:
        """
        Calculate the average z value of a heap.
        Args:
            heap (List[dict]): List of tags in the heap.
        Returns:
            float: Average z value of the heap.
        """
        return sum(tag['pose']['position']['z'] for tag in heap) / len(heap)


    def _calculate_heap_average_y(self, heap: List[dict]) -> float:
        """
        Calculate the average y value of a heap.
        Args:
            heap (List[dict]): List of tags in the heap.
        Returns:
            float: Average y value of the heap.
        """
        if not heap:
            return 0.0
        return sum(tag['pose']['position']['y'] for tag in heap) / len(heap)

    def _split_left_right_heaps(self, tags: List[dict]) -> Tuple[List[dict], List[dict]]:
        """
        Split tags into left and right heaps based on y-axis distance.
        If there is only one tag, determine left or right based on its y coordinate.
        If there are two or more tags, calculate the average y distance and use
        a security threshold to assign tags to left or right heaps.

        Args:
            tags (List[dict]): List of tags to split.

        Returns:
            Tuple[List[dict], List[dict]]: Left and right heaps of tags.
        """
        security_threshold = 0.05
        left_heap = []
        right_heap = []

        if len(tags) == 1:
            # If there is only one tag, determine left or right based on its x coordinate
            tag = tags[0]
            if tag['pose']['position']['y'] < 0:
                left_heap.append(tag)
            else:
                right_heap.append(tag)
        else:
            # Calculate the average y distance
            avg_y = sum(tag['pose']['position']['y'] for tag in tags) / len(tags)
            print(f"avg_y = {avg_y}")

            # Iterate through tags and assign to left or right heap based on the security threshold
            for tag in tags:
                y = tag['pose']['position']['y']
                if y < avg_y - security_threshold:
                    right_heap.append(tag)
                elif y > avg_y + security_threshold:
                    left_heap.append(tag)

        return left_heap, right_heap

    def _set_target_tag(self, target_tag: dict):
        """
        Set the target tag and update the blackboard.
        Args:
            target_tag (dict): The selected target tag.
        """
        self.target_id = target_tag['id']
        tag_pose = target_tag['pose']
        current_pose = Pose(
            pos=(tag_pose['position']['x'], tag_pose['position']['y'], tag_pose['position']['z']),
            quat=(tag_pose['orientation']['x'], tag_pose['orientation']['y'],
                  tag_pose['orientation']['z'], tag_pose['orientation']['w']),
            frame=Frame.ODOM
        )
        self.Tag = Tag(
            id=self.target_id,
            pose=current_pose
        )
        old_tag_euler = self.Tag.pose.get_euler(degrees=True)
        old_tag_euler[0] = 90  # 强行修正为垂直地面
        old_tag_euler[1] = 0.0  # 强行让tagZ轴水平
        self.Tag.pose = Pose.from_euler(
            pos=(tag_pose['position']['x'], tag_pose['position']['y'], tag_pose['position']['z']),
            euler=old_tag_euler,
            frame=Frame.ODOM,
            degrees=True
        )
        self.bb.TargetTag = self.Tag
        self.logger.info(f"Selected Tag ID: {self.Tag.id}")
        self.logger.info(f"Tag Position: {self.Tag.pose.pos}")
        self.logger.info(f"Tag Orientation: {self.Tag.pose.get_euler(degrees=True)}")

    @performance_monitor(method_name="update")
    def update(self):
        if self.success:
            print(f"Tag id is {self.Tag.id}, left or right: {self.bb.TagLeftOrRight}")
            return Status.SUCCESS
        else:
            return Status.FAILURE