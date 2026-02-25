from py_trees.behaviour import Behaviour
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path
from .performance_monitor import performance_monitor

class SelectTagByOrder(Behaviour):
    def __init__(self, name:str, label:str, namespace: str, params):
        super(SelectTagByOrder, self).__init__(name)

        self.params = params
        self.label = label.split('/', -1)[-1]
        self.success = False

        blackboard_namespace = namespace
        print(f"SelectTagByID function, blackboard_namespace = {blackboard_namespace}")
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)

        self.global_blackboard = self.attach_blackboard_client()
        self.global_blackboard.register_key(key="TargetTag", access=py_trees.common.Access.WRITE)  # 可读写
        self.global_blackboard.register_key(key="TargetTagList", access=py_trees.common.Access.READ)  # 可读
        self.axis = self.params.get('axis')
        self.order = self.params.get('order')

    @performance_monitor(method_name="initialise")
    def initialise(self):
        self.logger.debug(f"SelectTagBaseID::initialise {self.name}")
        target_data_list=self.global_blackboard.TargetTagList
        self.logger.info(f"In selectTagByOrder, target_data_list = {target_data_list}")

        if target_data_list is not None and len(target_data_list) > 0:
            # 找到self.axis轴高度highest or smallest的tag
            if self.order == "max":
                highest_index = max(
                    range(len(target_data_list)),
                    key=lambda i: target_data_list[i]['pose']['position'][self.axis]
                )
            elif self.order == "min":
                highest_index = min(
                    range(len(target_data_list)),
                    key=lambda i: target_data_list[i]['pose']['position'][self.axis]
                )
            else:
                print(f"you must set order to max or min")
                self.success = False
                return
            target_pose = target_data_list[highest_index]['pose']
            target_id = target_data_list[highest_index]['id']  # 更新当前target ID

            self.target_tag = Tag(
                id=target_id,
                pose=Pose(
                    pos=(target_pose['position']['x'], target_pose['position']['y'], target_pose['position']['z']),
                    quat=(target_pose['orientation']['x'], target_pose['orientation']['y'], target_pose['orientation']['z'],
                        target_pose['orientation']['w']),
                    frame=Frame.ODOM  # 假设感知到的tag位姿在odom坐标系下
                )
            )
            # FIXME: 如果latest_tag不垂直地面，强行改成垂直。(只保留yaw角)
            real_euler = self.target_tag.pose.get_euler(degrees=True)
            real_euler[0] = 90
            real_euler[1] = 0.0
            self.target_tag.pose = Pose.from_euler(
                pos=(target_pose['position']['x'], target_pose['position']['y'], target_pose['position']['z']),
                euler=real_euler,
                frame=Frame.ODOM,
                degrees=True
            )

            self.global_blackboard.TargetTag = self.target_tag   #写入抓取Tag的id，pose
            self.success=True
        else:
            self.target_tag = None
            self.success = False

    @performance_monitor(method_name="update")
    def update(self):
        if self.success:
            self.logger.info(f"SelectTagByOrder::update {self.name} 选择tag {self.target_tag.id}")
            self.logger.info(f"tag data is {self.target_tag}")
            return Status.SUCCESS
        else:
            return Status.FAILURE
