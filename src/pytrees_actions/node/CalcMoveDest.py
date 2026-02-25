from __future__ import annotations
import numpy as np
import rospy
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame, Tag
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventPercep
import ast
from .performance_monitor import performance_monitor

class CalcMoveDest(Behaviour):
    """计算箱子中心及机器人前方预停靠位姿（odom 系）"""

    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.params = params
        self.global_blackboard = self.attach_blackboard_client()
        self.global_blackboard.register_key(key='SLAMNavDest', access=py_trees.common.Access.WRITE)  # 写入腿部的移动停靠点位姿
        self.global_blackboard.register_key(key='TargetSMTID', access=py_trees.common.Access.READ)  # 读取目标 SMT ID
        self.global_blackboard.register_key(key='TargetSMTIDRow', access=py_trees.common.Access.READ)  # 读取目标 SMT 行号
        self.global_blackboard.register_key(key='TargetTag', access=py_trees.common.Access.READ)  # 读取目标 SMT 行号
        self.global_blackboard.register_key(key='TargetPoseOdom', access=py_trees.common.Access.WRITE)  # 读取目标 SMT 行号
        self.robot_type = None
        self.global_blackboard.register_key(key="Common_robot_type", access=py_trees.common.Access.READ)

        self.global_blackboard.register_key(key='GraspBox_PickPoseStandPositionInTag', access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key='SMTSquatStandControl_PickPoseStandPositionInTag', access=py_trees.common.Access.READ)
        print(f"GraspBox_PickPoseStandPositionInTag = {getattr(self.global_blackboard, 'GraspBox_PickPoseStandPositionInTag')}")
        self.global_blackboard.register_key(key='GraspBox_PlacePoseStandPositionInTag', access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key='SMTSquatStandControl_PlacePoseStandPositionInTag', access=py_trees.common.Access.READ)
        print(f"GraspBox_PlacePoseStandPositionInTag = {getattr(self.global_blackboard, 'GraspBox_PlacePoseStandPositionInTag')}")
        self.global_blackboard.register_key(key='GraspBox_PickPoseStandEulerInTag', access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key='SMTSquatStandControl_PickPoseStandEulerInTag', access=py_trees.common.Access.READ)
        print(f"GraspBox_PickPoseStandEulerInTag = {getattr(self.global_blackboard, 'GraspBox_PickPoseStandEulerInTag')}")
        self.global_blackboard.register_key(key='GraspBox_PlacePoseStandEulerInTag', access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key='SMTSquatStandControl_PlacePoseStandEulerInTag', access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key='Head_PercepHalfFov', access=py_trees.common.Access.READ)

        self.name = name
        self.robot_sdk = RobotSDK()

        self.mode = self.params.get("mode")
        self.initialise_success = False
        self.smt_pick_destination = {}
        self.smt_pick_lb = {}
        self.smt_place_destination = []

    @performance_monitor(method_name="initialise")
    def initialise(self):
        if self.mode.startswith('smt'):
            self._initialise_smt()
        elif self.mode.startswith('box'):
            self._initialise_box()

    @performance_monitor(method_name="update")
    def update(self):
        if self.mode.startswith('smt'):
            return self._update_smt()
        elif self.mode.startswith('box'):
            return self._update_box()

    def _initialise_box(self):
        self.percep_half_fov = int(getattr(self.global_blackboard, "Head_PercepHalfFov"))
        self.percep_event = EventPercep(
            robot_sdk=self.robot_sdk,
            half_fov=self.percep_half_fov,
            timeout=np.inf,
        )
        self.stand_in_tag_pos_in_pick = ast.literal_eval(getattr(self.global_blackboard, "GraspBox_PickPoseStandPositionInTag"))
        print(f"self.stand_in_tag_pos_in_pick = {self.stand_in_tag_pos_in_pick}")
        self.stand_in_tag_pos_in_place = ast.literal_eval(getattr(self.global_blackboard, "GraspBox_PlacePoseStandPositionInTag"))
        self.stand_in_tag_euler_in_pick = ast.literal_eval(getattr(self.global_blackboard, "GraspBox_PickPoseStandEulerInTag"))
        print(f"self.stand_in_tag_euler_in_pick = {self.stand_in_tag_euler_in_pick}")
        self.stand_in_tag_euler_in_place = ast.literal_eval(getattr(self.global_blackboard, "GraspBox_PlacePoseStandEulerInTag"))
        self.initialise_success = True

    def _initialise_smt(self) -> None:
        """每次被 tick 前重置标志"""
        self.feedback_message = ""
        rospy.logdebug(f"[{self.name}] 初始化完成")
        self.percep_half_fov = int(getattr(self.global_blackboard, "Head_PercepHalfFov"))
        self.percep_event = EventPercep(
            robot_sdk=self.robot_sdk,
            half_fov=self.percep_half_fov,
            timeout=np.inf,
        )
        self.stand_in_tag_pos_in_pick_smt = ast.literal_eval(getattr(self.global_blackboard, "SMTSquatStandControl_PickPoseStandPositionInTag"))
        self.stand_in_tag_pos_in_place_smt = ast.literal_eval(getattr(self.global_blackboard, "SMTSquatStandControl_PlacePoseStandPositionInTag"))
        self.stand_in_tag_euler_in_pick_smt = ast.literal_eval(getattr(self.global_blackboard, "SMTSquatStandControl_PickPoseStandEulerInTag"))
        self.stand_in_tag_euler_in_place_smt = ast.literal_eval(getattr(self.global_blackboard, "SMTSquatStandControl_PlacePoseStandEulerInTag"))

        # 从黑板读取 TargetSMTIDRow
        target_row = self.global_blackboard.TargetSMTIDRow
        if target_row is None:
            self.feedback_message = "TargetSMTIDRow 未在黑板中找到！"
            rospy.logerr(f"[{self.name}] {self.feedback_message}")
            self.initialise_success = False
            return

        # 映射 TargetSMTIDRow 到行名称
        row_mapping = { 2: "Second", 3: "Third", 4: "Fourth", 5: "Fifth", 6: "Sixth"}

        row_name = row_mapping[target_row]
        if row_name is None:
            self.feedback_message = f"无效的 TargetSMTIDRow 值: {target_row}"
            rospy.logerr(f"[{self.name}] {self.feedback_message}")
            return

        # 动态加载 pick_destination 和 place_destination
        try:
            # 从 board.json 动态加载抓取位置
            for column in range(3):  # 假设有 3 列
                key = f"SMTNavPos_PickPositionOfColumn{column}"
                self.smt_pick_destination[int(column)] = self._read_board_value(key)
                key_lb = f"SMTLB_PickPositionOfColumn{column}"
                self.smt_pick_lb[int(column)] = self._read_board_value(key_lb)

            # 从 board.json 动态加载放置位置
            self.smt_place_destination = self._read_board_value("SMTNavPos_PlacePosition")
            rospy.loginfo(f"[{self.name}] 成功加载抓取和放置位置")
            self.initialise_success = True
            return
        except KeyError as e:
            self.initialise_success = False
            self.feedback_message = f"从 board.json 加载位置参数失败: {e}"
            rospy.logerr(f"[{self.name}] {self.feedback_message}")

    def _update_box(self):
        try:
            tag = self.global_blackboard.TargetTag
            if tag is None:
                self.feedback_message = "TargetTag 为 None"
                rospy.logerr_throttle(2.0, f"[{self.name}] {self.feedback_message}")
                return Status.FAILURE

            print(f"self.stand_in_tag_pos_in_pick = {self.stand_in_tag_pos_in_pick}, type = {type(self.stand_in_tag_pos_in_pick)}")
            if self.mode == "box_pick":
                stand_pose_in_tag=Pose.from_euler(
                    pos=self.stand_in_tag_pos_in_pick,  # 站立位置在目标标签中的位置猜测，单位米
                    euler=self.stand_in_tag_euler_in_pick,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
                    frame=Frame.TAG,  # 使用标签坐标系
                    degrees=False
                )
                print(f"stand_pose_in_tag = {stand_pose_in_tag}")

                # # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
                # 检查tag是否已经是Pose对象，如果是，则创建一个临时Tag对象
                if hasattr(tag, 'pose'):
                    # 如果tag已经有pose属性，直接使用
                    stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
                else:
                    # 如果tag本身就是Pose对象，创建一个包含pose属性的临时Tag对象
                    temp_tag = Tag(id=0, pose=tag, size=0.1)
                    stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(temp_tag, stand_pose_in_tag)
                self.logger.info(f'🔵 目标位置在世界坐标系中的位置: {stand_pose_in_world}')

                #6.写黑板
                self.global_blackboard.TargetPoseOdom = stand_pose_in_world
                self.feedback_message = (
                    f"计算成功 → 停靠位姿: {stand_pose_in_world}"
                )
                rospy.loginfo(f"[{self.name}] {self.feedback_message}")
                return Status.SUCCESS

            elif self.mode == "box_place":
                stand_pose_in_tag=Pose.from_euler(
                    pos=self.stand_in_tag_pos_in_place,  # 站立位置在目标标签中的位置猜测，单位米
                    euler=self.stand_in_tag_euler_in_place,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
                    frame=Frame.TAG,  # 使用标签坐标系
                    degrees=False
                )
                if hasattr(tag, 'pose'):
                    # 如果tag已经有pose属性，直接使用
                    stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
                else:
                    # 如果tag本身就是Pose对象，创建一个包含pose属性的临时Tag对象
                    temp_tag = Tag(id=0, pose=tag, size=0.1)
                    stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(temp_tag, stand_pose_in_tag)
                self.logger.info(f'🔵 目标位置在世界坐标系中的位置: {stand_pose_in_world}')

                # # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义

                self.global_blackboard.TargetPoseOdom = stand_pose_in_world
                return Status.SUCCESS

            else:
                return Status.RUNNING
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"计算异常: {e}"
            rospy.logerr(f"[{self.name}] {self.feedback_message}")
            return Status.FAILURE


    def _update_smt(self) -> Status:
        """更新节点状态"""
        self.robot_type = self.global_blackboard.Common_robot_type
        print(f"self.robot_type = {self.robot_type}")
        if self.mode == 'smt_pick':
            self.feedback_message = "计算抓取箱子停靠点"
            aruco_id = self.global_blackboard.TargetSMTID
            if self.robot_type == "kuavo_lb":
                tag = self.global_blackboard.TargetTag
                if tag is None:
                    self.feedback_message = "TargetTag 为 None"
                    rospy.logerr_throttle(2.0, f"[{self.name}] {self.feedback_message}")
                    return Status.FAILURE
                print(f"aruco_id = {aruco_id}, type = {type(aruco_id)}")
                print(f"self.smt_pick_lb = {self.smt_pick_lb}")
                if aruco_id in self.smt_pick_lb:
                    # self.global_blackboard.SLAMNavDest = self.smt_pick_lb[aruco_id]
                    stand_pose_in_tag=Pose.from_euler(
                        pos=self.stand_in_tag_pos_in_pick_smt,  # 站立位置在目标标签中的位置猜测，单位米
                        euler=self.stand_in_tag_euler_in_pick_smt,  # 站立位置在目标标签中的姿态猜测，单位欧拉角（弧度）
                        frame=Frame.TAG,  # 使用标签坐标系
                        degrees=False
                    )
                    print(f"stand_pose_in_tag = {stand_pose_in_tag}")

                    # # 转换stand_pose_in_tag到世界坐标系。注意、需要搞清楚tag的坐标定义和机器人的坐标定义
                    # 检查tag是否已经是Pose对象，如果是，则创建一个临时Tag对象
                    if hasattr(tag, 'pose'):
                        # 如果tag已经有pose属性，直接使用
                        stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(tag, stand_pose_in_tag)
                    else:
                        # 如果tag本身就是Pose对象，创建一个包含pose属性的临时Tag对象
                        temp_tag = Tag(id=0, pose=tag, size=0.1)
                        stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(temp_tag, stand_pose_in_tag)
                    self.logger.info(f'🔵 目标位置在世界坐标系中的位置: {stand_pose_in_world}')

                    # 修改Z坐标：smt_pick_lb[aruco_id]是列表，需要取第一个元素
                    z_offset = self.smt_pick_lb[aruco_id][0] if isinstance(self.smt_pick_lb[aruco_id], list) else self.smt_pick_lb[aruco_id]
                    stand_pose_in_world.pos[2] += z_offset

                    #6.写黑板
                    self.global_blackboard.TargetPoseOdom = stand_pose_in_world
                    self.feedback_message = (
                        f"计算成功 → 停靠位姿: {stand_pose_in_world}"
                    )
                    rospy.loginfo(f"[{self.name}] {self.feedback_message}")
                    return Status.SUCCESS
                else:
                    self.feedback_message = f"未找到 Aruco ID {aruco_id} 的抓取停靠点"
                    rospy.logerr(f"[{self.name}] {self.feedback_message}")
                    return Status.FAILURE
            else:
                print(f"aruco_id = {aruco_id}, type = {type(aruco_id)}")
                print(f"self.smt_pick_destination = {self.smt_pick_destination}")
                if aruco_id in self.smt_pick_destination:
                    self.global_blackboard.SLAMNavDest = self.smt_pick_destination[aruco_id]
                else:
                    self.feedback_message = f"未找到 Aruco ID {aruco_id} 的抓取停靠点"
                    rospy.logerr(f"[{self.name}] {self.feedback_message}")
                    return Status.FAILURE
        elif self.mode == 'smt_place':
            self.feedback_message = "计算放置箱子停靠点"
            self.global_blackboard.SLAMNavDest = self.smt_place_destination
            print(f"place_destination = {self.global_blackboard.SLAMNavDest}")
        else:
            self.feedback_message = "未知模式"
            rospy.logerr(f"[{self.name}] {self.feedback_message}")
            return Status.FAILURE
        print("world")
        return Status.SUCCESS

    def _read_board_value(self, key):
        """
        从 board.json 中读取指定键的值
        Args:
            key (str): 键名
        Returns:
            Any: 对应的值
        """
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        value = self.global_blackboard.get(key)
        if value is None:
            raise KeyError(f"Key '{key}' not found in board.json")
        # 假设值是以逗号分隔的字符串，解析为浮点数列表
        return [float(x) for x in value.split(",")]
