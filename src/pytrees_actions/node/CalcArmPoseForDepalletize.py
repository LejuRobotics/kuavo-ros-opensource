from pickle import NONE
from time import sleep
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard

import sys
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy import grab_box_and_backward, place_box_and_backward
from .utils import filter_tree_path
from .performance_monitor import performance_monitor
class DepalletizeBoxParameter:
    def __init__(self, box_width: float, box_height:float, mass: float, force_ratio_z: float, lateral_force: float, behind_tag: float):
        self.box_width = box_width
        self.mass = mass
        self.force_ratio_z = force_ratio_z
        self.lateral_force = lateral_force
        self.behind_tag = behind_tag
        self.box_height = box_height

# ==========================================================
# 4. 抓起箱子
# ==========================================================
class CalcArmPoseForDepalletize(Behaviour):
    def __init__(self, name: str, label:str, **kwargs):
        super().__init__(name)
        print(f"label = {label}")
        self.executed = False
        self.label = label.split('/', -1)[-1]

        blackboard_namespace = filter_tree_path(label)
        self.logger.info(f"MoveArmBaseTargetPoint function, blackboard_namespace = {blackboard_namespace}")
        self.blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.blackboard.register_key(key="arm_pose_and_wrench", access=py_trees.common.Access.WRITE)
        self.robot_sdk = RobotSDK(  )

        self.params = kwargs.copy()
        self.move_mode = self.params.get('mode')
        self.logger.info(f"In MoveArmBaseTargetPoint, self.move_mode = {self.move_mode}")

        self.pick_box_parameter = DepalletizeBoxParameter(
            float(self.params.get('common.box_width', 0)),
            float(self.params.get('common.box_length', 0)),
            float(self.params.get('common.box_height', 0)),
            float(self.params.get('common.box_mass', 0)),
            float(self.params.get('depalletize.force_ratio_z', 0)),
            float(self.params.get('depalletize.lateral_force', 0)),
            float(self.params.get('depalletize.box_behind_tag_in_depalletize', 0)),
        )
        self.logger.info(f'self.pick_box_parameter = {self.pick_box_parameter.__dict__}')

        self.success = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点"""
        self.executed = False
        self.success = False
        self.feedback_message = "准备移动手臂"
        self.logger.info(f"开始移动手臂")
        self.logger.info(f"self.move_mode = {self.move_mode}, type = {type(self.move_mode)}")
        if self.move_mode.startswith('depalletize_step'):
            try:
                box_width = self.pick_box_parameter.box_width
                box_behind_tag = self.pick_box_parameter.behind_tag
                box_mass = self.pick_box_parameter.mass
                box_height = self.pick_box_parameter.box_height
                force_ratio_z = self.pick_box_parameter.force_ratio_z
                lateral_force = self.pick_box_parameter.lateral_force
                print(f'box_width   = {box_width}')

                grasp_left_pose=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(-box_width / 2 , box_height/2+0.20, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),
                    #左手抓取点位
                    Pose.from_euler(pos=(-box_width / 2 , box_height/2, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),
                    # lift a little.
                    Pose.from_euler(pos=(-box_width / 2, box_height/2 + 0.08, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),
                    #左手拖动点位
                    Pose.from_euler(pos=(-box_width / 2 -0.10, box_height/2 + 0.08, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(-box_width / 2 -0.10, box_height/2, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),
                    #wait point
                    Pose.from_euler(pos=(-box_width / 2 - 0.10 , box_height/2, -box_behind_tag), euler=(-90, 0, 0), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(0.30, box_width/2, 0.22), euler=(45, 0, -90), degrees=True,
                                    frame=Frame.BASE)
                ]

                roll_value  = 90
                pitch_value = 0
                yaw_value   = 180
                grasp_right_pose=[
                    #右手预抓取点位
                    Pose.from_euler(pos=(box_width / 2 , box_height/2+0.20, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_width / 2 , box_height/2+0.20, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_width / 2, box_height/2 + 0.20, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_width / 2, box_height/2 + 0.20, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_width / 2, box_height/2 + 0.20, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_width / 2 - 0.1 , box_height/2, -box_behind_tag), euler=(roll_value, pitch_value, yaw_value), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(0.30, -box_width / 2, 0.22), euler=(-45, 0, 90), degrees=True,
                                    frame=Frame.BASE)
                ]

                # ================ 计算每个关键点的力控目标（wrench） ================ #
                # 计算夹持力参数
                g = 9.8  # 重力加速度

                # 计算基础Z向力（考虑安全系数和经验比例）
                force_z = -abs(box_mass * g * force_ratio_z)

                # 判断是否为仿真模式
                left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
                right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

                grasp_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0, 0, force_z, 0, 0, 0],  # 第三关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第四关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第五关键点的扭矩
                    [0, 0, 0, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0]  # 第六关键点的扭矩
                ]

                grasp_right_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第三关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第四关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第五关键点的扭矩
                    [0, 0, 0, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0]  # 第六关键点的扭矩
                ]

                arm_traj = (grasp_left_pose, grasp_right_pose)
                arm_wrench = (grasp_left_arm_wrench, grasp_right_arm_wrench)
                step1_param = [(arm_traj[0][:2], arm_traj[1][:2]), (arm_wrench[0][:2], arm_wrench[1][:2])]
                step2_param = [(arm_traj[0][2:6], arm_traj[1][2:6]), (arm_wrench[0][2:6], arm_wrench[1][2:6])]
                step3_param = [(arm_traj[0][6:], arm_traj[1][6:]), (arm_wrench[0][6:], arm_wrench[1][6:])]
                if self.move_mode == 'depalletize_step1':
                    self.blackboard.arm_pose_and_wrench = step1_param
                    self.success = True
                elif self.move_mode == 'depalletize_step2':
                    self.blackboard.arm_pose_and_wrench = step2_param
                    self.success = True
                elif self.move_mode == 'depalletize_step3':
                    self.blackboard.arm_pose_and_wrench = step3_param
                    self.success = True
                else:
                    self.success = False
                    self.feedback_message = "抓取失败: step解析错误"

                if self.success:
                    self.feedback_message = "calculate success."
                    self.logger.info(f"In t, {self.feedback_message}")
                    return Status.SUCCESS
                else:
                    self.logger.info(f"In MoveArmBaseTargetPoint, {self.feedback_message}")
                    return Status.FAILURE

            except Exception as e:
                rospy.logerr(f"[{self.name}] 抓取失败: {e}")
                self.feedback_message = f"抓取失败: {e}"
                return Status.FAILURE

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if self.success:
            return Status.SUCCESS
        else:
            return Status.FAILURE
