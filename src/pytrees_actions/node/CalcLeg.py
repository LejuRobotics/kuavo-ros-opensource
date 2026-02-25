from time import sleep
import time
import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import py_trees
import numpy as np
from .utils import filter_tree_path
from .performance_monitor import performance_monitor

# 导入必要的SDK模块
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.data_type import Pose, Frame, Tag
from kuavo_humanoid_sdk.kuavo_strategy_pytree.nodes.api import TorsoAPI
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventPercep
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Transform3D
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode

class CalcLeg(Behaviour):
    """
    根据TargetTag计算并控制躯干位置的节点
    参考 prepare_torso_for_target_position 的实现
    使用 TorsoAPI 接口控制躯干，类似 case_test_torso.py 的方式
    从黑板中读取 TargetTag，计算躯干目标位置
    """
    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)

        self.params = params
        self.robot_sdk = RobotSDK()
        # self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
        # 创建 TorsoAPI 实例
        self.torso_api = TorsoAPI(robot_sdk=self.robot_sdk)

        # 获取感知事件参数
        percep_half_fov = int(self.params.get("percep_half_fov", 30))  # 默认30度
        self.percep_event = EventPercep(
            robot_sdk=self.robot_sdk,
            half_fov=percep_half_fov,
            timeout=np.inf,
        )

        # 黑板客户端
        blackboard_namespace = filter_tree_path(label)
        self.bb = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)
        self.bb.register_key(key="TargetTag", access=py_trees.common.Access.READ)

        # 从参数中获取配置，如果没有则使用默认值
        # 站立位置在tag坐标系下的位置和姿态
        stand_in_tag_pos_str = self.params.get("stand_in_tag_pos", "[-0.04, 0.2, 0.2]")  # 单位：米 /smtx 0.0/-0.5 z  or 0.1 chaiduo  0.2
        stand_in_tag_euler_str = self.params.get("stand_in_tag_euler", "[-1.57, 1.57, 0.0]")  # 单位：弧度（radians）

        # 解析字符串为列表
        import ast
        try:
            self.stand_in_tag_pos = ast.literal_eval(stand_in_tag_pos_str) if isinstance(stand_in_tag_pos_str, str) else stand_in_tag_pos_str
            self.stand_in_tag_euler = ast.literal_eval(stand_in_tag_euler_str) if isinstance(stand_in_tag_euler_str, str) else stand_in_tag_euler_str
        except:
            self.stand_in_tag_pos = [0.0, 0.0, 0.0]
            self.stand_in_tag_euler = [0.0, 0.0, 0.0]

        # 额外的Z偏移量
        self.leg_mode = self.params.get("leg_mode", "target")
        # 将字符串 "False"/"True" 转换为布尔值
        async_param = self.params.get("asynchronous", True)
        if isinstance(async_param, str):
            self.asynchronous = async_param.lower() in ("true", "1", "yes")
        else:
            self.asynchronous = bool(async_param)
        self.total_time = float(self.params.get("total_time", -1.0))
        self.offset_x = float(self.params.get("offset_x", 0.0))
        self.offset_y = float(self.params.get("offset_y", 0.0))
        self.offset_z = float(self.params.get("offset_z", 0.0))
        self.offset_yaw = float(self.params.get("offset_yaw", 0.0))

        self.stand_in_tag_pos[2] = self.stand_in_tag_pos[2] + self.offset_z
        self.stand_in_tag_pos[1] = self.stand_in_tag_pos[1] + self.offset_y
        self.stand_in_tag_pos[0] = self.stand_in_tag_pos[0] + self.offset_x
        # 是否控制底盘
        self.control_base = self.params.get("control_base", False)

        # 执行状态
        self.future = None
        self.initialised = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点，从黑板读取TargetTag，计算躯干目标位置并开始控制"""
        self.logger.info(f"CalcLeg::initialise {self.name}")
        self.future = None
        if self.leg_mode == "target":
            try:
                # 从黑板读取TargetTag
                target_tag = self.bb.TargetTag
                if target_tag is None:
                    self.logger.error("TargetTag 未在黑板中找到！")
                    self.initialised = False
                    return

                self.logger.info(f"读取到 TargetTag: id={target_tag.id}, pose={target_tag.pose}")

                # 1. 获取当前位置
                current_pos = self.robot_sdk.state.robot_position()
                current_quat = self.robot_sdk.state.robot_orientation()
                current_pose = Pose(pos=current_pos, quat=current_quat, frame=Frame.ODOM)

                self.logger.info(f"当前姿态: pos={current_pose.pos}, quat={current_pose.quat}")

                # 2. 计算站立位置在tag坐标系下的位姿
                stand_pose_in_tag = Pose.from_euler(
                    pos=self.stand_in_tag_pos,
                    euler=self.stand_in_tag_euler,
                    frame=Frame.TAG,
                    degrees=False
                )

                # 3. 将站立位置转换到世界坐标系
                stand_pose_in_world = self.percep_event.transform_pose_from_tag_to_world(
                    target_tag, stand_pose_in_tag
                )

                self.logger.info(f"站立位置: {stand_pose_in_world}")

                # 获取从 odom 到 base_link 的变换
                tf_pose = self.robot_sdk.tools.get_tf_transform("base_link_lb", "odom")
                odom_to_base_pose = Pose(
                    pos=tf_pose.position,
                    quat=tf_pose.orientation,
                    frame=Frame.BASE
                )
                transform_odom_to_base = Transform3D(
                    trans_pose=odom_to_base_pose,
                    source_frame=Frame.ODOM,
                    target_frame=Frame.BASE
                )
                target_torso_pose = transform_odom_to_base.apply_to_pose(stand_pose_in_world)
                # 将位姿从 odom 转换到 base_link
                # target_torso_pose = stand_pose_in_world

                self.logger.info(f"躯干目标位置 (base_link): pos={target_torso_pose.pos}, quat={target_torso_pose.quat}")

                # 7. 使用 TorsoAPI 异步控制躯干
                self.logger.info("开始调用 TorsoAPI.move_torso_pose...")
                self.future = self.torso_api.move_torso_pose(
                    desir_torso_pose=target_torso_pose,
                    asynchronous=False,
                    # control_base=self.control_base,
                    total_time=self.total_time
                )

                self.initialised = True
                self.logger.info("CalcLeg 初始化完成，开始控制躯干")

            except Exception as e:
                self.logger.error(f"CalcLeg::initialise 失败: {e}")
                import traceback
                self.logger.error(traceback.format_exc())
                self.initialised = False
            # time.sleep(0.5)
        else:
            try:
                self.future = self.torso_api.move_torso_pose(
                        desir_torso_pose=Pose.from_euler(pos=[self.offset_x, self.offset_y, self.offset_z], euler=[0, 0, self.offset_yaw], frame=Frame.BASE, degrees=True),
                        asynchronous=self.asynchronous,
                        # control_base=self.control_base,
                        total_time=self.total_time
                    )
                self.initialised = True
                self.logger.info("CalcLeg 初始化完成，开始控制躯干" + str(self.asynchronous))
            except Exception as e:
                self.logger.error(f"CalcLeg::initialise 失败: {e}")
                import traceback
                self.logger.error(traceback.format_exc())
                self.initialised = False
            # time.sleep(0.5)

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态（类似 NodeTorsoPose 的实现）"""
        if not self.initialised:
            return Status.FAILURE

        if self.future is None:
            self.logger.error(f"CalcLeg::update {self.name} - future is None in asynchronous mode")
            return Status.FAILURE

        if self.initialised:
            return Status.SUCCESS

        # if self.future.done():
        #     try:
        #         self.future.result()
        #         self.logger.info("CalcLeg 控制完成")
        #         return Status.SUCCESS
        #     except Exception as exc:
        #         self.logger.error(f"CalcLeg::update {self.name} failed: {exc}")
        #         return Status.FAILURE

        time.sleep(0.01)
        return Status.RUNNING

    def terminate(self, new_status):
        """终止节点"""
        self.logger.debug(f"CalcLeg::terminate {self.name} to {new_status}")
