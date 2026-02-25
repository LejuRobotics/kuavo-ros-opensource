from time import sleep, time
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import rospy
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_pytree.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import EventArmMoveKeyPoint
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
import py_trees
from py_trees.behaviour import Behaviour
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode

import sys
import os
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

# 添加上级目录到路径以导入 shared_robot_sdk
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)
from shared_robot_sdk import get_shared_robot_sdk
from .performance_monitor import performance_monitor


class PrepareArmForGrasp(Behaviour):
    """张开手臂到预抓取位置的节点"""

    def __init__(self, name: str, label: str, namespace: str, params: dict):
        super().__init__(name)
        self.executed = False

        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.robot_sdk = None  # 延迟初始化
        self.arm_event = None

        self.params = params

        # 注册黑板键（使用BASE坐标系，不需要TargetTag）
        self.global_blackboard.register_key(key="Arm_ArmEventTimeout", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Arm_PoseThreshold", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Arm_AngleThreshold", access=py_trees.common.Access.READ)
        self._ensure_robot_initialized()

    def _ensure_robot_initialized(self):
        """确保SDK已初始化（延迟初始化，在initialise()中创建）"""
        if self.robot_sdk is None:
            # 在initialise()中创建SDK，此时ROS应该已经完全启动（与case_test_arm.py的__main__类似）
            self.robot_sdk = RobotSDK()

            arm_timeout = int(getattr(self.global_blackboard, "Arm_ArmEventTimeout", 40))
            self.arm_control_mode = self.params.get('arm_control_mode', "fixed_base")
            arm_pos_threshold = float(getattr(self.global_blackboard, "Arm_PoseThreshold", 0.21))
            arm_angle_threshold = float(getattr(self.global_blackboard, "Arm_AngleThreshold", 0.45))

            self.arm_event = EventArmMoveKeyPoint(
                robot_sdk=self.robot_sdk,
                timeout=arm_timeout,
                arm_control_mode=self.arm_control_mode,
                pos_threshold=arm_pos_threshold,
                angle_threshold=arm_angle_threshold,
            )

    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点：计算预抓取点位并开始移动"""
        # 性能监控：记录各步骤耗时
        t_start = time()

        # self._ensure_robot_initialized()
        self.executed = False
        self.success = False
        self.feedback_message = "准备张开手臂到预抓取位置"

        t_before_logger1 = time()
        self.logger.info(f"开始张开手臂")
        t_after_logger1 = time()
        logger1_time = t_after_logger1 - t_before_logger1

        try:
            # 使用BASE坐标系的固定预抓取点位
            # pick_left_arm_pose = Pose.from_euler(
            #     pos=(0.3, 0.3, 0.2),
            #     euler=(0, -70, 0),
            #     degrees=True,
            #     frame=Frame.BASE
            # )

            # pick_right_arm_pose = Pose.from_euler(
            #     pos=(0.3, -0.3, 0.2),
            #     euler=(0, -70, 0),
            #     degrees=True,
            #     frame=Frame.BASE
            # )

            # des
            t_before_pose = time()
            pick_left_arm_pose = Pose.from_euler(
                # box
                # pos=(0.4, 0.35, 0.13), euler=(0, -90, 0), degrees=True,
                #                     frame=Frame.BASE
                # des
                pos=(0.2, 0.3, 0.3), euler=(0, -90, 0), degrees=True,
                                    frame=Frame.BASE
            )

            pick_right_arm_pose = Pose.from_euler(
                # box
                # pos=(0.4, -0.35, 0.13), euler=(0, -90, 0), degrees=True,
                #                     frame=Frame.BASE
                # des
                pos=(0.2, -0.3, 0.3), euler=(0, -90, 0), degrees=True,
                                    frame=Frame.BASE
            )
            t_after_pose = time()
            create_pose_time = t_after_pose - t_before_pose

            # 设置控制模式
            t_before_mpc = time()
            self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
            # if self.arm_control_mode == "manipulate_mpc":
            #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
            # elif self.arm_control_mode == "fixed_base":
            #     self.robot_sdk.control.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
            t_after_mpc = time()
            mpc_mode_time = t_after_mpc - t_before_mpc

            # 准备手臂轨迹和力控数据
            t_before_prepare = time()
            arm_traj = ([pick_left_arm_pose], [pick_right_arm_pose])
            arm_wrench = ([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            t_after_prepare = time()
            prepare_data_time = t_after_prepare - t_before_prepare

            # 打开手臂事件并设置目标（BASE坐标系不需要tag）
            t_before_open = time()
            self.arm_event.open_without_manipulation_mpc_mode_change()
            t_after_open = time()
            arm_event_open_time = t_after_open - t_before_open

            t_before_logger2 = time()
            self.logger.info(f"预抓取点位 - 左臂: {pick_left_arm_pose}, 右臂: {pick_right_arm_pose}")
            t_after_logger2 = time()
            logger2_time = t_after_logger2 - t_before_logger2

            t_before_set_target = time()
            if not self.arm_event.set_target(arm_traj, arm_wrench=arm_wrench, tag=None):
                self.logger.error("❌ 设置手臂预抓取位置失败")
                self.feedback_message = "设置手臂预抓取位置失败"
            else:
                self.feedback_message = "设置手臂预抓取位置成功"
                self.logger.info("✓ 设置手臂预抓取位置成功")
            t_after_set_target = time()
            set_target_time = t_after_set_target - t_before_set_target

            t_end = time()
            total_time = t_end - t_start

            # 输出性能分析
            self.logger.info(f"[性能分析] PrepareArmForGrasp.initialise 总耗时: {total_time:.3f}s")
            self.logger.info(f"  - logger.info (开始): {logger1_time:.3f}s")
            self.logger.info(f"  - 创建 Pose 对象: {create_pose_time:.3f}s")
            self.logger.info(f"  - set_manipulation_mpc_mode: {mpc_mode_time:.3f}s")
            self.logger.info(f"  - 准备轨迹和力控数据: {prepare_data_time:.3f}s")
            self.logger.info(f"  - arm_event.open(): {arm_event_open_time:.3f}s")
            self.logger.info(f"  - logger.info (预抓取点位): {logger2_time:.3f}s")
            self.logger.info(f"  - arm_event.set_target(): {set_target_time:.3f}s")
            other_time = total_time - logger1_time - create_pose_time - mpc_mode_time - prepare_data_time - arm_event_open_time - logger2_time - set_target_time
            self.logger.info(f"  - 其他操作: {other_time:.3f}s")

        except Exception as e:
            rospy.logerr(f"[{self.name}] 张开手臂失败: {e}")
            self.feedback_message = f"张开手臂失败: {e}"
            import traceback
            traceback.print_exc()

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if not hasattr(self, 'arm_event') or self.arm_event is None:
            return Status.FAILURE

        arm_status = self.arm_event.step()
        if arm_status == EventStatus.RUNNING:
            return Status.RUNNING
        elif arm_status == EventStatus.SUCCESS:
            self.arm_event.close()
            self.logger.info("手臂已张开到预抓取位置")
            return Status.SUCCESS
        else:
            self.arm_event.close()
            self.logger.error("手臂张开失败")
            return Status.FAILURE
