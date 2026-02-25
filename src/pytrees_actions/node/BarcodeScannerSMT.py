from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from py_trees.common import Status
import tf.transformations as tft

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK

from kuavo_humanoid_sdk.kuavo import KuavoRobotObservation

# 导入策略文件
from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy_slam_optimize import detect_18bit_barcode, arm_for_detector_code_multiMode_multiSegment

import py_trees
from py_trees.behaviour import Behaviour

from .utils import filter_tree_path
from .performance_monitor import performance_monitor

import sys
import threading
CONFIGS_PARENT_PATH = "/home/lab/garb_box/kuavo-ros-control/src/kuavo_humanoid_sdk/kuavo_humanoid_sdk/pytrees_actions"
if CONFIGS_PARENT_PATH not in sys.path:
    sys.path.append(CONFIGS_PARENT_PATH)

class BarcodeScannerSMT(Behaviour):
    def __init__(self, name: str, label: str, namespace, params):
        super(BarcodeScannerSMT, self).__init__(name)
        self.label = label.split('/', -1)[-1]
        self.params = params
        self.robot_sdk = RobotSDK()

        # 黑板客户端
        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=self.name, namespace=blackboard_namespace)

        # 通过黑板获取抓取的 目标ID
        self.global_blackboard = self.attach_blackboard_client()
        self.global_blackboard.register_key(key='TargetSMTID', access=py_trees.common.Access.READ)


        print(f"BarcodeScanner params = {self.params}")


    @performance_monitor(method_name="initialise")
    def initialise(self):
        """初始化节点"""

        self.target_id = self.global_blackboard.TargetSMTID

        # 全局控制变量, 用于识别到码后，停止手臂移动
        self.scan_stop_event = threading.Event()


    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态 - 非阻塞模式"""
        self.logger.info(f"slam move pose")

        try:
            thread_start_scan_18bit_barcode = threading.Thread(target=self._start_scan_18bit_barcode, args=(self.robot_sdk,))
            thread_move_head_toscan  = threading.Thread(target=self._move_arm_toscan, args=(self.robot_sdk,))
            thread_start_scan_18bit_barcode.start()
            thread_move_head_toscan.start()
            thread_start_scan_18bit_barcode.join()
            thread_move_head_toscan.join()

            return Status.SUCCESS

        except Exception as e:
            rospy.logerr(f"[{self.name}] 移动失败: {e}")
            self.feedback_message = f"移动失败: {e}"
            return Status.FAILURE

    def _start_scan_18bit_barcode(self, robot_sdk):

        detect_result = detect_18bit_barcode(robot_sdk=robot_sdk, max_retries=5)
        print(f" 第 {self.target_id} 码的料盘得到的18位数条形码结果是 {detect_result}")

        # 写入参数服务器中
        if rospy.has_param('/barcode_value'):
            rospy.delete_param('/barcode_value')
        try:
            rospy.set_param("/barcode_value", str(detect_result))
            rospy.loginfo(f"Barcode value stored to parameter server: {detect_result}")
        except Exception as e:
            rospy.logerr(f"Failed to store barcode value to parameter server: {e}")

        self.scan_stop_event.set()
        print("已发送停止信号给手臂运动")

    def _move_arm_toscan(self, robot_sdk):
        self.scan_stop_event.clear()  # 重置停止事件
        arm_for_detector_code_multiMode_multiSegment(robot_sdk, start_time=0.5, time_duration=0.3, mode=4, max_points=60, max_duration=2.0, stop_event=self.scan_stop_event)
