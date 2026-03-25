#!/usr/bin/env python3

import rospy
import json
import math
import time
import threading
import numpy as np
import os
import sys
import rospkg
import subprocess
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
    
# 使用 rospkg 获取 kuavo_common 包路径并导入 RobotVersion
try:
    kuavo_common_path = rospkg.RosPack().get_path('kuavo_common')
    kuavo_common_python_path = os.path.join(kuavo_common_path, 'python')
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion
except (rospkg.ResourceNotFound, ImportError) as e:
    # 如果 rospkg 不可用或包未找到，回退到相对路径方式
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    kuavo_common_python_path = os.path.abspath(os.path.join(current_file_dir, "../../../kuavo_common/python"))
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion
from humanoid_plan_arm_trajectory.msg import bezierCurveCubicPoint, jointBezierTrajectory
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData, sensorsData, robotWaistControl
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, getControllerList
from ocs2_msgs.msg import mpc_observation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from humanoid_plan_arm_trajectory.msg import RobotActionState
from humanoid_plan_arm_trajectory.srv import ExecuteArmAction, ExecuteArmActionResponse  # Import new service type
from std_srvs.srv  import Trigger, TriggerResponse  # 中断服务依赖 

# 根据机器人型号确定关节数据
KUAVO = "kuavo"
ROBAN = "roban"

class ArmTrajectoryBezierDemo:
    # START_FRAME_TIME = 0
    END_FRAME_TIME = 10000
    KUAVO_TACT_LENGTH = 28
    ROBAN_TACT_LENGTH = 23

    def __init__(self):
        self.START_FRAME_TIME = 0
        self.x_shift = self.START_FRAME_TIME
        self.joint_state = JointState()
        self.hand_state = robotHandPosition()
        self.head_state = robotHeadMotionData()
        self.waist_state = robotWaistControl()
        self.running_action = False
        self.arm_flag = False
        self._timer = None
        self.interrupt_flag  = False
        self.last_published_state = None  # 记录上一次发布的状态，用于减少日志打印  
        # 使用 RobotVersion 类创建版本号对象
        robot_version_int = int(os.environ.get("ROBOT_VERSION", "45"))
        self.robot_version = RobotVersion.create(robot_version_int) if RobotVersion.is_valid(robot_version_int) else RobotVersion(4, 5, 0)
        self.robot_class = KUAVO if self.robot_version.major() >= 4 else ROBAN
        self.kuavo_control_scheme = os.getenv("KUAVO_CONTROL_SCHEME", "multi")
        # KUAVO v50+ 有腰部关节
        self.has_waist = (self.robot_version.major() == 5) if self.robot_class == KUAVO else False
       
        if self.robot_class == KUAVO:
            # 根据是否有腰部关节确定TACT长度
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
            current_control_mode = self.get_current_control_mode()
            if current_control_mode == "rl":
                self.INIT_ARM_POS = [int(0)] * tact_length
            else:
                # 基础28个关节 + 可选的1个腰部关节
                # 默认值（如果 ROS 参数不存在时使用）
                default_base_init = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                
                # 从 ROS 参数读取 standJointState（手臂关节，14个）
                try:
                    stand_joint_state = None
                    while stand_joint_state is None and not rospy.is_shutdown():
                        stand_joint_state = rospy.get_param("/standJointState", None)
                        if stand_joint_state is None:
                            rospy.logwarn("ROS 参数 /standJointState 不存在，每隔 1s 检查一次，启动机器人时会自动加载")
                            rospy.sleep(1.0)
                    if stand_joint_state is not None and len(stand_joint_state) >= 14:
                        # 将弧度转换为度，并只取前14个手臂关节
                        self.arm_joints_deg = [math.degrees(rad) for rad in stand_joint_state[:14]]
                        # 构建完整的 base_init：前14个手臂关节 + 后14个默认值（手部、头部等）
                        base_init = self.arm_joints_deg + default_base_init[14:]
                        rospy.loginfo("从 ROS 参数 /standJointState 读取手臂初始位置: %s", self.arm_joints_deg)
                    else:
                        self.arm_joints_deg = default_base_init[:14]
                        base_init = default_base_init
                        rospy.logwarn("ROS 参数 /standJointState 不存在或数据不足，使用默认值")
                except Exception as e:
                    self.arm_joints_deg = default_base_init[:14]
                    base_init = default_base_init
                    rospy.logwarn("读取 ROS 参数 /standJointState 失败: %s，使用默认值", str(e))
                
                if self.has_waist:
                    self.INIT_ARM_POS = base_init + [0]  # 添加腰部初始位置
                else:
                    self.INIT_ARM_POS = base_init
            self.current_arm_joint_state = [0] * tact_length
        elif self.robot_class == ROBAN:
            self.INIT_ARM_POS = [22.91831, 10, 0, -45.83662, 22.91831, -10, 0, -45.83662, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]# task.info: shoudler_center: 0.4rad, elbow_center: -0.8rad
            self.current_arm_joint_state = [0] * self.ROBAN_TACT_LENGTH

        # rospy.spin()

        # Initialize ROS node
        rospy.init_node('autostart_arm_trajectory_bezier_demo')
        self.arm_restore_flag = rospy.get_param('~arm_restore_flag', True)
        
        # 检查是否是半身模式
        self.only_half_up_body = rospy.get_param('/only_half_up_body', False)
        if self.only_half_up_body:
            rospy.loginfo("检测到半身模式（only_half_up_body=True）")
        else:
            rospy.loginfo("全身体模式（only_half_up_body=False）")

        # Subscribers and Publishers
        rospy.loginfo(
            "***************************arm_trajectory_bezier_process_start*****************************************")
        self.traj_sub = rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self.traj_callback, queue_size=1,
                                         tcp_nodelay=True)
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self.control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1,
                                                tcp_nodelay=True)
        self.control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1,
                                                tcp_nodelay=True)
        self.control_waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=1, 
                                                tcp_nodelay=True)

        self.sensor_data_sub = rospy.Subscriber('/sensors_data_raw', 
                                                sensorsData,
                                                self.sensors_data_raw_callback,   
                                                queue_size=1, 
                                                tcp_nodelay=True)

        self.robot_hand_sub = rospy.Subscriber('/dexhand/state', 
                                                JointState,
                                                self.robot_hand_callback,   
                                                queue_size=1, 
                                                tcp_nodelay=True)

        # 订阅 /kuavo_arm_traj 用于 create_action_data 的起始帧（使用当前指令位姿而非关节反馈）
        self._last_kuavo_arm_traj_msg = None
        self.kuavo_arm_traj_sub = rospy.Subscriber(
            '/kuavo_arm_traj', JointState, self._kuavo_arm_traj_callback, queue_size=1, tcp_nodelay=True
        )

        # 添加发布者
        self.robot_action_state_pub = rospy.Publisher('/robot_action_state', RobotActionState, queue_size=1)

        # Add service to execute arm actions
        self.execute_service = rospy.Service('/execute_arm_action', ExecuteArmAction, self.handle_execute_action)
        self._interrupt_service = rospy.Service('/interrupt_arm_traj', Trigger, self.handle_interrupt  )

        # Store the file path base directory for actions
        # self.action_files_path = "/home/lab/kuavo-ros-control/src/humanoid-control/humanoid_plan_arm_trajectory/script/action_files"
        self.action_files_path = "/home/lab/.config/lejuconfig/action_files"
        rospy.loginfo("arm_trajectory_bezier_process is ready.")
        rospy.loginfo(
            "***************************arm_trajectory_bezier_process_end*****************************************")

        # self.run()
        rospy.spin()


    def sensors_data_raw_callback(self, msg):
        """更新关节数据"""
        self._last_joint_msg = msg

        if not hasattr(self, "_last_hand_msg"):
            dummy_hand = JointState()
            dummy_hand.position = [0.0] * 12
            self._last_hand_msg = dummy_hand

        self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    def robot_hand_callback(self, msg):
        """更新手部数据"""
        left = msg.position[:6] if len(msg.position) >= 6 else [0] * 6
        right = msg.position[6:12] if len(msg.position) >= 12 else [0] * 6

        self._last_hand_msg = msg

        if hasattr(self, "_last_joint_msg"):
            self._update_current_arm_joint_state(self._last_joint_msg, self._last_hand_msg)

    def _update_current_arm_joint_state(self, joint_msg, hand_msg):
        """整合 joint_msg 和 hand_msg，更新 current_arm_joint_state"""
        if self.robot_class == KUAVO:
            arm_part = list(joint_msg.joint_data.joint_q[12:26])
            hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12
            head_part = list(joint_msg.joint_data.joint_q[-2:])
            if self.has_waist:
                # KUAVO v50+: 腰部关节在joint_q[12]位置
                waist_part = [joint_msg.joint_data.joint_q[12]]
                self.current_arm_joint_state = arm_part + hand_part + head_part + waist_part
            else:
                self.current_arm_joint_state = arm_part + hand_part + head_part

        elif self.robot_class == ROBAN:
            # 按照 joint_q 索引顺序定义变量
            hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12  # 对应 joint_q[0:12]
            waist_part = [joint_msg.joint_data.joint_q[12]]  # 对应 joint_q[12]
            arm_part = list(joint_msg.joint_data.joint_q[13:21])  # 对应 joint_q[13:21]
            head_part = list(joint_msg.joint_data.joint_q[21:23])  # 对应 joint_q[21:23]
            # 保持最终组合顺序不变：arm_part + hand_part + head_part + waist_part
            self.current_arm_joint_state = arm_part + hand_part + head_part + waist_part

        self.current_arm_joint_state = [round(v, 5) for v in self.current_arm_joint_state]

    def _kuavo_arm_traj_callback(self, msg):
        """缓存 /kuavo_arm_traj 最新消息，供 create_action_data 使用"""
        self._last_kuavo_arm_traj_msg = msg

    def _get_servos_from_kuavo_arm_traj(self, tact_length):
        """从 /kuavo_arm_traj 获取起始关节角（度），不足部分按长度填充0。"""
        if getattr(self, '_last_kuavo_arm_traj_msg', None) is None or not getattr(
            self._last_kuavo_arm_traj_msg, 'position', None
        ):
            return [int(round(math.degrees(x))) for x in self.current_arm_joint_state[:tact_length]]

        msg = self._last_kuavo_arm_traj_msg
        pos = list(msg.position)
        n_from_topic = min(len(pos), tact_length)
        from_topic = [int(round(x)) for x in pos[:n_from_topic]]
        if n_from_topic >= tact_length:
            return from_topic[:tact_length]
        # 其余关节按长度填充0
        rest = [0] * (tact_length - n_from_topic)
        return from_topic + rest

    def traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]

        if self.robot_class == KUAVO:
            self.joint_state.name = [
                "l_arm_pitch",
                "l_arm_roll",
                "l_arm_yaw",
                "l_forearm_pitch",
                "l_hand_yaw",
                "l_hand_pitch",
                "l_hand_roll",
                "r_arm_pitch",
                "r_arm_roll",
                "r_arm_yaw",
                "r_forearm_pitch",
                "r_hand_yaw",
                "r_hand_pitch",
                "r_hand_roll",
            ]
            self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
            self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
            self.joint_state.effort = [0] * 14

            self.hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[14:20]]  # 无符号整数
            self.hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in
                                                point.positions[20:26]]  # 无符号整数
            
            self.head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:28]]
            if self.has_waist and len(point.positions) > 28:
                # KUAVO v50+: 腰部关节在joint_q[12]位置
                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [math.degrees(pos) for pos in point.positions[28:29]]
            
        elif self.robot_class == ROBAN:
            self.joint_state.name = [
                "l_arm_pitch",
                "l_arm_roll",
                "l_arm_yaw",
                "l_forearm_pitch",
                "r_arm_pitch",
                "r_arm_roll",
                "r_arm_yaw",
                "r_forearm_pitch",
            ]
            self.joint_state.position = [math.degrees(pos) for pos in point.positions[:8]]
            self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:8]]
            self.joint_state.effort = [0] * 8

            if len(point.positions) == self.ROBAN_TACT_LENGTH:

                self.hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[8:14]]  # 无符号整数
                self.hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in
                                                    point.positions[14:20]]  # 无符号整数
                
                self.head_state.joint_data = [math.degrees(pos) for pos in point.positions[20:22]]

                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [math.degrees(pos) for pos in point.positions[22:]]

    def call_change_arm_ctrl_mode_service(self, arm_ctrl_mode):
        result = True
        service_name = "arm_traj_change_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                "arm_traj_change_mode", changeArmCtrlMode
            )
            change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
            rospy.loginfo("Service call successful")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result

    def get_arm_ctrl_mode(self):
        """获取当前手臂控制模式"""
        service_name = "humanoid_get_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            get_arm_ctrl_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            req = changeArmCtrlModeRequest()
            req.control_mode = 0  # 查询模式时此参数不使用
            resp = get_arm_ctrl_mode(req)
            return resp.mode
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to get arm control mode: {e}")
            return -1
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            return -1

    def wait_for_arm_mode_change_complete(self, target_mode, timeout=2.0):
        """等待手臂控制模式切换完成"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            current_mode = self.get_arm_ctrl_mode()
            if current_mode == target_mode:
                rospy.loginfo(f"Arm control mode changed to {target_mode} successfully")
                return True
            # 如果获取模式失败（返回-1），继续等待
            if current_mode == -1:
                rospy.sleep(0.01)  # 10ms 检查间隔
                continue
            rospy.sleep(0.01)  # 10ms 检查间隔
        
        final_mode = self.get_arm_ctrl_mode()
        rospy.logwarn(f"Arm control mode change timeout after {timeout} seconds, current mode: {final_mode}, target: {target_mode}")
        return False

    def get_current_controller_name(self):
        """获取当前控制器名称（用于 multi 模式判断）
        :return: str, 当前控制器名称，如果获取失败返回 None
        """
        if self.kuavo_control_scheme != "multi":
            return None
        
        service_name = "/humanoid_controller/get_controller_list"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            get_controller_client = rospy.ServiceProxy(service_name, getControllerList)
            response = get_controller_client()
            if response.success:
                rospy.loginfo(f"Current controller in multi mode: {response.current_controller}")
                return response.current_controller
            else:
                rospy.logwarn(f"Get controller list failed: {response.message}")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(f"Service '{service_name}' call failed: {e}, assuming ocs2 behavior")
        return None

    def get_current_control_mode(self):
        """获取当前实际控制模式
        :return: str, 当前控制模式 ("rl" 或 "ocs2")，如果获取失败返回 "ocs2"（保守策略）
        """
        # 控制模式到控制器名称集合的映射
        mode_controllers = {
            "rl": {"amp_controller"},
            "ocs2": {"mpc"},
        }
        
        # 直接映射的控制方案
        control_scheme_list = ["ocs2","rl"]
        if self.kuavo_control_scheme in control_scheme_list:
            return self.kuavo_control_scheme
        
        # multi 模式需要查询当前控制器
        if self.kuavo_control_scheme == "multi":
            controller = self.get_current_controller_name()
            if controller:
                for mode, controllers in mode_controllers.items():
                    if controller.lower() in controllers:
                        return mode
                rospy.logwarn(f"Unknown controller '{controller}' in multi mode")
        
        # 默认返回 ocs2（保守策略）
        return "ocs2"

    def load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def validate_tact_file(self, data):
        """
        验证tact文件的合法性，以抱拳.tact为标准
        :param data: 加载的JSON数据
        :return: (is_valid, error_message)
        """
        errors = []
        
        # 检查必需字段
        required_fields = ["frames", "finish", "first", "robotType"]
        for field in required_fields:
            if field not in data:
                return False, f"缺少必需字段: {field}"
        
        # 通过机器人版本判断机器人类型，确定servos数组的期望长度
        if self.robot_class == ROBAN:
            expected_servos_length = self.ROBAN_TACT_LENGTH
            expected_attribute_keys = set(str(i) for i in range(1, self.ROBAN_TACT_LENGTH + 1))
        else:  # KUAVO
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
            expected_servos_length = tact_length
            expected_attribute_keys = set(str(i) for i in range(1, tact_length + 1))
        
        frames = data.get("frames", [])
        if not frames:
            return False, "frames数组为空"
        
        # 检查每个frame
        for frame_idx, frame in enumerate(frames):
            # 检查servos数组
            if "servos" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少servos字段")
                continue
            
            servos = frame["servos"]
            if not isinstance(servos, list):
                errors.append(f"frame[{frame_idx}]: servos必须是数组")
                continue
            
            # 检查servos数组长度
            if len(servos) != expected_servos_length:
                errors.append(f"frame[{frame_idx}]: servos数组长度应为{expected_servos_length}，实际为{len(servos)}")
            
            # 检查servos数组中的值（不能为null，必须是数字）
            for servo_idx, servo_value in enumerate(servos):
                if servo_value is None:
                    errors.append(f"frame[{frame_idx}]: servos[{servo_idx}]不能为null")
                elif not isinstance(servo_value, (int, float)):
                    errors.append(f"frame[{frame_idx}]: servos[{servo_idx}]必须是数字，实际类型为{type(servo_value).__name__}")
            
            # 检查keyframe
            if "keyframe" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少keyframe字段")
            else:
                keyframe = frame["keyframe"]
                if not isinstance(keyframe, (int, float)):
                    errors.append(f"frame[{frame_idx}]: keyframe必须是数字")
                elif keyframe < 0:
                    errors.append(f"frame[{frame_idx}]: keyframe不能为负数")
            
            # 检查attribute
            if "attribute" not in frame:
                errors.append(f"frame[{frame_idx}]: 缺少attribute字段")
                continue
            
            attribute = frame["attribute"]
            if not isinstance(attribute, dict):
                errors.append(f"frame[{frame_idx}]: attribute必须是对象-f")
                continue
            
            # 检查attribute中的键
            actual_keys = set(attribute.keys())
            missing_keys = expected_attribute_keys - actual_keys
            if missing_keys:
                errors.append(f"frame[{frame_idx}]: attribute缺少键: {sorted(missing_keys)}")
            
            # 检查每个attribute项的结构
            for key in sorted(expected_attribute_keys):
                if key not in attribute:
                    continue
                
                attr_item = attribute[key]
                if not isinstance(attr_item, dict):
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']必须是对象")
                    continue
                
                # 检查CP字段
                if "CP" not in attr_item:
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']缺少CP字段")
                else:
                    CP = attr_item["CP"]
                    if not isinstance(CP, list) or len(CP) != 2:
                        errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP必须是包含2个元素的数组")
                    else:
                        for cp_idx, cp_point in enumerate(CP):
                            if not isinstance(cp_point, list) or len(cp_point) != 2:
                                errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP[{cp_idx}]必须是[x, y]格式的数组")
                            else:
                                if not all(isinstance(x, (int, float)) for x in cp_point):
                                    errors.append(f"frame[{frame_idx}]: attribute['{key}'].CP[{cp_idx}]的元素必须是数字")
                
                # 检查CPType字段
                if "CPType" not in attr_item:
                    errors.append(f"frame[{frame_idx}]: attribute['{key}']缺少CPType字段")
                else:
                    CPType = attr_item["CPType"]
                    if not isinstance(CPType, list) or len(CPType) != 2:
                        errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType必须是包含2个元素的数组")
                    else:
                        if not all(isinstance(x, str) for x in CPType):
                            errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType的元素必须是字符串")
                        # 检查CPType的值（通常是"AUTO"）
                        for cp_type_idx, cp_type_value in enumerate(CPType):
                            if cp_type_value not in ["AUTO", "MANUAL"]:
                                errors.append(f"frame[{frame_idx}]: attribute['{key}'].CPType[{cp_type_idx}]应为'AUTO'或'MANUAL'，实际为'{cp_type_value}'")
        
        # 检查finish和first字段
        numeric_fields = ["finish", "first"]
        for field in numeric_fields:
            field_value = data.get(field)
            if field_value is not None:
                if not isinstance(field_value, (int, float)):
                    errors.append(f"{field}字段必须是数字")
                elif field_value < 0:
                    errors.append(f"{field}字段不能为负数")
        
        if errors:
            error_msg = "文件合法性检查失败:\n" + "\n".join(f"  - {err}" for err in errors)
            return False, error_msg
        
        return True, None

    def create_init_stand_frame(self, frames, is_rl=False):
        """
        创建初始站立帧（0f处）
        如果0f处没有动作帧，使用指定的初始站立姿态
        :param frames: 现有的frames列表，用于获取servos长度和attribute结构
        :return: 初始站立帧字典
        """
        # 根据机器人类型确定初始站立帧的servos值
        if self.robot_class == KUAVO:
            # KUAVO的初始站立帧值（前14个关节）
            init_stand_servos = [0] * len(self.arm_joints_deg) if is_rl else self.arm_joints_deg
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
        else:  # ROBAN
            # ROBAN的初始站立帧值（前8个关节），与抱拳.tact等标准动作文件保持一致
            init_stand_servos = [22.6, 10, 0, -54.4, 22.6, -10, 0, -54.4]
            tact_length = self.ROBAN_TACT_LENGTH
        
        # 如果frames不为空，使用第一个frame来确定servos长度和attribute结构
        if frames and len(frames) > 0:
            first_frame = frames[0]
            expected_length = len(first_frame.get("servos", []))
            # 使用第一个frame的attribute结构作为模板
            template_attribute = first_frame.get("attribute", {})
        else:
            expected_length = tact_length
            template_attribute = {}
        
        # 补全servos数组到期望长度
        if len(init_stand_servos) < expected_length:
            servos = init_stand_servos + [0] * (expected_length - len(init_stand_servos))
        else:
            servos = init_stand_servos[:expected_length]
        
        # 构造attribute，如果没有模板则创建默认结构
        if template_attribute:
            # 深拷贝模板attribute，保持与原始文件结构一致（包括是否有select字段）
            import copy
            attribute = copy.deepcopy(template_attribute)
        else:
            # 创建默认的attribute结构，与标准tact文件格式保持一致
            attribute = {}
            for i in range(1, expected_length + 1):
                attribute[str(i)] = {
                    "CP": [[0, 0], [0, 0]],  # 与标准tact文件格式一致
                    "CPType": ["AUTO", "AUTO"]
                    # 注意：不添加"select"字段，与第一个frame的格式保持一致
                }
        
        # 创建初始站立帧
        init_frame = {
            "servos": servos,
            "keyframe": 0,
            "attribute": attribute
        }

        return init_frame

    def calculate_transition_time(self, source_angles, target_angles, min_keyframe=50, max_keyframe=400, default_keyframe=200):
        """
        根据两个角度数组的差值动态计算过渡时间（优化版本）
        
        优化点：
        1. 考虑不同关节的速度限制（肩膀关节速度更快）
        2. 计算每个关节所需时间，取最大值（更精确）
        3. 使用更合理的速度限制值（基于实际硬件参数）
        
        :param source_angles: 源角度数组（度）
        :param target_angles: 目标角度数组（度）
        :param min_keyframe: 最小过渡时间（keyframe单位，默认50，即0.5秒）
        :param max_keyframe: 最大过渡时间（keyframe单位，默认400，即4.0秒）
        :param default_keyframe: 默认过渡时间（keyframe单位，默认200，即2.0秒）
        :return: 过渡时间（keyframe单位）
        """
        # 整条手臂关节索引：14个手臂关节（索引0-13）
        arm_joint_indices = list(range(14))
        
        # 关节速度限制配置（度/秒）
        # 肩膀关节（索引0和7，左右臂第一个关节）速度更快，其他关节较慢
        # 基于实际硬件参数：普通关节约50度/秒，肩膀关节约120度/秒（每步限制）
        # 为安全起见，使用保守值：普通关节40度/秒，肩膀关节100度/秒
        SHOULDER_JOINT_INDICES = [0, 7]  # 左右臂肩膀关节索引
        NORMAL_JOINT_VELOCITY = 40.0  # 普通关节速度限制（度/秒）
        SHOULDER_JOINT_VELOCITY = 100.0  # 肩膀关节速度限制（度/秒）
        
        # 计算每个关节所需的时间，取最大值
        max_required_time = 0.0
        total_angle_diff = 0.0
        valid_joint_count = 0
        
        for idx in arm_joint_indices:
            if idx < len(source_angles) and idx < len(target_angles):
                source_angle = source_angles[idx]
                target_angle = target_angles[idx]
                angle_diff = abs(source_angle - target_angle)
                
                # 根据关节类型选择速度限制
                if idx in SHOULDER_JOINT_INDICES:
                    joint_velocity = SHOULDER_JOINT_VELOCITY
                else:
                    joint_velocity = NORMAL_JOINT_VELOCITY
                
                # 计算该关节所需的时间（秒）
                # 时间 = 角度差 / 速度限制
                if joint_velocity > 0:
                    required_time = angle_diff / joint_velocity
                    # 转换为keyframe（1 keyframe = 0.01秒）
                    required_keyframe = required_time * 100.0
                    if required_keyframe > max_required_time:
                        max_required_time = required_keyframe
                
                total_angle_diff += angle_diff
                valid_joint_count += 1
        
        # 根据计算结果确定过渡时间
        if valid_joint_count > 0:
            # 使用最大所需时间，并添加安全余量（20%）
            # 这样可以确保所有关节都能平滑过渡
            transition_keyframe_raw = max_required_time * 1.2
            
            # 如果最大所需时间很小，考虑平均角度差值作为补充``
            # 避免单个关节的小幅移动导致时间过短
            if max_required_time < 50:  # 如果最大时间小于0.5秒
                avg_angle_diff = total_angle_diff / valid_joint_count
                # 使用平均速度限制计算平均时间
                avg_velocity = (NORMAL_JOINT_VELOCITY * (valid_joint_count - 2) + 
                               SHOULDER_JOINT_VELOCITY * 2) / valid_joint_count if valid_joint_count > 2 else NORMAL_JOINT_VELOCITY
                avg_required_time = (avg_angle_diff / avg_velocity) * 100.0 if avg_velocity > 0 else 0
                # 取最大值和平均值的较大者
                transition_keyframe_raw = max(transition_keyframe_raw, avg_required_time * 0.8)
            
            transition_keyframe = int(transition_keyframe_raw)
            # 限制在合理范围内
            transition_keyframe = max(min_keyframe, min(max_keyframe, transition_keyframe))
        else:
            # 如果没有有效的关节数据，使用默认值
            transition_keyframe = default_keyframe
        
        return transition_keyframe

    def add_init_frame(self, frames, is_rl=False, is_first_stage=True):
        action_data = {}

        # rl 要在刚开始插入当前状态为初始值来平滑过渡，ocs2 不需要
        if is_rl:
            import copy

            # 检查当前状态和第一帧的差异
            first_frame = frames[0]

            if is_first_stage and not self.interrupt_flag:
                # 检查当前状态和第一帧的差异
                current_angles_deg = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(first_frame["servos"])]]
            else:
                current_angles_deg = self.servos_start

            # 使用 calculate_transition_time 精确计算过渡时间
            # 考虑每个关节的实际差异和速度限制，比简单的平均差异更准确
            transition_keyframes = self.calculate_transition_time(
                current_angles_deg,
                first_frame["servos"],
                min_keyframe=10,   # 最小0.1秒（姿态几乎一致）
                max_keyframe=100,  # 最大1秒（姿态差异大）
                default_keyframe=50  # 默认0.5秒
            )

            rospy.loginfo(f"RL模式过渡时间：{transition_keyframes} keyframes ({transition_keyframes * 0.01:.2f}秒)")

            for frame in frames:
                frame["keyframe"] += transition_keyframes
            frame0 = copy.deepcopy(frames[0])
            # 如果原来的长度长则补全，否则就需要裁剪
            if is_first_stage and not self.interrupt_flag:
                if len(self.current_arm_joint_state) > len(frame0["servos"]):
                    # 当前状态长度更长，需要裁剪到原始长度
                    frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(frame0["servos"])]]
                else:
                    # 当前状态长度更短或相等，需要补全到原始长度
                    frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state] + [0] * (len(frame0["servos"]) - len(self.current_arm_joint_state))
            else:
                frame0["servos"] = self.servos_start
            frame0["keyframe"] = 0
            frames.insert(0, frame0)
        
        # ocs2 模式和半身模式：在第一帧之前插入当前手臂姿态作为第一帧
        if not is_rl and self.robot_class == KUAVO and self.only_half_up_body and len(frames) > 0:
            import copy
            
            # 获取第一帧
            first_frame = frames[0]
            
            # 检查当前姿态是否可用
            if hasattr(self, 'current_arm_joint_state') and len(self.current_arm_joint_state) > 0:
                # 将当前姿态转换为度
                current_angles_deg = [math.degrees(pos) for pos in self.current_arm_joint_state]
                # 确保 standJointState 数据可用
                if hasattr(self, 'arm_joints_deg') and len(self.arm_joints_deg) >= 14:
                    # 计算过渡时间：从当前姿态到站立帧（standJointState），最小0.5秒，最大4.0秒
                    # 这样更合理：当前位置先过渡到站立帧，然后再从站立帧过渡到第一帧
                    transition_keyframe = self.calculate_transition_time(
                        current_angles_deg, 
                        self.arm_joints_deg,
                        min_keyframe=50,  # 0.5秒
                        max_keyframe=400,  # 4.0秒
                        default_keyframe=200  # 2.0秒
                    )
                else:
                    # 如果没有站立帧数据，则使用默认值
                    transition_keyframe = 100  # 1.0秒
                
                # 先将所有现有帧的keyframe向后偏移
                for frame in frames:
                    frame["keyframe"] += transition_keyframe
                
                # 创建过渡帧，使用当前姿态
                current_frame = copy.deepcopy(first_frame)
                current_frame["keyframe"] = 0
                
                # 更新过渡帧的servos：使用当前实际姿态
                # 如果当前状态长度更长，需要裁剪到原始长度
                if len(self.current_arm_joint_state) > len(current_frame["servos"]):
                    current_frame["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(current_frame["servos"])]]
                else:
                    # 当前状态长度更短或相等，需要补全到原始长度
                    current_frame["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state] + [0] * (len(current_frame["servos"]) - len(self.current_arm_joint_state))
                
                # 将当前姿态帧插入到第一帧位置
                frames.insert(0, current_frame)
        
        # ocs2 模式和半身模式：在最后一帧后面添加一帧，整条手臂平滑过渡到 standJointState
        if not is_rl and self.robot_class == KUAVO and self.only_half_up_body:
            import copy
            
            # 找到最后一帧（keyframe 最大的帧）
            last_frame = max(frames, key=lambda f: f.get("keyframe", 0))
            last_keyframe = last_frame.get("keyframe", 0)
            
            # 确保 standJointState 数据可用
            if hasattr(self, 'arm_joints_deg') and len(self.arm_joints_deg) >= 14:
                # 计算过渡时间：从最后一帧到standJointState，最小1.0秒，最大4.0秒
                transition_keyframe = self.calculate_transition_time(
                    last_frame["servos"],
                    self.arm_joints_deg,
                    min_keyframe=100,  # 1.0秒
                    max_keyframe=400,  # 4.0秒
                    default_keyframe=200  # 2.0秒
                )
                
                # 创建新的一帧，基于最后一帧
                smooth_frame = copy.deepcopy(last_frame)
                smooth_frame["keyframe"] = last_keyframe + transition_keyframe
                
                # 更新整条手臂关节角度：从最后一帧过渡到 standJointState
                # 整条手臂关节索引：14个手臂关节（索引0-13）
                for idx in range(14):
                    if idx < len(smooth_frame["servos"]) and idx < len(self.arm_joints_deg):
                        # 使用 standJointState 的手臂关节角度
                        smooth_frame["servos"][idx] = self.arm_joints_deg[idx]
                
                # 将新帧添加到 frames 列表
                frames.append(smooth_frame)
            else:
                # 即使没有 standJointState，也创建一个默认的过渡帧（2秒），保持最后一帧的角度不变
                smooth_frame = copy.deepcopy(last_frame)
                smooth_frame["keyframe"] = last_keyframe + 200
                frames.append(smooth_frame)

        # 计算结束键值：KUAVO根据是否有腰部关节调整
        if self.robot_class == KUAVO:
            end_key = (self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)) + 1
        else:
            end_key = self.ROBAN_TACT_LENGTH + 1

        for frame in frames:
            servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
            for index, value in enumerate(servos):
                key = index + 1
                if key == end_key:
                    break
                if key not in action_data:
                    action_data[key] = []
                    if keyframe != 0 and len(action_data[key]) == 0:
                        if key <= len(self.INIT_ARM_POS):
                            action_data[key].append([
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                                [0, math.radians(self.INIT_ARM_POS[key - 1])],
                            ])
                if value is not None:
                    CP = attribute[str(key)]["CP"]
                    left_CP, right_CP = CP
                    action_data[key].append([
                        [round(keyframe / 100, 5), math.radians(value)],
                        [round((keyframe + left_CP[0]) / 100, 5), math.radians(value + left_CP[1])],
                        [round((keyframe + right_CP[0]) / 100, 5), math.radians(value + right_CP[1])],
                    ])
        return action_data

    def filter_data(self, action_data):
        filtered_action_data = {}
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i + 1]
                end_time = next_frame[0][0]
                if not found_start and end_time >= self.START_FRAME_TIME:
                    found_start = True
                    p0 = np.array([0, self.current_arm_joint_state[key - 1]])
                    p3 = np.array([next_frame[0][0] - self.x_shift, next_frame[0][1]])

                    # 计算控制点，但使用更保守的方法避免过度摆动
                    # 使用较短的控制杆长度以减少过渡期间的运动幅度
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left


                    # 创建新帧
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]

                    # 修改下一帧的左控制点
                    next_frame[1] = p2.tolist()

                    filtered_frames.append(frame1)
                    skip_next = True

                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - self.x_shift, 5), round(frame[0][1], 5)]
                    left_control_point = [round(frame[1][0] - self.x_shift, 5), round(frame[1][1], 5)]
                    right_control_point = [round(frame[2][0] - self.x_shift, 5), round(frame[2][1], 5)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    def delayed_publish_action_state(self, delay):
        """
        延时发布动作完成状态。（增加中断检查）
        :param delay: 延迟时间（秒）
        """
        rospy.loginfo(f"Delaying action completion state for {delay} seconds...")
        self._timer = rospy.Timer(rospy.Duration(delay), self._on_timer_trigger, oneshot=True)

    def reset_robot_state(self):
        # 复位开始，发布 state=1 表示正在复位
        rospy.loginfo("[RESET_START] Starting robot reset, publishing action state=1")
        self.publish_action_state(1)

        current_control_mode = self.get_current_control_mode()
        if current_control_mode == "rl":
            self.rl_reset_robot_state()
        else:
            # 做完动作之后恢复自然摆臂状态，并且手、头、腰部关节归位
            self.call_change_arm_ctrl_mode_service(1)
            self.hand_state.left_hand_position = [0] * 6
            self.hand_state.right_hand_position = [0] * 6
            self.control_hand_pub.publish(self.hand_state)
            self.head_state.joint_data = [0] * 2
            self.control_head_pub.publish(self.head_state)
            # 复位腰部（KUAVO v50+ 或 ROBAN）
            if (self.robot_class == KUAVO and self.has_waist) or self.robot_class == ROBAN:
                self.waist_state.header.stamp = rospy.Time.now()
                self.waist_state.data.data = [0]
                self.control_waist_pub.publish(self.waist_state)

            # OCS2/MPC 复位完成，发布 state=2
            rospy.loginfo("[RESET_COMPLETE] OCS2/MPC reset finished, publishing action state=2")
            self.publish_action_state(2)

    def create_action_data(self, finish_time, is_rl=False):
        # 根据是否有腰部关节确定TACT长度
        if self.robot_class == KUAVO:
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
        else:
            tact_length = self.ROBAN_TACT_LENGTH
        if is_rl:
            servos_end = [0] * tact_length
        else:
            servos_end = self.INIT_ARM_POS
        # # 起始帧从 /kuavo_arm_traj 获取（当前指令位姿）；无数据时回退到 current_arm_joint_state
        self.servos_start = self._get_servos_from_kuavo_arm_traj(tact_length)
        frames = [
            {
                "servos": self.servos_start,
                "keyframe": 0,
                "attribute": {str(i+1): {"CP": [[0,0],[0,0]]} for i in range(tact_length)}
            },
            {
                "servos": servos_end,
                "keyframe": finish_time * 100,
                "attribute": {str(i+1): {"CP": [[0,0],[0,0]]} for i in range(tact_length)}
            },
        ]
        return {"frames": frames}

    def rl_reset_robot_state(self):
        self.arm_flag = True
        self.START_FRAME_TIME = 0
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift
        finish_time = 2
        data = self.create_action_data(finish_time, is_rl=True)

        # 不需要额外增加时间，add_init_frame会根据实际差异动态添加过渡帧
        self.END_FRAME_TIME = finish_time

        action_data = self.add_init_frame(data["frames"], is_rl=True, is_first_stage=False)

        # RL模式下，add_init_frame可能插入了过渡帧，需要更新END_FRAME_TIME
        current_control_mode = self.get_current_control_mode()
        if current_control_mode == "rl":
            frames = data["frames"]
            if frames:
                last_keyframe = max(f.get("keyframe", 0) for f in frames)
                # 将 keyframe 转换为秒并更新 END_FRAME_TIME
                self.END_FRAME_TIME = last_keyframe * 0.01

        filtered_data = self.filter_data(action_data)
        bezier_request = self.create_bezier_request(filtered_data)

        success = self.plan_arm_trajectory_bezier_curve_client(bezier_request)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            # 清除中断标志位，启动发布线程执行回归初始位的轨迹
            self.interrupt_flag = False
            threading.Thread(target=self.run).start()
            # 在复位动作完成后，仅停止发布，不再触发再次复位
            self._timer = rospy.Timer(
                rospy.Duration(self.END_FRAME_TIME), self._on_reset_timer_trigger, oneshot=True
            )
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            return ExecuteArmActionResponse(success=False, message="Failed to execute action")

    def _on_timer_trigger(self, event):
        self.running_action = False  # 结束 state=1 的发布
        self.publish_action_state(2)
        self.arm_flag = False
        # 动作播放完成以后恢复机器人初始状态
        if self.arm_restore_flag:
            self.reset_robot_state()
        rospy.loginfo(f"After the action playback is complete, revert the robot initial state ")

    def stop_action(self):
        if self._timer:
            self._timer.shutdown()

    def _on_reset_timer_trigger(self, event):
        """复位动作结束后停止发布，并恢复手臂模式为自动摆臂（AMP/RL 下行走时摆手）。"""
        self.arm_flag = False
        rospy.loginfo(f"[RESET_COMPLETE] Reset trajectory finished at {time.time():.3f}. Stopping publishers. [DEBUG] arm_flag={self.arm_flag}, running_action={self.running_action}")
        # 复位完成，发布 state=2
        self.publish_action_state(2)
        # AMP/RL 下做完动作会切到 mode 2，复位轨迹播完后需切回 mode 1，否则拨动摇杆行走时不摆手
        current_control_mode = self.get_current_control_mode()
        if current_control_mode == "rl":
            self.call_change_arm_ctrl_mode_service(1)
            # rospy.loginfo("RL reset done: arm mode switched back to 1 (auto swing) for walking.")

    def publish_running_action_state(self):
        """持续发布 state=1"""
        rate = rospy.Rate(10)  # 每秒发布 2 次
        while self.running_action:
            self.publish_action_state(1)
            rate.sleep()

    def publish_action_state(self, state):
        """
        发布手臂动作状态
        :param state: 动作状态 (0: 失败, 1:执行 2: 成功)
        :param message: 状态描述信息
        """
        state_msg = RobotActionState()
        state_msg.state = state
        self.robot_action_state_pub.publish(state_msg)
        # 只在状态变化时打印日志，减少重复打印
        if self.last_published_state != state:
            rospy.loginfo(f"Robot action state published: state={state}")
            self.last_published_state = state

    def create_bezier_request(self, action_data):
        req = planArmTrajectoryBezierCurveRequest()
        for key, value in action_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = self.START_FRAME_TIME
        req.end_frame_time = self.END_FRAME_TIME
        # 基础关节名称（14个手臂关节）
        base_joint_names = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
        ]
        # KUAVO v50+: 添加腰部关节
        if self.robot_class == KUAVO and self.has_waist:
            req.joint_names = base_joint_names + ["waist_yaw_joint"]
        else:
            req.joint_names = base_joint_names
        return req

    def plan_arm_trajectory_bezier_curve_client(self, req):
        service_name = '/bezier/plan_arm_trajectory'
        rospy.wait_for_service(service_name)
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_nodelet_manager_alive(self):
        """检查 nodelet_manager 节点是否真正在线且可通信
        
        Returns:
            bool: True 如果节点在线且可通信，False 否则
        """
        # 先快速检查：使用 get_num_connections 检查是否有订阅者
        # 这是最快的检查方法，可以避免不必要的子进程调用
        num_subscribers = self.kuavo_arm_traj_pub.get_num_connections()
        if num_subscribers == 0:
            rospy.logwarn("话题 /kuavo_arm_traj 没有订阅者")
            return False
        
        # 有订阅者，进一步验证 nodelet_manager 节点是否真正可通信
        # 方法1: 先检查节点是否在节点列表中
        try:
            node_result = subprocess.run(
                ['rosnode', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )
            if node_result.returncode == 0:
                node_list = node_result.stdout.decode('utf-8', errors='ignore')
                # 检查 nodelet_manager 是否在节点列表中
                if '/nodelet_manager' not in node_list:
                    # 检查是否有包含 nodelet_manager 的节点名
                    found_nodelet = False
                    for line in node_list.split('\n'):
                        if 'nodelet_manager' in line.strip():
                            found_nodelet = True
                            break
                    if not found_nodelet:
                        rospy.logwarn("nodelet_manager 节点不在节点列表中")
                        return False
            else:
                # rosnode list 失败，继续尝试 ping
                stderr_output = node_result.stderr.decode('utf-8', errors='ignore')
                rospy.logwarn(f"rosnode list 命令失败: {stderr_output.strip()}")
        except Exception as e:
            rospy.logwarn(f"检查节点列表时出错: {e}，继续尝试 ping")
        
        # 方法2: 使用 rosnode ping 验证节点是否真正可通信（最可靠）
        # 这会真正尝试与节点建立连接，即使节点在 master 中注册但已崩溃也会返回 False
        # 注意：rosnode ping 即使失败也可能返回 0，需要检查输出内容
        max_retries = 2
        for attempt in range(max_retries):
            try:
                result = subprocess.run(
                    ['rosnode', 'ping', '/nodelet_manager', '-c', '1'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,  # 将 stderr 合并到 stdout，因为错误信息可能在这里
                    timeout=3,  # 增加超时时间，匹配 rosnode ping 的默认超时
                    text=True  # 直接返回字符串而不是 bytes
                )
                
                # 当 stderr=subprocess.STDOUT 时，所有输出都在 stdout 中
                output = result.stdout or ""
                
                # 检查输出中是否包含错误信息
                output_lower = output.lower()
                has_error = 'error' in output_lower or 'connection refused' in output_lower or 'failed' in output_lower
                has_success = 'xmlrpc reply from' in output_lower
                
                # rosnode ping 成功时会显示 "xmlrpc reply from" 或类似的成功信息
                # 失败时会显示 "ERROR" 或 "connection refused"
                if not has_error and (has_success or result.returncode == 0):
                    # ping 成功，节点真正在线
                    rospy.loginfo("nodelet_manager 节点正常运行")
                    return True
                else:
                    # ping 失败，节点在 master 中注册但无法通信（可能已崩溃）
                    error_msg = output.strip() if output.strip() else "未知错误"
                    if attempt < max_retries - 1:
                        # 不是最后一次尝试，稍等再试（可能是节点刚启动）
                        rospy.logwarn(f"节点 nodelet_manager ping 失败（尝试 {attempt + 1}/{max_retries}），稍后重试: {error_msg}")
                        time.sleep(0.5)
                    else:
                        # 最后一次尝试也失败
                        rospy.logwarn(f"节点 nodelet_manager 无法通信（可能已崩溃）: {error_msg}")
                        return False
                        
            except subprocess.TimeoutExpired:
                if attempt < max_retries - 1:
                    rospy.logwarn(f"检查 nodelet_manager 节点状态超时（尝试 {attempt + 1}/{max_retries}），稍后重试")
                    time.sleep(0.5)
                else:
                    rospy.logwarn("检查 nodelet_manager 节点状态超时（最终失败）")
                    return False
            except Exception as e:
                rospy.logwarn(f"检查 nodelet_manager 节点状态时出错: {e}")
                return False
        
        return False

    def handle_interrupt(self, req):
        """处理中断请求的服务回调"""
        rospy.loginfo("[%s]  接收到机械臂中断指令", rospy.get_time())  
        
        # 设置中断标志位 
        self.interrupt_flag  = True 
        self.arm_flag  = False 
        self.running_action  = False 
        
        # 发布动作完成状态
        self.publish_action_state(2)   
        
        # 停止等待动作的timer
        self.stop_action()

        # 恢复机器人初始状态
        self.reset_robot_state()
        
        # 返回标准Trigger响应 
        return TriggerResponse(
            success=True,
            message=f"动作于{time.strftime('%Y-%m-%d  %H:%M:%S')}成功中断"
        )

    def handle_execute_action(self, req):
        action_name = req.action_name

        # 检查是否有动作正在执行
        if self.arm_flag or self.running_action:
            rospy.logwarn(f"Action '{action_name}' rejected: Another action is already executing")
            return ExecuteArmActionResponse(
                success=False,
                message=f"另一个动作正在执行中，请等待当前动作完成后再试"
            )

        # 清理旧动作的定时器，防止旧定时器在新动作执行时触发
        if hasattr(self, '_timer') and self._timer:
            rospy.loginfo(f"Stopping old timer before executing: {action_name}")
            self._timer.shutdown()
            self._timer = None

        # 停止旧动作的执行线程
        if self.arm_flag:
            rospy.loginfo(f"Stopping old action thread before executing: {action_name}")
            self.interrupt_flag = True
            rospy.sleep(0.05)  # 给旧线程一点时间退出
            self.interrupt_flag = False

        file_path = f"{self.action_files_path}/{action_name}.tact"
        data = self.load_json_file(file_path)
        if not data:
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=f"Action file {action_name} not found")

        if not self.check_nodelet_manager_alive():
            msg = "话题 /kuavo_arm_traj 的订阅者 nodelet_manager 节点无法通信或不存在。请检查 nodelet_manager 节点是否正常运行。"
            rospy.logerr(msg)
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=msg)

        robot_type_raw = data.get("robotType", None)
        
        # 验证文件合法性（以抱拳.tact为标准）
        is_valid, validation_error = self.validate_tact_file(data)
        if not is_valid:
            rospy.logerr(f"Tact file validation failed: {validation_error}")
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=f"文件合法性检查失败: {validation_error}")
        
        if robot_type_raw is None:
            msg = "Action file missing required field: robotType"
            rospy.logerr(msg)
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=msg)
        try:
            tact_robot_version = int(robot_type_raw)
        except (TypeError, ValueError):
            msg = f"Invalid robotType in action file: {robot_type_raw}"
            rospy.logerr(msg)
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=msg)

        # 版本兼容关系映射
        version_compat_map = {
            41: [41],
            42: [42],
            45: [43, 45, 46, 48, 49, 100045, 100049, 200049],
            52: [52, 53, 54],
            11: [11, 13, 14, 15, 16, 17],
            13: [11, 13, 14, 15, 16, 17],
            14: [11, 13, 14, 15, 16, 17],
            15: [11, 13, 14, 15, 16, 17],
            16: [11, 13, 14, 15, 16, 17],
            17: [11, 13, 14, 15, 16, 17],

        }
        allowed_robot_versions = version_compat_map.get(tact_robot_version, [tact_robot_version])
        # 使用 version_number() 获取版本号数字进行比较
        robot_version_number = self.robot_version.version_number()
        if robot_version_number not in allowed_robot_versions:
            msg = (
                f"Version mismatch: tact {tact_robot_version} is incompatible with robot {robot_version_number} ({self.robot_version.version_name()})"
            )
            rospy.logerr(msg)
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=msg)


        self.call_change_arm_ctrl_mode_service(2)

        # 等待手臂控制模式切换完成
        if not self.wait_for_arm_mode_change_complete(2, timeout=2.0):
            rospy.logwarn("Arm control mode change may not be complete, but continuing...")


        # 获取初始帧时间
        self.arm_flag = True
        first_value = data.get("first", 0)
        self.START_FRAME_TIME = round(first_value * 0.01, 2)  # 转换为秒，取两位小数
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift

        # 读取动作完成时间
        finish_time = data.get("finish", 0) * 0.01 # 转换为秒
        current_control_mode = self.get_current_control_mode()
        # RL模式和OCS2模式的过渡时间都会在 add_init_frame 中动态添加，这里不需要额外增加时间
        # ocs2 模式的过渡时间将在 add_init_frame 中动态计算并更新
        self.END_FRAME_TIME = finish_time

        # 检查0f处是否有动作帧，如果没有则添加初始站立帧
        # 注意：RL模式下即使没有0f帧，也不需要插入站立帧（会插入当前帧）
        # 注意：半身模式下即使没有0f帧，也不需要插入站立帧（会插入当前姿态帧）
        frames = data["frames"]
        has_frame_at_0f = any(frame.get("keyframe", -1) == 0 for frame in frames)
        
        
        # 只有在非RL模式下，且没有0f帧时，才插入站立帧
        # 半身模式下会在 add_init_frame 中插入当前姿态帧，所以这里不需要插入站立帧
        if not has_frame_at_0f and current_control_mode == "ocs2":
            # 半身模式下不插入站立帧，因为会在 add_init_frame 中插入当前姿态帧
            if not (self.robot_class == KUAVO and self.only_half_up_body):
                # 创建初始站立帧
                init_stand_frame = self.create_init_stand_frame(frames, is_rl=current_control_mode == "rl")
                frames.insert(0, init_stand_frame)
                rospy.loginfo("0f处没有动作帧，已添加初始站立帧")

        action_data = self.add_init_frame(frames, is_rl=current_control_mode == "rl")

        # 根据实际计算的过渡时间更新结束时间
        # RL模式和OCS2半身模式都需要更新，因为add_init_frame可能插入了过渡帧
        if current_control_mode == "rl" or (current_control_mode == "ocs2" and self.only_half_up_body):
            # 找到最后一帧的 keyframe（包括新添加的过渡帧）
            if frames:
                last_keyframe = max(f.get("keyframe", 0) for f in frames)
                # 将 keyframe 转换为秒并更新 END_FRAME_TIME
                self.END_FRAME_TIME = last_keyframe * 0.01
        filtered_data = self.filter_data(action_data)
        bezier_request = self.create_bezier_request(filtered_data)

        rospy.loginfo(f"Planning arm trajectory for action: {action_name}...")
        # 开始执行，持续发布 state=1
        self.running_action = True
        threading.Thread(target=self.publish_running_action_state).start()

        success = self.plan_arm_trajectory_bezier_curve_client(bezier_request)
        # self.call_change_arm_ctrl_mode_service(1)
        if success:
            rospy.loginfo("Arm trajectory planned successfully")
            threading.Thread(target=self.run).start()
            # 使用更新后的 END_FRAME_TIME（包含过渡帧时间）
            # RL模式和OCS2半身模式都会在add_init_frame中添加过渡帧并更新END_FRAME_TIME
            if current_control_mode == "rl" or (current_control_mode == "ocs2" and self.only_half_up_body):
                self.delayed_publish_action_state(self.END_FRAME_TIME)
            else:
                self.delayed_publish_action_state(finish_time)
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            rospy.logerr("Failed to plan arm trajectory")
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message="Failed to exefcute action")

    def run(self):
        rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        while self.arm_flag and not self.interrupt_flag:
            try:
                if len(self.joint_state.position) != 0:
                    self.kuavo_arm_traj_pub.publish(self.joint_state)
                if len(self.hand_state.right_hand_position) != 0 or len(self.hand_state.left_hand_position) != 0:
                    self.control_hand_pub.publish(self.hand_state)
                if len(self.head_state.joint_data) != 0:
                    self.control_head_pub.publish(self.head_state)
                # 发布腰部数据（KUAVO v50+ 或 ROBAN）
                if (self.robot_class == KUAVO and self.has_waist) or self.robot_class == ROBAN:
                    if len(self.waist_state.data.data) != 0:
                        self.control_waist_pub.publish(self.waist_state)
            except Exception as e:
                rospy.logerr(f"Failed to publish arm trajectory: {e}")
            except KeyboardInterrupt:
                break
            rate.sleep()

        if self.interrupt_flag: 
            self.interrupt_flag  = False

if __name__ == "__main__":
    demo = ArmTrajectoryBezierDemo()
