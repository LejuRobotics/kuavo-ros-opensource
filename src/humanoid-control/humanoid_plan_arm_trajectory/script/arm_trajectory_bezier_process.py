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
from kuavo_msgs.msg import robotHandPosition, robotHeadMotionData, sensorsData
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
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
        self.waist_state = Float64MultiArray()
        self.running_action = False
        self.arm_flag = False
        self._timer = None
        self.interrupt_flag  = False  
        # 使用 RobotVersion 类创建版本号对象
        robot_version_int = int(os.environ.get("ROBOT_VERSION", "45"))
        self.robot_version = RobotVersion.create(robot_version_int) if RobotVersion.is_valid(robot_version_int) else RobotVersion(4, 5, 0)
        self.robot_class = KUAVO if self.robot_version.major() >= 4 else ROBAN
        self.kuavo_control_scheme = os.getenv("KUAVO_CONTROL_SCHEME", "ocs2")
        # KUAVO v50+ 有腰部关节
        self.has_waist = (self.robot_version.major() == 5) if self.robot_class == KUAVO else False
        
        if self.robot_class == KUAVO:
            # 根据是否有腰部关节确定TACT长度
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
            if self.kuavo_control_scheme == "rl":
                self.INIT_ARM_POS = [int(0)] * tact_length
            else:
                # 基础28个关节 + 可选的1个腰部关节
                base_init = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                if self.has_waist:
                    self.INIT_ARM_POS = base_init + [0]  # 添加腰部初始位置
                else:
                    self.INIT_ARM_POS = base_init
            self.current_arm_joint_state = [0] * tact_length
        elif self.robot_class == ROBAN:
            self.INIT_ARM_POS = [22.91831, 0, 0, -45.83662, 22.91831, 0, 0, -45.83662, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]# task.info: shoudler_center: 0.4rad, elbow_center: -0.8rad
            self.current_arm_joint_state = [0] * self.ROBAN_TACT_LENGTH

        # rospy.spin()

        # Initialize ROS node
        rospy.init_node('autostart_arm_trajectory_bezier_demo')
        self.arm_restore_flag = rospy.get_param('~arm_restore_flag', True)

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
        self.control_waist_pub = rospy.Publisher('/robot_waist_motion_data', Float64MultiArray, queue_size=1, 
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
            arm_part = list(joint_msg.joint_data.joint_q[13:21])
            hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12
            head_part = list(joint_msg.joint_data.joint_q[-2:])
            waist_part = [joint_msg.joint_data.joint_q[0]]
            self.current_arm_joint_state = arm_part + hand_part + head_part + waist_part

        self.current_arm_joint_state = [round(v, 5) for v in self.current_arm_joint_state]

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
                self.waist_state.data = [math.degrees(pos) for pos in point.positions[28:29]]
            
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

                self.waist_state.data = [math.degrees(pos) for pos in point.positions[22:]]

    def call_change_arm_ctrl_mode_service(self, arm_ctrl_mode):
        result = True
        service_name = "humanoid_change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
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

    def load_json_file(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error reading file {file_path}: {e}")
            return None

    def add_init_frame(self, frames):
        action_data = {}

        # rl 要在刚开始插入当前状态为初始值来平滑过渡，ocs2 不需要
        if self.kuavo_control_scheme == "rl":
            import copy
            for frame in frames:
                frame["keyframe"] += 100
            frame0 = copy.deepcopy(frames[0])
            # 如果原来的长度长则补全，否则就需要裁剪
            if len(self.current_arm_joint_state) > len(frame0["servos"]):
                # 当前状态长度更长，需要裁剪到原始长度
                frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state[:len(frame0["servos"])]]
            else:
                # 当前状态长度更短或相等，需要补全到原始长度
                frame0["servos"] = [math.degrees(pos) for pos in self.current_arm_joint_state] + [0] * (len(frame0["servos"]) - len(self.current_arm_joint_state))
            frame0["keyframe"] = 0
            frames.insert(0, frame0)

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
        if self.kuavo_control_scheme == "rl":
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
                self.waist_state.data = [0]
                self.control_waist_pub.publish(self.waist_state)

    def create_action_data(self, finish_time):
        # 根据是否有腰部关节确定TACT长度
        if self.robot_class == KUAVO:
            tact_length = self.KUAVO_TACT_LENGTH + (1 if self.has_waist else 0)
        else:
            tact_length = self.ROBAN_TACT_LENGTH
        frames = [
            {
                "servos": [int(round(math.degrees(x))) for x in self.current_arm_joint_state],
                "keyframe": 0,
                "attribute": {str(i+1): {"CP": [[0,0],[0,0]]} for i in range(tact_length)}
            },
            {
                "servos": self.INIT_ARM_POS,
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
        data = self.create_action_data(finish_time)

        if self.kuavo_control_scheme == "rl":
            finish_time += 1
        self.END_FRAME_TIME = finish_time

        action_data = self.add_init_frame(data["frames"])
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
        """复位动作结束后停止发布，避免递归复位。"""
        self.arm_flag = False
        rospy.loginfo("Reset trajectory finished. Stopping publishers.")

    def publish_running_action_state(self):
        """持续发布 state=1"""
        rate = rospy.Rate(1)  # 每秒发布 2 次
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
        rospy.loginfo(f"Robot action state published: state={state}")

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

        file_path = f"{self.action_files_path}/{action_name}.tact"
        data = self.load_json_file(file_path)
        if not data:
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message=f"Action file {action_name} not found")

        robot_type_raw = data.get("robotType", None)
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
            45: [43, 45, 46, 48, 49],
            11: [11, 13, 14],
            13: [11, 13, 14],
            14: [11, 13, 14],
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

        # 获取初始帧时间
        self.arm_flag = True
        first_value = data.get("first", 0)
        self.START_FRAME_TIME = round(first_value * 0.01, 2)  # 转换为秒，取两位小数
        self.x_shift = self.START_FRAME_TIME  # 动态调整 x_shift

        # 读取动作完成时间
        finish_time = data.get("finish", 0) * 0.01 # 转换为秒
        if self.kuavo_control_scheme == "rl":
            finish_time += 1.0
        self.END_FRAME_TIME = finish_time

        action_data = self.add_init_frame(data["frames"])
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
            # threading.Thread(target=self.delayed_publish_action_state, args=(finish_time,)).start()
            self.delayed_publish_action_state(finish_time)
            return ExecuteArmActionResponse(success=True, message="Action executed successfully")
        else:
            rospy.logerr("Failed to plan arm trajectory")
            self.publish_action_state(0)
            return ExecuteArmActionResponse(success=False, message="Failed to execute action")

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
                    if len(self.waist_state.data) != 0:
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
