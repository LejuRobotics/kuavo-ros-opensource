#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import sys
import os
import json
import time
import signal
from collections import deque
import rospy
import tf
import numpy as np
import math
import netifaces
from pprint import pprint
from kuavo_ros_interfaces.msg import robotHeadMotionData
from noitom_hi5_hand_udp_python.msg import PoseInfoList, PoseInfo
from kuavo_msgs.msg import JoySticks, headCtrlMode
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import threading
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray, String, Empty, Float64
from std_srvs.srv import Trigger
# Add the parent directory to the system path to allow relative imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

# Import the hand_pose_pb2 module
import protos.hand_pose_pb2 as event_pb2
import protos.robot_info_pb2 as robot_info_pb2
import protos.hand_wrench_srv_pb2 as hand_wrench_srv_pb2

from robot_state_server import RobotStateServer
from quest_vr_config import Quest3VrConfig, HandWrenchConfig
from head_control_manager import HeadControlManager, HeadControlMode
from kuavo_msgs.srv import SetHeadControlMode, SetHeadControlModeResponse

class Quest3BoneFramePublisher:
    MAX_PENDING_REQUESTS = 32
    DEFAULT_JOYSTICK_TIMEOUT_SEC = 0.3
    DEFAULT_QUEST_DISCONNECT_TIMEOUT_SEC = 1.0
    UDP_RECEIVER_POLL_INTERVAL_SEC = 0.1

    def __init__(self):
        self.bone_names = [
            "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
            "LeftHandPalm", "RightHandPalm", "LeftHandThumbMetacarpal",
            "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
            "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
            "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
            "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
            "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip",
            "Root", "Chest", "Neck", "Head"
        ]

        self.exit_listen_thread_for_quest3_broadcast = False
        self.bone_name_to_index = {name: index for index, name in enumerate(self.bone_names)}
        self.index_to_bone_name = {index: name for index, name in enumerate(self.bone_names)}
        
        self.current_file_dir = os.path.dirname(os.path.abspath(__file__))
        self.CONFIG_FILE = os.path.join(self.current_file_dir, "config.json")
        self.calibrated_head_quat_matrix_inv = None
        self.head_motion_range = self.get_head_motion_range()
        self.chest_motion_range = self.get_chest_motion_range()
        self.chest_motion_range = self.get_chest_motion_range()
        
        self.sock = None
        self.socket_lock = threading.Lock()
        self.server_address = None
        self.port = None
        self.udp_packets_received = 0
        self.stale_pose_frames_dropped = 0
        self.last_reported_stale_pose_frames = 0
        self.receiver_condition = threading.Condition()
        self.receiver_stop_event = threading.Event()
        self.receiver_thread = None
        self.receiver_socket = None
        self.receiver_error = None
        self.latest_pose_event = None
        self.latest_pose_receive_time = None
        self.pending_requests = deque()
        self.pending_request_keys = set()
        self.joystick_publish_lock = threading.Lock()
        self.joystick_neutral_published = False
        self.shutdown_lock = threading.Lock()
        self.shutdown_started = False

        self.listening_udp_ports_cnt = 0
        
        rospy.init_node('Quest3_bone_frame_publisher', anonymous=True)
        self.reset_vr_torso_state = rospy.ServiceProxy('/quest3/reset_vr_torso_state', Trigger)
        self.vr_torso_state_reset = False
        self.joystick_timeout_sec = max(
            0.05,
            float(rospy.get_param("~joystick_timeout_sec", self.DEFAULT_JOYSTICK_TIMEOUT_SEC)),
        )
        self.quest_disconnect_timeout_sec = max(
            self.joystick_timeout_sec,
            float(
                rospy.get_param(
                    "~quest_disconnect_timeout_sec",
                    self.DEFAULT_QUEST_DISCONNECT_TIMEOUT_SEC,
                )
            ),
        )
        self.rate = rospy.Rate(100.0)
        
        self.br = tf.TransformBroadcaster()
        # Bone targets are real-time commands: never retain more than the newest
        # unsent frame for a slow subscriber connection.
        self.pose_pub = rospy.Publisher('/leju_quest_bone_poses', PoseInfoList, queue_size=1)
        # 延迟诊断默认关闭，避免常规遥操作时的统计、ROS发布和日志开销。
        self.enable_vr_latency_diagnostics = rospy.get_param(
            '~enable_vr_latency_diagnostics', False
        )
        self.latency_pub = None
        if self.enable_vr_latency_diagnostics:
            self.latency_pub = rospy.Publisher(
                '/quest3/node_processing_latency_ms', Float64, queue_size=10
            )
        self._latency_samples = []
        self._latency_log_every = 100
        rospy.loginfo(
            "Quest3 VR latency diagnostics: %s",
            "enabled" if self.enable_vr_latency_diagnostics else "disabled",
        )
        self.head_data_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        self.chest_data_pub = rospy.Publisher('/robot_chest_motion_data', Float64MultiArray, queue_size=10)
        self.head_pose_pub = rospy.Publisher('/robot_head_pose', PoseStamped, queue_size=10)
        self.chest_pose_pub = rospy.Publisher('/robot_chest_pose', PoseStamped, queue_size=10)
        # JoySticks is a current-state command. Keep only the newest outbound snapshot
        # so a slow subscriber cannot make the robot replay stale stick positions.
        self.joysticks_pub = rospy.Publisher('quest_joystick_data', JoySticks, queue_size=1)
        
        # 末端力配置发布器
        self.hand_wrench_config_pub = rospy.Publisher('/quest3/hand_wrench_config', Float64MultiArray, queue_size=1)
        
        self.listener = tf.TransformListener()
        self.hand_finger_tf_pub = rospy.Publisher('/quest_hand_finger_tf', TFMessage, queue_size=10)
        # 批量发布所有骨骼TF的发布器
        self.bone_tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

        self.enable_head_control = rospy.get_param("~enable_head_control", True)
        rospy.loginfo(f"enable_head_control: {self.enable_head_control}")
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGHUP, self.signal_handler)
        rospy.on_shutdown(self.shutdown)
        self.broadcast_ips = []
        self.robot_info_sent_initial_broadcast = False
        self.robot_info_lock = threading.Lock()
        
        # RobotStateServer 引用，用于处理 hand wrench 请求
        self.robot_state_server = None
        
        # 配置管理器
        self.config_manager = Quest3VrConfig()
        
        # 头部控制管理器
        self.head_control_manager = HeadControlManager(self.config_manager)
        self._init_head_control_manager()
        
        # 创建头部控制模式设置服务
        self.head_control_mode_service = rospy.Service(
            '/quest3/set_head_control_mode',
            SetHeadControlMode,
            self._handle_set_head_control_mode
        )
        rospy.loginfo("Head control mode service started at /quest3/set_head_control_mode")

        # 创建头部控制模式发布节点
        self.head_ctrl_mode_pub = rospy.Publisher('/quest3/head_control_mode', headCtrlMode, queue_size=10)
        # 每次 set_head_control_mode 成功设为 fixed 时发布，供 QuestControlFSM 在手臂复位窗口内更新「解锁后恢复」快照（含重复 fixed）
        self._pub_head_fixed_user_intent = rospy.Publisher(
            '/quest3/head_fixed_forward_user_intent', Empty, queue_size=1, latch=False
        )

        rospy.loginfo("Head reset service started at /quest3/reset_head_pose")

        # 初始化VR连接状态参数
        rospy.set_param('/quest3/connected', False)

    def set_robot_state_server(self, robot_state_server):
        """设置 RobotStateServer 引用
        
        Args:
            robot_state_server: RobotStateServer 实例
        """
        self.robot_state_server = robot_state_server
        rospy.loginfo("RobotStateServer reference set for hand wrench processing")
    
    def update_broadcast_ips(self, ips_list):
        """Updates the list of broadcast IPs."""
        if isinstance(ips_list, list):
            self.broadcast_ips = sorted(list(set(ips_list))) # Store unique sorted IPs
            rospy.loginfo(f"Updated broadcast IPs to: {self.broadcast_ips}")
        else:
            rospy.logwarn("Failed to update broadcast IPs: input is not a list.")

    def send_robot_info_on_broadcast_ips(self, robot_name, robot_version, start_port, end_port):
        """Sends RobotDescription protobuf message to all broadcast IPs within a port range."""
        if not self.broadcast_ips:
            rospy.logwarn("No broadcast IPs configured. Cannot send robot info.")
            return
        rospy.loginfo(f"Broadcasting robot info: Name='{robot_name}', Version={robot_version}, "
                      f"Ports={start_port}-{end_port} to IPs: {self.broadcast_ips}")
        robot_desc = robot_info_pb2.RobotDescription()
        robot_desc.robot_name = robot_name
        robot_desc.robot_version = robot_version
        serialized_message = robot_desc.SerializeToString()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as send_sock:
            send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            for ip in self.broadcast_ips:
                for port in range(start_port, end_port + 1):
                    try:
                        send_sock.sendto(serialized_message, (ip, port))
                        rospy.logdebug(f"Sent robot info to {ip}:{port}")
                    except Exception as e:
                        rospy.logerr(f"Error sending robot info to {ip}:{port}: {e}")
 
    def load_config(self):
        if os.path.exists(self.CONFIG_FILE):
            with open(self.CONFIG_FILE, 'r') as f:
                return json.load(f)
        return {}

    def save_config(self, config):
        with open(self.CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)

    def get_head_motion_range(self):
        config = self.load_config()
        return config.get("head_motion_range", None)
    
    def get_chest_motion_range(self):
        config = self.load_config()
        return config.get("chest_motion_range", None)
    
    def _init_head_control_manager(self):
        """初始化头部控制管理器"""
        config = self.load_config()
        
        # 读取头部控制配置（统一配置来源）
        head_control_config = config.get("head_control", {})
        
        # 设置关节限制（优先从head_control读取，如果没有则从head_motion_range读取，最后使用默认值）
        joint_limits = head_control_config.get("joint_limits", {})
        if joint_limits:
            yaw_limit = joint_limits.get("yaw", [-80, 80])
            pitch_limit = joint_limits.get("pitch", [-25, 25])
        elif self.head_motion_range:
            # 兼容旧配置：从head_motion_range读取
            yaw_limit = self.head_motion_range.get("yaw", [-80, 80])
            pitch_limit = self.head_motion_range.get("pitch", [-25, 25])
        else:
            # 默认值
            yaw_limit = [-80, 80]
            pitch_limit = [-25, 25]
        
        self.head_control_manager.set_joint_limits(yaw_limit, pitch_limit)
        
        # 初始化头部控制模式 （从config.json读取，默认vr_follow模式）
        mode_str = str(head_control_config.get("mode", "vr_follow") or "vr_follow").strip().lower()
        mode = HeadControlMode.from_string(mode_str)
        if mode is None:
            legacy_mode_map = {
                "auto_track": HeadControlMode.AUTO_TRACK_ACTIVE,
                "fixed_main": HeadControlMode.FIXED_MAIN_HAND,
            }
            mode = legacy_mode_map.get(mode_str, HeadControlMode.VR_FOLLOW)
        fixed_hand = head_control_config.get("fixed_main_hand", "right")
        self.head_control_manager.set_mode(mode, fixed_hand)
        
        # 设置平滑滤波系数（从配置读取，默认0.15）
        smoothing_factor = head_control_config.get("smoothing_factor", 0.15)
        self.head_control_manager.set_smoothing_factor(smoothing_factor)
        
        # 设置主动手检测阈值（从配置读取，默认0.02）
        active_hand_threshold = head_control_config.get("active_hand_threshold", 0.02)
        self.head_control_manager.set_active_hand_threshold(active_hand_threshold)
        
        rospy.loginfo(f"Head control manager initialized: mode={mode_str}, fixed_hand={fixed_hand}, "
                     f"yaw_limit={yaw_limit}, pitch_limit={pitch_limit}")

    def signal_handler(self, sig, frame):
        print('Exiting gracefully...')
        self.exit_listen_thread_for_quest3_broadcast = True
        self.receiver_stop_event.set()
        with self.receiver_condition:
            self.receiver_condition.notify_all()
        # Neutralize motion commands before ROS publishers and services begin shutdown.
        self.publish_neutral_joystick("termination signal received")
        # 在 ROS 标记为 shutdown 前完成服务调用，保证仍可连接到常驻 Quest FSM
        self.reset_vr_torso_state_on_shutdown()
        rospy.signal_shutdown('Quest3 monitor received a termination signal')

    def shutdown(self):
        """Stop the receiver exactly once and leave the last joystick command neutral."""
        with self.shutdown_lock:
            if self.shutdown_started:
                return
            self.shutdown_started = True

        self.exit_listen_thread_for_quest3_broadcast = True
        self.publish_neutral_joystick("Quest3 monitor shutting down")
        self.reset_vr_torso_state_on_shutdown()
        self.stop_udp_receiver()

    def reset_vr_torso_state_on_shutdown(self):
        if self.vr_torso_state_reset:
            return

        self.vr_torso_state_reset = True
        try:
            rospy.wait_for_service('/quest3/reset_vr_torso_state', timeout=1.0)
            response = self.reset_vr_torso_state()
            if response.success:
                rospy.loginfo('Quest VR torso state reset during monitor shutdown.')
            else:
                rospy.logwarn('Quest VR torso-state reset failed during shutdown: %s', response.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn('Could not reset Quest VR torso state during shutdown: %s', error)

    def setup_socket(self, server_address, port):
        with self.socket_lock:
            if self.sock is not None:
                print("Socket is already established, skip creating a new one.")
            else:
                self.server_address = (server_address, port)
                self.port = port
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.settimeout(self.DEFAULT_QUEST_DISCONNECT_TIMEOUT_SEC)
        return (server_address, port)


    def send_initial_message(self):
        message = b'hi'
        max_retries = 200
        for attempt in range(max_retries):
            if rospy.is_shutdown():
                return False
            try:
                with self.socket_lock:
                    handshake_socket = self.sock
                if handshake_socket is None:
                    rospy.logerr("Cannot connect to Quest3: UDP socket is not available")
                    return False
                handshake_socket.sendto(message, self.server_address)
                handshake_socket.recvfrom(1024)
                print(f"\033[92mAcknowledgment From Quest3 received on attempt {attempt + 1}, start to receiving data...\033[0m")
                # 连接成功后设置参数服务器参数
                rospy.set_param('/quest3/connected', True)
                rospy.loginfo("VR连接成功！已设置参数: /quest3/connected=True")
                return True
            except socket.timeout:
                print(f"\033[91mQuest3_timeout: Attempt {attempt + 1} timed out. Retrying...\033[0m")
            except OSError as error:
                if rospy.is_shutdown():
                    return False
                rospy.logerr("Quest3 UDP handshake failed: %s", error)
                return False
            except KeyboardInterrupt:
                print("Force quit by Ctrl-c.")
                self.signal_handler(signal.SIGINT, None)
                return False
        print("Failed to send message after 200 attempts.")
        return False

    def convert_position_to_right_hand(self, left_hand_position):
        return {
            "x": 0 - left_hand_position["z"],
            "y": 0 - left_hand_position["x"],
            "z": left_hand_position["y"]
        }

    def convert_quaternion_to_right_hand(self, left_hand_quat):
        return (
            0 - left_hand_quat[2],
            0 - left_hand_quat[0],
            left_hand_quat[1],
            left_hand_quat[3]
        )

    def updateAFrame(self, frame_name, frame_position, frame_rotation_quat, time_now):
        self.br.sendTransform((frame_position["x"], frame_position["y"], frame_position["z"]), 
                              frame_rotation_quat, time_now, frame_name, "torso")
        
    def update_quest_hand_finger_tf(self):
        tf_msg = TFMessage()
        for frame_name in [
                "LeftHandPalm", "RightHandPalm","LeftHandThumbMetacarpal",
                "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
                "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
                "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
                "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
                "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip"
            ]:
                try:
                    if "Left" in frame_name:
                        relative_position, relative_rotation = self.listener.lookupTransform("LeftHandPalm", frame_name, rospy.Time(0))
                    else:   
                        relative_position, relative_rotation = self.listener.lookupTransform("RightHandPalm", frame_name, rospy.Time(0))
                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "LeftHandPalm" if "Left" in frame_name else "RightHandPalm"
                    transform.child_frame_id = frame_name
                    transform.transform.translation.x = relative_position[0]
                    transform.transform.translation.y = relative_position[1]
                    transform.transform.translation.z = relative_position[2]
                    transform.transform.rotation.x = relative_rotation[0]
                    transform.transform.rotation.y = relative_rotation[1]
                    transform.transform.rotation.z = relative_rotation[2]
                    transform.transform.rotation.w = relative_rotation[3]
                    tf_msg.transforms.append(transform)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF lookup failed: {e}")
                    return
        self.hand_finger_tf_pub.publish(tf_msg)

    def normalize_degree_in_180(self, degree):
        if degree > 180:
            degree -= 180
        elif degree < -180:
            degree += 180
        return degree

    def pub_head_motion_data(self, cur_quat):
        """发布头部运动数据（根据控制模式）"""
        if not self.enable_head_control:
            return
        
        # VR随动模式：使用原有逻辑
        if self.head_control_manager.mode == HeadControlMode.VR_FOLLOW:
            try:
                # Get the transform from Chest to Head using TF listener
                (trans, rot) = self.listener.lookupTransform("Chest", "Head", rospy.Time(0))
                
                # Convert quaternion to euler angles (roll, pitch, yaw)
                rpy = tf.transformations.euler_from_quaternion(rot)
                rpy_deg = [r * 180 / math.pi for r in rpy]
                
                # Extract pitch (around X-axis) and yaw (around Y-axis)
                pitch = max(min(self.normalize_degree_in_180(round(rpy_deg[0], 2)), self.head_motion_range["pitch"][1]), self.head_motion_range["pitch"][0])
                yaw = max(min(self.normalize_degree_in_180(round(rpy_deg[1], 2)), self.head_motion_range["yaw"][1]), self.head_motion_range["yaw"][0])
                
                msg = robotHeadMotionData()
                msg.joint_data = [yaw, pitch] 
                self.head_data_pub.publish(msg)
                
                # Publish head pose (position set to zero)
                head_pose_msg = PoseStamped()
                head_pose_msg.header.stamp = rospy.Time.now()
                head_pose_msg.header.frame_id = "Chest"
                head_pose_msg.pose.position.x = 0.0
                head_pose_msg.pose.position.y = 0.0
                head_pose_msg.pose.position.z = 0.0
                head_pose_msg.pose.orientation.x = rot[0]
                head_pose_msg.pose.orientation.y = rot[1]
                head_pose_msg.pose.orientation.z = rot[2]
                head_pose_msg.pose.orientation.w = rot[3]
                self.head_pose_pub.publish(head_pose_msg)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"TF lookup failed for Chest->Head transform: {e}")
                return
        # 其他模式：由_update_head_control_from_hands()处理
    
    def pub_chest_motion_data(self, cur_quat):
        try:
            # Get the transform from torso to Chest using TF listener
            (trans, rot) = self.listener.lookupTransform("torso", "Chest", rospy.Time(0))
            
            # Convert quaternion to euler angles (roll, pitch, yaw)
            rpy = tf.transformations.euler_from_quaternion(rot)
            rpy_deg = [r * 180 / math.pi for r in rpy]
            
            # Extract pitch (around Y-axis) and yaw (around Z-axis)
            # Normalize first, then apply range limits if configured (similar to head motion)
            pitch = self.normalize_degree_in_180(round(rpy_deg[0], 2))
            yaw = self.normalize_degree_in_180(round(rpy_deg[1], 2))
            
            # Apply range limits if configured
            if self.chest_motion_range:
                if "pitch" in self.chest_motion_range:
                    pitch = max(min(pitch, self.chest_motion_range["pitch"][1]), self.chest_motion_range["pitch"][0])
                if "yaw" in self.chest_motion_range:
                    yaw = max(min(yaw, self.chest_motion_range["yaw"][1]), self.chest_motion_range["yaw"][0])
            
            # Extract position (x, y, z) from translation vector
            x = round(trans[0], 3)
            y = round(trans[1], 3)
            z = round(trans[2], 3)
            
            # Create 1x5 vector: [yaw, pitch, x, y, z]
            chest_data = [yaw, pitch, x, y, z]
            
            msg = Float64MultiArray()
            msg.data = chest_data
            self.chest_data_pub.publish(msg)
            
            # Publish chest pose (position set to zero)
            chest_pose_msg = PoseStamped()
            chest_pose_msg.header.stamp = rospy.Time.now()
            chest_pose_msg.header.frame_id = "torso"
            chest_pose_msg.pose.position.x = 0.0
            chest_pose_msg.pose.position.y = 0.0
            chest_pose_msg.pose.position.z = 0.0
            chest_pose_msg.pose.orientation.x = rot[0]
            chest_pose_msg.pose.orientation.y = rot[1]
            chest_pose_msg.pose.orientation.z = rot[2]
            chest_pose_msg.pose.orientation.w = rot[3]
            self.chest_pose_pub.publish(chest_pose_msg)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed for torso->Chest transform: {e}")
            return
    
    def pub_chest_pose_in_torso_frame(self, chest_position, chest_quat, root_offset, time_now):
        chest_pose_msg = PoseStamped()
        chest_pose_msg.header.stamp = time_now
        chest_pose_msg.header.frame_id = "torso"
        chest_pose_msg.pose.position.x = chest_position["x"] - root_offset["x"]
        chest_pose_msg.pose.position.y = chest_position["y"] - root_offset["y"]
        chest_pose_msg.pose.position.z = chest_position["z"]
        chest_pose_msg.pose.orientation.x = chest_quat[0]
        chest_pose_msg.pose.orientation.y = chest_quat[1]
        chest_pose_msg.pose.orientation.z = chest_quat[2]
        chest_pose_msg.pose.orientation.w = chest_quat[3]
        self.chest_pose_pub.publish(chest_pose_msg)

    def _get_robot_hand_tcp_positions(self):
        """
        获取机器人本体左右手末端TCP位置（相对于base_link）
        
        Returns:
            (left_tcp_pos, right_tcp_pos): 左手和右手TCP位置 [x, y, z]，如果获取失败返回None
        """
        left_tcp_pos = None
        right_tcp_pos = None
        
        try:
            (trans, _) = self.listener.lookupTransform("base_link", "zarm_l7_end_effector", rospy.Time(0))
            left_tcp_pos = list(trans)  # [x, y, z]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TF查询失败，不打印错误（可能手部还未发布）
            pass
        
        try:
            (trans, _) = self.listener.lookupTransform("base_link", "zarm_r7_end_effector", rospy.Time(0))
            right_tcp_pos = list(trans)  # [x, y, z]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # TF查询失败，不打印错误（可能手部还未发布）
            pass
        
        return left_tcp_pos, right_tcp_pos
    
    def _get_robot_head_pos_from_tf(self):
        """
        从TF树获取机器人本体头部位置（相对于base_link）
        
        Returns:
            head_pos: 头部位置 [x, y, z]，如果获取失败返回None
        """
        # 优先尝试获取camera_base，如果失败则尝试zhead_2_link
        try:
            (trans, _) = self.listener.lookupTransform("base_link", "camera_base", rospy.Time(0))
            return list(trans)  # [x, y, z] - 实时位置
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            try:
                (trans, _) = self.listener.lookupTransform("base_link", "zhead_2_link", rospy.Time(0))
                return list(trans)  # [x, y, z] - 实时位置
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # TF查询失败，不打印错误（可能Head还未发布），返回None使用配置值
                return None
    
    def _update_head_control_from_hands(self):
        """从机器人本体手部末端TCP位置更新头部控制"""
        # 获取机器人本体左右手末端TCP位置（相对于base_link）
        left_tcp_pos, right_tcp_pos = self._get_robot_hand_tcp_positions()
        
        # 从TF树获取机器人本体头部位置（相对于base_link）
        head_pos = self._get_robot_head_pos_from_tf()
        
        # 历史代码每帧完整调用本方法两次。保留两次滤波更新以维持最终头部
        # 目标响应，但只查询一次TF并只发布一次，避免重复的昂贵工作。
        self.head_control_manager.update(
            left_hand_tcp_pos=left_tcp_pos,
            right_hand_tcp_pos=right_tcp_pos,
            head_pos=head_pos  # 使用实时位置，如果为None则使用配置值
        )
        self.head_control_manager.update(
            left_hand_tcp_pos=left_tcp_pos,
            right_hand_tcp_pos=right_tcp_pos,
            head_pos=head_pos
        )
        
        # 发布头部控制命令
        self.head_control_manager.publish_head_command(self.head_data_pub)
    
    def _handle_set_head_control_mode(self, req):
        """
        处理头部控制模式设置服务请求
        
        Args:
            req: SetHeadControlMode服务请求
            
        Returns:
            SetHeadControlModeResponse服务响应
        """
        response = SetHeadControlModeResponse()
        
        try:
            # 将字符串模式转换为枚举值
            mode = HeadControlMode.from_string(req.mode)
            
            if mode is None:
                response.success = False
                response.message = f"Invalid mode: {req.mode}. Valid modes are: fixed, auto_track_active, fixed_main_hand, vr_follow"
                response.current_mode = HeadControlMode.to_string(self.head_control_manager.mode)
                rospy.logwarn(f"Failed to set head control mode: {response.message}")
                return response
            
            # 处理fixed_hand参数（仅在fixed_main_hand模式时需要）
            if mode == HeadControlMode.FIXED_MAIN_HAND:
                # fixed_main_hand模式需要指定fixed_hand参数
                if not req.fixed_hand or req.fixed_hand.strip() == "":
                    response.success = False
                    response.message = f"For 'fixed_main_hand' mode, fixed_hand parameter is required. Must be 'left' or 'right'"
                    response.current_mode = HeadControlMode.to_string(self.head_control_manager.mode)
                    rospy.logwarn(f"Failed to set head control mode: {response.message}")
                    return response
                
                fixed_hand = req.fixed_hand.lower().strip()
                if fixed_hand not in ["left", "right"]:
                    response.success = False
                    response.message = f"Invalid fixed_hand: '{req.fixed_hand}'. Must be 'left' or 'right'"
                    response.current_mode = HeadControlMode.to_string(self.head_control_manager.mode)
                    rospy.logwarn(f"Failed to set head control mode: {response.message}")
                    return response
            else:
                # 其他模式不需要fixed_hand参数，使用默认值
                fixed_hand = "right"  # 默认值，不会被使用
            
            # 设置模式
            self.head_control_manager.set_mode(mode, fixed_hand)
            if mode == HeadControlMode.FIXED:
                self._pub_head_fixed_user_intent.publish(Empty())
            
            response.success = True
            response.message = f"Head control mode set to: {req.mode}" + (f", fixed_hand: {fixed_hand}" if mode == HeadControlMode.FIXED_MAIN_HAND else "")
            response.current_mode = HeadControlMode.to_string(self.head_control_manager.mode)
            rospy.loginfo(f"Head control mode changed via service: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error setting head control mode: {str(e)}"
            response.current_mode = HeadControlMode.to_string(self.head_control_manager.mode)
            rospy.logerr(f"Exception in set_head_control_mode service: {e}")
            import traceback
            traceback.print_exc()
        
        return response
    
    def process_item_mass_force_request(self, request):
        """处理物品质量与力请求
        
        Args:
            request: hand_wrench_srv_pb2.ItemMassForceRequest 请求消息
        """
        if not self.robot_state_server:
            rospy.logwarn("RobotStateServer not set, cannot process item mass force request")
            return
        try:
            if request.operation == hand_wrench_srv_pb2.ItemMassForceOperation.GET:
                # GET 操作：返回所有配置
                rospy.loginfo("Received GET item mass force request")
                response = hand_wrench_srv_pb2.ItemMassForceResponse()
                response.operation = hand_wrench_srv_pb2.ItemMassForceOperation.GET
                
                # 获取所有配置
                success, errmsg, hand_wrench_cases = self.config_manager.get_all_hand_wrench_cases()
                print(f"hand_wrench_cases: {hand_wrench_cases}")
                print(f"success: {success}")
                print(f"errmsg: {errmsg}")
                if not success:
                    response.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.ERROR
                    response.description = errmsg
                else:
                    # 将所有配置添加到响应中
                    for case_name, config in hand_wrench_cases.items():
                        item_mass_force = response.item_mass_forces.add()
                        item_mass_force.case_name = case_name
                        item_mass_force.description = config.description
                        item_mass_force.item_mass = config.itemMass
                        item_mass_force.lforce_x = config.lforceX
                        item_mass_force.lforce_y = config.lforceY
                        item_mass_force.lforce_z = config.lforceZ
                    
                    response.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.SUCCESS
                    response.description = f"Successfully retrieved {len(hand_wrench_cases)} configurations"
                    rospy.loginfo(f"Returning {len(hand_wrench_cases)} hand wrench configurations")
                
                self.robot_state_server.add_item_mass_force_response(response)
                
            elif request.operation == hand_wrench_srv_pb2.ItemMassForceOperation.SET:
                # SET 操作：设置物品质量与力
                rospy.loginfo(f"Received SET item mass force request: case_name={request.data.case_name}, "
                             f"mass={request.data.item_mass}kg, force=({request.data.lforce_x}, "
                             f"{request.data.lforce_y}, {request.data.lforce_z})N")
                
                # 创建配置对象
                hand_wrench_config = HandWrenchConfig(
                    default=False,
                    description=request.data.description,
                    itemMass=request.data.item_mass,
                    lforceX=request.data.lforce_x,
                    lforceY=request.data.lforce_y,
                    lforceZ=request.data.lforce_z
                )
                
                # 保存配置
                success = self.config_manager.set_hand_wrench_config(request.data.case_name, hand_wrench_config)
                
                # 创建响应
                response = hand_wrench_srv_pb2.ItemMassForceResponse()
                response.operation = hand_wrench_srv_pb2.ItemMassForceOperation.SET
                
                if success:
                    response.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.SUCCESS
                    response.description = f"Successfully set item mass force for case: {request.data.case_name}"
                    rospy.loginfo(f"✓ Configuration saved for case: {request.data.case_name}")
                    
                    # 发布末端力配置到 QuestControlFSMNode
                    # 消息格式: [item_mass, lforce_x, lforce_y, lforce_z, rforce_x, rforce_y, rforce_z]
                    from std_msgs.msg import Float64MultiArray
                    config_msg = Float64MultiArray()
                    config_msg.data = [
                        request.data.item_mass,      # [0] 物品质量
                        request.data.lforce_x,       # [1] 左手 X 方向力
                        request.data.lforce_y,       # [2] 左手 Y 方向力
                        request.data.lforce_z,       # [3] 左手 Z 方向力
                        request.data.lforce_x,       # [4] 右手 X 方向力 (与左手相同)
                        -request.data.lforce_y,      # [5] 右手 Y 方向力 (取反)
                        request.data.lforce_z        # [6] 右手 Z 方向力 (与左手相同)
                    ]
                    self.hand_wrench_config_pub.publish(config_msg)
                    rospy.loginfo(f"✓ Published hand wrench config to /quest3/hand_wrench_config: "
                                 f"mass={request.data.item_mass:.2f}kg, "
                                 f"left_force=({request.data.lforce_x:.2f}, {request.data.lforce_y:.2f}, {request.data.lforce_z:.2f})N, "
                                 f"right_force=({request.data.lforce_x:.2f}, {-request.data.lforce_y:.2f}, {request.data.lforce_z:.2f})N")
                else:
                    response.status = hand_wrench_srv_pb2.ItemMassForceResponse.OperationStatus.ERROR
                    response.description = f"Failed to save configuration for case: {request.data.case_name}"
                    rospy.logerr(f"✗ Failed to save configuration for case: {request.data.case_name}")
                
                # 将响应添加到队列
                self.robot_state_server.add_item_mass_force_response(response)
                
            else:
                rospy.logerr(f"Unknown item mass force operation: {request.operation}")
                
        except Exception as e:
            rospy.logerr(f"Error processing item mass force request: {e}")
            import traceback
            traceback.print_exc()

    def _publish_receiver_error(self, error):
        with self.receiver_condition:
            self.receiver_error = error
            self.receiver_condition.notify_all()

    def publish_neutral_joystick(self, reason):
        """Publish one all-zero snapshot when input becomes stale or disconnects."""
        with self.joystick_publish_lock:
            if self.joystick_neutral_published:
                return False
            try:
                self.joysticks_pub.publish(JoySticks())
            except Exception as error:
                rospy.logerr("Failed to neutralize Quest3 joystick: %s", error)
                return False
            self.joystick_neutral_published = True

        rospy.logwarn("Quest3 joystick neutralized: %s", reason)
        return True

    def _udp_receiver_loop(self, receiver_socket, stop_event):
        """Continuously drain UDP independently from the slower ROS/TF processing."""
        last_packet_time = time.monotonic()
        receiver_socket.settimeout(self.UDP_RECEIVER_POLL_INTERVAL_SEC)

        while not stop_event.is_set() and not rospy.is_shutdown():
            try:
                datagram, _ = receiver_socket.recvfrom(4096)
                receive_time = None
                if self.enable_vr_latency_diagnostics:
                    # 同一数据包：UDP接收到原始数据的时刻。
                    receive_time = rospy.Time.now()
            except socket.timeout as error:
                if stop_event.is_set() or rospy.is_shutdown():
                    return

                idle_time = time.monotonic() - last_packet_time
                if idle_time >= self.joystick_timeout_sec:
                    self.publish_neutral_joystick(
                        "no valid Quest3 packet for {:.0f} ms".format(idle_time * 1000.0)
                    )
                if idle_time >= self.quest_disconnect_timeout_sec:
                    self._publish_receiver_error(
                        socket.timeout(
                            "no Quest3 UDP packet for {:.3f} seconds".format(idle_time)
                        )
                    )
                    return
                continue
            except OSError as error:
                if not stop_event.is_set():
                    self.publish_neutral_joystick("Quest3 UDP socket error")
                    self._publish_receiver_error(error)
                return

            if stop_event.is_set():
                return

            event = event_pb2.LejuHandPoseEvent()
            try:
                event.ParseFromString(datagram)
            except Exception as error:
                rospy.logwarn_throttle(5.0, "Invalid Quest3 UDP protobuf ignored: %s", error)
                continue

            last_packet_time = time.monotonic()

            # Joystick commands are safety-sensitive snapshots. Publish each original
            # snapshot unchanged; never merge states from different UDP frames. The ROS
            # publisher queue is one, so transport keeps the freshest complete snapshot.
            try:
                self.process_joystick_data(event, JoySticks(), stop_event)
            except Exception as error:
                rospy.logerr_throttle(5.0, "Failed to publish Quest3 joystick data: %s", error)

            if stop_event.is_set():
                return

            with self.receiver_condition:
                self.udp_packets_received += 1

                if len(event.poses) > 0:
                    if self.latest_pose_event is not None:
                        self.stale_pose_frames_dropped += 1
                    self.latest_pose_event = event
                    self.latest_pose_receive_time = receive_time

                if event.HasField('item_mass_force_request'):
                    request = hand_wrench_srv_pb2.ItemMassForceRequest()
                    request.CopyFrom(event.item_mass_force_request)
                    request_key = request.SerializeToString()
                    if request_key not in self.pending_request_keys:
                        if len(self.pending_requests) >= self.MAX_PENDING_REQUESTS:
                            discarded = self.pending_requests.popleft()
                            self.pending_request_keys.discard(discarded.SerializeToString())
                            rospy.logerr_throttle(
                                5.0,
                                "Quest3 request queue full; oldest request discarded",
                            )
                        self.pending_requests.append(request)
                        self.pending_request_keys.add(request_key)

                self.receiver_condition.notify()

    def start_udp_receiver(self):
        if self.receiver_thread is not None and self.receiver_thread.is_alive():
            return

        with self.socket_lock:
            receiver_socket = self.sock
        if receiver_socket is None:
            raise RuntimeError("Cannot start Quest3 UDP receiver without a socket")

        self.receiver_stop_event = threading.Event()
        with self.receiver_condition:
            self.receiver_error = None
            self.latest_pose_event = None
            self.latest_pose_receive_time = None
            self.pending_requests.clear()
            self.pending_request_keys.clear()
        self.receiver_thread = threading.Thread(
            target=self._udp_receiver_loop,
            args=(receiver_socket, self.receiver_stop_event),
            name="quest3_udp_receiver",
            daemon=True,
        )
        self.receiver_socket = receiver_socket
        self.receiver_thread.start()

    def stop_udp_receiver(self):
        stop_event = self.receiver_stop_event
        receiver_thread = self.receiver_thread
        receiver_socket = self.receiver_socket

        stop_event.set()
        with self.receiver_condition:
            self.receiver_condition.notify_all()

        self.publish_neutral_joystick("Quest3 UDP receiver stopped")

        # The receiver polls at 100 ms, so it normally observes stop_event and exits
        # before its socket is closed. This avoids close/recvfrom races and fd reuse.
        if (
            receiver_thread is not None
            and receiver_thread.is_alive()
            and receiver_thread is not threading.current_thread()
        ):
            receiver_thread.join(timeout=self.UDP_RECEIVER_POLL_INTERVAL_SEC + 0.5)

        receiver_socket_forced_closed = False
        if receiver_thread is not None and receiver_thread.is_alive():
            rospy.logwarn("Quest3 UDP receiver did not stop in time; forcing socket close")
            if receiver_socket is not None:
                try:
                    receiver_socket.close()
                    receiver_socket_forced_closed = True
                except OSError:
                    pass
            if receiver_thread is not threading.current_thread():
                receiver_thread.join(timeout=0.5)

        with self.socket_lock:
            socket_to_close = self.sock
            self.sock = None
        if (
            socket_to_close is not None
            and not (receiver_socket_forced_closed and socket_to_close is receiver_socket)
        ):
            try:
                socket_to_close.close()
            except OSError:
                pass

        if receiver_thread is not None and receiver_thread.is_alive():
            rospy.logerr("Quest3 UDP receiver thread is still alive; refusing socket restart")
            return False

        self.receiver_thread = None
        self.receiver_socket = None
        return True

    def _log_latency_stats(self, vr_timestamp):
        """打印最近一批数据包的节点处理延迟统计(单位: ms)"""
        samples = self._latency_samples
        if not samples:
            return
        arr = np.array(samples)
        rospy.loginfo(
            f"[Latency] samples={len(arr)} "
            f"mean={arr.mean():.3f}ms min={arr.min():.3f}ms max={arr.max():.3f}ms"
        )
        self._latency_samples = []

    def receive_latest_events(self):
        """Return the freshest pose, its receive time, and queued requests.

        Teleoperation needs fresh data rather than lossless delivery.  Processing every
        queued pose makes latency grow without bound whenever TF/ROS work is briefly
        slower than Quest's sender.  A dedicated receiver drains the socket continuously;
        joystick snapshots are published there unchanged, while this consumer takes only
        the newest pose and a bounded, de-duplicated list of one-shot requests.
        """
        with self.receiver_condition:
            self.receiver_condition.wait_for(
                lambda: (
                    self.latest_pose_event is not None
                    or bool(self.pending_requests)
                    or self.receiver_error is not None
                    or self.receiver_stop_event.is_set()
                    or rospy.is_shutdown()
                ),
                timeout=1.2,
            )

            # Consume pending data before reporting a timeout/error from the receiver.
            if self.latest_pose_event is None and not self.pending_requests:
                if self.receiver_error is not None:
                    error = self.receiver_error
                    self.receiver_error = None
                    raise error
                return None, None, []

            latest_pose_event = self.latest_pose_event
            latest_pose_receive_time = self.latest_pose_receive_time
            requests = list(self.pending_requests)
            dropped_total = self.stale_pose_frames_dropped
            self.latest_pose_event = None
            self.latest_pose_receive_time = None
            self.pending_requests.clear()
            self.pending_request_keys.clear()

        if dropped_total > self.last_reported_stale_pose_frames:
            rospy.logwarn_throttle(
                5.0,
                "Quest3 UDP backlog: %d stale pose frames discarded in total; "
                "newest frame kept",
                dropped_total,
            )
            self.last_reported_stale_pose_frames = dropped_total

        return latest_pose_event, latest_pose_receive_time, requests

    def run(self):
        self.start_udp_receiver()
        try:
            while not rospy.is_shutdown():
                try:
                    pose_event, pose_receive_time, item_mass_force_requests = self.receive_latest_events()
                    if pose_event is None and not item_mass_force_requests:
                        continue

                    time_now = rospy.Time.now()

                    # Requests are independent from superseded pose frames.
                    for request in item_mass_force_requests:
                        self.process_item_mass_force_request(request)

                    if pose_event is not None:
                        pose_info_list = PoseInfoList()
                        self.process_pose_data(pose_event, pose_info_list, time_now)
                        # 使用本机Unix毫秒时间戳，供下游节点与ros::Time::now()同钟计算延迟。
                        pose_info_list.timestamp_ms = int(time.time() * 1000)
                        pose_info_list.is_high_confidence = pose_event.IsDataHighConfidence
                        pose_info_list.is_hand_tracking = pose_event.IsHandTracking
                        if self.enable_vr_latency_diagnostics:
                            # 同一数据包：即将发布到姿态话题的时刻。
                            pose_publish_time = rospy.Time.now()
                        self.pose_pub.publish(pose_info_list)
                        if self.enable_vr_latency_diagnostics and pose_receive_time is not None:
                            delay_ms = (pose_publish_time - pose_receive_time).to_sec() * 1000.0
                            self.latency_pub.publish(Float64(data=delay_ms))
                            self._latency_samples.append(delay_ms)
                            if len(self._latency_samples) >= self._latency_log_every:
                                self._log_latency_stats(pose_event.timestamp)

                    self.head_control_manager.publish_head_control_mode(
                        self.head_ctrl_mode_pub,
                        HeadControlMode.to_string(self.head_control_manager.mode),
                        self.head_control_manager.fixed_main_hand,
                    )
                    self.rate.sleep()
                except socket.timeout:
                    print('Timeout occurred, no data received. Restarting socket...')
                    rospy.set_param('/quest3/connected', False)
                    rospy.logwarn("VR连接断开！已设置参数: /quest3/connected=False")
                    if not self.stop_udp_receiver():
                        break
                    if not self.restart_socket():
                        break
                    self.start_udp_receiver()
                except Exception as e:
                    print(f'An error occurred: {e}')
                    rospy.set_param('/quest3/connected', False)
                    rospy.logwarn("VR连接异常！已设置参数: /quest3/connected=False")
                    if not self.stop_udp_receiver():
                        break
                    if not self.restart_socket():
                        break
                    self.start_udp_receiver()
        finally:
            self.stop_udp_receiver()

    def process_joystick_data(self, event, joysticks_msg, stop_event=None):
        joysticks_msg.left_x = event.left_joystick.x
        joysticks_msg.left_y = event.left_joystick.y
        joysticks_msg.left_trigger = event.left_joystick.trigger
        joysticks_msg.left_grip = event.left_joystick.grip
        joysticks_msg.left_first_button_pressed = event.left_joystick.firstButtonPressed
        joysticks_msg.left_second_button_pressed = event.left_joystick.secondButtonPressed
        joysticks_msg.left_first_button_touched = event.left_joystick.firstButtonTouched
        joysticks_msg.left_second_button_touched = event.left_joystick.secondButtonTouched
        joysticks_msg.right_x = event.right_joystick.x
        joysticks_msg.right_y = event.right_joystick.y
        joysticks_msg.right_trigger = event.right_joystick.trigger
        joysticks_msg.right_grip = event.right_joystick.grip
        joysticks_msg.right_first_button_pressed = event.right_joystick.firstButtonPressed
        joysticks_msg.right_second_button_pressed = event.right_joystick.secondButtonPressed
        joysticks_msg.right_first_button_touched = event.right_joystick.firstButtonTouched
        joysticks_msg.right_second_button_touched = event.right_joystick.secondButtonTouched
        with self.joystick_publish_lock:
            if stop_event is not None and stop_event.is_set():
                return False
            self.joysticks_pub.publish(joysticks_msg)
            self.joystick_neutral_published = False
        return True

    def add_transform_to_tf_message(self, tf_msg, time_now, bone_name, scaled_position, right_hand_quat):
        """
        创建TransformStamped并添加到TFMessage中
        
        Args:
            tf_msg: TFMessage对象，用于批量发布TF变换
            time_now: 时间戳
            bone_name: 骨骼名称
            scaled_position: 缩放后的位置字典，包含x, y, z
            right_hand_quat: 四元数，格式为(x, y, z, w)
        """
        transform = TransformStamped()
        transform.header.stamp = time_now
        transform.header.frame_id = "torso"
        transform.child_frame_id = bone_name
        transform.transform.translation.x = scaled_position["x"]
        transform.transform.translation.y = scaled_position["y"]
        transform.transform.translation.z = scaled_position["z"]
        transform.transform.rotation.x = right_hand_quat[0]
        transform.transform.rotation.y = right_hand_quat[1]
        transform.transform.rotation.z = right_hand_quat[2]
        transform.transform.rotation.w = right_hand_quat[3]
        tf_msg.transforms.append(transform)

    def process_pose_data(self, event, pose_info_list, time_now):
        scale_factor = {"x": 3.0, "y": 3.0, "z": 3.0}
        # 创建TFMessage用于批量发布所有骨骼的TF变换
        tf_msg = TFMessage()
        root_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        root_index = self.bone_name_to_index.get("Root", None)
        if root_index is not None and root_index < len(event.poses):
            root_pose = event.poses[root_index]
            root_position = {"x": root_pose.position.x, "y": root_pose.position.y, "z": root_pose.position.z}
            root_offset = self.convert_position_to_right_hand(root_position)
        
        for i, pose in enumerate(event.poses):
            bone_name = self.index_to_bone_name[i]
            frame_position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
            frame_rotation_quat = (pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w)
            
            right_hand_position = self.convert_position_to_right_hand(frame_position)
            right_hand_quat = self.convert_quaternion_to_right_hand(frame_rotation_quat)
            
            pose_info = PoseInfo()
            pose_info.position = Point(x=right_hand_position["x"], y=right_hand_position["y"], z=right_hand_position["z"])
            pose_info.orientation = Quaternion(x=right_hand_quat[0], y=right_hand_quat[1], z=right_hand_quat[2], w=right_hand_quat[3])
            pose_info_list.poses.append(pose_info)
            
            # 应用缩放因子
            scaled_position = {}
            for axis in ["x", "y", "z"]:
                scaled_position[axis] = right_hand_position[axis] * scale_factor[axis]
            
            # 创建TransformStamped并添加到TFMessage中，而不是立即发布
            self.add_transform_to_tf_message(tf_msg, time_now, bone_name, scaled_position, right_hand_quat)

            if bone_name == "Head" and self.enable_head_control:
                self.pub_head_motion_data(right_hand_quat)
            
            if bone_name == "Chest":
                self.pub_chest_pose_in_torso_frame(right_hand_position, right_hand_quat, root_offset, time_now)
        
        # 批量发布所有骨骼的TF变换（一次性发布，而不是循环中逐个发布）
        if len(tf_msg.transforms) > 0:
            self.bone_tf_pub.publish(tf_msg)
        
        # 非VR模式：更新头部控制（在TF发布后，确保可以查询到手部位置）
        if self.enable_head_control and self.head_control_manager.mode != HeadControlMode.VR_FOLLOW:
            self._update_head_control_from_hands()

    def restart_socket(self):
        print("Restarting socket connection...")
        if self.receiver_thread is not None and self.receiver_thread.is_alive():
            rospy.logerr("Refusing to restart Quest3 socket while receiver thread is alive")
            return False

        new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        new_socket.settimeout(self.DEFAULT_QUEST_DISCONNECT_TIMEOUT_SEC)
        with self.socket_lock:
            if self.sock is not None:
                rospy.logerr("Refusing to overwrite an active Quest3 socket")
                new_socket.close()
                return False
            self.sock = new_socket

        if not self.send_initial_message():
            print("Failed to restart socket connection.")
            with self.socket_lock:
                failed_socket = self.sock
                self.sock = None
            if failed_socket is not None:
                failed_socket.close()
            return False
        print("Socket connection restarted successfully.")
        # send_initial_message() 成功时已设置 /quest3/connected=True
        return True

    def _periodic_robot_info_broadcaster(self):
        """Periodically broadcasts robot info while waiting for Quest3."""
        rospy.loginfo("Starting periodic robot info broadcaster thread (kuavo, 0, ports 11050-11060).")
        robot_version = int(rospy.get_param('/robot_version', 45))
        while not self.exit_listen_thread_for_quest3_broadcast and not rospy.is_shutdown():
            if not self.broadcast_ips:
                rospy.logwarn_throttle(10, "_periodic_robot_info_broadcaster: broadcast_ips is empty. Robot info will not be sent until IPs are updated.")

            self.send_robot_info_on_broadcast_ips("kuavo", robot_version, 11050, 11060)
            # Sleep for 1 second, but check the exit condition more frequently
            # to allow faster shutdown if needed.
            for _ in range(10): # Check every 0.1 seconds
                if self.exit_listen_thread_for_quest3_broadcast or rospy.is_shutdown():
                    break
                time.sleep(0.1)
        rospy.loginfo("Stopping periodic robot info broadcaster thread.")

    def broadcast_robot_info_and_wait_for_quest3(self):
        """Broadcasts robot information and waits for a Quest3 device to connect."""
        start_port = 11000
        end_port = 11010
        threads = []

        # Start the periodic robot info broadcaster thread
        periodic_broadcaster_thread = threading.Thread(target=self._periodic_robot_info_broadcaster)
        periodic_broadcaster_thread.daemon = False # Ensure it completes before program exit if main threads finish
        periodic_broadcaster_thread.start()
 
        for port in range(start_port, end_port + 1):
            thread = threading.Thread(target=self.listen_for_quest3_broadcasts, args=(port,))
            thread.daemon = False  # Set as non-daemon thread to wait for threads to finish
            thread.start()
            threads.append(thread)

        import os

        if self.listening_udp_ports_cnt == 0:
            print("\033[91m" + "carlos_ All UDP broadcast ports are occupied. Please check using the command 'lsof -i :11000-11010' to see the process which occupy the ports." + "\033[0m")
            os._exit(1)

        for thread in threads:
            thread.join()

        # Wait for the periodic broadcaster thread to finish as well
        periodic_broadcaster_thread.join()

        if self.server_address and self.server_address[0]:
            print(f"\033[92mQuest3 device found at IP: {self.server_address[0]}\033[0m")
        print("\033[92m" + "Received Quest3 Broadcast, starting to connect." + "\033[0m")

    def listen_for_quest3_broadcasts(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', port))  # Listen on all interfaces
        except OSError as e:
            pass
            return
        sock.settimeout(1)  # Set timeout to 1 second
        self.listening_udp_ports_cnt += 1

        while not self.exit_listen_thread_for_quest3_broadcast:
            try:
                data, addr = sock.recvfrom(1024)
                self.exit_listen_thread_for_quest3_broadcast = True
                print(f"carlos_ Received message from Quest3: {data.decode()} from {addr[0]} on port {port} - Setting up socket connection")
                self.setup_socket(addr[0], 10019)
                break
            except socket.timeout:
                continue

def get_local_broadcast_ips():
    """
    Gets a list of local IPv4 broadcast IP addresses for all active interfaces.
    Requires the 'netifaces' library to be imported.
    """
    broadcast_ips = []
    excluded_prefixes = ("docker", "br-", "veth")
    try:
        for iface_name in netifaces.interfaces():
            if any(iface_name.startswith(prefix) for prefix in excluded_prefixes):
                continue
            if_addresses = netifaces.ifaddresses(iface_name)
            if netifaces.AF_INET in if_addresses:
                for link_addr in if_addresses[netifaces.AF_INET]:
                    # Ensure 'broadcast' key exists and its value is not None or empty
                    if 'broadcast' in link_addr and link_addr['broadcast']:
                        broadcast_ips.append(link_addr['broadcast'])
        # Return unique broadcast IPs, sorted for consistency
        return sorted(list(set(broadcast_ips)))
    except Exception as e: # Catch any error during netifaces operations
        rospy.logerr(f"Error getting broadcast IPs using 'netifaces': {e}. Ensure 'netifaces' is installed and network interfaces are configured correctly.")
        return []

if __name__ == "__main__":
    # Create a Quest3BoneFramePublisher instance
    publisher = Quest3BoneFramePublisher()
    #######################################################
    def get_package_path(package_name):
        try:
            import rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            return package_path
        except rospkg.ResourceNotFound:
            return None

    # Start the robot state server
    if rospy.has_param("/end_effector_type"):
        ee_type = rospy.get_param("/end_effector_type")
        print(f"\033[92mend_effector_type from rosparm: {ee_type}\033[0m")    
    else:
        kuavo_assests_path = get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
        import json
        with open(config_file, 'r') as f:
            config = json.load(f)
            ee_type = config.get("EndEffectorType", ["qiangnao", "qiangnao"])[0]
        print("\033[91mend_effector_type not found in rosparm, using from kuavo.json\033[0m")
    # RobotStateServer to Vr App
    subscribe_sensor_data = True # 订阅sensors_data_raw
    robot_state_server = RobotStateServer(ee_type=ee_type, udp_port=15170, publish_rate=20, subscribe_sensor_data=subscribe_sensor_data)
    if not robot_state_server.start():
        print("\033[91mRobotStateServer 启动失败\033[0m")
        sys.exit(1)
    
    # 设置 robot_state_server 引用到 publisher，用于处理 hand wrench 请求
    publisher.set_robot_state_server(robot_state_server)
    #######################################################

    broadcast_ips = get_local_broadcast_ips()
    print(f"Local broadcast IPs: {broadcast_ips}")

    publisher.update_broadcast_ips(broadcast_ips)
    if len(sys.argv) < 2 or "." not in sys.argv[1]:
        print("IP not specified. Waiting for Quest3 to connect. Please ensure Quest3 and the robot are on the same LAN and the router has broadcast mode enabled.\n未指定IP。正在等待 Quest3 主动连接。请确保 Quest3 和机器人在同一个局域网下，并且路由器已开启广播模式。")
        publisher.broadcast_robot_info_and_wait_for_quest3()
    else:
        try:

            if ':' in sys.argv[1]:
                server_address, port = sys.argv[1].split(':')
                port = int(port)
            else:
                server_address = sys.argv[1]
                port = 10019

        except ValueError:
            print("Argument must be in the format <server_address[:port]> and port must be an integer")
            sys.exit(1)


        publisher.setup_socket(server_address, port)

    if publisher.send_initial_message():
        publisher.run()
    else:
        print("Failed to establish initial connection.")

    # Close the socket
    robot_state_server.stop()    
