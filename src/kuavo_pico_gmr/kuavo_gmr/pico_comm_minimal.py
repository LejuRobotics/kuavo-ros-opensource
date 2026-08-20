#!/usr/bin/env python3
"""
最小化 PICO 通信模块

从 pico-body-tracking-server 提取的核心功能：
- UDP 通信
- Protobuf 解析
- 骨骼数据发布
- IP 广播（让 PICO 设备发现服务器）

依赖:
- rospy
- kuavo_msgs
- protobuf (需要将 body_tracking_extended_pb2.py 放在同目录)
- netifaces (用于获取网络接口信息)
"""

import socket
import threading
import queue
import numpy as np
import json
import time
import os
import rospy
from geometry_msgs.msg import Point, Quaternion
from kuavo_msgs.msg import picoPoseInfo, picoPoseInfoList, robotBodyMatrices, JoySticks
from kuavo_msgs.srv import switchToNextController
from std_srvs.srv import SetBool, Trigger
import tf
from typing import Optional, Tuple, List
import body_tracking_extended_pb2 as proto
from pico_diagnostic_runtime import (
    JsonlDiagnosticLogger,
    PicoDiagnosticsRuntime,
    ReceivedDatagram,
    monotonic_ns,
)
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion,
    euler_from_matrix,
    quaternion_from_euler,
    translation_matrix
)
try:
    import netifaces
    HAS_NETIFACES = True
except ImportError:
    HAS_NETIFACES = False
    rospy.logwarn_once("netifaces not installed, IP broadcast disabled. Install: pip install netifaces")

# 骨骼名称定义
BODY_TRACKER_ROLES = [
    "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", "RIGHT_KNEE",
    "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3", "LEFT_FOOT", "RIGHT_FOOT",
    "NECK", "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND",
    "FOOT_STATE"
]

INDEX_TO_BONE_NAME = {index: name for index, name in enumerate(BODY_TRACKER_ROLES)}
BONE_NAME_TO_INDEX = {name: index for index, name in enumerate(BODY_TRACKER_ROLES)}


LEFT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND"
]]

RIGHT_ARM_IDXS = [BODY_TRACKER_ROLES.index(name) for name in [
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND"
]]

ALIGN_TO_ROBOT_URDF_JOINTS = [
    "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"
]

CONTROL_MODES = ["WholeBody", "UpperBody", "LowerBody"]


def get_localip_and_broadcast_ips() -> List[Tuple[str, str]]:
    """
    获取本地 IPv4 地址和对应的广播地址（有线和无线接口）
    
    Returns:
        List of tuples [(local_ip, broadcast_ip), ...]
    """
    if not HAS_NETIFACES:
        rospy.logwarn_once("netifaces not available, returning empty IP list")
        return []
    
    ip_pairs = []
    excluded_prefixes = ("docker", "br-", "veth", "lo")  # 排除虚拟和回环接口
    wired_prefixes = ("eth", "en", "em")  # 有线网卡前缀
    wireless_prefixes = ("wl", "wlan")  # 无线网卡前缀
    
    try:
        for iface_name in netifaces.interfaces():
            # 跳过排除的接口
            if any(iface_name.startswith(prefix) for prefix in excluded_prefixes):
                continue
            
            # 只包含有线和无线接口
            if not any(iface_name.startswith(prefix) for prefix in wired_prefixes + wireless_prefixes):
                continue
            
            if_addresses = netifaces.ifaddresses(iface_name)
            if netifaces.AF_INET in if_addresses:
                for link_addr in if_addresses[netifaces.AF_INET]:
                    local_ip = link_addr.get('addr')
                    broadcast_ip = link_addr.get('broadcast')
                    
                    # 确保本地 IP 和广播 IP 都存在且有效
                    if local_ip and broadcast_ip:
                        # 跳过回环地址 127.x.x.x
                        if not local_ip.startswith('127.') and not broadcast_ip.startswith('127.'):
                            ip_pairs.append((local_ip, broadcast_ip))
        
        # 返回唯一的 IP 对，排序以保持一致性
        return sorted(list(set(ip_pairs)))
        
    except Exception as e:
        rospy.logerr(f"Error getting local and broadcast IPs: {e}")
        return []


class RobotInfoBroadcaster:
    """
    机器人信息广播器
    
    功能：
    - 定期广播机器人的 IP 地址和基本信息
    - 让 PICO 设备能够自动发现服务器
    - 支持暂停/恢复广播
    """
    
    def __init__(self, robot_name: str = "KUAVO", broadcast_port: int = 8443):
        """
        初始化广播器
        
        Args:
            robot_name: 机器人名称
            broadcast_port: 广播端口
        """
        self.robot_name = os.getenv("ROBOT_NAME", robot_name)
        self.broadcast_port = broadcast_port
        self.ip_pairs = get_localip_and_broadcast_ips()
        self.is_paused = False
        self.running = True
        self.broadcast_condition = threading.Condition()
        
        # 机器人信息
        self.robot_info = {
            "data": {
                "robot_name": self.robot_name,
                "robot_ip": '',
                "eef_type": rospy.get_param('end_effector_type', None),
            }
        }
        
        # 打印广播信息
        rospy.loginfo("="*50)
        rospy.loginfo("Robot Info Broadcaster Initialized")
        rospy.loginfo("="*50)
        for local_ip, broadcast_ip in self.ip_pairs:
            robot_info = self.robot_info.copy()
            robot_info["data"]["robot_ip"] = local_ip
            message = json.dumps(robot_info).encode("utf-8")
            rospy.loginfo(f"Will broadcast to {broadcast_ip}:{self.broadcast_port}")
            rospy.loginfo(f"Message: {message.decode('utf-8')}")
        rospy.loginfo("="*50)
        
    
    def pause_broadcast(self):
        """暂停广播"""
        with self.broadcast_condition:
            self.is_paused = True
            self.broadcast_condition.notify_all()
        rospy.loginfo("Robot info broadcast paused")
    
    def resume_broadcast(self):
        """恢复广播"""
        with self.broadcast_condition:
            self.is_paused = False
            self.broadcast_condition.notify_all()
        rospy.loginfo("Robot info broadcast resumed")
    
    def quit(self):
        """停止广播"""
        self.running = False
        with self.broadcast_condition:
            self.broadcast_condition.notify_all()
    
    def broadcast(self):
        """广播线程主函数"""
        if not self.ip_pairs:
            rospy.logwarn("No valid IP addresses found, skipping broadcast")
            return
        
        # 创建 UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            for local_ip, broadcast_ip in self.ip_pairs:
                rospy.loginfo(f"Broadcasting from {local_ip} to {broadcast_ip}")
            
            while self.running and not rospy.is_shutdown():
                try:
                    with self.broadcast_condition:
                        # 如果暂停了，等待恢复信号
                        while self.is_paused and not rospy.is_shutdown():
                            self.broadcast_condition.wait(1.0)
                        
                        # 广播机器人信息
                        for local_ip, broadcast_ip in self.ip_pairs:
                            self.robot_info["data"]["robot_ip"] = local_ip
                            message = json.dumps(self.robot_info).encode("utf-8")
                            sock.sendto(message, (broadcast_ip, self.broadcast_port))
                    
                    time.sleep(1.0)  # 每秒广播一次
                    
                except Exception as e:
                    rospy.logerr(f"Error broadcasting robot info: {e}")
                    break
                    
        finally:
            rospy.loginfo("Robot info broadcast thread exited")
            sock.close()


class MinimalPicoReceiver:
    """
    最小化 PICO 数据接收器
    
    功能：
    1. 接收 PICO 设备通过 UDP 发送的 Protobuf 数据
    2. 解析骨骼数据（25 个骨骼点）
    3. 转换坐标系（PICO → 世界坐标系）
    4. 发布 ROS 消息
    5. 广播 TF 变换
    """
    
    def __init__(self, host: str = '0.0.0.0', port: int = 12345, 
                 publish_tf: bool = True, enable_ip_broadcast: bool = True,
                 robot_name: str = "KUAVO", broadcast_port: int = 8443,
                 enable_diagnostics: bool = True,
                 diagnostic_log_enable: bool = False,
                 diagnostic_log_dir: str = "~/.ros/pico_diagnostics",
                 diagnostic_log_max_file_mb: float = 300.0,
                 diagnostic_log_compress: bool = True,
                 diagnostic_udp_reply: bool = True,
                 enable_time_sync: bool = True,
                 diagnostic_publish_hz: float = 1.0,
                 diagnostic_allow_legacy: bool = False,
                 diagnostic_ping_enable: bool = True,
                 diagnostic_ping_lower_ip: str = "",
                 diagnostic_ping_interval_sec: float = 5.0,
                 diagnostic_ping_timeout_sec: float = 1.0,
                 diagnostic_sync_max_valid_rtt_ms: float = 20.0,
                 diagnostic_sync_max_valid_residual_ms: float = 5.0):
        """
        初始化 PICO 接收器
        
        Args:
            host: UDP 监听地址
            port: UDP 监听端口
            publish_tf: 是否发布 TF 变换
            enable_ip_broadcast: 是否启用 IP 广播（让 PICO 设备发现服务器）
            robot_name: 机器人名称（用于广播）
            broadcast_port: 广播端口
        """
        # UDP socket
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.settimeout(1.0)
        
        # 标志
        self.publish_tf = publish_tf
        self.running = False
        
        # 数据队列（异步处理）
        self.data_queue = queue.Queue(maxsize=10)

        self.diagnostic_logger = JsonlDiagnosticLogger(
            enabled=diagnostic_log_enable,
            log_dir=diagnostic_log_dir,
            max_file_size_mb=diagnostic_log_max_file_mb,
            compress_closed_files=diagnostic_log_compress,
        )
        self.diagnostics = PicoDiagnosticsRuntime(
            sock=self.socket,
            logger=self.diagnostic_logger,
            enabled=enable_diagnostics,
            udp_reply_enabled=diagnostic_udp_reply,
            sync_enabled=enable_time_sync,
            diagnostic_publish_hz=diagnostic_publish_hz,
            allow_legacy_vrdata=diagnostic_allow_legacy,
            ping_enabled=diagnostic_ping_enable,
            ping_lower_ip=diagnostic_ping_lower_ip,
            ping_interval_sec=diagnostic_ping_interval_sec,
            ping_timeout_sec=diagnostic_ping_timeout_sec,
            sync_max_valid_rtt_ms=diagnostic_sync_max_valid_rtt_ms,
            sync_max_valid_residual_ms=diagnostic_sync_max_valid_residual_ms,
        )
        
        # ROS 发布器
        self._init_publishers()
        
        # TF 广播器
        if self.publish_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()
        
        # IP 广播器（让 PICO 设备自动发现服务器）
        self.broadcaster = None
        self.broadcast_thread = None
        self.cached_matrices = np.tile(np.eye(4), (len(BODY_TRACKER_ROLES), 1, 1))
        self.bone_names = BODY_TRACKER_ROLES
        self.left_arm_idxs = LEFT_ARM_IDXS
        self.right_arm_idxs = RIGHT_ARM_IDXS
        self.bone_name_to_index = BONE_NAME_TO_INDEX
        self.index_to_bone_name = INDEX_TO_BONE_NAME

        if enable_ip_broadcast:
            self.broadcaster = RobotInfoBroadcaster(
                robot_name=robot_name,
                broadcast_port=broadcast_port
            )
            self.broadcast_thread = threading.Thread(
                target=self.broadcaster.broadcast,
                daemon=True
            )
            self.broadcast_thread.start()
            rospy.loginfo("IP broadcast enabled")
        else:
            rospy.loginfo("IP broadcast disabled")
        
        rospy.loginfo(f"MinimalPicoReceiver initialized on {host}:{port}")
        if diagnostic_log_enable:
            rospy.loginfo(f"PICO diagnostic log: {self.diagnostic_logger.path}")
        
        # ========== PICO 手柄按键处理（VMP推流中断/恢复） ==========
        self._init_joy_handler()
    
    def _init_publishers(self):
        """初始化 ROS 发布器"""
        # 世界坐标系骨骼位姿
        self.world_poses_pub = rospy.Publisher(
            '/pico/world_bone_poses',
            picoPoseInfoList,
            queue_size=10
        )
        
        # 原始骨骼位姿（PICO 坐标系）
        self.raw_poses_pub = rospy.Publisher(
            '/pico/raw_bone_poses',
            picoPoseInfoList,
            queue_size=10
        )
        
        # PICO 手柄按键数据
        self.pico_joy_pub = rospy.Publisher(
            '/pico/joy',
            JoySticks,
            queue_size=10
        )
        
        # 世界坐标系下的骨骼变换矩阵（供 pico_head_control_node 等下游节点使用）
        self.body_matrices_pub = rospy.Publisher(
            '/robot_body_matrices',
            robotBodyMatrices,
            queue_size=10
        )

    def _init_joy_handler(self):
        """初始化 PICO 手柄按键处理（轻量级，用于 VMP 推流中断/恢复 + GMR 校准）"""
        # Trigger 阈值（与 pico-body-tracking-server 的 JoySticksHandler 一致）
        self._trigger_threshold = 0.5
        
        # 按钮前一帧状态（用于上升沿检测）
        self._prev_btn = {'X': False, 'Y': False, 'A': False, 'B': False}
        
        # VMP 推流控制服务客户端（惰性初始化）
        self._vmp_stream_control_client = None
        # GMR 校准服务客户端（惰性初始化）
        self._gmr_calibrate_client = None          # RT+B → 半身校准
        self._gmr_calibrate_whole_client = None     # LT+B → 全身校准
        # RL 控制器切换服务客户端（惰性初始化）
        self._switch_next_controller_client = None
        self._switch_previous_controller_client = None
        
        rospy.loginfo(
            "PICO joy handler initialized (RT+Y=pause, RT+X=resume VMP stream, "
            "RT+B=半身校准, LT+B=全身校准, RG+A=下一个控制器, RG+B=上一个控制器)"
        )
    
    def _process_controller_data(self, controller_data):
        """
        解析 protobuf ControllerData 并发布 /pico/joy，
        同时处理 VMP 推流控制按键组合（RT+Y=暂停, RT+X=恢复）和 GMR 校准（RT+B=半身, LT+B=全身）
        """
        try:
            if not controller_data.HasField("controllers"):
                return
            
            states = controller_data.controllers
            
            # 解析左右手柄状态
            left = self._parse_controller_state(states.left) if states.HasField("left") else \
                   {"buttons": [False] * 4, "axes": [0.0] * 4}
            right = self._parse_controller_state(states.right) if states.HasField("right") else \
                    {"buttons": [False] * 4, "axes": [0.0] * 4}
            
            # 构造 JoySticks 消息
            joy_msg = JoySticks()
            joy_msg.left_x = left["axes"][0]
            joy_msg.left_y = left["axes"][1]
            joy_msg.left_grip = left["axes"][2]
            joy_msg.left_trigger = left["axes"][3]
            joy_msg.left_first_button_pressed = left["buttons"][0]   # X
            joy_msg.left_second_button_pressed = left["buttons"][1]  # Y
            joy_msg.left_first_button_touched = False
            joy_msg.left_second_button_touched = False
            
            joy_msg.right_x = right["axes"][0]
            joy_msg.right_y = right["axes"][1]
            joy_msg.right_grip = right["axes"][2]
            joy_msg.right_trigger = right["axes"][3]
            joy_msg.right_first_button_pressed = right["buttons"][0]   # A
            joy_msg.right_second_button_pressed = right["buttons"][1]  # B
            joy_msg.right_first_button_touched = False
            joy_msg.right_second_button_touched = False
            
            # 发布 /pico/joy
            if not rospy.is_shutdown():
                self.pico_joy_pub.publish(joy_msg)
            
            # 处理 VMP 推流 / GMR 校准 / 控制器切换按键组合
            right_grip_pressed = right["buttons"][2] or right["axes"][2] >= self._trigger_threshold
            self._handle_vmp_stream_buttons(joy_msg, right_grip_pressed=right_grip_pressed)
            
        except Exception as e:
            rospy.logerr(f"Error processing controller data: {e}")
    
    @staticmethod
    def _parse_controller_state(state) -> dict:
        """解析单个手柄的 protobuf ControllerState"""
        thumbstick = state.thumbstick
        thumb_x = float(thumbstick[0]) if len(thumbstick) > 0 else 0.0
        thumb_y = float(thumbstick[1]) if len(thumbstick) > 1 else 0.0
        return {
            "buttons": [
                bool(state.primary_button),    # X/A
                bool(state.secondary_button),  # Y/B
                bool(state.grip_button),       # Grip
                bool(state.trigger_button)     # Trigger
            ],
            "axes": [
                thumb_x,
                thumb_y,
                float(state.grip_value),
                float(state.trigger_value)
            ]
        }
    
    def _handle_vmp_stream_buttons(self, joy_msg: JoySticks, right_grip_pressed: bool = False):
        """
        处理 VMP 推流控制、GMR 校准与 RL 控制器切换的按键组合：
        - RT + Y（上升沿）→ 暂停推流
        - RT + X（上升沿）→ 恢复推流
        - RT + B（上升沿）→ GMR 半身校准 (upper_body)
        - LT + B（上升沿）→ GMR 全身校准 (whole_body)
        - RG + A（上升沿）→ 下一个 RL 控制器
        - RG + B（上升沿）→ 上一个 RL 控制器
        """
        # 当前按钮状态
        curr_x = bool(joy_msg.left_first_button_pressed)   # X 按钮
        curr_y = bool(joy_msg.left_second_button_pressed)  # Y 按钮
        curr_a = bool(joy_msg.right_first_button_pressed)  # A 按钮
        curr_b = bool(joy_msg.right_second_button_pressed) # B 按钮
        
        # RT（右扳机）/ LT（左扳机）是否按下
        rt_pressed = joy_msg.right_trigger >= self._trigger_threshold
        lt_pressed = joy_msg.left_trigger >= self._trigger_threshold
        
        # 检测上升沿（当前按下且前一帧未按下）
        y_rising = curr_y and not self._prev_btn['Y']
        x_rising = curr_x and not self._prev_btn['X']
        a_rising = curr_a and not self._prev_btn['A']
        b_rising = curr_b and not self._prev_btn['B']
        
        # RT + Y 上升沿 → 暂停推流
        if rt_pressed and y_rising:
            self._call_vmp_stream_control(True)
        
        # RT + X 上升沿 → 恢复推流
        if rt_pressed and x_rising:
            self._call_vmp_stream_control(False)
        
        # RT + B 上升沿 → GMR 半身校准
        if rt_pressed and b_rising:
            self._call_gmr_calibrate("upper_body")
        
        # LT + B 上升沿 → GMR 全身校准
        if lt_pressed and b_rising:
            self._call_gmr_calibrate("whole_body")
        
        # RG（右手握把边键）+ A/B 上升沿 → RL 控制器循环切换（B 与 RT+B 半身校准时互斥）
        if right_grip_pressed and a_rising:
            self._call_switch_controller("next")
        elif right_grip_pressed and b_rising and not rt_pressed:
            self._call_switch_controller("previous")
        
        # 更新前一帧状态
        self._prev_btn['X'] = curr_x
        self._prev_btn['Y'] = curr_y
        self._prev_btn['A'] = curr_a
        self._prev_btn['B'] = curr_b
    
    def _call_vmp_stream_control(self, pause: bool):
        """调用 /vmp/pico_stream_control 服务"""
        action = "暂停" if pause else "恢复"
        combo = "RT+Y" if pause else "RT+X"
        try:
            if self._vmp_stream_control_client is None:
                service_name = "/vmp/pico_stream_control"
                rospy.wait_for_service(service_name, timeout=1.0)
                self._vmp_stream_control_client = rospy.ServiceProxy(service_name, SetBool)
            
            resp = self._vmp_stream_control_client(pause)
            if resp.success:
                rospy.loginfo(f"\033[93mVMP推流已{action} ({combo})\033[0m")
            else:
                rospy.logwarn(f"VMP推流{action}失败: {resp.message}")
        except rospy.ROSException:
            rospy.logwarn_once(f"VMP推流控制服务 /vmp/pico_stream_control 不可用（VMPController可能未启动）")
            self._vmp_stream_control_client = None
        except rospy.ServiceException as e:
            rospy.logerr(f"VMP推流控制服务调用失败: {e}")
            self._vmp_stream_control_client = None

    def _call_switch_controller(self, direction: str):
        """调用 RL 控制器切换服务（RG+A=下一个，RG+B=上一个）"""
        if direction == "next":
            service_name = "/humanoid_controller/switch_to_next_controller"
            combo_label = "RG+A"
            client_attr = "_switch_next_controller_client"
        elif direction == "previous":
            service_name = "/humanoid_controller/switch_to_previous_controller"
            combo_label = "RG+B"
            client_attr = "_switch_previous_controller_client"
        else:
            return

        try:
            client = getattr(self, client_attr)
            if client is None:
                rospy.wait_for_service(service_name, timeout=1.0)
                client = rospy.ServiceProxy(service_name, switchToNextController)
                setattr(self, client_attr, client)

            resp = client()
            if resp.success:
                rospy.loginfo(
                    "\033[93m控制器切换成功 (%s): %s -> %s\033[0m",
                    combo_label,
                    resp.current_controller,
                    resp.next_controller,
                )
            else:
                rospy.logwarn("控制器切换失败 (%s): %s", combo_label, resp.message)
        except rospy.ROSException:
            rospy.logwarn_once("控制器切换服务不可用: %s", service_name)
            setattr(self, client_attr, None)
        except rospy.ServiceException as e:
            rospy.logerr("控制器切换服务调用失败 (%s): %s", combo_label, e)
            setattr(self, client_attr, None)

    def _call_gmr_calibrate(self, cali_option: str = "upper_body"):
        """调用 GMR 校准服务
        
        Args:
            cali_option: "upper_body" (RT+B) 或 "whole_body" (LT+B)
        
        服务立即返回（校准在 retarget 节点的 main loop 中异步执行），
        最终结果通过 retarget 节点的日志输出。
        """
        if cali_option == "whole_body":
            service_name = '/pico/gmr_calibrate_whole'
            combo_label = "LT+B"
            label = "全身校准"
            client_attr = '_gmr_calibrate_whole_client'
        else:
            service_name = '/pico/gmr_calibrate'
            combo_label = "RT+B"
            label = "半身校准"
            client_attr = '_gmr_calibrate_client'

        try:
            client = getattr(self, client_attr)
            if client is None:
                rospy.wait_for_service(service_name, timeout=1.0)
                client = rospy.ServiceProxy(service_name, Trigger)
                setattr(self, client_attr, client)
            
            resp = client()
            if resp.success:
                rospy.loginfo(f"\033[93mGMR {label}已触发 ({combo_label}), 等待 retarget 节点完成...\033[0m")
            else:
                rospy.logwarn(f"GMR {label}请求被拒绝: {resp.message}")
        except rospy.ROSException:
            rospy.logwarn_once(f"GMR 校准服务不可用: {service_name}")
            setattr(self, client_attr, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"调用 GMR 校准服务失败: {e}")
            setattr(self, client_attr, None)

    def _proto_timestamp_to_ros_time(self, proto_timestamp):
        """
        将 protobuf Timestamp 转换为 ROS Time
        
        Args:
            proto_timestamp: google.protobuf.timestamp_pb2.Timestamp 对象
            
        Returns:
            rospy.Time: ROS 时间戳对象
        """
        if proto_timestamp is None:
            return rospy.Time.now()
        
        try:
            # protobuf Timestamp 有 seconds 和 nanos 字段
            # ROS Time 需要 secs 和 nsecs
            seconds = proto_timestamp.seconds
            nanos = proto_timestamp.nanos
            
            # 转换为 ROS Time (secs, nsecs)
            return rospy.Time(secs=seconds, nsecs=nanos)
        except Exception as e:
            rospy.logwarn(f"Failed to convert protobuf timestamp to ROS time: {e}, using current time")
            return rospy.Time.now()
    
    def get_robot_urdf_matrix_from_proto(self, proto_data):
        """Get the robot URDF matrix from FullBodyData protobuf bytes."""
        matrices = self.parse_full_body_to_matrix(proto_data)
        if matrices is None or matrices.size == 0:
            rospy.logwarn("Received invalid matrices data")
            return
        
        # 从 protobuf 消息中提取 Header 中的时间戳
        try:
            message = proto.VRData()
            message.ParseFromString(proto_data)
            
            # 检查是否有 header 和时间戳
            # 在 proto3 中，非 optional 字段可能没有 HasField 方法，直接检查是否为 None
            if message.header is not None and message.header.timestamp is not None:
                # 检查 timestamp 是否有有效值（seconds 不为 0，表示时间戳已设置）
                # 注意：seconds=0 表示 1970-01-01，这在大多数情况下不太可能是实际数据时间
                if message.header.timestamp.seconds > 0:
                    current_time = self._proto_timestamp_to_ros_time(message.header.timestamp)
                else:
                    rospy.logwarn_once("Timestamp in protobuf header appears invalid (seconds=0), using current time")
                    current_time = rospy.Time.now()
            else:
                rospy.logwarn_once("No header or timestamp in protobuf message, using current time")
                current_time = rospy.Time.now()
        except Exception as e:
            rospy.logwarn(f"Failed to extract timestamp from protobuf: {e}, using current time")
            current_time = rospy.Time.now()
        
        # 发布原始矩阵数据（Pico 坐标系，未经转换）
        self.publish_raw_matrices(matrices.copy(), current_time)
        
        return matrices, current_time
    
    @staticmethod
    def _message_payload_fields(message) -> List[str]:
        fields = []
        for field_name in [
            "full_body",
            "upper_body",
            "controller",
            "robot_data",
            "vr_command",
            "delayed_diagnosis_command",
        ]:
            try:
                if message.HasField(field_name):
                    fields.append(field_name)
            except ValueError:
                pass
        return fields

    def _parse_protobuf(self, datagram) -> Optional[picoPoseInfoList]:
        """
        解析 Protobuf 数据
        
        流程：
        1. 解析 Protobuf → 矩阵
        2. 坐标系转换（PICO → 世界）
        3. 矩阵 → ROS 消息
        4. 解析手柄数据 → /pico/joy + VMP推流控制
        
        Args:
            datagram: UDP 接收的二进制数据或 ReceivedDatagram
        
        Returns:
            picoPoseInfoList 或 None
        """
        payload_fields = []
        handled_controller = False
        handled_full_body = False
        try:
            if isinstance(datagram, ReceivedDatagram):
                data = datagram.payload
            else:
                data = datagram
                datagram = ReceivedDatagram(
                    raw_data=data,
                    payload=data,
                    addr=("", 0),
                    recv_monotonic_ns=monotonic_ns(),
                    legacy=True,
                )

            if datagram.process_start_monotonic_ns <= 0:
                datagram.process_start_monotonic_ns = monotonic_ns()

            # 解析 protobuf
            message = proto.VRData()
            message.ParseFromString(data)
            datagram.protobuf_parse_done_monotonic_ns = monotonic_ns()
            payload_fields = self._message_payload_fields(message)
            self.diagnostics.record_vrdata_fields(datagram, payload_fields, parse_ok=True)
            
            # 处理手柄数据（不依赖 full_body 是否存在）
            if message.HasField('controller'):
                self._process_controller_data(message.controller)
                handled_controller = True
            
            # 检查是否有全身数据
            if not message.HasField('full_body'):
                self.diagnostics.record_processing_timing(
                    datagram,
                    payload_fields,
                    process_done_monotonic_ns=monotonic_ns(),
                    handled_controller=handled_controller,
                    handled_full_body=handled_full_body,
                    process_ok=True,
                )
                return None

            matrices, current_time = self.get_robot_urdf_matrix_from_proto(data)
            handled_full_body = True
            process_done_ns = monotonic_ns()
            self.diagnostics.record_processing_timing(
                datagram,
                payload_fields,
                process_done_monotonic_ns=process_done_ns,
                handled_controller=handled_controller,
                handled_full_body=handled_full_body,
                process_ok=True,
            )
            return matrices, current_time
            
        except Exception as e:
            rospy.logerr(f"Error parsing protobuf: {e}")
            if isinstance(datagram, ReceivedDatagram):
                if datagram.process_start_monotonic_ns <= 0:
                    datagram.process_start_monotonic_ns = monotonic_ns()
                self.diagnostics.record_parse_failure(datagram, str(e))
                self.diagnostics.record_processing_timing(
                    datagram,
                    payload_fields,
                    process_done_monotonic_ns=monotonic_ns(),
                    handled_controller=handled_controller,
                    handled_full_body=handled_full_body,
                    process_ok=False,
                    error=str(e),
                )
            import traceback
            traceback.print_exc()
            return None
    
    def _publish_tf(self, pose_list: picoPoseInfoList):
        """
        发布 TF 变换
        
        Args:
            pose_list: 骨骼位姿列表
        """
        if not self.publish_tf:
            return
        
        try:
            current_time = rospy.Time.now()
            
            for i, pose in enumerate(pose_list.poses):
                if i >= len(BODY_TRACKER_ROLES):
                    break
                
                bone_name = BODY_TRACKER_ROLES[i]
                
                # 提取位置
                translation = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ]
                
                # 提取旋转（四元数 [x, y, z, w]）
                rotation = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ]
                
                # 发布 TF
                self.tf_broadcaster.sendTransform(
                    translation,
                    rotation,
                    current_time,
                    f"pico_{bone_name}",
                    "world"
                )
                
        except Exception as e:
            rospy.logerr(f"Error publishing TF: {e}")

    def parse_full_body_to_matrix(self, data_bytes: bytes):
        """Parse protobuf FullBodyData bytes into [N, 4, 4] matrices."""
        try:
            full_body_msg = proto.VRData()
            full_body_msg.ParseFromString(data_bytes)
            
            poses = full_body_msg.full_body.full_body
            if not poses:
                rospy.logwarn("No poses in full_body")
                return None
            
            for i, pose in enumerate(poses):
                translation = np.array([pose.pos_x, pose.pos_y, pose.pos_z])
                rotation = np.array([pose.rot_qx, pose.rot_qy, pose.rot_qz, pose.rot_qw])
                
                self.cached_matrices[i][:3, :3] = quaternion_matrix(rotation)[:3, :3]
                self.cached_matrices[i][:3, 3] = translation
            
            return self.cached_matrices
        except Exception as e:
            rospy.logerr(f"Error parsing FullBodyData protobuf: {e}")
            return None

    def publish_raw_matrices(self, raw_matrices, current_time):
        """
        处理 Pico 原始矩阵数据：
        1. 发布世界坐标系下的位姿 (/pico/world_bone_poses)
        2. 发布世界坐标系下的变换矩阵 (/robot_body_matrices)，供 pico_head_control_node 等下游节点使用
        
        Args:
            raw_matrices: numpy array of shape [N, 4, 4], Pico 坐标系下的原始变换矩阵
            current_time: rospy.Time, 当前时间戳
        """
        try:
            # 坐标轴变换：Pico 坐标系 → ROS 世界坐标系
            world_matrices = self.transform_matrix_to_ros(raw_matrices.copy())
            
            # 发布 robotBodyMatrices（世界坐标系），供 pico_head_control_node 使用
            mat_msg = robotBodyMatrices()
            mat_msg.header.stamp = rospy.Time.now()
            mat_msg.header.frame_id = "world"
            mat_msg.timestamp = current_time
            mat_msg.num_matrices = len(world_matrices)
            mat_msg.body_parts = BODY_TRACKER_ROLES.copy()
            mat_msg.data_source = "pico_gmr"
            
            matrices_flat = []
            for matrix in world_matrices:
                matrices_flat.extend(matrix.flatten().tolist())
            mat_msg.matrices_data = matrices_flat
            
            self.body_matrices_pub.publish(mat_msg)
            
            # 发布世界坐标系下的位姿数据
            self.publish_world_poses(raw_matrices, current_time)
        except Exception as e:
            rospy.logerr(f"Error publishing raw matrices: {e}")


    @staticmethod
    def homogeneous_matrix_roll(angle_degrees: float) -> np.ndarray:
        """Generate homogeneous transformation matrix for rotation around X axis."""
        angle_radians = np.radians(angle_degrees)
        c = np.cos(angle_radians)
        s = np.sin(angle_radians)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])

    @staticmethod
    def reverse_y_z_rotation_vectorized_reflection(ros_matrices):
        """Vectorized reflection matrix approach - fastest method for reversing y and z rotations."""
        result = ros_matrices.copy()
        
        # Reflection matrix for y and z rotation reversal
        # This matrix when multiplied flips the signs of y and z rotation components
        reflection_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])
        
        # Vectorized operation: apply reflection to all matrices at once
        rotation_matrices = ros_matrices[:, :3, :3]
        
        # S * R * S^T for all matrices at once using einsum for maximum performance
        # This is equivalent to: reflection_matrix @ rotation_matrices @ reflection_matrix.T
        result[:, :3, :3] = np.einsum('ij,njk,kl->nil', reflection_matrix, rotation_matrices, reflection_matrix.T)
        
        return result

    def transform_matrix_to_ros(self, matrix):
        """Transform Unity coordinate system matrices to robot URDF coordinate system."""
        # Unity to ROS transformation
        T_unity_2_ros = np.array([[0, 0, -1, 0],
                                 [-1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 0, 1]])
        
        T_ros_2_robot_urdf = np.array([[0, -1, 0, 0],
                                      [0, 0, 1, 0],
                                      [-1, 0, 0, 0],
                                      [0, 0, 0, 1]])
        
        # Arm correction matrices
        T_LEFT_ARM_CORRECTION = self.homogeneous_matrix_roll(90)
        T_RIGHT_ARM_CORRECTION = self.homogeneous_matrix_roll(-90)
        
        ros_matrices = np.matmul(T_unity_2_ros, matrix)
        ros_matrices = np.matmul(ros_matrices, T_ros_2_robot_urdf)

        # Reverse x position
        ros_matrices[:,0,3] = -ros_matrices[:,0,3]

        # start_time = time.time()
        # Reverse y and z rotation using vectorized reflection method (76x faster than original)
        ros_matrices = self.reverse_y_z_rotation_vectorized_reflection(ros_matrices)
        # ros_matrices = self.reverse_y_z_rotation_original(ros_matrices)
        # end_time = time.time()
        # print(f"Time taken: {end_time - start_time} seconds")

        # Align to robot urdf
        for idx in self.left_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_LEFT_ARM_CORRECTION
        for idx in self.right_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_RIGHT_ARM_CORRECTION
        return ros_matrices

    def publish_world_poses(self, raw_matrices, current_time):
        """
        发布世界坐标系下的位姿数据（仅做坐标轴变换）
        
        Args:
            raw_matrices: numpy array of shape [N, 4, 4], Pico 坐标系下的原始变换矩阵
            current_time: rospy.Time, 当前时间戳
        """
        try:
            # 坐标轴变换：Pico 坐标系 → ROS 世界坐标系
            world_matrices = self.transform_matrix_to_ros(raw_matrices.copy())
            
            pose_info_list = picoPoseInfoList()
            pose_info_list.timestamp_ms = int(current_time.to_sec() * 1000)
            pose_info_list.is_high_confidence = True
            pose_info_list.is_hand_tracking = True
            
            for i, matrix in enumerate(world_matrices):
                try:
                    # 从世界坐标系矩阵中提取位置和旋转
                    pose_info = picoPoseInfo()
                    
                    # 提取位置 (平移向量)
                    position = matrix[:3, 3]
                    pose_info.position = Point(
                        x=float(position[0]),
                        y=float(position[1]),
                        z=float(position[2])
                    )
                    
                    # 提取旋转矩阵并转换为四元数
                    quat = quaternion_from_matrix(matrix)
                    
                    # 确保四元数已归一化
                    quat_norm = np.linalg.norm(quat)
                    if quat_norm > 0:
                        quat = quat / quat_norm
                    else:
                        quat = np.array([0.0, 0.0, 0.0, 1.0])
                    
                    pose_info.orientation = Quaternion(
                        x=float(quat[0]),
                        y=float(quat[1]),
                        z=float(quat[2]),
                        w=float(quat[3])
                    )
                    
                    pose_info_list.poses.append(pose_info)
                    
                except Exception as e:
                    rospy.logerr(f"Error converting world matrix {i} to pose: {e}")
                    continue
            
            # 通过发布器发布世界坐标系位姿数据
            self.world_poses_pub.publish(pose_info_list)
        except Exception as e:
            rospy.logerr(f"Error publishing world poses: {e}")
    
    def _process_data_thread(self):
        """数据处理线程"""
        rospy.loginfo("Data processing thread started")
        
        while self.running and not rospy.is_shutdown():
            try:
                # 从队列获取数据
                datagram = self.data_queue.get(timeout=0.1)
                if isinstance(datagram, ReceivedDatagram):
                    datagram.process_start_monotonic_ns = monotonic_ns()
                
                # 解析数据
                pose_list = self._parse_protobuf(datagram)
                
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in processing thread: {e}")
    
    def start(self):
        """启动接收器"""
        self.running = True
        self.diagnostics.start()
        
        # 启动数据处理线程
        self.process_thread = threading.Thread(
            target=self._process_data_thread,
            daemon=True
        )
        self.process_thread.start()
        
        rospy.loginfo(f"PICO Receiver listening on {self.host}:{self.port}")
        rospy.loginfo("Waiting for PICO data...")
        
        # 主循环：接收 UDP 数据
        while self.running and not rospy.is_shutdown():
            try:
                # 接收数据
                data, addr = self.socket.recvfrom(65535)
                recv_ns = monotonic_ns()
                datagram = self.diagnostics.decode_datagram(data, addr, recv_ns)
                if datagram is None:
                    continue
                datagram.enqueue_monotonic_ns = monotonic_ns()
                
                # 放入队列（异步处理）
                try:
                    self.data_queue.put_nowait(datagram)
                except queue.Full:
                    # 队列满时，丢弃最旧的数据
                    try:
                        dropped = self.data_queue.get_nowait()
                        self.diagnostics.record_queue_drop(dropped)
                        self.data_queue.put_nowait(datagram)
                    except queue.Empty:
                        pass
                    
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Error receiving data: {e}")
    
    def stop(self):
        """停止接收器"""
        self.running = False
        self.diagnostics.stop()
        self.diagnostic_logger.close()
        
        # 停止广播器
        if self.broadcaster:
            self.broadcaster.quit()
            rospy.loginfo("IP broadcaster stopped")
        
        if self.socket:
            self.socket.close()
        
        rospy.loginfo("PICO Receiver stopped")


def main():
    """主函数"""
    rospy.init_node('pico_receiver_minimal', anonymous=True)
    
    # 获取参数
    host = rospy.get_param('~host', '0.0.0.0')
    port = rospy.get_param('~port', 12345)
    publish_tf = rospy.get_param('~publish_tf', True)
    enable_ip_broadcast = rospy.get_param('~enable_ip_broadcast', True)
    robot_name = rospy.get_param('~robot_name', 'KUAVO')
    broadcast_port = rospy.get_param('~broadcast_port', 8443)
    enable_diagnostics = rospy.get_param('~enable_diagnostics', True)
    diagnostic_log_enable = rospy.get_param('~diagnostic_log_enable', False)
    diagnostic_log_dir = rospy.get_param('~diagnostic_log_dir', '~/.ros/pico_diagnostics')
    diagnostic_log_max_file_mb = rospy.get_param('~diagnostic_log_max_file_mb', 300.0)
    diagnostic_log_compress = rospy.get_param('~diagnostic_log_compress', True)
    diagnostic_udp_reply = rospy.get_param('~diagnostic_udp_reply', True)
    enable_time_sync = rospy.get_param('~enable_time_sync', True)
    diagnostic_publish_hz = rospy.get_param('~diagnostic_publish_hz', 1.0)
    diagnostic_allow_legacy = rospy.get_param('~diagnostic_allow_legacy', False)
    diagnostic_ping_enable = rospy.get_param('~diagnostic_ping_enable', True)
    diagnostic_ping_lower_ip = rospy.get_param('~diagnostic_ping_lower_ip', '')
    diagnostic_ping_interval_sec = rospy.get_param('~diagnostic_ping_interval_sec', 5.0)
    diagnostic_ping_timeout_sec = rospy.get_param('~diagnostic_ping_timeout_sec', 1.0)
    diagnostic_sync_max_valid_rtt_ms = rospy.get_param('~diagnostic_sync_max_valid_rtt_ms', 20.0)
    diagnostic_sync_max_valid_residual_ms = rospy.get_param('~diagnostic_sync_max_valid_residual_ms', 5.0)
    
    # 创建接收器
    receiver = MinimalPicoReceiver(
        host=host, 
        port=port, 
        publish_tf=publish_tf,
        enable_ip_broadcast=enable_ip_broadcast,
        robot_name=robot_name,
        broadcast_port=broadcast_port,
        enable_diagnostics=enable_diagnostics,
        diagnostic_log_enable=diagnostic_log_enable,
        diagnostic_log_dir=diagnostic_log_dir,
        diagnostic_log_max_file_mb=diagnostic_log_max_file_mb,
        diagnostic_log_compress=diagnostic_log_compress,
        diagnostic_udp_reply=diagnostic_udp_reply,
        enable_time_sync=enable_time_sync,
        diagnostic_publish_hz=diagnostic_publish_hz,
        diagnostic_allow_legacy=diagnostic_allow_legacy,
        diagnostic_ping_enable=diagnostic_ping_enable,
        diagnostic_ping_lower_ip=diagnostic_ping_lower_ip,
        diagnostic_ping_interval_sec=diagnostic_ping_interval_sec,
        diagnostic_ping_timeout_sec=diagnostic_ping_timeout_sec,
        diagnostic_sync_max_valid_rtt_ms=diagnostic_sync_max_valid_rtt_ms,
        diagnostic_sync_max_valid_residual_ms=diagnostic_sync_max_valid_residual_ms,
    )
    
    try:
        # 启动
        receiver.start()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    finally:
        receiver.stop()


if __name__ == '__main__':
    main()
