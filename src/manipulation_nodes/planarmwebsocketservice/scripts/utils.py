#!/usr/bin/env python3
import os
import sys
import netifaces
import json
import hashlib
import math
import numpy as np
import rospy
import rospkg

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

def get_wifi_ip():

    # 获取所有网络接口
    interfaces = netifaces.interfaces()

    # 尝试先找以 'wl' 开头的接口（通常是 WiFi）
    wifi_interface = next((iface for iface in interfaces if iface.startswith("wl")), None)

    if wifi_interface:
        addresses = netifaces.ifaddresses(wifi_interface)
        if netifaces.AF_INET in addresses:
            return addresses[netifaces.AF_INET][0]["addr"]

    # 如果找不到 wl 开头的，再尝试找 enp 开头的（有线网络）
    ethernet_interface = next((iface for iface in interfaces if iface.startswith("enp")), None)

    if ethernet_interface:
        addresses = netifaces.ifaddresses(ethernet_interface)
        if netifaces.AF_INET in addresses:
            return addresses[netifaces.AF_INET][0]["addr"]

    # 若两个都找不到，抛出异常
    rospy.logwarn("未找到有效网络接口，返回 None")
    return None



def get_hotspot_ip():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the hotspot interface (usually starts with 'ap')
        hotspot_interface = next(
            (iface for iface in interfaces if iface.startswith("ap")), None
        )

        if hotspot_interface:
            # Get the IPv4 address of the hotspot interface
            addresses = netifaces.ifaddresses(hotspot_interface)
            if netifaces.AF_INET in addresses:
                return addresses[netifaces.AF_INET][0]["addr"]

        return None
    except Exception as e:
        return None


# ... (keep the get_wifi() function as is)


import logging
import subprocess

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_wifi():
    try:
        logger.info("Attempting to get WiFi SSID using nmcli")
        # Run the nmcli command to get the SSID
        result = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], 
                                capture_output=True, text=True, check=True)
        # Parse the output to find the active connection
        for line in result.stdout.strip().split('\n'):
            active, ssid = line.split(':')
            if active == 'yes':
                logger.info(f"Connected to WiFi: {ssid}")
                return ssid
        
        logger.warning("No active WiFi connection found")
        return "Not connected to WiFi"
    except subprocess.CalledProcessError as e:
        logger.error(f"Error running nmcli command: {e}")
        return f"Error: {e}"
    except Exception as e:
        logger.error(f"Unexpected error getting WiFi SSID: {e}")
        return f"Error: {e}"

def get_hotspot():
    try:
        # 读取环境文件中的ROBOT_SERIAL_NUMBER
        with open('/etc/environment.d/RRNIS.env', 'r') as file:
            for line in file:
                if line.startswith('ROBOT_SERIAL_NUMBER'):
                    robot_serial_number = line.split('=')[1].strip()
                    return f"{robot_serial_number}的热点"
                
        logger.warning("No hotspot found")
        return "No hotspot found"
    except FileNotFoundError as e:
        logger.warning("No environment file found")
        return f"Error: {e}"
    except Exception as e:
        logger.error(f"Unexpected error getting hotspot: {e}")
        return f"Error: {e}"

# 使用 RobotVersion 类创建版本号对象
robot_version_int = int(os.environ.get("ROBOT_VERSION", "45"))
robot_version = RobotVersion.create(robot_version_int) if RobotVersion.is_valid(robot_version_int) else RobotVersion(4, 5, 0)

if robot_version.version_number() >= 40:
    INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
else:
    INIT_ARM_POS = [22.91831, 0, 0, -45.83662, 22.91831, 0, 0, -45.83662, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # task.info: shoudler_center: 0.4rad, elbow_center: -0.8rad

def frames_to_custom_action_data(file_path: str):
    with open(file_path, "r") as f:
        data = json.load(f)
    frames = data["frames"]
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, INIT_ARM_POS[key-1]],
                            [0, 0],
                            [0, INIT_ARM_POS[key-1]],
                        ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/100, 1), int(value)],
                    [round((keyframe+left_CP[0])/100, 1), int(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), int(value+right_CP[1])],
                ])
    return action_data

def frames_to_custom_action_data_ocs2(file_path: str, start_frame_time: float, current_arm_joint_state: list):
    def filter_data(action_data, start_frame_time, current_arm_joint_state):
        filtered_action_data = {}
        x_shift = start_frame_time - 1
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]

                if not found_start and end_time >= start_frame_time:
                    found_start = True
                    
                    p0 = np.array([0, current_arm_joint_state[key-1]])
                    p3 = np.array([next_frame[0][0] - x_shift, next_frame[0][1]])
                    
                    # Calculate control points for smooth transition
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left
                    
                    # Create new frame
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]
                    
                    # Modify next_frame's left control point
                    next_frame[1] = p2.tolist()
                    
                    filtered_frames.append(frame1)
                    skip_next = True
                
                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data 

    with open(file_path, "r") as f:
        data = json.load(f)
    frames = data["frames"]
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/100, 1), math.radians(value)],
                    [round((keyframe+left_CP[0])/100, 1), math.radians(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/100, 1), math.radians(value+right_CP[1])],
                ])
    return filter_data(action_data, start_frame_time, current_arm_joint_state)

def verify_robot_version(file_path: str):
    with open(file_path, "r") as f:
        data = json.load(f)
    
    robot_type_raw = data.get("robotType", None)
    if robot_type_raw is None:
        msg = "Action file missing required field: robotType"
        rospy.logerr(msg)
        return False, msg
    try:
        tact_robot_version = int(robot_type_raw)
    except (TypeError, ValueError):
        msg = f"Invalid robotType in action file: {robot_type_raw}"
        rospy.logerr(msg)
        return False, msg

    # 版本兼容关系映射
    version_compat_map = {
        41: [41],
        42: [42],
        45: [43, 45, 46, 48, 49, 100045, 100049, 200049],
        52: [52, 53, 54],
        11: [11, 13, 14],
        13: [11, 13, 14],
        14: [11, 13, 14]
    }
    allowed_robot_versions = version_compat_map.get(tact_robot_version, [tact_robot_version])
    robot_version_number = robot_version.version_number()
    if robot_version_number not in allowed_robot_versions:
        msg = (
            f"Version mismatch: tact {tact_robot_version} is incompatible with robot {robot_version_number}"
        )
        rospy.logerr(msg)
        return False, msg
    
    rospy.loginfo(f"Version match: tact {tact_robot_version} is compatible with robot {robot_version_number}")
    return True, None

def get_start_end_frame_time(file_path: str):
    with open(file_path, "r") as f:
        data = json.load(f)
    start_frame_time = data["first"] / 100
    end_frame_time = data["finish"] / 100
    return start_frame_time, end_frame_time

def calculate_file_md5(file_path: str) -> str:
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def get_wifi_mac_address():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the WiFi interface (usually starts with 'wl')
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            # Get the MAC address of the WiFi interface
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_LINK in addresses:
                return addresses[netifaces.AF_LINK][0]['addr']

        return "WiFi interface not found"
    except Exception as e:
        return f"Error: {e}"

def get_hotspot_mac_address():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the Hotspot interface (usually starts with 'ap' or similar)
        hotspot_interface = next(
            (iface for iface in interfaces if iface.startswith("ap")), None
        )

        if hotspot_interface:
            # Get the MAC address of the Hotspot interface
            addresses = netifaces.ifaddresses(hotspot_interface)
            if netifaces.AF_LINK in addresses:
                return addresses[netifaces.AF_LINK][0]['addr']

        return "Hotspot interface not found"
    except Exception as e:
        return f"Error: {e}"
