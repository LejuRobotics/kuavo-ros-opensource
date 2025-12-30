#!/usr/bin/env python3
import os
import netifaces
import json
import hashlib
import math
import numpy as np

def get_wifi_ip():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the WiFi interface (usually starts with 'wl')
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            # Get the IPv4 address of the WiFi interface
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_INET in addresses:
                return addresses[netifaces.AF_INET][0]["addr"]

        return None
    except Exception as e:
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

INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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
