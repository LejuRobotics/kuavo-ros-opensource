# -*- coding: utf-8 -*-
import rospy
import rospkg
import os
import pwd
import sys
import asyncio
import signal
import json
from queue import Queue
from dataclasses import dataclass
import tf
import geometry_msgs.msg
import traceback
import vr_manager

# 全局变量存储地图信息，用于坐标系转换
global_map_info = None

# 全局 TF Listener（复用以提高效率）
tf_listener = None

# 地图推送状态缓存
last_status_check_time = 0
should_push_map_cache = True
STATUS_CHECK_INTERVAL = 2.0  # 2秒检查一次


def get_tf_listener():
    """获取全局 TF Listener"""
    global tf_listener
    if tf_listener is None:
        tf_listener = tf.TransformListener()
    return tf_listener


def odometry_to_png(odom_x, odom_y, map_info=None):
    """
    Odometry坐标系转PNG坐标系

    Args:
        odom_x: Odometry坐标系X坐标（米）
        odom_y: Odometry坐标系Y坐标（米）
        map_info: 地图信息字典，包含resolution, origin, width, height

    Returns:
        tuple: (png_x, png_y) PNG坐标系坐标（像素）
    """
    global global_map_info

    if map_info is None:
        map_info = global_map_info

    if not map_info:
        print("警告: 地图信息不可用，无法进行坐标转换")
        return None, None

    try:
        # 获取地图参数
        resolution = map_info.get("resolution", 0.05)  # 米/像素
        origin_x = map_info.get("origin", {}).get("x", 0.0)  # 地图原点X（米）
        origin_y = map_info.get("origin", {}).get("y", 0.0)  # 地图原点Y（米）
        width = map_info.get("width", 0)  # 图片宽度（像素）
        height = map_info.get("height", 0)  # 图片高度（像素）

        if resolution <= 0 or width <= 0 or height <= 0:
            print("错误: 地图参数无效")
            return None, None

        # 使用 TF 将 odom 坐标转换到 map 坐标系
        try:
            listener = get_tf_listener()
            listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(1.0))

            # 转换到 map 坐标系
            odom_point_stamped = geometry_msgs.msg.PointStamped()
            odom_point_stamped.header.stamp = rospy.Time(0)
            odom_point_stamped.header.frame_id = "odom"
            odom_point_stamped.point.x = odom_x
            odom_point_stamped.point.y = odom_y
            odom_point_stamped.point.z = 0.0

            map_point = listener.transformPoint("map", odom_point_stamped)
            map_x = map_point.point.x
            map_y = map_point.point.y

        except Exception as tf_error:
            # TF 转换失败时，回退到直接使用 odom 坐标
            print(f"TF 转换失败，使用 odom 坐标: {tf_error}")
            map_x = odom_x
            map_y = odom_y

        # Map坐标系 -> PNG坐标系
        # PNG_X = (Map_X - Origin_X) / Resolution
        # PNG_Y = Height - (Map_Y - Origin_Y) / Resolution
        png_x = (map_x - origin_x) / resolution
        png_y = height - (map_y - origin_y) / resolution

        # 调试信息：显示转换前后的坐标
        #print(f"坐标转换调试: Odom({odom_x:.3f}, {odom_y:.3f}) -> Map({map_x:.3f}, {map_y:.3f}) -> PNG({png_x:.1f}, {png_y:.1f})")
        #print(f"地图参数: resolution={resolution}, origin=({origin_x:.3f}, {origin_y:.3f}), size={width}x{height}")

        # 检查原始PNG坐标是否超出边界
        original_png_x = png_x
        original_png_y = png_y

        # 确保坐标在图片范围内
        png_x = max(0, min(width - 1, png_x))
        png_y = max(0, min(height - 1, png_y))

        # 如果坐标被边界检查调整了，输出警告
        if (abs(png_x - original_png_x) > 0.1 or abs(png_y - original_png_y) > 0.1):
            pass
            #print(f"警告: 坐标超出地图范围，已调整: 原始PNG({original_png_x:.1f}, {original_png_y:.1f}) -> 调整后PNG({png_x:.1f}, {png_y:.1f})")

        return round(png_x, 2), round(png_y, 2)

    except Exception as e:
        print(f"Odometry转PNG坐标失败: {e}")
        return None, None


def png_to_map(png_x, png_y, map_info=None):
    """
    PNG坐标系转Map坐标系（不使用TF，直接转换）

    用于任务点保存：任务点应存储在map坐标系中，而不是odom坐标系

    Args:
        png_x: PNG坐标系X坐标（像素）
        png_y: PNG坐标系Y坐标（像素）
        map_info: 地图信息字典，包含resolution, origin, width, height

    Returns:
        tuple: (map_x, map_y) Map坐标系坐标（米）
    """
    global global_map_info

    if map_info is None:
        map_info = global_map_info

    if not map_info:
        print("警告: 地图信息不可用，无法进行坐标转换")
        return None, None

    try:
        # 获取地图参数
        resolution = map_info.get("resolution", 0.05)  # 米/像素
        origin_x = map_info.get("origin", {}).get("x", 0.0)  # 地图原点X（米）
        origin_y = map_info.get("origin", {}).get("y", 0.0)  # 地图原点Y（米）
        width = map_info.get("width", 0)  # 图片宽度（像素）
        height = map_info.get("height", 0)  # 图片高度（像素）

        if resolution <= 0 or width <= 0 or height <= 0:
            print("错误: 地图参数无效")
            return None, None

        # PNG坐标系 -> Map坐标系
        # Map_X = Origin_X + PNG_X * Resolution
        # Map_Y = Origin_Y + (Height - PNG_Y) * Resolution
        map_x = origin_x + png_x * resolution
        map_y = origin_y + (height - png_y) * resolution

        #print(f"坐标转换: PNG({png_x:.1f}, {png_y:.1f}) -> Map({map_x:.3f}, {map_y:.3f})")

        return map_x, map_y

    except Exception as e:
        print(f"PNG转Map坐标失败: {e}")
        return None, None


def map_to_png(map_x, map_y, map_info=None):
    """
    Map坐标系转PNG坐标系（不使用TF，直接转换）

    用于任务点读取：任务点存储在map坐标系中，需要转换为PNG坐标用于前端显示

    Args:
        map_x: Map坐标系X坐标（米）
        map_y: Map坐标系Y坐标（米）
        map_info: 地图信息字典，包含resolution, origin, width, height

    Returns:
        tuple: (png_x, png_y) PNG坐标系坐标（像素）
    """
    global global_map_info

    if map_info is None:
        map_info = global_map_info

    if not map_info:
        #print("警告: 地图信息不可用，无法进行坐标转换")
        return None, None

    try:
        # 获取地图参数
        resolution = map_info.get("resolution", 0.05)  # 米/像素
        origin_x = map_info.get("origin", {}).get("x", 0.0)  # 地图原点X（米）
        origin_y = map_info.get("origin", {}).get("y", 0.0)  # 地图原点Y（米）
        width = map_info.get("width", 0)  # 图片宽度（像素）
        height = map_info.get("height", 0)  # 图片高度（像素）

        if resolution <= 0 or width <= 0 or height <= 0:
            print("错误: 地图参数无效")
            return None, None

        # Map坐标系 -> PNG坐标系
        # PNG_X = (Map_X - Origin_X) / Resolution
        # PNG_Y = Height - (Map_Y - Origin_Y) / Resolution
        png_x = (map_x - origin_x) / resolution
        png_y = height - (map_y - origin_y) / resolution

        # 确保坐标在图片范围内
        png_x = max(0, min(width - 1, png_x))
        png_y = max(0, min(height - 1, png_y))

        #print(f"坐标转换: Map({map_x:.3f}, {map_y:.3f}) -> PNG({png_x:.1f}, {png_y:.1f})")

        return round(png_x, 2), round(png_y, 2)

    except Exception as e:
        print(f"Map转PNG坐标失败: {e}")
        return None, None


def get_mapping_status():
    """获取建图状态服务"""
    try:
        # 使用 rosservice 命令调用服务
        service_name = '/get_mapping_status'

        # 调用 rosservice 命令
        cmd = f"rosservice call {service_name}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)

        if result.returncode == 0:
            output = result.stdout.strip()

            # 提取 message 字段的值
            import re
            import json

            # 匹配 message: "..." （使用贪婪匹配和 DOTALL 标志）
            match = re.search(r'message:\s*"(.*)"', output, re.DOTALL)
            if match:
                message_str = match.group(1)
                # 处理YAML行续行符：删除反斜杠和后面的换行符
                message_str = re.sub(r'\\\s*\n\s*', '', message_str)
                # 处理转义字符
                message_str = message_str.replace('\\"', '"')
                data = json.loads(message_str)

                # 创建一个简单对象来存储结果
                class MappingStatus:
                    def __init__(self, is_mapping, current_map_name, active_processes):
                        self.is_mapping = is_mapping
                        self.current_map_name = current_map_name
                        self.active_processes = active_processes
                return MappingStatus(
                    data.get('is_mapping', False),
                    data.get('current_map_name', ''),
                    data.get('active_processes', 0)
                )
            else:
                print(f"Failed to parse message from output: {output}")
        return None
    except Exception as e:
        print(f"获取建图状态失败: {e}")
        return None


def get_load_map_status():
    """获取当前加载地图状态服务"""
    try:
        from kuavo_mapping.srv import GetCurrentMap, GetCurrentMapRequest

        service_name = '/get_current_map'
        rospy.wait_for_service(service_name, timeout=0.5)
        proxy = rospy.ServiceProxy(service_name, GetCurrentMap)

        response = proxy()

        # 创建一个简单对象来存储结果
        # current_map 为空字符串说明没有加载地图，转换为 None
        map_name = response.current_map if response.current_map else None
        class LoadMapStatus:
            def __init__(self, map_name):
                self.map_name = map_name
        return LoadMapStatus(map_name)
    except Exception as e:
        print(f"获取地图加载状态失败: {e}")
        return None


def should_push_map():
    """轻量级的状态检查，使用缓存避免频繁查询"""
    global last_status_check_time, should_push_map_cache

    current_time = time.time()

    # 只在间隔时间到达时才查询服务状态
    if current_time - last_status_check_time > STATUS_CHECK_INTERVAL:
        try:
            # 查询建图状态
            mapping_status = get_mapping_status()
            mapping_active = getattr(mapping_status, 'is_mapping', False)

            # 查询加载地图状态
            load_status = get_load_map_status()
            map_loaded = getattr(load_status, 'map_name', None) is not None

            # 更新缓存
            should_push_map_cache = mapping_active or map_loaded
            last_status_check_time = current_time

            print(f"地图推送状态更新: 建图={mapping_active}, 地图已加载={map_loaded}, 推送={should_push_map_cache}")

        except Exception as e:
            print(f"地图推送状态检查失败: {e}")
            # 默认允许推送，避免影响正常功能
            should_push_map_cache = True

    return should_push_map_cache


def update_map_info(map_info):
    """
    更新全局地图信息

    Args:
        map_info: 地图信息字典
    """
    global global_map_info
    global_map_info = map_info
from typing import Any, Union, Dict
import websockets
import threading
import time
import math
import subprocess
import base64
from pathlib import Path
import yaml
from utils import calculate_file_md5, frames_to_custom_action_data, get_start_end_frame_time, frames_to_custom_action_data_ocs2, verify_robot_version, robot_version
import shutil
import rosnode
import glob
from std_srvs.srv import Trigger, TriggerResponse


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
from kuavo_ros_interfaces.srv import planArmTrajectoryBezierCurve, stopPlanArmTrajectory, planArmTrajectoryBezierCurveRequest, ocs2ChangeArmCtrlMode
from kuavo_ros_interfaces.msg import planArmState, jointBezierTrajectory, bezierCurveCubicPoint, robotHeadMotionData
from kuavo_msgs.msg import robotHandPosition, robotWaistControl
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.msg import AprilTagDetectionArray
from h12pro_controller_node.msg import UpdateH12CustomizeConfig
from kuavo_msgs.srv import adjustZeroPoint, adjustZeroPointRequest, LoadMap, LoadMapRequest, GetAllMaps, GetAllMapsRequest,SetInitialPose, SetInitialPoseRequest, robotSwitchPose, robotSwitchPoseRequest,changeArmCtrlModeRequest, changeArmCtrlMode
from std_msgs.msg import Bool, Float64MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry
import cv2
import numpy as np
import tf
import argparse
import subprocess
from typing import Tuple
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 导航功能需要的消息类型
from sensor_msgs.msg import PointCloud2
import re

# Replace multiprocessing values with simple variables
plan_arm_state_progress = 0
plan_arm_state_status = False
should_stop = False
terminate_process_result = False
process = None
response_queue = Queue()

# 全局变量用于存储零点数据
zero_point_data = []
adjusted_zero_point_data = []
frist_get_zero_point_flag = False

active_threads: Dict[websockets.WebSocketServerProtocol, threading.Event] = {}
ACTION_FILE_FOLDER = "~/.config/lejuconfig/action_files"
ROBAN_TACT_LENGTH = 23
g_robot_type = ""
ocs2_current_joint_state = []
# robot_version 从 utils 模块导入，避免重复创建
print(f"robot_version: {robot_version}")
# Global variables for robot control
ROS_MASTER_URI = os.getenv("ROS_MASTER_URI")
ROS_IP = os.getenv("ROS_IP")
ROS_HOSTNAME = os.getenv("ROS_HOSTNAME")

# Get KUAVO_ROS_CONTROL_WS_PATH from environment variable, if not found, try to find it
KUAVO_ROS_CONTROL_WS_PATH = os.getenv("KUAVO_ROS_CONTROL_WS_PATH")
if not KUAVO_ROS_CONTROL_WS_PATH:
    # Try to find the kuavo-ros-control workspace path by searching upward from this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    while current_dir != '/':
        # Check if this directory contains the标志性 files of kuavo-ros-control workspace
        if os.path.exists(os.path.join(current_dir, 'src')) and \
           os.path.exists(os.path.join(current_dir, 'devel/setup.bash')):
            KUAVO_ROS_CONTROL_WS_PATH = current_dir
            break
        current_dir = os.path.dirname(current_dir)

ROBOT_VERSION = os.environ.get("ROBOT_VERSION")
WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME = "websocket_humanoid_robot_service"
# 为了兼容性，保留 ROBOT_VERSION 字符串用于环境变量传递


# 机器人状态跟踪变量，用于跟踪机器人当前状态: unlaunch, crouching, standing
robot_status = "unlaunch"  # 默认状态为未启动

def update_robot_status_from_service():
    """
    从 /humanoid_controller/real_launch_status 服务获取机器人状态并更新全局 robot_status
    """
    global robot_status
    try:
        success, status_message = robot_instance.get_robot_launch_status()
        if success:
            # print(f"robot_status: {status_message}")

            # 映射服务返回的状态到我们的状态系统
            if status_message == "ready_stance":
                robot_status = "crouching"
            elif status_message == "launched":
                robot_status = "standing"
            elif status_message == "zero_point_cali":
                robot_status = "zero_point_cali"
            else:
                robot_status = "unlaunch"
        else:
            robot_status = "unlaunch"
    except Exception as e:
        print(f"Failed to update robot status from service: {e}")
        robot_status = "unlaunch"

def check_real_kuavo():
    try:
        # optimize: 简单通过检查零点文件来判断是否为实物, 可优化判断条件
        offset_file = os.path.expanduser("~/.config/lejuconfig/offset.csv")
        config_file = os.path.expanduser("~/.config/lejuconfig/config.yaml")
        
        offset_file_exists = os.path.exists(offset_file)
        config_file_exists = os.path.exists(config_file)
        print(f"offset_file: {offset_file}, exists: {offset_file_exists}")
        print(f"config_file: {config_file}, exists: {config_file_exists}")
        return offset_file_exists and config_file_exists
    except Exception as e:
        return False

def tmux_run_cmd(session_name:str, cmd:str, sudo:bool=False)->Tuple[bool, str]:
    launch_cmd = cmd
        
    print(f"launch_cmd: {launch_cmd}")
    
    try:
        subprocess.run(["tmux", "kill-session", "-t", session_name],
                        stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Failed to kill session: {e}")
        # 这里不返回错误，因为可能session不存在
        pass

    print(f"If you want to check the session, please run 'tmux attach -t {session_name}'")
    
    # 构建完整的命令，确保环境变量正确设置 
    full_cmd = f"source ~/.bashrc && \
        source {KUAVO_ROS_CONTROL_WS_PATH}/devel/setup.bash && \
        export ROBOT_VERSION={ROBOT_VERSION} && \
        {launch_cmd}"
    
    tmux_cmd = [
        "tmux", "new-session",
        "-s", session_name,
        "-d",
        full_cmd
    ]
    if sudo:
        tmux_cmd.insert(0, "sudo")
    
    print(f"tmux_cmd: {tmux_cmd}")

    # 执行tmux命令并等待结果
    try:
        result = subprocess.run(
            tmux_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10  # 设置超时时间
        )
        
        # 检查命令执行是否出错
        if result.returncode != 0:
            error_msg = f"Failed to execute tmux command: {result.stderr}"
            print(error_msg)
            return False, error_msg
    except subprocess.TimeoutExpired:
        error_msg = "tmux command execution timed out"
        print(error_msg)
        return False, error_msg
    except Exception as e:
        error_msg = f"Failed to execute tmux command: {str(e)}"
        print(error_msg)
        return False, error_msg

    # 等待一段时间让session启动
    time.sleep(3.0)

    # 检查session是否成功创建
    check_cmd = ["tmux", "has-session", "-t", session_name]
    if sudo:
        check_cmd.insert(0, "sudo")
    check_result = subprocess.run(check_cmd, capture_output=True)
    ret = False
    if check_result.returncode == 0:
        ret = True
        msg = f"Started {session_name} in tmux session: {session_name}"
    else:
        msg = f"Failed to start {session_name}"
    return ret, msg

def check_rosnode_exists(node_name:str)->bool:
    try:
        nodes = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
        return node_name in nodes
    except subprocess.CalledProcessError as e:
       print(f"Error checking if node {node_name} exists: {e}")
       return False
    except Exception as e:
        print(f"Error checking if node {node_name} exists: {e}")
        return False

class KuavoRobot:
    def __init__(self):
        pass

    def start_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("start_robot is not implemented")

    def stop_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("stop_robot is not implemented")

    def stand_robot(self)->Tuple[bool, str]:
        raise NotImplementedError("stand_robot is not implemented")

    def switch_robot_pose(self)->Tuple[bool, str]:
        raise NotImplementedError("switch_robot_pose is not implemented")

    def get_robot_launch_status(self)->Tuple[bool, str]:
        raise NotImplementedError("get_robot_launch_status is not implemented")

class KuavoRobotReal(KuavoRobot):
    def __init__(self, debug=False):
        super().__init__()
        self.debug = debug
        self.stop_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def start_robot(self)->Tuple[bool, str]:
        global WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME
        global KUAVO_ROS_CONTROL_WS_PATH
        global ROS_MASTER_URI
        global ROS_IP
        global ROS_HOSTNAME
 
        if self.debug:
            launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch"
            if not ROS_MASTER_URI or not ROS_IP or not ROS_HOSTNAME:
                ROS_MASTER_URI = "http://kuavo_master:11311"
                ROS_IP = "kuavo_master"
                ROS_HOSTNAME = "kuavo_master"
        else:
            launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch start_way:=auto"

        return tmux_run_cmd(WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME, launch_cmd, sudo=True)
    
    def stop_robot(self)->Tuple[bool, str]:
        try:
            # 已经启动则下蹲再停止
            if robot_status == "launched" or robot_status == "standing" or robot_status == "zero_point_cali":
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                msg.linear.z = -0.05
                for i in range(10):
                    self.cmd_vel_pub.publish(msg)
                    time.sleep(0.1)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            return True, "success"
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            return False, f"Failed to stop robot: {e}"
            
    def stand_robot(self)->Tuple[bool, str]:
        try:
            client = rospy.ServiceProxy('/humanoid_controller/real_initial_start', Trigger)
            req = TriggerRequest()
            client.wait_for_service(timeout=2.0)
            # Call the service

            if client.call(req):
                #print(f"RealInitializeSrv service call successful")
                return True, "Success"
            else:
                print(f"Failed to callRealInitializeSrv service")
                return False, "Failed to callRealInitializeSrv service"
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"Service call failed: {e}"

    def switch_robot_pose(self, pose="")->Tuple[bool, str]:
        try:
            client = rospy.ServiceProxy('/hardware/switch_robot_state', robotSwitchPose)
            req = robotSwitchPoseRequest()
            req.status = pose
            client.wait_for_service(timeout=2.0)
            # Call the service
            response = client.call(req)
            if response.success:
                print(f"Switch robot pose service call successful")
                return True, response.message
            else:
                print(f"Failed to call switch robot pose service: {response.message}")
                return False, response.message
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"Service call failed: {e}"

    def get_robot_launch_status(self)->Tuple[bool, str]:
        client = rospy.ServiceProxy('/humanoid_controller/real_launch_status', Trigger)
        try:
            client.wait_for_service(timeout=2.0)
        except rospy.ROSException as e:
            # 等待服务超时（服务不存在）
            print(f"Service does not exist: {e}")
            return True, "unknown"  # 关键修改：服务不存在时返回(True, "unknown")
    
        try:
            req = TriggerRequest()
            client.wait_for_service(timeout=1.5)
            # Call the service
            response = client.call(req)
            if response.success:
                print(f"RealInitializeSrv service call successful")
                return True, response.message
            else:
                print(f"Failed to callRealInitializeSrv service")
                return False, "unknown"
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False, f"unknown"

    def zero_point_robot(self,data)->Tuple[bool, str]:
        global WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME
        global KUAVO_ROS_CONTROL_WS_PATH
        global ROS_MASTER_URI
        global ROS_IP
        global ROS_HOSTNAME

        global robot_status
        global frist_get_zero_point_flag

 
        if self.debug:
            launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch"
            if not ROS_MASTER_URI or not ROS_IP or not ROS_HOSTNAME:
                ROS_MASTER_URI = "http://kuavo_master:11311"
                ROS_IP = "kuavo_master"
                ROS_HOSTNAME = "kuavo_master"

        else:
            if robot_status == "unlaunch" and data == "start":
                launch_cmd = "roslaunch humanoid_controllers load_kuavo_real.launch cali_set_zero:=true"
            elif data == "exit":
                result, msg = robot_instance.switch_robot_pose("exit")
                if not result:
                    return False, msg
                    
                # 添加10秒超时机制
                start_time = time.time()
                while(1) :
                    success, status_message = robot_instance.get_robot_launch_status()
                    if success:
                        # 先切换到绷直状态在解锁
                        if status_message == "zero_point_cali":
                            time.sleep(1)
                            self.stop_robot()
                            robot_status = "unlaunch"
                            frist_get_zero_point_flag = True
                            break

                    if time.time() - start_time > 10:  # 超时10秒
                        return False, "failed"
                        # break
                    time.sleep(0.1)  # 短暂延时以减少CPU占用
                    
                return True, "success"
            else:
                # 处处不满足任何条件的情况
                return False, "Invalid operation: robot_status is '{}' and data is '{}'".format(robot_status, data)

        return tmux_run_cmd(WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME, launch_cmd, sudo=True)

class KuavoRobotSim(KuavoRobot):
    def __init__(self, debug=False):
        super().__init__()
        self.debug = debug
        self.stop_pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def start_robot(self)->Tuple[bool, str]:
        global WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME
        global KUAVO_ROS_CONTROL_WS_PATH
        global ROS_MASTER_URI
        global ROS_IP
        global ROS_HOSTNAME

        if not ROS_MASTER_URI or not ROS_IP or not ROS_HOSTNAME:
            ROS_MASTER_URI = "http://localhost:11311"
            ROS_IP = "localhost"
            ROS_HOSTNAME = "localhost"

        launch_cmd = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch && export DISPLAY=:1"
        return tmux_run_cmd(WEBSOCKET_HUMANOID_ROBOT_SESSION_NAME, launch_cmd, sudo=False)
    
    def stop_robot(self)->Tuple[bool, str]:
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            msg.linear.z = -0.05
            for i in range(50):
                self.cmd_vel_pub.publish(msg)
                time.sleep(0.1)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            self.stop_pub.publish(True)
            return True, "success"
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            return False, f"Failed to stop robot: {e}"
    
    def stand_robot(self)->Tuple[bool, str]:
        return self.get_robot_launch_status()
    
    def get_robot_launch_status(self)->Tuple[bool, str]:
        if check_rosnode_exists("/humanoid_sqp_mpc") and check_rosnode_exists("/nodelet_controller"):
            return True, "launched"
        else:
            return True, "unlaunch"

# Initialize robot instance
if check_real_kuavo():
    robot_instance = KuavoRobotReal()
else:
    robot_instance = KuavoRobotSim()

package_name = 'planarmwebsocketservice'
package_path = rospkg.RosPack().get_path(package_name)

UPLOAD_FILES_FOLDER = package_path + "/upload_files"
KUAVO_TACT_LENGTH = 28
ROBAN_TACT_LENGTH = 23

# 下位机音乐文件存放路径，如果不存在则进行创建
sudo_user = os.environ.get("SUDO_USER")
if sudo_user:
    user_info = pwd.getpwnam(sudo_user)
    home_path = user_info.pw_dir
else:
    home_path = os.path.expanduser("~")

MUSIC_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'music')
ACTION_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'action_files')
try:
    Path(MUSIC_FILE_FOLDER).mkdir(parents=True, exist_ok=True)
    Path(ACTION_FILE_FOLDER).mkdir(parents=True, exist_ok=True)
except Exception as e:
    print(f"创建目录时出错: {e}")

# H12 遥控器按键功能配置文件路径
h12_package_name = "h12pro_controller_node"
h12_package_path = rospkg.RosPack().get_path(h12_package_name)
joy_package_name = "joy"
joy_package_path = rospkg.RosPack().get_path(joy_package_name)
if robot_version.major() >= 4:
    H12_CONFIG_PATH = h12_package_path + "/config/customize_config.json"
    current_arm_joint_state = [0] * KUAVO_TACT_LENGTH
elif robot_version.major() == 1:
    H12_CONFIG_PATH = joy_package_path + "/config/customize_config.json"
    current_arm_joint_state = [0] * ROBAN_TACT_LENGTH
_last_joint_msg = None
_last_hand_msg = None

# 获取仓库路径
 # 获取当前 Python 文件的路径
current_file = os.path.abspath(__file__)
# 获取文件所在目录
current_dir = os.path.dirname(current_file)
repo_path_result = subprocess.run(
    ['git', 'rev-parse', '--show-toplevel'],
    capture_output=True,
    text=True,
    cwd=current_dir
)
REPO_PATH = repo_path_result.stdout.strip()

g_robot_type = ""
ocs2_current_joint_state = []

if robot_version.major() >= 4:
    joint_names = [
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
else:
    joint_names = [
        "zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link",
        "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link",
    ]
ocs2_joint_state = JointState()
ocs2_hand_state = robotHandPosition()
ocs2_head_state = robotHeadMotionData()
ocs2_waist_state = robotWaistControl()
robot_settings = {
    "kuavo":{
        "plan_arm_trajectory_bezier_service_name": "/plan_arm_trajectory_bezier_curve",
        "stop_plan_arm_trajectory_service_name": "/stop_plan_arm_trajectory",
        "arm_traj_state_topic_name": "/robot_plan_arm_state",
    },
    "ocs2":{
        "plan_arm_trajectory_bezier_service_name": "/bezier/plan_arm_trajectory",
        "stop_plan_arm_trajectory_service_name": "/bezier/stop_plan_arm_trajectory",
        "arm_traj_state_topic_name": "/bezier/arm_traj_state",
    }
}

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
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

def sensors_data_callback(msg):
    """更新关节数据"""
    global _last_joint_msg, _last_hand_msg, ocs2_hand_state
    _last_joint_msg = msg

    if _last_hand_msg is None:
        from sensor_msgs.msg import JointState
        dummy_hand = JointState()
        dummy_hand.position = [0.0] * 12
        _last_hand_msg = dummy_hand

    _update_current_arm_joint_state(_last_joint_msg, _last_hand_msg)

def robot_hand_callback(msg):
    """更新手部数据"""
    global _last_hand_msg, _last_joint_msg, ocs2_hand_state
    left = msg.position[:6] if len(msg.position) >= 6 else [0] * 6
    right = msg.position[6:12] if len(msg.position) >= 12 else [0] * 6

    _last_hand_msg = msg

    if _last_joint_msg is not None:
        _update_current_arm_joint_state(_last_joint_msg, _last_hand_msg)

def _update_current_arm_joint_state(joint_msg, hand_msg):
    """整合 joint_msg 和 hand_msg，更新 current_arm_joint_state"""

    global ocs2_joint_state,current_arm_joint_state
    if robot_version.major() == 5:
        arm_part = list(joint_msg.joint_data.joint_q[13:27])
        hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12
        head_part = list(joint_msg.joint_data.joint_q[-2:])
        waist_part = [joint_msg.joint_data.joint_q[12]]
        current_arm_joint_state = arm_part + hand_part + head_part + waist_part
    elif robot_version.major() == 4:
        arm_part = list(joint_msg.joint_data.joint_q[12:26])
        hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12
        head_part = list(joint_msg.joint_data.joint_q[-2:])
        current_arm_joint_state = arm_part + hand_part + head_part

    elif robot_version.major() == 1:
        # 按照 joint_q 索引顺序定义变量
        hand_part = list(hand_msg.position[:12]) if len(hand_msg.position) >= 12 else [0.0] * 12  # 对应 joint_q[0:12]
        waist_part = [joint_msg.joint_data.joint_q[12]]  # 对应 joint_q[12]
        arm_part = list(joint_msg.joint_data.joint_q[13:21])  # 对应 joint_q[13:21]
        head_part = list(joint_msg.joint_data.joint_q[21:23])  # 对应 joint_q[21:23]
        # 保持最终组合顺序不变：arm_part + hand_part + head_part + waist_part
        current_arm_joint_state = arm_part + hand_part + head_part + waist_part

    current_arm_joint_state = [round(v, 2) for v in current_arm_joint_state]

def traj_callback(msg):
    global ocs2_joint_state
    if len(msg.points) == 0:
        return
    point = msg.points[0]
    
    if robot_version.major() == 5:
        ocs2_joint_state.name = joint_names
        ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        ocs2_joint_state.effort = [0] * 14
        ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
        ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]
        ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:28]]
        if len(point.positions) > 28:
            ocs2_waist_state.header.stamp = rospy.Time.now()
            ocs2_waist_state.data.data = [math.degrees(pos) for pos in point.positions[28:]]
        else:
            ocs2_waist_state.data.data = []
    elif robot_version.major() == 4:
        ocs2_joint_state.name = joint_names
        ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        ocs2_joint_state.effort = [0] * 14
        ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
        ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]
        ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]

    elif robot_version.major() == 1:
        ocs2_joint_state.name = joint_names
        ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:8]]
        ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:8]]
        ocs2_joint_state.effort = [0] * 8

        if len(point.positions) == ROBAN_TACT_LENGTH:
            ocs2_hand_state.left_hand_position = [max(0, int(math.degrees(pos))) for pos in point.positions[8:14]]  # 无符号整数
            ocs2_hand_state.right_hand_position = [max(0, int(math.degrees(pos))) for pos in
                                                point.positions[14:20]]  # 无符号整数
            
            ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[20:22]]

            ocs2_waist_state.header.stamp = rospy.Time.now()
            ocs2_waist_state.data.data = [math.degrees(pos) for pos in point.positions[22:]]


kuavo_arm_traj_pub = None
control_hand_pub = None
control_head_pub = None
control_waist_pub = None
update_h12_config_pub = None
update_joy_config_pub = None

def timer_callback(event):
    global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, control_waist_pub
    if g_robot_type == "ocs2" and len(ocs2_joint_state.position) > 0 and plan_arm_state_status is False and vr_manager.recording_state != vr_manager.RecordingState.CONVERTING:
        if len(ocs2_joint_state.position) != 0:
            kuavo_arm_traj_pub.publish(ocs2_joint_state)
        if len(ocs2_hand_state.left_hand_position) != 0 or len(ocs2_hand_state.right_hand_position) != 0:
            control_hand_pub.publish(ocs2_hand_state)
        if len(ocs2_head_state.joint_data) != 0:
            control_head_pub.publish(ocs2_head_state)
        if len(ocs2_waist_state.data.data) != 0:
            control_waist_pub.publish(ocs2_waist_state)

def robot_status_timer_callback(event):
    """
    定时更新机器人状态的回调函数
    """
    update_robot_status_from_service()

def init_publishers():
    global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, control_waist_pub, update_h12_config_pub, update_joy_config_pub, load_map_pub
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
    control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
    control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)
    control_waist_pub = rospy.Publisher('/robot_waist_motion_data', robotWaistControl, queue_size=1, tcp_nodelay=True)
    update_h12_config_pub = rospy.Publisher('/update_h12_customize_config', UpdateH12CustomizeConfig, queue_size=1, tcp_nodelay=True)
    update_joy_config_pub = rospy.Publisher('/update_joy_customize_config', String, queue_size=1, tcp_nodelay=True)

    # 初始化地图订阅者
    print("Initializing map publisher...")
    init_map_publisher()

    # 初始化机器人位置发布者
    print("Initializing robot position publisher...")
    init_robot_position_publisher()

async def init_ros_node():
    print("Initializing ROS node")
    rospy.init_node("arm_action_server", anonymous=True, disable_signals=True)
    robot_plan_arm_state_topic_name = robot_settings[g_robot_type]["arm_traj_state_topic_name"]
    rospy.Subscriber(robot_plan_arm_state_topic_name, planArmState, plan_arm_state_callback)
    rospy.Subscriber('/bezier/arm_traj', JointTrajectory, traj_callback, queue_size=1, tcp_nodelay=True)
    # rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, mpc_obs_callback)
    rospy.Subscriber('/sensors_data_raw', sensorsData, sensors_data_callback, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/dexhand/state', JointState, robot_hand_callback, queue_size=1, tcp_nodelay=True)

    init_publishers()

    # Create a timer that calls timer_callback every 10ms (100Hz)
    rospy.Timer(rospy.Duration(0.01), timer_callback)

    # Create a timer that calls robot_status_timer_callback every 1 second to update robot status
    rospy.Timer(rospy.Duration(1.0), robot_status_timer_callback)

    print("ROS node initialized")

def create_bezier_request(action_data, start_frame_time, end_frame_time):
    req = planArmTrajectoryBezierCurveRequest()
    for key, value in action_data.items():
        msg = jointBezierTrajectory()
        for frame in value:
            point = bezierCurveCubicPoint()
            point.end_point, point.left_control_point, point.right_control_point = frame
            msg.bezier_curve_points.append(point)
        req.multi_joint_bezier_trajectory.append(msg)
    req.start_frame_time = start_frame_time
    req.end_frame_time = end_frame_time
    if robot_version.major() >= 4:
        req.joint_names = ["l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll", "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    else:
        req.joint_names = ["zarm_l1_link", "zarm_l2_link", "zarm_l3_link", "zarm_l4_link", "zarm_r1_link", "zarm_r2_link", "zarm_r3_link", "zarm_r4_link"]
    return req

def plan_arm_trajectory_bezier_curve_client(req):
    service_name = robot_settings[g_robot_type]["plan_arm_trajectory_bezier_service_name"]
    # Check if service exists
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
    except Exception as e:
        rospy.logerr(f"Service {service_name} not available")
        return False
    
    try:
        plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
        res = plan_service(req)
        return res.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def stop_plan_arm_trajectory_client():
    service_name = robot_settings[g_robot_type]["stop_plan_arm_trajectory_service_name"]
    
    # Check if service exists
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
    except Exception as e:
        rospy.logerr(f"Service {service_name} not available")
        return False
    
    try:
        if g_robot_type == "ocs2":
            stop_service = rospy.ServiceProxy(service_name, Trigger)
        else:
            stop_service = rospy.ServiceProxy(service_name, stopPlanArmTrajectory)

        stop_service()
        return
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
@dataclass
class Payload:
    cmd: str
    data: dict

@dataclass
class Response:
    payload: Any
    target: Union[str, websockets.WebSocketServerProtocol]

def plan_arm_state_callback(msg: planArmState):
    global plan_arm_state_progress, plan_arm_state_status
    plan_arm_state_progress = msg.progress
    plan_arm_state_status = msg.is_finished


def update_preview_progress(response: Response, stop_event: threading.Event):
    payload = response.payload
    update_interval = 0.001 
    global plan_arm_state_progress, plan_arm_state_status
    last_progress = None
    last_status = None
    
    while not stop_event.is_set():
        current_progress = plan_arm_state_progress
        current_status = plan_arm_state_status

        if current_progress != last_progress or current_status != last_status:
            last_progress = current_progress
            last_status = current_status
            
            payload.data["progress"] = current_progress
            payload.data["status"] = 0 if current_status else 1
            response_queue.put(response)
            
            if current_status:
                return
        
        time.sleep(update_interval)

async def websocket_message_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    cmd = data.get("cmd", "")
    cmd_handler = f"{cmd}_handler"
    if cmd_handler in globals() and callable(globals()[cmd_handler]):
        await globals()[cmd_handler](websocket, data)
    else:
        print(f"Unknown command: {cmd}")
        error_payload = Payload(
            cmd="error", data={"message": f"Unknown command: {cmd}"}
        )
        error_response = Response(payload=error_payload, target=websocket)
        response_queue.put(error_response)


async def broadacast_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    payload = Payload(
        cmd="broadcast",
        data={"code": 0, "message": "Broadcast message"},
    )
    response = Response(
        payload=payload,
        target="all",
    )

    response_queue.put(response)


async def preview_action_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global g_robot_type, active_threads
    print(g_robot_type)
    # Cancel existing thread for this client if it exists
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]

    payload = Payload(
        cmd="preview_action", data={"code": 0, "status": 0, "progress": 0}
    )

    data = data["data"]
    print(f"received message data: {data}")

    action_filename = data["action_filename"]
    global ACTION_FILE_FOLDER, current_arm_joint_state
    action_file_path = os.path.expanduser(f"{ACTION_FILE_FOLDER}/{action_filename}")
    if not os.path.exists(action_file_path):
        print(f"Action file not found: {action_file_path}")
        payload.data["code"] = 1
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return

    if calculate_file_md5(action_file_path) != data["action_file_MD5"]:
        print(f"Action file MD5 mismatch: {action_file_path}")
        payload.data["code"] = 2
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    
    if current_arm_joint_state is None:
        payload.data["code"] = 3
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    
    is_compatible, msg = verify_robot_version(action_file_path)
    if not is_compatible:
        payload.data["code"] = 4
        payload.data["message"] = msg
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    
    start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)

    if g_robot_type == "ocs2":
        action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
        end_frame_time += 1
        call_change_arm_ctrl_mode_service(2)
    else:
        action_frames = frames_to_custom_action_data(action_file_path)

    req = create_bezier_request(action_frames, start_frame_time, end_frame_time)
    if not plan_arm_trajectory_bezier_curve_client(req):
        payload.data["code"] = 4
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    time.sleep(0.5)
    # If valid, create initial response
    response = Response(
        payload=payload,
        target=websocket,
    )

    # Create a new stop event for this thread
    stop_event = threading.Event()
    active_threads[websocket] = stop_event
    print("---------------------add event-------------------------------")
    print(f"active_threads: {active_threads}")

    # Start a thread to update the progress
    thread = threading.Thread(
        target=update_preview_progress, args=(response, stop_event)
    )
    print("Starting thread to update progress")
    thread.start()

async def adjust_zero_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global zero_point_data, adjusted_zero_point_data,adjusted_zero_point_data_copy
    payload = Payload(
        cmd="adjust_zero_point", data={"code": 0, "message": "Zero point adjusted successfully"}
    )

    data = data["data"]
    print(f"received adjust zero point data: {data}")

    try:
        motor_index = data.get("motor_index")
        adjust_pos = data.get("adjust_pos")

        adjust_pos_rec = adjust_pos

        if motor_index is None or adjust_pos is None:
            payload.data["code"] = 1
            payload.data["message"] = "Missing required parameters motor_index or adjust_pos"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return


        # 如果是第一次调整，则复制原始零点数据作为调整的基础
        if len(adjusted_zero_point_data) == 0:
            adjusted_zero_point_data = zero_point_data.copy()

        # 前13个电机直接使用收到的值，之后的电机使用当前值减去上一次的值
        if motor_index >= 13:
            # 对于第13个及之后的电机，使用当前值减去上一次的值
            if motor_index < len(adjusted_zero_point_data):
                adjust_pos = adjust_pos - adjusted_zero_point_data[motor_index]
        # 前13个电机(索引0-12)直接使用收到的值，不需要做任何处理
        
        # 更新调整后的零点数据
        if motor_index < len(adjusted_zero_point_data):
            adjusted_zero_point_data[motor_index] = adjust_pos_rec

        adjusted_zero_point_data_copy = adjusted_zero_point_data.copy()

        # Check if nodelet_manager is running
        running_nodes = rosnode.get_node_names()
        if '/nodelet_manager' in running_nodes:

            # Check hardware ready status first
            rospy.wait_for_service('hardware/get_hardware_ready', timeout=0.5)
            get_hardware_ready = rospy.ServiceProxy('hardware/get_hardware_ready', Trigger)

            hw_res = get_hardware_ready()
            if hw_res.message == 'Hardware is ready':
            
                payload.data["code"] = 2
                payload.data["message"] = f"current robot is ready pose, cannot adjust motor zero point, please run `roslaunch humanoid_controllers load_kuavo_real.launch cali_set_zero:=true` to reboot robot"
                response = Response(payload=payload, target=websocket)
                response_queue.put(response)
                return
        
        rospy.wait_for_service('hardware/adjust_zero_point', timeout=0.5)
        adjust_zero_point = rospy.ServiceProxy('hardware/adjust_zero_point', adjustZeroPoint)
            # Create request
        req = adjustZeroPointRequest()
        req.motor_index = motor_index
        req.offset = adjust_pos
        
        # Call service
        res = adjust_zero_point(req)
        print(f'motor_index : {motor_index}, adjust_pos: {adjust_pos}')
        # return
        if not res.success:
            payload.data["code"] = 3
            payload.data["message"] = f"Failed to adjust zero point: {res.message}"


        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        payload.data["code"] = 4
        payload.data["message"] = f"Service call failed: {str(e)}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    except rospy.ROSException:
        print("Service adjust_zero_point not available")
        payload.data["code"] = 5
        payload.data["message"] = "Service adjust_zero_point not available"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return
    except Exception as e:
        print(f"Failed to check nodelet_manager status: {e}")
        payload.data["code"] = 6
        payload.data["message"] = f"Failed to check nodelet_manager status: {str(e)}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return


async def set_zero_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global adjusted_zero_point_data,frist_get_zero_point_flag
    payload = Payload(
        cmd="set_zero_point", data={"code": 0, "message": "Zero point set successfully"}
    )

    data = data["data"]
    print(f"received zero point data: {data}")
    
    arm_zero_file_path = os.path.expanduser(f"~/.config/lejuconfig/arms_zero.yaml")
    leg_zero_file_path = os.path.expanduser(f"~/.config/lejuconfig/offset.csv")
    
    # Get kuavo_assets package path
    kuavo_assets_path = rospkg.RosPack().get_path('kuavo_assets')
    robot_version_str = robot_version.to_string()
    with open(f"{kuavo_assets_path}/config/kuavo_v{robot_version_str}/kuavo.json", 'r') as f:
        json_config = json.load(f)
    
    arm_joints = json_config["NUM_ARM_JOINT"]
    head_joints = json_config["NUM_HEAD_JOINT"]
    if robot_version.major() == 1:
        waist_joints = json_config["NUM_WAIST_JOINT"]
    else:
        waist_joints = 0
    num_joints = json_config["NUM_JOINT"]
    ec_joints = num_joints - arm_joints - head_joints
    
    try:
        # 如果有调整后的零点数据，则使用它；否则使用传入的数据
        if len(adjusted_zero_point_data) > 0:
            zero_pos = adjusted_zero_point_data
        elif "zero_pos" in data:
            zero_pos = data["zero_pos"]
        else:
            payload.data["code"] = 1
            payload.data["message"] = "Invalid zero point data"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        arm_zero_data = zero_pos[ec_joints:]
        ec_zero_data = zero_pos[:ec_joints]

        # Backup the zero point files
        arm_zero_backup = f"{arm_zero_file_path}.bak"
        leg_zero_backup = f"{leg_zero_file_path}.bak"
        
        try:
            shutil.copy2(arm_zero_file_path, arm_zero_backup)
            shutil.copy2(leg_zero_file_path, leg_zero_backup)
        except Exception as e:
            print(f"Failed to backup zero point files: {e}")
            payload.data["code"] = 3 
            payload.data["message"] = f"Failed to backup zero point files: {str(e)}"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        with open(arm_zero_file_path, 'r') as f:
            arm_zero_data_origin = yaml.safe_load(f)
            arm_zero_data_origin_size = len(arm_zero_data_origin['arms_zero_position'])
        
        with open(leg_zero_file_path, 'r') as f:
            ec_zero_data_origin = f.read()
            ec_zero_data_origin = ec_zero_data_origin.split('\n')
            # -1 去掉末尾的换行
            if ec_zero_data_origin[-1] == '':
                ec_zero_data_origin_size = len(ec_zero_data_origin) - 1

        arm_zero_data = arm_zero_data + [0] * (arm_zero_data_origin_size - len(arm_zero_data))
        # Convert degrees to radians for arm zero data
        arm_zero_data = [math.radians(float(x)) for x in arm_zero_data]
        ec_zero_data = ec_zero_data + [0] * (ec_zero_data_origin_size - len(ec_zero_data))

        
        with open(arm_zero_file_path, 'w') as f:
            arm_zero_data_origin['arms_zero_position'] = arm_zero_data
            yaml.dump(arm_zero_data_origin, f)
    
        with open(leg_zero_file_path, 'w') as f:
            f.write('\n'.join(str(x) for x in ec_zero_data))
            f.write('\n')
    
        # 重置调整后的数据，为下一次调整做准备
        adjusted_zero_point_data = []
        frist_get_zero_point_flag = True
    
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
    except Exception as e:
        print(f"Error saving zero point file: {e}")
        payload.data["code"] = 2
        payload.data["message"] = f"Error saving zero point file: {str(e)}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)

async def get_zero_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    print(f"received get_zero_point data: {data}")

    global zero_point_data, adjusted_zero_point_data_copy, frist_get_zero_point_flag
    payload = Payload(
        cmd="get_zero_point", data={"code": 0, "message": "Zero point retrieved successfully"}
    )

    # Get kuavo_assets package path
    kuavo_assets_path = rospkg.RosPack().get_path('kuavo_assets')
    robot_version_str = robot_version.to_string()
    with open(f"{kuavo_assets_path}/config/kuavo_v{robot_version_str}/kuavo.json", 'r') as f:
        json_config = json.load(f)
    
    arm_joints = json_config["NUM_ARM_JOINT"]
    head_joints = json_config["NUM_HEAD_JOINT"]
    if robot_version.major() == 1:
        waist_joints = json_config["NUM_WAIST_JOINT"]
    else:
        waist_joints = 0
    num_joints = json_config["NUM_JOINT"]
    num_joints = json_config["NUM_JOINT"]
    ec_joints = num_joints - arm_joints - head_joints

    arm_zero_file_path = os.path.expanduser(f"~/.config/lejuconfig/arms_zero.yaml")
    leg_zero_file_path = os.path.expanduser(f"~/.config/lejuconfig/offset.csv")

    print("Please make sure the arms_zero.yaml and offset.csv files exist.")
    
    if not os.path.exists(arm_zero_file_path) or not os.path.exists(leg_zero_file_path):
        payload.data["code"] = 1
        payload.data["message"] = f"Zero point file not found: {arm_zero_file_path} or {leg_zero_file_path}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return

    try:
        print("Reading zero point data from file")

        # Read the zero point data from file
        with open(arm_zero_file_path, 'r') as file:
            arm_zero_data = yaml.safe_load(file)

        arm_zero_data = arm_zero_data['arms_zero_position']
        arm_zero_data = arm_zero_data[:arm_joints]
        # Convert arm zero data from radians to degrees
        arm_zero_data = [math.degrees(float(x)) for x in arm_zero_data]

        with open(leg_zero_file_path, 'r') as file: 
            leg_zero_data = file.read()
        leg_zero_data = [float(i) for i in leg_zero_data.split('\n') if i]
        leg_zero_data = leg_zero_data[:ec_joints]
        print(leg_zero_data + arm_zero_data)

        
        if frist_get_zero_point_flag :

            frist_get_zero_point_flag = False

            zero_point_data = leg_zero_data + arm_zero_data
            adjusted_zero_point_data_copy = zero_point_data.copy()

            payload.data["zero_pos"] = zero_point_data
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
        else :
            payload.data["zero_pos"] = adjusted_zero_point_data_copy
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)

        
    except Exception as e:
        print(f"Error reading zero point file: {e}")
        payload.data["code"] = 2
        payload.data["message"] = f"Error reading zero point file: {str(e)}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        
async def stop_preview_action_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]
    
    payload = Payload(
        cmd="stop_preview_action", data={"code":0}
    )
    response = Response(
        payload=payload,
        target=websocket,
    )
    stop_plan_arm_trajectory_client()
    response_queue.put(response)


async def get_robot_info_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):

    # 从参数服务器获取版本号，如果不存在则使用全局 robot_version
    try:
        robot_version_param = rospy.get_param('robot_version')
        if RobotVersion.is_valid(robot_version_param):
            robot_version_info = RobotVersion.create(robot_version_param)
        else:
            robot_version_info = robot_version
    except:
        robot_version_info = robot_version
    # 获取VR录制保存路径
    vr_recording_path = os.path.expanduser(vr_manager.RECORDING_SAVE_PATH)
    payload = Payload(
        cmd="get_robot_info",
        data={
            "code": 0,
            "robot_type": robot_version_info.version_number(),
            "music_folder_path": MUSIC_FILE_FOLDER,
            "maps_folder_path": MAP_FILE_FOLDER,
            "h12_config_path": H12_CONFIG_PATH,
            "repo_path": REPO_PATH,
            "vr_recording_path": vr_recording_path,
            "workspace_setup_path": os.path.join(KUAVO_ROS_CONTROL_WS_PATH, "devel", "setup.bash")
        }
    )

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

# 传入脚本名称或者脚本路径来运行目标脚本。
async def execute_python_script_handler(
        websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="execute_python_script", data={"code": 0, "message": "Python script executed successfully"}
    )

    print(f"received execute_python_script data: {data}")
    data = data["data"]
    action_data = data.get("action_data")
    scripts_name = data.get("scripts_name")
    if action_data == "start_action":
        try:
            if not scripts_name:
                payload.data["code"] = 1
                payload.data["message"] = "scripts_name parameter not provided"
            else:
                # 如果scripts_name是绝对路径，直接使用；否则认为是相对UPLOAD_FILES_FOLDER目录的路径
                # 处理脚本路径中的"~"等转义符
                scripts_name_expanded = os.path.expanduser(scripts_name)
                if os.path.isabs(scripts_name_expanded):
                    script_path = scripts_name_expanded
                else:
                    # 默认脚本目录为UPLOAD_FILES_FOLDER
                    global UPLOAD_FILES_FOLDER
                    script_path = os.path.join(UPLOAD_FILES_FOLDER, scripts_name_expanded)
                print(f"script_path: {script_path}")
                if not os.path.isfile(script_path):
                    payload.data["code"] = 2
                    payload.data["message"] = f"Script file does not exist: {script_path}"
                else:
                    # 后台执行python脚本
                    try:
                        # 使用subprocess.Popen后台执行，不等待脚本结束
                        process = subprocess.Popen(
                            ["python3", script_path],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True,
                            encoding='utf-8'
                        )
                        payload.data["code"] = 0
                        payload.data["message"] = f"Python script started in background: {scripts_name}"
                    except Exception as e:
                        payload.data["code"] = 3
                        payload.data["message"] = f"Failed to start Python script in background: {scripts_name}, error: {str(e)}"
        except Exception as e:
            payload.data["code"] = 4
            payload.data["message"] = f"Exception occurred while executing script: {str(e)}"
    elif action_data == "stop_action":
        # 查找scripts_name对应的进程并kill掉
        try:
            if not scripts_name:
                payload.data["code"] = 1
                payload.data["message"] = "scripts_name parameter not provided, cannot stop process"
            else:
                # 处理脚本路径中的"~"等转义符
                scripts_name_expanded = os.path.expanduser(scripts_name)
                # 只取文件名部分，防止路径影响
                script_basename = os.path.basename(scripts_name_expanded)
                # 使用pgrep查找所有python3进程中包含该脚本名的进程
                # 只查找python3进程，避免误杀
                # 获取所有python3进程
                ps = subprocess.Popen(['ps', '-eo', 'pid,cmd'], stdout=subprocess.PIPE, text=True)
                output, _ = ps.communicate()
                killed = False
                for line in output.splitlines():
                    if 'python3' in line and script_basename in line:
                        try:
                            pid = int(line.strip().split()[0])
                            if pid == os.getpid():
                                continue  # 不杀掉自己
                            os.kill(pid, signal.SIGTERM)
                            killed = True
                            print(f"Killed process: {pid} ({line})")
                        except Exception as e:
                            print(f"Failed to kill process: {e}")
                if killed:
                    payload.data["code"] = 0
                    payload.data["message"] = f"Tried to kill all python3 processes containing script name {script_basename}"
                else:
                    payload.data["code"] = 2
                    payload.data["message"] = f"No python3 process found containing script name {script_basename}"
        except Exception as e:
            payload.data["code"] = 3
            payload.data["message"] = f"Exception occurred while stopping script process: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def get_robot_status_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    payload = Payload(
        cmd="get_robot_status", data={"code": 0, "isrun": True}
    )

    if not active_threads:
        payload.data["isrun"] = False
        print(f"No activate threads: {active_threads}")
    else:
        for key, value in active_threads.items():
            print(f"Key: {key}, Value: {value}")

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


async def run_node_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]

    payload = Payload(
        cmd="run_node", data={"code": 0, "msg": "msg"}
    )

    data = data["data"]
    print(f"received message data: {data}")
    execute_path = data["path"]

    response = Response(
        payload=payload,
        target=websocket,
    )

    # Create a new stop event for this thread
    stop_event = threading.Event()

    # Start a thread to update the progress
    thread = threading.Thread(
        target=execute_command_progress, args=(websocket, response, stop_event, execute_path, False, None)  # 明确传递None作为parameters
    )
    print("Starting thread to update progress")
    thread.start()


def execute_command_progress(websocket: websockets.WebSocketServerProtocol, response: Response, stop_event: threading.Event, execute_path, use_ros_env: bool = False, parameters: dict = None):
    global active_threads
    payload = response.payload
    update_interval = 0.001 

    if not os.path.exists(execute_path):
        payload.data["code"] = 1
        payload.data["msg"] = "File not found."
        response_queue.put(response)
        return

    py_exe = sys.executable
    command_list = [py_exe, execute_path]
    
    if parameters:
        for key, value in parameters.items():
            if isinstance(value, bool):
                if value:
                    command_list.append(f"{key}")
                    command_list.append(str(value))
            else:
                command_list.append(f"{key}")
                command_list.append(str(value))
    
    # 当需要加载 ros 环境时，动态查找工作空间路径
    if use_ros_env:
        # 获取当前脚本所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 向上查找直到找到包含devel/setup.bash的目录
        workspace_path = None
        search_dir = current_dir
        while search_dir != '/':
            setup_bash_path = os.path.join(search_dir, 'devel', 'setup.bash')
            if os.path.exists(setup_bash_path):
                workspace_path = search_dir
                break
            search_dir = os.path.dirname(search_dir)
        
        if workspace_path:
            # 使用bash -c来执行source命令和Python脚本
            setup_bash_path = os.path.join(workspace_path, 'devel', 'setup.bash')
            if parameters:
                args_str = ' '.join(command_list[2:])
                command_list = ['bash', '-c', f'source {setup_bash_path} && {py_exe} {execute_path} {args_str}']
            else:
                command_list = ['bash', '-c', f'source {setup_bash_path} && {py_exe} {execute_path}']
            print(f"Found workspace at: {workspace_path}")
        else:
            print("Warning: Could not find ROS workspace")
    
    print(f"Executing command: {command_list}")
    global process
    try: 
        env = os.environ.copy()
        process = subprocess.Popen(command_list, env=env)
    except Exception as e:
        print("An error occurred while trying to execute the command:")
        print(e)
        payload.data["code"] = 1
        payload.data["msg"] = "Command execution failed."
        response_queue.put(response)
        print("Command execution failed.")   
    else:
        active_threads[websocket] = stop_event
        payload.data["code"] = 0
        payload.data["msg"] = "Command executed successfully."
        response_queue.put(response)
        
    while not stop_event.is_set():
        time.sleep(update_interval)

def monitor_and_stop(process):
    global should_stop, terminate_process_result
    terminate_process_result = False
    while True:
        time.sleep(1)
        if should_stop:
            print("Kill the target process")
            process.terminate()
            try:
                process.wait(timeout=3)
                terminate_process_result = True

                current_dir = os.path.dirname(os.path.abspath(__file__))
                while current_dir != '/':
                    # Check if this directory contains the标志性 files of kuavo-ros-control workspace
                    if os.path.exists(os.path.join(current_dir, 'src')) and \
                            os.path.exists(os.path.join(current_dir, 'devel/setup.bash')):
                        KUAVO_ROS_CONTROL_WS_PATH = current_dir
                        break
                    current_dir = os.path.dirname(current_dir)

                # 将选定代码中的硬编码路径替换为使用 current_dir 变量
                try:
                    subprocess.run([
                        "bash", "-c",
                        f"cd {current_dir} && "
                        "source /home/lab/.bashrc && "
                        "source /opt/ros/noetic/setup.bash && "
                        f"source {current_dir}/devel/setup.bash && "
                        "rostopic pub --once /robot_head_motion_data kuavo_msgs/robotHeadMotionData 'joint_data: [0,0]'"
                    ], shell=False, timeout=1)  # 最多等 1 秒
                except subprocess.TimeoutExpired:
                    print("Error: rostopic pub timed out. Is roscore running?")

            except subprocess.TimeoutExpired:
                print("Forced kill the target process")
                process.kill()
                try:
                    process.wait(timeout=2)
                    terminate_process_result = True
                except subprocess.TimeoutExpired:
                    terminate_process_result = False
            break

async def stop_run_node_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]
    
    payload = Payload(
        cmd="stop_run_node", data={"code":0}
    )

    global process, should_stop, terminate_process_result
    monitor_thread = threading.Thread(target=monitor_and_stop, args=(process,))
    monitor_thread.start()
    should_stop = True
    monitor_thread.join()
    print("terminate_process_result: ", terminate_process_result)
    
    if not terminate_process_result:
        payload.data["code"] = 1

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)
def is_player_in_body():
    # 检查下位机是否有音频播放设备
    result = False
    try:
        result_headphone = subprocess.run("pactl list | grep -i Headphone", shell=True, capture_output=True, text=True)
        result_speaker = subprocess.run("pactl list | grep -i Speaker", shell=True, capture_output=True, text=True)
        result_audio = subprocess.run("sudo aplay -l | grep -i Audio", shell=True, capture_output=True, text=True)
        if result_headphone.stdout.strip() or result_speaker.stdout.strip() or result_audio.stdout.strip():
            # 下位机有音频设备
            result = True
    except Exception as e:
        print(f"An error occurred while checking the music path: {e}")
    
    return result

def upload_music_file(music_filename=""):
    # 将音乐文件上传到上位机指定路径
    # TODO: 上位机是 AGX 用户名为 leju_kuavo 但这种情况下音频设备在下位机，不需要拷贝文件

    result = subprocess.run(['systemctl', 'is-active', 'isc-dhcp-server'])
    if result.returncode == 0:
        remote_host = "192.168.26.12"
    else:
        remote_host = "192.168.26.1"

    remote_user = "kuavo"
    remote_path = "/home/kuavo/.config/lejuconfig/"

    encoded_password = os.getenv("KUAVO_REMOTE_PASSWORD")
    if encoded_password is None:
        raise ValueError("Failed to get remote password.")
    remote_password = base64.b64decode(encoded_password).decode('utf-8')

    if not music_filename:
        # 将全部音频文件拷贝过去, scp 命令中添加 -o StrictHostKeyChecking=no 参数，跳过主机验证，防止传输失败
        scp_cmd = ["scp", "-r", "-o", "StrictHostKeyChecking=no", MUSIC_FILE_FOLDER]
    else:
        # 拷贝单个音频文件
        scp_cmd = ["scp", "-o", "StrictHostKeyChecking=no", MUSIC_FILE_FOLDER, music_filename]
    cmd = [
        "sshpass", "-p", remote_password,
        *scp_cmd,
        f"{remote_user}@{remote_host}:{remote_path}"
    ]
    # print(f"cmd: {cmd}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    return result
    


async def check_music_path_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="check_music_path", data={"code": 0, "msg": "msg"}
    )

    is_reset_cmd = data.get("is_reset_cmd", False)
    music_filename = data.get("music_filename", "")

    try:
        if is_player_in_body():
            # 下位机有音频设备，无需额外操作
            payload.data["code"] = 0
            payload.data["msg"] = "Body NUC"
        else:
            # 下位机没有音频设备，需要将音频文件拷贝到上位机
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            import asyncio
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, upload_music_file, music_filename)
            if result.returncode == 0:
                payload.data["code"] = 0
                payload.data["msg"] = "Head NUC"
            else:
                payload.data["code"] = 1
                payload.data["msg"] = "Failed to copy music file."
    except Exception as e:
        print(f"An error occurred while checking the music path: {e}")
        payload.data["code"] = 1
        payload.data["msg"] = e
    
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def update_h12_config_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global update_h12_config_pub, update_joy_config_pub
    payload = Payload(
        cmd="update_h12_config", data={"code": 0, "msg": "msg"}
    )

    # 更新H12遥控器按键配置
    if robot_version.major() >= 4:
        msg = UpdateH12CustomizeConfig()
        msg.update_msg = "Update H12 Config"
        update_h12_config_pub.publish(msg)
    elif robot_version.major() == 1:
        msg = String()
        msg.data = "Update Joy Config"
        update_joy_config_pub.publish(msg)
    
    try:
        # 确认音乐文件在上位机还是下位机播放，并进行相应处理
        if is_player_in_body():
            payload.data["code"] = 0
            payload.data["msg"] = "Body NUC"
        else:
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            import asyncio
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, upload_music_file, "")
            if result.returncode == 0:
                payload.data["code"] = 0
                payload.data["msg"] = "Head NUC"
            else:
                payload.data["code"] = 1
                payload.data["msg"] = "Failed to copy music file."
    except Exception as e:
        print(f"An error occurred while updating the H12 config: {e}")
        payload.data["code"] = 1
        payload.data["msg"] = e
    
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

# 登录到上位机上面执行下载指令
def download_data_pilot_to_head():
    # TODO: 上位机是 AGX 用户名为 leju_kuavo 但这种情况下音频设备在下位机，不需要拷贝文件

    result = subprocess.run(['systemctl', 'is-active', 'isc-dhcp-server'])
    if result.returncode == 0:
        remote_host = "192.168.26.12"
    else:
        remote_host = "192.168.26.1"

    remote_user = "leju_kuavo"
    remote_path = "/home/leju_kuavo/kuavo_data_pilot/src/kuavo_data_pilot_bin/dist/app"
    encoded_password = os.getenv("KUAVO_REMOTE_PASSWORD")
    if encoded_password is None:
        raise ValueError("Failed to get remote password.")
    remote_password = base64.b64decode(encoded_password).decode('utf-8')

    # 远程登录到上位机，执行下载 https://kuavo.lejurobot.com/kuavo_data_pilot_app/kuavo_data_pilot_app_latest 文件
    ssh_cmd = [
        "sshpass", "-p", remote_password,
        "ssh", f"{remote_user}@{remote_host}",
        "mkdir", "-p", remote_path,
        "&&", "cd", remote_path,
        "&&", "wget", "-O", "kuavo_data_pilot_app_latest", "https://kuavo.lejurobot.com/kuavo_data_pilot_app/kuavo_data_pilot_app_latest",
        "&&", "chmod", "+x", "kuavo_data_pilot_app_latest"
    ]
    result = subprocess.run(ssh_cmd, stdout=None, stderr=None)
    return result

# 更新训练场上位机 agx 程序
async def update_data_pilot_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="update_data_pilot", data={"code": 0, "msg": "msg"}
    )

    response = Response(
        payload=payload,
        target=websocket,
    )

    try:
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        import asyncio
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, download_data_pilot_to_head)
        if result.returncode == 0:
            payload.data["code"] = 0
            payload.data["msg"] = "Success"
        else:
            payload.data["code"] = 1
            payload.data["msg"] = "Failed to download data pilot to head."
    except Exception as e:
        print(f"An error occurred while updating the data pilot: {e}")
        payload.data["code"] = 1
        payload.data["msg"] = f"Error occurred while updating the data pilot: {str(e)}"

    response_queue.put(response)


def get_music_list():
    # 指定路径下指定格式的文件列表
    music_list = []

    # 检查下位机是否有音频播放设备
    if is_player_in_body():
        music_folder = os.path.expanduser("/home/lab/.config/lejuconfig/music")

        for root, dirs, files in os.walk(music_folder):
            for file in files:
                if file.endswith(".wav") or file.endswith(".mp3"):
                    # 使用 os.path.join 规范路径拼接
                    full_path = os.path.join(music_folder, file)
                    # 确保UTF-8编码格式
                    music_list.append(full_path.encode('utf-8').decode('utf-8'))

    else:
        result = subprocess.run(['systemctl', 'is-active', 'isc-dhcp-server'])
        if result.returncode == 0:
            remote_host = "192.168.26.12"
        else:
            remote_host = "192.168.26.1"


        remote_user = "kuavo"
        remote_path = "/home/kuavo/.config/lejuconfig/music"
        encoded_password = os.getenv("KUAVO_REMOTE_PASSWORD")

        if encoded_password is None:
            raise ValueError("Failed to get remote password.")
        remote_password = base64.b64decode(encoded_password).decode('utf-8')

        # 获取远程路径下的音乐列表
        ssh_cmd = (
            f"sshpass -p '{remote_password}' ssh {remote_user}@{remote_host} "
            f"'cd \"{remote_path}\" && find . -maxdepth 1 -type f \\( -name \"*.mp3\" -o -name \"*.wav\" \\) -printf \"%P\\n\"'"
        )

        result = subprocess.run(
            ssh_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True
        )

        if result.returncode == 0:
            # 成功获取远程音乐文件列表
            music_list = result.stdout.strip().split('\n') if result.stdout.strip() else []
            music_list = [os.path.join(remote_path, file) for file in music_list]
        else:
            music_list = []

    return music_list

async def get_music_list_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    music_list = get_music_list()
    payload = Payload(
        cmd="get_music_list", data={"code": 0, "music_list": music_list}

    )
    response = Response(
        payload=payload,
        target=websocket,
    )

    response_queue.put(response)

async def execute_demo_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]

    payload = Payload(
        cmd="execute_demo", data={"code": 0, "msg": "Demo execution started"}
    )

    data = data["data"]
    print(f"received demo execution data: {data}")
    demo_name = data["demo_name"]
    parameters = data.get("parameters", {})
    
    # 构建demo脚本路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建demo脚本路径：demo_name_demo/main.py
    execute_path = os.path.join(current_dir, f"{demo_name}_demo", "main.py")

    response = Response(
        payload=payload,
        target=websocket,
    )

    # 创建新的停止事件
    stop_event = threading.Event()

    thread = threading.Thread(
        target=execute_command_progress, 
        args=(websocket, response, stop_event, execute_path, True, parameters)
    )
    print("Starting thread to execute demo")
    thread.start()

async def stop_execute_demo_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]
    
    payload = Payload(
        cmd="stop_execute_demo", data={"code":0}
    )

    global process, should_stop, terminate_process_result
    monitor_thread = threading.Thread(target=monitor_and_stop, args=(process,))
    monitor_thread.start()
    should_stop = True
    monitor_thread.join()
    print("terminate_process_result: ", terminate_process_result)
    
    if not terminate_process_result:
        payload.data["code"] = 1
        payload.data["msg"] = "Failed to stop demo execution"

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def load_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]
    payload = Payload(
        cmd="load_map", data={"code":0}
    )
    data = data["data"]
    map_name = data["map_name"]
    try:
        service_name = '/load_map'
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        loop = asyncio.get_event_loop()

        # 等待服务
        await loop.run_in_executor(None, rospy.wait_for_service, service_name, 3.0)
        load_map_client = rospy.ServiceProxy(service_name, LoadMap)

        # request
        req = LoadMapRequest()
        req.map_name = map_name

        # response - 在后台线程执行服务调用
        res = await loop.run_in_executor(None, load_map_client, req)
        if res.success:
            if await download_map_file(map_name):
                payload.data["code"] = 0
                payload.data["msg"] = "Map loaded successfully"
                payload.data["map_path"] = MAP_FILE_FOLDER +"/"+ map_name+".png"
            else:
                payload.data["code"] = 1
                payload.data["msg"] = "Map loaded successfully, but failed to download map file"

    except rospy.ServiceException as e:
        payload.data["code"] = 2
        payload.data["msg"] = f"Service `load_map` call failed: {e}"
    except rospy.ROSException as e:
        payload.data["code"] = 2
        payload.data["msg"] = f"Service `load_map` call failed: {e}"
    except Exception as e:
        payload.data["code"] = 2
        payload.data["msg"] = f"Service `load_map` call failed: {e}"
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)
    
async def init_localization_by_pose_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="init_localization_by_pose", data={"code":0}
    )

    data = data["data"]
    print(f"received init localization by pose data: {data}")
    x = data["x"]
    y = data["y"]
    z = data["z"]
    roll = data["roll"]
    pitch = data["pitch"]
    yaw = data["yaw"]
    
    try:
        service_name = 'set_initialpose'
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        loop = asyncio.get_event_loop()

        # 等待服务
        await loop.run_in_executor(None, rospy.wait_for_service, service_name, 3.0)
        init_localization_by_pose_client = rospy.ServiceProxy(service_name, SetInitialPose)

        # 将 roll pitch yaw 转换为四元数
        orientation = tf.transformations.quaternion_from_euler(yaw, pitch, roll)

        # request
        req = SetInitialPoseRequest()
        req.initial_pose.header.frame_id = "map"
        req.initial_pose.header.stamp = rospy.Time.now()
        req.initial_pose.pose.pose.position.x = x
        req.initial_pose.pose.pose.position.y = y
        req.initial_pose.pose.pose.position.z = z
        req.initial_pose.pose.pose.orientation.x = orientation[0]
        req.initial_pose.pose.pose.orientation.y = orientation[1]
        req.initial_pose.pose.pose.orientation.z = orientation[2]
        req.initial_pose.pose.pose.orientation.w = orientation[3]

        # response - 在后台线程执行服务调用
        res = await loop.run_in_executor(None, init_localization_by_pose_client, req)
        if res.success:
            payload.data["code"] = 0
            payload.data["msg"] = "Localization initialized successfully"
        else:
            payload.data["code"] = 1
            payload.data["msg"] = "Failed to initialize localization"
    except rospy.ServiceException as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Service `init_localization_by_pose` call failed: {e}"
    except rospy.ROSException as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Service `init_localization_by_pose` call failed: {e}"  
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def download_map_file(map_name: str = ""):
    """
    异步订阅/map话题，将地图数据保存为png格式图片
    使用 asyncio.run_in_executor 避免阻塞 WebSocket 事件循环
    """
    global MAP_FILE_FOLDER
    try:
        # 在后台线程执行 rospy.wait_for_message，避免阻塞事件循环
        loop = asyncio.get_event_loop()
        msg = await loop.run_in_executor(None, rospy.wait_for_message, '/map', OccupancyGrid)

        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 映射到图像灰度：0=白色，100=黑色，-1=灰色
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 128
        img[data == 0] = 255
        img[data == 100] = 0
        # 可选：线性插值灰度（处理 0~100 范围的值）
        mask = (data > 0) & (data < 100)
        img[mask] = 255 - (data[mask] * 255 // 100)

        # OpenCV 默认原点在左上，而 ROS 地图原点通常在左下 → 上下翻转图像
        img = cv2.flip(img, 0)

        # 保存为 PNG
        cv2.imwrite(MAP_FILE_FOLDER+"/"+map_name+".png", img)
        rospy.loginfo(f"地图已保存为 {MAP_FILE_FOLDER}/{map_name}.png")
        return True
    except Exception as e:
        rospy.logerr(f"Failed to download map file: {e}")
        return False


async def start_robot_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    启动机器人（缩腿）
    """
    global robot_status
    payload = Payload(
        cmd="start_robot",
        data={"code": 0, "message": "Robot started successfully"}
    )
    
    try:
        # 直接调用本地实现
        result, msg = robot_instance.start_robot()
        if result:
            payload.data["code"] = 0
            payload.data["message"] = msg
            # 更新机器人状态为"crouching"
            robot_status = "crouching"
        else:
            payload.data["code"] = 1
            payload.data["message"] = msg
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to start robot: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def set_align_stair_param_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    通过识别 tag 码设置当前的偏置参数
    """
    payload = Payload(
        cmd="set_align_stair_param",
        data={"code": 0, "message": "Align stair param set successfully"}
    )
    
    try:    
        target_tag_id = 66     # 固定使用 tag_id=10 的 tag 码进行识别
        timeout_duration = 30  # 总超时时间：30秒
        start_time = time.time()
        tag_found = False
        
        while True:
            # 检查是否超时
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout_duration:
                payload.data["code"] = 1
                payload.data["message"] = f"timeout: {timeout_duration} seconds, tag_id: {target_tag_id}"
                break
            
            try:
                # 等待从话题 "/robot_tag_info" 接收到 AprilTagDetectionArray 消息
                msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=2)
                
                if not msg.detections:
                    continue
                
                # 查找目标tag
                target_detection = None
                for detection in msg.detections:
                    if detection.id[0] == target_tag_id:
                        target_detection = detection
                        break
                
                if target_detection is None:
                    continue
                
                # 找到目标tag
                pos = target_detection.pose.pose.pose.position
                offset_x = -pos.x
                offset_y = -pos.y
                
                # 将识别结果保存到文件（参考 simple-singleStepStairs-roban-21.py 的保存逻辑）
                try:
                    from datetime import datetime
                    
                    # 确保上传文件夹存在
                    global UPLOAD_FILES_FOLDER
                    if not os.path.exists(UPLOAD_FILES_FOLDER):
                        os.makedirs(UPLOAD_FILES_FOLDER)
                    
                    # 构建配置数据（与参考文件保持一致的数据结构）
                    config = {
                        'tag_id': target_tag_id,
                        'offset_x': float(offset_x),
                        'offset_y': float(offset_y),
                        'offset_yaw': 0.0,
                        'offset_set': True
                    }
                    
                    # 1. 保存到固定文件名（用于后续程序读取的最新配置）
                    config_filename = 'stair_alignment_config.json'
                    config_filepath = os.path.join(UPLOAD_FILES_FOLDER, config_filename)
                    with open(config_filepath, 'w', encoding='utf-8') as f:
                        json.dump(config, f, indent=4, ensure_ascii=False)
                    
                    payload.data["code"] = 0
                    payload.data["message"] = "Align stair param set successfully"
                    
                except Exception as save_error:
                    payload.data["code"] = 1
                    payload.data["message"] = f"Failed to set align stair param: {save_error}"
                break
            except rospy.ROSException:
                continue
            except Exception as e:
                payload.data["code"] = 1
                payload.data["message"] = f"Failed to set align stair param: {e}"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to set align stair param: {e}"

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def robot_switch_pose_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    机器人姿态切换处理函数，用于在站立和蹲下姿态之间切换
    支持传入字符参数：
    'ready': 缩腿
    'stand': 退出缩腿，全身绷直阻尼状态
    """
    payload = Payload(
        cmd="robot_switch_pose", data={"code": 0, "message": "Robot pose switch command sent successfully"}
    )

    try:
        # 获取传入的字符参数
        robot_pose = data.get("data", {}).get("robot_pose", "")
        print(f"Received robot_pose: {robot_pose}")
        
        # 根据传入的字符执行相应的操作
        if robot_pose == "cali_zero":
            # 调用机器人缩腿服务
            result, msg = robot_instance.switch_robot_pose(robot_pose)
            if result:
                payload.data["code"] = 0
                payload.data["message"] = f"零点模式执行成功: {msg}"
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"零点模式令执行失败: {msg}"
        elif robot_pose == "ready":
            # 调用机器人缩腿服务
            result, msg = robot_instance.switch_robot_pose(robot_pose)
            if result:
                payload.data["code"] = 0
                payload.data["message"] = f"缩腿命令执行成功: {msg}"
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"缩腿命令执行失败: {msg}"
        elif robot_pose == "stand":
            # 调用机器人退出缩腿，全身绷直阻尼状态服务
            result, msg = robot_instance.switch_robot_pose(robot_pose)
            if result:
                payload.data["code"] = 0
                payload.data["message"] = f"退出缩腿命令执行成功: {msg}"
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"退出缩腿命令执行失败: {msg}"
        else:
            payload.data["code"] = 1
            payload.data["message"] = f"不支持的命令字符: {robot_pose}，仅支持 'ready' 或 'stand'"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"切换机器人姿态失败: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


async def stop_robot_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    停止机器人
    """
    global robot_status
    payload = Payload(
        cmd="stop_robot",
        data={"code": 0, "message": "Robot stopped successfully"}
    )
    
    try:
        # 直接调用本地实现
        result, msg = robot_instance.stop_robot()
        if result:
            payload.data["code"] = 0
            payload.data["message"] = msg
            # 更新机器人状态为"unlaunch"
            robot_status = "unlaunch"
        else:
            payload.data["code"] = 1
            payload.data["message"] = msg
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to stop robot: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def stand_robot_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    站立机器人（伸直腿）
    """
    global robot_status
    payload = Payload(
        cmd="stand_robot",
        data={"code": 0, "message": "Robot stand command sent successfully"}
    )
    
    try:
        # 直接调用本地实现
        result, msg = robot_instance.stand_robot()
        if result:
            payload.data["code"] = 0
            payload.data["message"] = msg
            # 更新机器人状态为"standing"
            robot_status = "standing"
        else:
            payload.data["code"] = 1
            payload.data["message"] = msg
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to stand robot: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

# 机器人零点调试模式
async def zero_point_debug_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    global robot_status,frist_get_zero_point_flag
    payload = Payload(
        cmd="zero_point_debug", data={"code": 0, "message": "Zero point debug mode entered successfully"}
    )

    
    try:
        zero_point_debug_status = data.get("data", {}).get("zero_point_debug_status", "")
    
        result, msg = robot_instance.zero_point_robot(zero_point_debug_status)
        
        if result:
            frist_get_zero_point_flag  = True
            payload.data["code"] = 0
            payload.data["message"] = msg
        else:
            payload.data["code"] = 1
            payload.data["message"] = msg
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to zero point robot: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


async def get_robot_launch_status_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    获取机器人当前状态
    """
    global robot_status
    payload = Payload(
        cmd="get_robot_launch_status",
        data={"code": 0, "status": robot_status, "message": "Get robot status successfully"}
    )
    
    try:
        # 从服务动态获取机器人状态
        update_robot_status_from_service()
        payload.data["code"] = 0
        payload.data["status"] = robot_status
        payload.data["message"] = "Get robot status successfully"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to get robot status: {e}"
        
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

MOVE_VOICE_KEYWORDS = {
  "前进一步": {
    "type": "SINGLE_STEP",
    "keywords": ["往前走", "往前走一步", "前进一步"],
    "data": {
      "direction": "前",
      "step": 1
    }
  },
  "后退一步": {
    "type": "SINGLE_STEP",
    "keywords": ["往后走", "往后走一步", "后退一步"],
    "data": {
      "direction": "后",
      "step": 1
    }
  },
  "左转一步": {
    "type": "SINGLE_STEP",
    "keywords": ["左转", "左转一步"],
    "data": {
      "direction": "左转",
      "step": 1
    }
  },
  "右转一步": {
    "type": "SINGLE_STEP",
    "keywords": ["右转", "右转一步"],
    "data": {
      "direction": "右转",
      "step": 1
    }
  },
  "左移一步": {
    "type": "SINGLE_STEP",
    "keywords": ["左移", "往左走", "左边走"],
    "data": {
      "direction": "左",
      "step": 1
    }
  },
  "右移一步": {
    "type": "SINGLE_STEP",
    "keywords": ["右移", "往右走", "右边走"],
    "data": {
      "direction": "右",
      "step": 1
    }
  }
}

def _merge_keywords_config_by_action_files(voice_keywords_config: dict, action_folder_path: str):
    """
    根据本地动作文件列表，重置关键词配置
    :param voice_keywords_config: 原始配置字典
    :param action_folder_path: 动作文件的绝对路径
    """
    
    # 获取动作文件列表
    action_files_pattern = os.path.join(action_folder_path, "*.tact")
    action_files = glob.glob(action_files_pattern)
    # 使用 os.path.basename 获取不带目录的文件名，再使用 os.path.splitext 安全去除文件后缀
    action_file_names = set(os.path.splitext(os.path.basename(f))[0] for f in action_files)
    rospy.loginfo(f"Found {action_file_names} action files in {action_folder_path}.")

    updated_config = {}
    
    # 1. 处理原始配置中的项
    for key, value in voice_keywords_config.items():
        if value.get("type") == "SINGLE_STEP":
            # 保留非动作文件类型的配置
            updated_config[key] = value
        elif value.get("type") == "ARM_ACTION":
            # 如果是 ARM_ACTION，检查文件是否存在
            if key in action_file_names:
                updated_config[key] = value
            else:
                # 文件不存在，记录日志（实现清理逻辑，不加入 updated_config 即为删除）
                rospy.logwarn(f"Removing '{key}' from config because the action file is missing.")
    
    # 2. 添加新发现的动作文件
    for action_name in action_file_names:
        # 如果配置里没有这个动作，则添加
        if action_name not in updated_config:
            # 再次检查：防止同名但类型为 SINGLE_STEP 的冲突
            if action_name in voice_keywords_config and voice_keywords_config[action_name].get("type") != "ARM_ACTION":
                rospy.logwarn(f"Skipping auto-add '{action_name}', name conflict with existing {voice_keywords_config[action_name]['type']}")
                continue
                
            updated_config[action_name] = {
                "type": "ARM_ACTION",
                "keywords": [],
                "data": action_name
            }
            rospy.loginfo(f"Auto-added new action file to config: {action_name}")

    return updated_config

async def get_voice_keywords_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    payload = Payload(
        cmd="get_voice_keywords",
        data={"code": 0, "message": "Read voice keywords successfully", "result": {}}
    )
    # 配置文件路径：voice_control_node/scripts/key_words.json

    VOICE_CONTROL_PKG_NAME = "voice_control_node"

    try:    

        voice_control_keywords_path = rospkg.RosPack().get_path(VOICE_CONTROL_PKG_NAME)+"/scripts"
        KEYWORDS_FILE_PATH = os.path.join(voice_control_keywords_path, "key_words.json")
        action_folder = os.path.expanduser("~/.config/lejuconfig/action_files")     
        
        # 1. 读取配置
        with open(KEYWORDS_FILE_PATH, "r") as f:
                voice_keywords_config = json.load(f)

        # 2. 同步更新逻辑
        merged_config = _merge_keywords_config_by_action_files(voice_keywords_config, action_folder)

        payload.data["result"] = merged_config
        rospy.loginfo(f"Read voice keywords successfully: {merged_config}")
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to read {KEYWORDS_FILE_PATH}: {e}"
        payload.data["result"] = {}
        rospy.logerr(f"Failed to read {KEYWORDS_FILE_PATH}: {e}")

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)

async def update_voice_keywords_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """
    更新语音关键词
    """
    payload = Payload(
        cmd="update_voice_keywords",
        data={"code": 0, "message": "Update voice keywords successfully"}
    )

    # 1.找配置文件

    VOICE_CONTROL_PKG_NAME = "voice_control_node"
    voice_control_keywords_path = rospkg.RosPack().get_path(VOICE_CONTROL_PKG_NAME)+"/scripts"
    KEYWORDS_FILE_PATH = os.path.join(voice_control_keywords_path, "key_words.json")
    
    # 2.解析参数，并更新配置文件
    voice_keywords_config = {}
    try:
        # 如果文件不存在，创建一个其父目录
        if not Path(KEYWORDS_FILE_PATH).is_file():
            
            os.makedirs(os.path.dirname(KEYWORDS_FILE_PATH), exist_ok=True)
            rospy.loginfo(f"Created new voice keywords config file: {KEYWORDS_FILE_PATH}")
    
        real_data = data.get("data", {})
        for action, keywords in real_data.items():
            # TODO 检查lejuconfig下的对应动作配置文件是否存在
            voice_keywords_config[action] = {
                "type": "ARM_ACTION",
                "keywords": keywords,
                "data": action
            }
        voice_keywords_config = {**MOVE_VOICE_KEYWORDS, **voice_keywords_config}
        with open(KEYWORDS_FILE_PATH, "w") as f:
            json.dump(voice_keywords_config, f, ensure_ascii=False, indent=2)

        # 如果文件更新成功，但是服务重载失败。依然算成功，因为下一次语音控制节点启动的时候，仍会加载到最新的配置文件
        try:
            rospy.wait_for_service('/voice_control/reload_keywords', timeout=2.0)
            reload_client = rospy.ServiceProxy('/voice_control/reload_keywords', Trigger)
            req = TriggerRequest()
            response = reload_client(req)
            if response.success:
                rospy.loginfo("已通知 voice_control_node 重载配置")
            else:
                rospy.logwarn("Failed to reload voice keywords.")
        except rospy.ROSException:
            rospy.logerr("Service /voice_control/reload_keywords not available")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        # Call the service
        payload.data["code"] = 0
        payload.data["message"] = "Update voice keywords successfully"
        rospy.loginfo(f"Updated voice keywords: {voice_keywords_config}")
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to update voice keywords: {e}."
        rospy.logwarn(f"Failed to update voice keywords: {e}.")

    # 返回转换后的配置文件数据
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


# Add a function to clean up when a websocket connection is closed
def cleanup_websocket(websocket: websockets.WebSocketServerProtocol):
    global active_threads
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]

# Add a new function to set the robot type
def set_robot_type(robot_type: str):
    global g_robot_type
    g_robot_type = robot_type

def set_folder_path(action_file_folder: str, upload_folder: str, map_file_folder: str):
    global ACTION_FILE_FOLDER, UPLOAD_FILES_FOLDER, MAP_FILE_FOLDER
    ACTION_FILE_FOLDER = action_file_folder
    UPLOAD_FILES_FOLDER = upload_folder
    MAP_FILE_FOLDER = map_file_folder


# ==================== 导航功能 WebSocket Handlers ====================

async def check_lidar_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     检测雷达功能
    检查雷达点云话题是否有数据：/livox/cloud
    返回 True 或 False
    """
    payload = Payload(
        cmd="check_lidar",
        data={"code": 0, "message": "Lidar check completed successfully"}
    )

    try:
        print("=== DEBUG: 开始检查雷达话题 /livox/cloud ===")

        # 检查/livox/cloud话题是否存在
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        loop = asyncio.get_event_loop()
        topics = await loop.run_in_executor(None, rospy.get_published_topics)

        lidar_topic_exists = any('/livox/cloud' == topic for topic, _ in topics)
        print(f"DEBUG: lidar_topic_exists = {lidar_topic_exists}")

        if not lidar_topic_exists:
            payload.data["code"] = 1
            payload.data["message"] = "Lidar topic /livox/cloud not found"
            payload.data["lidar_active"] = False
            print("DEBUG: 雷达话题不存在")
        else:
            print("DEBUG: 找到雷达话题，尝试等待消息...")
            # 尝试等待一次消息来确认是否有数据
            try:
                print("DEBUG: 调用 rospy.wait_for_message('/livox/cloud', PointCloud2, timeout=2.0)")
                # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
                loop = asyncio.get_event_loop()
                msg = await loop.run_in_executor(None, rospy.wait_for_message, '/livox/cloud', PointCloud2, 2.0)
                payload.data["lidar_active"] = True
                payload.data["message"] = "Lidar is active and receiving data"
                print(f"DEBUG: 雷达检查成功: 收到 {len(msg.data)} 字节数据")
            except rospy.ROSException as e:
                payload.data["code"] = 1
                payload.data["lidar_active"] = False
                payload.data["message"] = f"Lidar topic exists but no data received: {str(e)}"
                print(f"DEBUG: 雷达话题存在但无法接收数据，异常: {str(e)}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["lidar_active"] = False
        payload.data["message"] = f"Failed to check lidar: {str(e)}"
        print(f"DEBUG: 雷达检查异常: {str(e)}")
        import traceback
        traceback.print_exc()

    print("=== DEBUG: 雷达检查结束 ===")
    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def create_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     新建地图功能
    采用服务的方式接收地图名，启动 launch 来开始建图
    """
    payload = Payload(
        cmd="create_map",
        data={"code": 0, "message": "Map creation started successfully"}
    )

    try:
        # 获取地图名称
        map_name = data.get("data", {}).get("map_name", "")
        if not map_name:
            payload.data["code"] = 1
            payload.data["message"] = "Map name is required"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 调用上位机的建图启动服务
        print(f"Creating map: {map_name}")

        import subprocess  # 移到函数开始处导入

        try:
            # 调用上位机的建图启动服务
            service_name = '/start_mapping_service'
            rospy.wait_for_service(service_name, timeout=5.0)

            # 使用rosservice命令调用建图启动服务
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            import asyncio
            loop = asyncio.get_event_loop()

            cmd = f"rosservice call {service_name} '{{map_name: \"{map_name}\", lidar_type: \"livox\"}}'"
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            # 使用 lambda 包装以正确传递关键字参数
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, encoding='utf-8', timeout=30))

            if result.returncode == 0:
                payload.data["message"] = f"建图启动成功: {map_name}"
                payload.data["map_name"] = map_name
                print(f"Start mapping service call successful: {result.stdout}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"建图启动失败: {result.stderr}"
                print(f"Start mapping service failed: {result.stderr}")

        except (rospy.ServiceException, subprocess.TimeoutExpired) as e:
            payload.data["code"] = 1
            payload.data["message"] = f"无法连接到建图启动服务: {str(e)}"
            print(f"Failed to connect to start mapping service: {str(e)}")

        payload.data["map_name"] = map_name

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to create map: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def save_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    保存地图功能
    采用服务的方式接受信号，参考 save_map.sh 中的信号，来结束保存地图
    """
    payload = Payload(
        cmd="save_map",
        data={"code": 0, "message": "Map save command sent successfully"}
    )

    # 从请求数据中获取地图名，如果没有则使用空字符串
    map_name = ""
    if data and data.get("data") and data["data"].get("map_name"):
        map_name = data["data"]["map_name"]

    try:
        print(f"Saving current map with name: '{map_name}'...")

        # 调用上位机的保存地图服务
        import asyncio
        try:
            # 调用上位机的保存地图服务
            service_name = '/save_map_service'
            rospy.wait_for_service(service_name, timeout=5.0)

            # 使用rosservice命令调用保存地图服务
            # SaveMap服务需要一个map_name参数
            # 确保使用 UTF-8 编码环境，支持中文地图名
            env = os.environ.copy()
            env['LC_ALL'] = 'C.UTF-8'
            env['LANG'] = 'C.UTF-8'
            cmd = f'rosservice call {service_name} "map_name: \'{map_name}\'"'
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=env
            )
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=60)
            result_stdout = stdout.decode('utf-8') if stdout else ""
            result_stderr = stderr.decode('utf-8') if stderr else ""

            # 创建一个类似subprocess.run返回的对象
            class AsyncResult:
                def __init__(self, returncode, stdout, stderr):
                    self.returncode = returncode
                    self.stdout = stdout
                    self.stderr = stderr

            result = AsyncResult(proc.returncode, result_stdout, result_stderr)

            if result.returncode == 0:
                # 解析ROSService返回的YAML格式响应
                output_lines = result.stdout.strip().split('\n')
                map_file = ""
                success = False
                message = "地图保存成功"

                for line in output_lines:
                    if "success: True" in line:
                        success = True
                    elif "success: False" in line:
                        success = False
                    elif "message:" in line:
                        # 提取消息内容
                        message = line.split("message:", 1)[1].strip()
                    elif "map_path:" in line:
                        # 从map_path中提取文件名，例如从 "map_path: map_2024-01-15_14-30-25" 中提取
                        map_file = line.split("map_path:", 1)[1].strip().replace('"', '')

                if success:
                    payload.data["message"] = message
                    payload.data["map_file"] = map_file if map_file else "unknown"
                    payload.data["output"] = result.stdout
                    print(f"Save map service call successful: {message}")
                else:
                    payload.data["code"] = 1
                    payload.data["message"] = message
                    print(f"Save map service call failed: {message}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"地图保存失败: {result.stderr}"
                print(f"Save map service call failed: {result.stderr}")

        except (rospy.ServiceException, asyncio.TimeoutError) as e:
            payload.data["code"] = 1
            payload.data["message"] = f"无法连接到保存地图服务: {str(e)}"
            print(f"Failed to connect to save map service: {str(e)}")
   
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to save map: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def stop_mapping_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    结束建图功能
    """
    payload = Payload(
        cmd="stop_mapping",
        data={"code": 0, "message": "Mapping stop command sent successfully"}
    )

    try:
        print("Stopping mapping...")

        # 调用上位机停止建图服务
        import asyncio
        try:
            # 调用上位机的停止建图服务
            service_name = '/stop_mapping_service'
            rospy.wait_for_service(service_name, timeout=5.0)

            # 使用rosservice命令调用停止建图服务
            # StopMapping服务没有请求参数，所以直接调用
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            cmd = f"rosservice call {service_name}"
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, encoding='utf-8', timeout=30))

            if result.returncode == 0:
                # 解析ROSService返回的YAML格式响应
                output_lines = result.stdout.strip().split('\n')
                success = False
                message = "建图已停止"

                for line in output_lines:
                    if "success: True" in line:
                        success = True
                    elif "success: False" in line:
                        success = False
                    elif "message:" in line:
                        # 提取消息内容
                        message = line.split("message:", 1)[1].strip()

                if success:
                    payload.data["message"] = message
                    payload.data["output"] = result.stdout
                    print(f"Stop mapping service call successful: {message}")
                else:
                    payload.data["code"] = 1
                    payload.data["message"] = message
                    print(f"Stop mapping service call failed: {message}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"停止建图失败: {result.stderr}"
                print(f"Stop mapping service call failed: {result.stderr}")

        except (rospy.ServiceException, subprocess.TimeoutExpired, asyncio.TimeoutError) as e:
            payload.data["code"] = 1
            payload.data["message"] = f"无法连接到停止建图服务: {str(e)}"
            print(f"Failed to connect to stop mapping service: {str(e)}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to stop mapping: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def rename_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    重命名地图功能
    """
    payload = Payload(
        cmd="rename_map",
        data={"code": 0, "message": "Map rename command sent successfully"}
    )

    try:
        # 获取原地图名称和新地图名称
        old_name = data.get("data", {}).get("old_name", "")
        new_name = data.get("data", {}).get("new_name", "")

        if not old_name or not new_name:
            payload.data["code"] = 1
            payload.data["message"] = "原地图名称和新地图名称不能为空"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        print(f"Renaming map: {old_name} -> {new_name}")

        # 调用上位机的重命名地图服务
        import asyncio
        try:
            # 调用上位机的重命名地图服务
            service_name = '/rename_map_service'
            rospy.wait_for_service(service_name, timeout=5.0)

            # 使用rosservice命令调用重命名地图服务
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            cmd = f"rosservice call {service_name} '{{old_name: \"{old_name}\", new_name: \"{new_name}\"}}'"
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, encoding='utf-8', timeout=30))

            if result.returncode == 0:
                # 解析ROSService返回的YAML格式响应
                output_lines = result.stdout.strip().split('\n')
                success = False
                message = f"地图重命名成功: {old_name} -> {new_name}"

                for line in output_lines:
                    if "success: True" in line:
                        success = True
                    elif "success: False" in line:
                        success = False
                    elif "message:" in line:
                        # 提取消息内容
                        message = line.split("message:", 1)[1].strip()

                if success:
                    payload.data["message"] = message
                    payload.data["old_name"] = old_name
                    payload.data["new_name"] = new_name
                    print(f"Rename map service call successful: {message}")
                else:
                    payload.data["code"] = 1
                    payload.data["message"] = message
                    payload.data["old_name"] = old_name
                    payload.data["new_name"] = new_name
                    print(f"Rename map service call failed: {message}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"地图重命名失败: {result.stderr}"
                payload.data["old_name"] = old_name
                payload.data["new_name"] = new_name
                print(f"Rename map service call failed: {result.stderr}")

        except (rospy.ServiceException, subprocess.TimeoutExpired, asyncio.TimeoutError) as e:
            payload.data["code"] = 1
            payload.data["message"] = f"无法连接到重命名地图服务: {str(e)}"
            payload.data["old_name"] = old_name
            payload.data["new_name"] = new_name
            print(f"Failed to connect to rename map service: {str(e)}")


    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to rename map: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def delete_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    删除地图功能
    """
    payload = Payload(
        cmd="delete_map",
        data={"code": 0, "message": "删除地图命令发送成功"}
    )

    try:
        # 获取要删除的地图名称
        map_name = data.get("data", {}).get("map_name", "")

        if not map_name:
            payload.data["code"] = 2
            payload.data["message"] = "地图名称不能为空"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        print(f"Deleting map: {map_name}")

        # 调用上位机的删除地图服务
        import asyncio
        try:
            # 调用上位机的删除地图服务
            service_name = '/delete_map_service'
            rospy.wait_for_service(service_name, timeout=5.0)

            # 使用rosservice命令调用删除地图服务
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            cmd = f"rosservice call {service_name} '{{map_name: \"{map_name}\"}}'"
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, encoding='utf-8', timeout=30))

            if result.returncode == 0:
                # 解析ROSService返回的YAML格式响应
                output_lines = result.stdout.strip().split('\n')
                success = False
                message = f"地图删除成功: {map_name}"
                error_code = 0  # 0=成功, 1=正在使用, 2=名称为空, 3=其他错误

                for line in output_lines:
                    if "success: True" in line:
                        success = True
                    elif "success: False" in line:
                        success = False
                    elif "message:" in line:
                        english_message = line.split("message:", 1)[1].strip()
                        if "Map name cannot be empty" in english_message:
                            message = "地图名称不能为空"
                            error_code = 2
                        elif "Cannot delete map currently being mapped" in english_message:
                            message = "无法删除当前正在建图的地图"
                            error_code = 1
                        elif "Cannot delete map currently in use for navigation" in english_message:
                            message = "无法删除当前正在使用的导航地图"
                            error_code = 1
                        elif "Map deleted successfully" in english_message:
                            message = f"地图删除成功: {map_name}"
                            error_code = 0
                        elif "Failed to delete map" in english_message:
                            message = f"地图删除失败: {map_name}"
                            error_code = 3
                        elif "Error deleting map" in english_message:
                            message = "删除地图时发生错误"
                            error_code = 3
                        else:
                            message = english_message
                            error_code = 3

                if success:
                    payload.data["message"] = message
                    payload.data["map_name"] = map_name
                    print(f"Delete map service call successful: {message}")
                else:
                    payload.data["code"] = error_code
                    payload.data["message"] = message
                    payload.data["map_name"] = map_name
                    print(f"Delete map service call failed: {message}")
            else:
                payload.data["code"] = 3
                payload.data["message"] = f"地图删除失败: {result.stderr}"
                payload.data["map_name"] = map_name
                print(f"Delete map service call failed: {result.stderr}")

        except (rospy.ServiceException, subprocess.TimeoutExpired, asyncio.TimeoutError) as e:
            payload.data["code"] = 3
            payload.data["message"] = f"无法连接到删除地图服务: {str(e)}"
            payload.data["map_name"] = map_name
            print(f"Failed to connect to delete map service: {str(e)}")


    except Exception as e:
        payload.data["code"] = 3
        payload.data["message"] = f"Failed to delete map: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


# ==================== 2D地图显示功能 

# 全局变量存储地图数据
latest_map_data = None
map_subscriber = None

# 地图列表检测改为按需主动获取，无需全局变量和回调函数

def map_callback(msg):
    """
    /map话题的回调函数，接收地图数据并主动推送给客户端
    """
    global latest_map_data, latest_map_info

    try:
        # 保存最新的地图数据
        latest_map_data = msg

        # 更新地图信息，用于坐标系转换
        map_info = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "z": msg.info.origin.position.z
            }
        }

        # 更新全局地图信息
        update_map_info(map_info)

        # 将OccupancyGrid转换为base64编码的PNG图片
        map_base64 = convert_map_to_base64(msg)

        # 检查是否应该推送地图
        if not should_push_map():
            return

        if map_base64:
            # 创建推送消息
            payload = Payload(
                cmd="map_update",
                data={
                    "code": 0,
                    "message": "Map data updated",
                    "map_data": map_base64,
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "resolution": msg.info.resolution,
                    "origin": {
                        "x": msg.info.origin.position.x,
                        "y": msg.info.origin.position.y,
                        "z": msg.info.origin.position.z
                    },
                    "timestamp": rospy.Time.now().to_sec()
                }
            )

            # 推送给所有连接的客户端
            response = Response(payload=payload, target="all")
            response_queue.put(response)
            # print(f"Map update pushed to all clients: {msg.info.width}x{msg.info.height}")  # 注释掉以避免大量日志输出

    except Exception as e:
        print(f"Error processing map data: {e}")

def convert_map_to_base64(map_msg):
    """
    将OccupancyGrid消息转换为base64编码的PNG图片
    """
    try:
        import base64
        from io import BytesIO
        import numpy as np
        import cv2

        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # 映射到图像灰度：0=白色，100=黑色，-1=灰色
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 128  # 未知区域为灰色
        img[data == 0] = 255   # 自由空间为白色
        img[data == 100] = 0   # 障碍物为黑色

        # 可选：线性插值灰度（处理 0~100 范围的值）
        mask = (data > 0) & (data < 100)
        img[mask] = 255 - (data[mask] * 255 // 100)

        # ROS地图原点在左下，OpenCV图像原点在左上 -> 上下翻转图像
        img = cv2.flip(img, 0)

        # 编码为PNG
        success, buffer = cv2.imencode('.png', img)
        if not success:
            print("Failed to encode image as PNG")
            return None
        png_data = buffer.tobytes()

        # 转换为base64
        base64_data = base64.b64encode(png_data).decode('utf-8')

        return base64_data

    except Exception as e:
        print(f"Error converting map to base64: {e}")
        return None

def init_robot_position_publisher():
    """
    初始化机器人位置发布者 - 订阅/Odometry话题并启动主动推送
    """
    global odometry_subscriber, latest_odometry_data

    print("=== init_robot_position_publisher called ===")
    print(f"ROS master status: {rospy.core.is_initialized()}")

    try:
        # 订阅/Odometry话题
        print("Trying to subscribe to /Odometry topic...")
        odometry_subscriber = rospy.Subscriber('/Odometry', Odometry, odometry_callback, queue_size=1)
        print("Robot position publisher initialized: subscribed to /Odometry topic for active push")

        # 检查话题是否存在
        topics = rospy.get_published_topics()
        odom_topics = [topic for topic, _ in topics if 'odom' in topic.lower()]
        print(f"Available odometry topics: {odom_topics}")

        # 延迟一下，等待/Odometry话题稳定
        rospy.Timer(rospy.Duration(2.0), check_and_publish_initial_position)

    except Exception as e:
        print(f"init_robot_position_publisher error: {e}")

def check_and_publish_initial_position(event):
    """
    检查是否有位置数据，如果有则发布初始位置
    """
    global latest_odometry_data
    #print("check_and_publish_initial_position: checking for initial position data...")

    if latest_odometry_data:
        pass
        #print(f"Found cached odometry data, publishing initial position")
        # 这里可以发布初始位置给客户端
        # 但由于odometry_callback已经在主动推送，这里不需要重复
    else:
        pass
        #print("No cached odometry data available yet")

def init_map_publisher():
    """
    初始化地图发布者 - 订阅/map话题并启动主动推送
    """
    global map_subscriber, latest_map_data

    print("=== init_map_publisher called ===")
    print(f"ROS master status: {rospy.core.is_initialized()}")

    try:
        # 订阅/map话题 (kuavo_slam_ws发布的是/map话题)
        print("Trying to subscribe to /map topic...")
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
        print("Map publisher initialized: subscribed to /map topic for active push")

        # 检查话题是否存在
        topics = rospy.get_published_topics()
        map_topics = [topic for topic, _ in topics if 'map' in topic.lower()]
        print(f"Available map topics: {map_topics}")

          # 延迟一下，等待/map话题稳定，然后检查是否有地图数据
        rospy.Timer(rospy.Duration(3.0), check_and_publish_map)

        return True
    except Exception as e:
        print(f"Failed to initialize map publisher: {e}")
        import traceback
        traceback.print_exc()
        return False



def check_and_publish_map(event):
    """
    检查是否有地图数据，如果没有则尝试获取
    """
    global latest_map_data

    #print("=== check_and_publish_map called ===")

    if latest_map_data:
        #print("Already have map data, forcing update...")
        # 如果已经有地图数据，强制触发一次更新
        map_callback(latest_map_data)
    else:
        #print("No map data available yet, will wait for /map topic updates...")
        pass

def get_latest_map_data():
    """
    获取最新的地图数据 - 用于新连接的客户端
    """
    global latest_map_data
    return latest_map_data




async def check_robot_standing_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    查询机器人是否站立功能
    """
    payload = Payload(
        cmd="check_robot_standing",
        data={"code": 0, "is_standing": False, "message": "Robot standing status checked successfully"}
    )

    try:
        # 检查机器人状态参数或从全局状态获取
        global robot_status

        # 通过全局机器人状态判断是否站立
        if robot_status == "standing" or robot_status == "launched":
            payload.data["is_standing"] = True
            payload.data["message"] = f"Robot is standing (status: {robot_status})"
        else:
            payload.data["is_standing"] = False
            payload.data["message"] = f"Robot is not standing (status: {robot_status})"

        print(f"Robot standing check: {payload.data['is_standing']} (status: {robot_status})")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to check robot standing status: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def assist_scan_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    辅助扫描功能
    采用话题的方式控制机器人的头部关节的 yaw 角
    先左转再右转，再居中
    """
    payload = Payload(
        cmd="assist_scan",
        data={"code": 0, "message": "Assist scan completed successfully"}
    )

    try:
        print("Starting assist scan (head movement)...")

        # 获取全局的头部控制发布者
        global control_head_pub

        print(f"DEBUG: control_head_pub = {control_head_pub}")

        if control_head_pub is None:
            payload.data["code"] = 1
            payload.data["message"] = "Head publisher not initialized"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 检查话题是否有订阅者
        num_subscribers = control_head_pub.get_num_connections()
        print(f"DEBUG: Number of subscribers to /robot_head_motion_data: {num_subscribers}")

        if num_subscribers == 0:
            print("WARNING: No subscribers to /robot_head_motion_data topic")

        # 头部扫描参数
        scan_angles = [-30.0, 30.0, 0.0]  # 居中→左→右→居中
        step_delay = 4  # 每个位置保持时间(秒)

        print("Starting head scan sequence...")

        for i, angle in enumerate(scan_angles):
            print(f"Position {i+1}/{len(scan_angles)}: {angle}°")

            # 发布角度命令
            head_msg = robotHeadMotionData()
            head_msg.joint_data = [angle, 0.0]  # [yaw, pitch]
            control_head_pub.publish(head_msg)

            # 等待到位
            await asyncio.sleep(step_delay)

        payload.data["message"] = "Head assist scan completed (left-right-center movement)"
        print("Assist scan completed successfully")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to perform assist scan: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def edit_map_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     地图编辑功能 - 支持显示模式和编辑模式
     编辑模式会自动保存到地图文件，必须指定地图名称
    """
    payload = Payload(
        cmd="edit_map",
        data={"code": 0, "message": "地图编辑成功"}
    )

    try:
        # 获取编辑参数
        map_name = data.get("data", {}).get("map_name", "")
        points = data.get("data", {}).get("points", [])  # 四个点的PNG坐标
        operation = data.get("data", {}).get("operation", "")  # "fill" 或 "clear"

        # 必须提供地图名称
        if not map_name:
            payload.data["code"] = 1
            payload.data["message"] = "必须指定地图名称"
            print("地图编辑失败：必须指定地图名称")
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 检查点和操作参数
        if not points or len(points) != 8:  # 四个点，每个点有x,y坐标
            payload.data["code"] = 1
            payload.data["message"] = "需要提供四个点的坐标 (8个数值: x1,y1,x2,y2,x3,y3,x4,y4)"
            print("地图编辑失败：需要提供四个点的坐标")
        elif operation not in ["fill", "clear"]:
            payload.data["code"] = 1
            payload.data["message"] = "操作类型必须是 'fill' 或 'clear'"
            print("地图编辑失败：操作类型必须是 'fill' 或 'clear'")
        else:
            print(f"执行地图编辑，地图: {map_name}, 操作: {operation}, 点: {points}")

            # 桌面软件传来的坐标是PNG坐标系（像素坐标）
            # 前端显示的地图图像和PGM文件尺寸一致，PNG坐标可以直接用于绘制
            # points格式: [x1, y1, x2, y2, x3, y3, x4, y4] - PNG坐标
            # 直接发送PNG坐标，不需要转换
            png_points = points
            print(f"使用PNG像素坐标: {png_points}")

            # 调用上位机的地图编辑服务
            try:
                import rospy
                from kuavo_mapping.srv import EditMap, EditMapRequest, EditMapResponse

                # 检查服务是否可用
                service_name = '/edit_map_service'
                print(f"等待地图编辑服务: {service_name}")

                try:
                    rospy.wait_for_service(service_name, timeout=10.0)
                except rospy.ROSException as e:
                    payload.data["code"] = 1
                    payload.data["message"] = f"地图编辑服务不可用: {str(e)}"
                    print(f"地图编辑服务不可用: {str(e)}")
                    response = Response(payload=payload, target=websocket)
                    response_queue.put(response)
                    return

                # 创建服务代理
                edit_map_service = rospy.ServiceProxy(service_name, EditMap)

                # 构建请求 - 注意：points期望是8个float64值（4个点）
                request = EditMapRequest()
                request.map_name = map_name
                request.points = png_points  # 直接使用PNG像素坐标
                request.operation = operation

                print(f"调用地图编辑服务: map_name={map_name}, operation={operation}, points={png_points}")

                # 调用服务
                response = edit_map_service(request)

                if response.success:
                    payload.data["code"] = 0
                    payload.data["message"] = response.message
                    # 编辑服务返回的图片数据是编辑后的结果
                    payload.data["map_image"] = response.image_data
                    payload.data["map_name"] = map_name
                    print(f"接收到编辑后的图片数据，长度: {len(response.image_data) if response.image_data else 0}")

                    # 获取地图元数据信息，与获取地图图片的格式保持一致
                    try:
                        # 先加载地图以获取最新信息
                        load_service_name = '/load_map'
                        rospy.wait_for_service(load_service_name, timeout=3.0)
                        load_map_client = rospy.ServiceProxy(load_service_name, LoadMap)
                        load_req = LoadMapRequest()
                        load_req.map_name = map_name
                        load_map_client(load_req)

                        # 等待地图数据发布并获取 - 使用异步sleep避免阻塞
                        await asyncio.sleep(1)
                        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
                        loop = asyncio.get_event_loop()
                        map_msg = await loop.run_in_executor(None, rospy.wait_for_message, '/map', OccupancyGrid, 5)

                        # 构建地图信息
                        map_info = {
                            "width": map_msg.info.width,
                            "height": map_msg.info.height,
                            "resolution": map_msg.info.resolution,
                            "origin": {
                                "x": map_msg.info.origin.position.x,
                                "y": map_msg.info.origin.position.y,
                                "z": map_msg.info.origin.position.z
                            }
                        }
                        payload.data["map_info"] = map_info
                        print(f"地图编辑成功，地图尺寸: {map_msg.info.width}x{map_msg.info.height}")

                    except Exception as map_e:
                        print(f"获取地图元数据失败，但编辑操作成功: {str(map_e)}")
                        # 即使获取元数据失败，也返回基本的地图信息
                        payload.data["map_info"] = {
                            "width": 0,
                            "height": 0,
                            "resolution": 0,
                            "origin": {"x": 0, "y": 0, "z": 0}
                        }

                    print(f"地图编辑成功: {response.message}")
                else:
                    payload.data["code"] = 1
                    payload.data["message"] = response.message
                    print(f"地图编辑失败: {response.message}")

            except Exception as e:
                payload.data["code"] = 1
                payload.data["message"] = f"调用地图编辑服务异常: {str(e)}"
                print(f"调用地图编辑服务异常: {str(e)}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"地图编辑异常: {str(e)}"
        print(f"地图编辑异常: {str(e)}")

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def check_map_exists(map_name):
    """检查指定地图是否存在于地图列表中"""
    try:
        import subprocess

        # 调用get_all_maps服务
        service_name = '/get_all_maps'
        cmd = f"rosservice call {service_name}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)

        if result.returncode == 0:
            # 检查输出中是否包含指定地图名
            if map_name in result.stdout:
                return True
            else:
                print(f"地图 '{map_name}' 不在地图列表中")
                return False
        else:
            print(f"获取地图列表失败: {result.stderr}")
            return False

    except Exception as e:
        print(f"检查地图是否存在时出错: {e}")
        return False


async def get_map_image_for_editing(map_name):
    """获取指定地图的图片数据用于编辑 - 先加载地图，然后从ROS话题获取"""
    try:
        import subprocess
        import time

        # 如果指定了地图名，先加载地图
        if map_name:
            service_name = '/load_map'
            print(f"正在加载地图: {map_name}")

            # 调用load_map服务
            cmd = f'rosservice call {service_name} "map_name: \\"{map_name}\\""'
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15))

            if result.returncode != 0:
                print(f"加载地图失败: {result.stderr}")
                return None

            # 等待地图数据发布 - 使用异步sleep避免阻塞
            await asyncio.sleep(2)

        # 从/map话题获取地图数据
        print("从/map话题获取地图数据...")
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        loop = asyncio.get_event_loop()
        msg = await loop.run_in_executor(None, rospy.wait_for_message, '/map', OccupancyGrid, 10)

        if msg:
            # 将OccupancyGrid转换为base64编码的PNG图片
            map_base64 = convert_map_to_base64(msg)

            if map_base64:
                return {
                    "image": map_base64,
                    "info": {
                        "width": msg.info.width,
                        "height": msg.info.height,
                        "resolution": msg.info.resolution,
                        "origin": {
                            "x": msg.info.origin.position.x,
                            "y": msg.info.origin.position.y,
                            "z": msg.info.origin.position.z,
                            "w": msg.info.origin.orientation.w
                        },
                        "timestamp": rospy.Time.now().to_sec()
                    }
                }
            else:
                print("地图数据转换失败")
                return None
        else:
            print("无法获取地图数据")
            return None

    except Exception as e:
        print(f"获取地图图片数据失败: {e}")
        return None






async def task_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     任务点管理功能 - 添加、更新、删除、获取任务点
    """
    payload = Payload(
        cmd="task_point",
        data={"code": 0, "message": "任务点操作成功"}
    )

    try:
        # 获取操作参数
        operation = data.get("data", {}).get("operation", 3)  # 默认GET操作
        task_point_data = data.get("data", {}).get("task_point", {})
        name = data.get("data", {}).get("name", "")
        use_robot_current_pose = data.get("data", {}).get("use_robot_current_pose", False)

        print(f"任务点操作: operation={operation}, name={name}, use_robot_current_pose={use_robot_current_pose}")

        # 如果需要获取机器人当前位姿，先检查机器人位置是否可用
        if use_robot_current_pose:
            print("Debug: 需要获取机器人当前位姿，使用统一的位置获取函数...")

            # 使用统一的机器人位置获取函数
            success, result = get_robot_position_for_task_point()

            if not success:
                payload.data["code"] = 1
                payload.data["message"] = f"Failed to get robot pose: {result}\n请确保机器人正在运行且定位系统正常工作"
                response = Response(payload=payload, target=websocket)
                response_queue.put(response)
                return
            else:
                print(f"Debug: 机器人位置验证成功，直接提供位置数据给任务点")
                # 直接使用验证成功的位置数据，避免ROS服务再次获取
                robot_pose = result
                task_point_data = task_point_data or {}
                task_point_data["pose"] = {
                    "position": {
                        "x": robot_pose.position.x,
                        "y": robot_pose.position.y,
                        "z": robot_pose.position.z
                    },
                    "orientation": {
                        "x": robot_pose.orientation.x,
                        "y": robot_pose.orientation.y,
                        "z": robot_pose.orientation.z,
                        "w": robot_pose.orientation.w
                    }
                }
                # 设置use_robot_current_pose=False，因为我们已经提供了位置数据
                use_robot_current_pose = False

        # 调用任务点服务
        import rospy
        from kuavo_msgs.srv import TaskPointOperation, TaskPointOperationRequest
        from kuavo_msgs.msg import TaskPoint
        from geometry_msgs.msg import Pose, Point, Quaternion

        # 检查服务是否可用
        service_name = '/task_point'
        print(f"等待任务点服务: {service_name}")

        try:
            rospy.wait_for_service(service_name, timeout=10.0)
        except rospy.ROSException as e:
            payload.data["code"] = 1
            payload.data["message"] = f"任务点服务不可用: {str(e)}"
            print(f"任务点服务不可用: {str(e)}")
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 创建服务代理
        task_point_service = rospy.ServiceProxy(service_name, TaskPointOperation)

        # 构建请求
        request = TaskPointOperationRequest()
        request.operation = operation
        request.use_robot_current_pose = use_robot_current_pose

        # 构建任务点对象
        task_point = TaskPoint()

        # 确定任务点名称
        task_point_name = task_point_data.get("name", name) if task_point_data else name
        task_point.name = task_point_name
        request.name = task_point_name  # 确保name字段也正确设置

        print(f"Debug: 设置任务点名称: task_point.name={task_point.name}, request.name={request.name}")

        if task_point_data and "pose" in task_point_data:
            pose_data = task_point_data.get("pose", {})
            position_data = pose_data.get("position", {})

            if use_robot_current_pose:
                # 使用机器人当前位置时，get_robot_position_for_task_point返回的是PNG坐标
                # 需要转换为Map坐标存储（任务点应存储在map坐标系中）
                png_x = position_data.get("x", 0.0)
                png_y = position_data.get("y", 0.0)
                png_z = position_data.get("z", 0.0)

                print(f"Debug: 使用机器人当前位置，坐标系统: PNG({png_x:.3f}, {png_y:.3f}, {png_z:.3f})")

                # PNG坐标转换为Map坐标
                map_x, map_y = png_to_map(png_x, png_y)
                if map_x is not None and map_y is not None:
                    print(f"坐标转换: PNG({png_x}, {png_y}) -> Map({map_x:.3f}, {map_y:.3f})")
                    position = Point()
                    position.x = map_x
                    position.y = map_y
                    position.z = png_z  # 保留原始Z值
                else:
                    print(f"坐标转换失败，使用原始值")
                    position = Point()
                    position.x = png_x
                    position.y = png_y
                    position.z = png_z
            else:
                # 桌面软件传来的坐标是PNG坐标系，需要转换为Map坐标系存储
                png_x = position_data.get("x", 0.0)
                png_y = position_data.get("y", 0.0)

                # PNG坐标转换为Map坐标（不使用odom，任务点应存储在map坐标系中）
                map_x, map_y = png_to_map(png_x, png_y)
                if map_x is not None and map_y is not None:
                    print(f"坐标转换: PNG({png_x}, {png_y}) -> Map({map_x:.3f}, {map_y:.3f})")
                    position = Point()
                    position.x = map_x
                    position.y = map_y
                    position.z = 0.0  # PNG坐标没有Z信息，设为0
                else:
                    print(f"坐标转换失败，使用原始值")
                    position = Point()
                    position.x = png_x
                    position.y = png_y
                    position.z = position_data.get("z", 0.0)

            orientation_data = pose_data.get("orientation", {})
            orientation = Quaternion()
            orientation.x = orientation_data.get("x", 0.0)
            orientation.y = orientation_data.get("y", 0.0)
            orientation.z = orientation_data.get("z", 0.0)
            orientation.w = orientation_data.get("w", 1.0)

            pose = Pose()
            pose.position = position
            pose.orientation = orientation
            task_point.pose = pose
            print(f"Debug: 设置任务点位姿: position({position.x}, {position.y}, {position.z})")
        else:
            # 如果没有提供位姿数据，使用默认位姿
            from geometry_msgs.msg import Pose, Point, Quaternion
            default_pose = Pose()
            default_pose.position = Point(0, 0, 0)
            default_pose.orientation = Quaternion(0, 0, 0, 1)
            task_point.pose = default_pose
            print(f"Debug: 使用默认位姿")

        request.task_point = task_point

        print(f"调用任务点服务: operation={operation}, task_point_name={task_point_name}")
        print(f"Debug: 调用任务点服务，请求参数: operation={request.operation}, name={request.name}, use_robot_current_pose={request.use_robot_current_pose}")
        try:
            response = task_point_service(request)
            print(f"Debug: 任务点服务响应: success={response.success}, message={response.message}")

            if response.success:
                payload.data["code"] = 0
                payload.data["message"] = response.message
                payload.data["task_points"] = []

                # 如果是GET操作，返回任务点列表
                if operation == 3:  # GET操作
                    print(f"Debug: 返回任务点列表，数量: {len(response.task_points)}")
                    for tp in response.task_points:
                        # 任务点存储在Map坐标系中，需要转换为PNG坐标用于前端显示
                        tp_x = float(tp.pose.position.x)
                        tp_y = float(tp.pose.position.y)

                        png_x, png_y = map_to_png(tp_x, tp_y)
                        if png_x is not None and png_y is not None:
                            # 使用PNG坐标作为position字段
                            position_x = png_x
                            position_y = png_y
                            print(f"Debug: 任务点 {tp.name} 转换为PNG坐标: Map({tp_x:.3f}, {tp_y:.3f}) -> PNG({png_x:.1f}, {png_y:.1f})")
                        else:
                            # PNG转换失败，使用原始坐标
                            position_x = tp_x
                            position_y = tp_y
                            print(f"Debug: 任务点 {tp.name} PNG转换失败，使用原始坐标: ({tp_x:.3f}, {tp_y:.3f})")

                        tp_dict = {
                            "name": tp.name,
                            "pose": {
                                "position": {
                                    "x": position_x,  # PNG坐标（如果转换成功）
                                    "y": position_y,  # PNG坐标（如果转换成功）
                                    "z": float(tp.pose.position.z)
                                },
                                "orientation": {
                                    "x": tp.pose.orientation.x,
                                    "y": tp.pose.orientation.y,
                                    "z": tp.pose.orientation.z,
                                    "w": tp.pose.orientation.w
                                }
                            }
                        }

                        # 保存原始map坐标作为map_position字段
                        tp_dict["map_position"] = {
                            "x": tp_x,  # 原始map坐标
                            "y": tp_y,  # 原始map坐标
                            "z": float(tp.pose.position.z)
                        }

                        # 如果有PNG坐标，也保存png_position字段
                        if png_x is not None and png_y is not None:
                            tp_dict["png_position"] = {
                                "x": png_x,
                                "y": png_y
                            }
                            # 检查position是否已经是PNG坐标
                            if abs(position_x - png_x) < 0.1 and abs(position_y - png_y) < 0.1:
                                tp_dict["coordinate_type"] = "PNG"
                            else:
                                tp_dict["coordinate_type"] = "Mixed"
                        else:
                            tp_dict["png_position"] = None
                            tp_dict["coordinate_type"] = "Odom"

                        payload.data["task_points"].append(tp_dict)

                print(f"任务点操作成功: {response.message}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"任务点操作失败: {response.message}"
                print(f"任务点操作失败: {response.message}")

        except rospy.ServiceException as e:
            payload.data["code"] = 1
            payload.data["message"] = f"任务点服务调用失败: {str(e)}"
            print(f"任务点服务调用失败: {str(e)}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"任务点操作异常: {str(e)}"
        print(f"任务点操作异常: {str(e)}")

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def get_task_points_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     获取任务点列表功能 -
    """
    payload = Payload(
        cmd="get_task_points",
        data={"code": 0, "message": "获取任务点列表成功", "task_points": []}
    )

    try:
        print("=== 开始获取任务点列表 ===")

        # 调用任务点管理服务，使用GET操作
        import rospy
        from kuavo_msgs.srv import TaskPointOperation, TaskPointOperationRequest
        from kuavo_msgs.msg import TaskPoint
        from geometry_msgs.msg import Pose, Point, Quaternion

        # 检查服务是否可用
        service_name = '/task_point'
        try:
            rospy.wait_for_service(service_name, timeout=10.0)
        except rospy.ROSException as e:
            payload.data["code"] = 1
            payload.data["message"] = f"任务点服务不可用: {str(e)}"
            print(f"任务点服务不可用: {str(e)}")
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 创建服务代理
        task_point_service = rospy.ServiceProxy(service_name, TaskPointOperation)

        # 构建GET请求
        request = TaskPointOperationRequest()
        request.operation = 3  # GET操作
        request.name = ""
        request.use_robot_current_pose = False

        # 创建空的任务点对象
        task_point = TaskPoint()
        task_point.name = ""
        pose = Pose()
        pose.position = Point(0.0, 0.0, 0.0)
        pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        task_point.pose = pose
        request.task_point = task_point

        print("调用任务点服务获取任务点列表...")

        # 调用服务
        response = task_point_service(request)

        if response.success:
            payload.data["code"] = 0
            payload.data["message"] = response.message
            payload.data["task_points"] = []

            # 解析任务点列表
            for tp in response.task_points:
                # 任务点存储在Map坐标系中，需要转换为PNG坐标用于前端显示
                tp_x = float(tp.pose.position.x)
                tp_y = float(tp.pose.position.y)

                png_x, png_y = map_to_png(tp_x, tp_y)
                if png_x is not None and png_y is not None:
                    # 使用PNG坐标作为position字段
                    position_x = png_x
                    position_y = png_y
                    print(f"Debug: 任务点 {tp.name} 转换为PNG坐标: Map({tp_x:.3f}, {tp_y:.3f}) -> PNG({png_x:.1f}, {png_y:.1f})")
                else:
                    # PNG转换失败，使用原始坐标
                    position_x = tp_x
                    position_y = tp_y
                    print(f"Debug: 任务点 {tp.name} PNG转换失败，使用原始坐标: ({tp_x:.3f}, {tp_y:.3f})")

                tp_dict = {
                    "name": tp.name,
                    "pose": {
                        "position": {
                            "x": position_x,  # PNG坐标（如果转换成功）
                            "y": position_y,  # PNG坐标（如果转换成功）
                            "z": float(tp.pose.position.z)
                        },
                        "orientation": {
                            "x": float(tp.pose.orientation.x),
                            "y": float(tp.pose.orientation.y),
                            "z": float(tp.pose.orientation.z),
                            "w": float(tp.pose.orientation.w)
                        }
                    }
                }

                # 保存原始map坐标作为map_position字段
                tp_dict["map_position"] = {
                    "x": tp_x,  # 原始map坐标
                    "y": tp_y,  # 原始map坐标
                    "z": float(tp.pose.position.z)
                }

                # 如果有PNG坐标，也保存png_position字段
                if png_x is not None and png_y is not None:
                    tp_dict["png_position"] = {
                        "x": png_x,
                        "y": png_y
                    }
                    # 检查position是否已经是PNG坐标
                    if abs(position_x - png_x) < 0.1 and abs(position_y - png_y) < 0.1:
                        tp_dict["coordinate_type"] = "PNG"
                    else:
                        tp_dict["coordinate_type"] = "Mixed"
                else:
                    tp_dict["png_position"] = None
                    tp_dict["coordinate_type"] = "Odom"

                payload.data["task_points"].append(tp_dict)

            print(f"获取任务点列表成功: {len(payload.data['task_points'])} 个任务点")
        else:
            payload.data["code"] = 1
            payload.data["message"] = response.message
            print(f"获取任务点列表失败: {response.message}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"获取任务点列表异常: {str(e)}"
        print(f"获取任务点列表异常: {str(e)}")

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def navigate_to_task_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     导航到任务点功能
    """
    payload = Payload(
        cmd="navigate_to_task_point",
        data={"code": 0, "message": "导航到任务点启动成功"}
    )

    try:
        # 获取任务点名称
        task_name = data.get("data", {}).get("task_name", "")

        if not task_name:
            payload.data["code"] = 1
            payload.data["message"] = "需要提供任务点名称"
            print("导航到任务点失败：需要提供任务点名称")
        else:
            print(f"导航到任务点: {task_name}")

            # 调用导航到任务点服务
            import rospy
            from kuavo_mapping.srv import NavigateToTaskPoint, NavigateToTaskPointRequest

            # 检查服务是否可用
            service_name = '/navigate_to_task_point'
            print(f"等待导航到任务点服务: {service_name}")

            try:
                rospy.wait_for_service(service_name, timeout=10.0)
            except rospy.ROSException as e:
                payload.data["code"] = 1
                payload.data["message"] = f"导航到任务点服务不可用: {str(e)}"
                print(f"导航到任务点服务不可用: {str(e)}")
                response = Response(payload=payload, target=websocket)
                response_queue.put(response)
                return

            # 创建服务代理
            navigate_service = rospy.ServiceProxy(service_name, NavigateToTaskPoint)

            # 构建请求
            request = NavigateToTaskPointRequest()
            request.task_name = task_name

            print(f"调用导航到任务点服务: task_name={task_name}")

            # 调用服务
            response = navigate_service(request)

            if response.success:
                payload.data["code"] = 0
                payload.data["message"] = response.message
                print(f"导航到任务点启动成功: {response.message}")
            else:
                payload.data["code"] = 1
                payload.data["message"] = response.message
                print(f"导航到任务点启动失败: {response.message}")

    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"导航到任务点异常: {str(e)}"
        print(f"导航到任务点异常: {str(e)}")

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)

async def get_robot_position_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="get_robot_position", data={"code":0}
    )
    try:
        # 获取base_link在map坐标系下的位置
        # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
        loop = asyncio.get_event_loop()

        # 包装 TF 变换操作
        def get_tf_transform():
            listener = tf.TransformListener()
            listener.waitForTransform("map", "livox_frame", rospy.Time(0), rospy.Duration(2.0))
            return listener.lookupTransform("map", "livox_frame", rospy.Time(0))

        (trans, rot) = await loop.run_in_executor(None, get_tf_transform)
        # trans为(x, y, z)
        x, y, z = trans

        # 获取地图信息 - 使用 asyncio 在后台线程执行
        map_msg = await loop.run_in_executor(None, rospy.wait_for_message, '/map', OccupancyGrid, 2.0)
        origin = map_msg.info.origin
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        # 这里的origin_grid_x和origin_grid_y表示map坐标系下(0,0)点在栅格坐标系下的坐标
        # 也就是map坐标系的(0,0)点对应的像素点
        origin_x = origin.position.x
        origin_y = origin.position.y

        origin_grid_x = int((0.0 - origin_x) / resolution)
        origin_grid_y = int((0.0 - origin_y) / resolution)

        origin_grid_y =  height - 1 - origin_grid_y

        # 将base_link的map坐标转换为栅格坐标
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        # 转换为PNG图片上的像素坐标
        # PNG图片通过cv2.flip(img, 0)上下翻转，所以Y坐标需要转换
        png_x = grid_x
        png_y = height - 1 - grid_y

        payload.data["position"] = {
            "png_x": png_x,  # PNG图片上的X像素坐标
            "png_y": png_y,  # PNG图片上的Y像素坐标
            "origin_grid_x": origin_grid_x,  # 地图原点在栅格坐标系下的X
            "origin_grid_y": origin_grid_y,  # 地图原点在栅格坐标系下的Y
        }
        payload.data["msg"] = "Get robot position successfully"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Failed to get robot position: {e}"
    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


async def get_all_maps_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
     给出地图列表功能 -
    """
    payload = Payload(
        cmd="get_all_maps",
        data={"code": 0, "message": "Map list retrieved successfully", "maps": []}
    )

    try:
        print("=== 开始获取地图列表 ===")

        # 调用map_manager的get_all_maps服务
        service_name = '/get_all_maps'
        try:
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, rospy.wait_for_service, service_name, 5.0)
            print(f"Found service: {service_name}")

            # 使用rosservice命令调用get_all_maps服务
            cmd = f"rosservice call {service_name}"
            # 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True, encoding='utf-8', timeout=10))

            if result.returncode == 0:
                # 解析服务返回结果
                print(f"Raw output: {result.stdout}")
                output_lines = result.stdout.strip().split('\n')
                maps_list = []

                # 处理YAML格式输出 - 可能是多行的
                for i, line in enumerate(output_lines):
                    line = line.strip()
                    if "maps:" in line:
                        # 如果这一行包含数据，直接解析
                        if "[" in line:
                            # 单行格式: maps: ['map1', 'map2']
                            import re
                            match = re.search(r'\[.*?\]', line)
                            if match:
                                maps_str = match.group(0)
                                try:
                                    maps_list = eval(maps_str)
                                except:
                                    maps_list = [item.strip().strip("'\"") for item in maps_str.strip('[]').split(',') if item.strip()]
                        else:
                            # 多行格式，查找下一行的数据
                            if i + 1 < len(output_lines):
                                next_line = output_lines[i + 1].strip()
                                if next_line.startswith('[') and next_line.endswith(']'):
                                    # ['map1', 'map2']
                                    maps_str = next_line
                                    try:
                                        maps_list = eval(maps_str)
                                    except:
                                        maps_list = [item.strip().strip("'\"") for item in maps_str.strip('[]').split(',') if item.strip()]
                                elif next_line.startswith('-'):
                                    # - map1
                                    # - map2
                                    maps_list = []
                                    for j in range(i, len(output_lines)):
                                        item_line = output_lines[j].strip()
                                        if item_line.startswith('- '):
                                            maps_list.append(item_line[2:].strip().strip("'\""))
                                        elif item_line == '' or (not item_line.startswith(' ') and j > i):
                                            break
                        break

                # 如果还是没有找到数据，尝试直接解析整个输出
                if not maps_list:
                    # 尝试直接从输出中提取所有引号内的内容
                    import re
                    all_matches = re.findall(r'["\']([^"\']+)["\']', result.stdout)
                    if all_matches:
                        maps_list = all_matches

                # 解码 Unicode 转义序列 (如 \u4E2D -> 中)
                decoded_maps = []
                for s in maps_list:
                    try:
                        # 解码 Unicode 转义序列
                        decoded = s.encode('utf-8').decode('unicode_escape')
                        decoded_maps.append(decoded)
                    except:
                        # 如果解码失败，保留原字符串
                        decoded_maps.append(s)

                payload.data["maps"] = decoded_maps
                payload.data["message"] = f"Found {len(maps_list)} maps"
                print(f"Parsed maps list: {maps_list}")
                print(f"Debug: Total lines: {len(output_lines)}, Content: {output_lines}")
            else:
                payload.data["code"] = 0
                payload.data["maps"] = []
                payload.data["message"] = f"获取地图列表服务失败: {result.stderr}"
                print(f"Get all maps failed: {result.stderr}")

        except (rospy.ServiceException, subprocess.TimeoutExpired) as e:
            payload.data["code"] = 0
            payload.data["maps"] = []
            payload.data["message"] = f"无法连接到地图列表服务: {str(e)}"
            print(f"Failed to connect to get_all_maps service: {str(e)}")

        print("=== DEBUG: 地图列表检查结束 ===")
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)

    except Exception as e:
        payload.data["code"] = 0
        payload.data["maps"] = []
        payload.data["message"] = f"获取地图列表异常: {str(e)}"
        print(f"Get map list exception: {str(e)}")
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)


# 全局变量存储最新的机器人位置
latest_odometry_data = None
odometry_subscriber = None
tf_listener = None
last_robot_position_publish_time = 0  # 上次推送机器人位置的时间
ROBOT_POSITION_PUBLISH_INTERVAL = 0.1  # 机器人位置推送间隔（秒），降低频率到原来的一半


def odometry_callback(msg):
    """
    /Odometry话题的回调函数，接收机器人位置数据并主动推送给客户端
    """
    global latest_odometry_data, tf_listener, last_robot_position_publish_time

    try:
        # 保存最新的里程计数据
        latest_odometry_data = msg

        # 限制机器人位置推送频率，降低到原来的一半
        current_time = time.time()
        if current_time - last_robot_position_publish_time < ROBOT_POSITION_PUBLISH_INTERVAL:
            return  # 未到推送时间，直接返回

        # 更新上次推送时间
        last_robot_position_publish_time = current_time

        pose = msg.pose.pose
        twist = msg.twist.twist

        # 将Odom坐标转换为PNG坐标供前端显示
        odom_x = float(pose.position.x)
        odom_y = float(pose.position.y)

        # 打印当前地图信息状态
        global global_map_info
        if global_map_info:
            #print(f"Debug: 当前地图信息可用: resolution={global_map_info.get('resolution')}, origin={global_map_info.get('origin')}, size={global_map_info.get('width')}x{global_map_info.get('height')}")
            pass
        else:
            #print("Debug: 地图信息不可用，无法进行坐标转换")
            pass

        png_x, png_y = odometry_to_png(odom_x, odom_y)

        if png_x is not None and png_y is not None:
            # 成功转换为PNG坐标，用于前端显示
            display_x = png_x
            display_y = png_y
            #print(f"机器人位置坐标转换: Odom({odom_x:.3f}, {odom_y:.3f}) -> PNG({display_x:.1f}, {display_y:.1f})")
        else:
            # 转换失败，使用原始odom坐标
            display_x = odom_x
            display_y = odom_y
            #print(f"机器人位置坐标转换失败，使用原始Odom坐标: ({odom_x:.3f}, {odom_y:.3f})")

        # 构建位置数据
        position_data = {
            "code": 0,
            "message": "Robot position updated",
            "position": {
                "x": display_x,  # PNG坐标用于前端显示
                "y": display_y,  # PNG坐标用于前端显示
                "z": float(pose.position.z)
            },
            "odom_position": {
                "x": odom_x,    # 原始odom坐标，用于调试
                "y": odom_y,    # 原始odom坐标，用于调试
                "z": float(pose.position.z)
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w)
            },
            "linear_velocity": {
                "x": float(twist.linear.x),
                "y": float(twist.linear.y),
                "z": float(twist.linear.z)
            },
            "angular_velocity": {
                "x": float(twist.angular.x),
                "y": float(twist.angular.y),
                "z": float(twist.angular.z)
            },
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "timestamp": msg.header.stamp.to_sec()
        }

        # 尝试获取机器人在地图坐标系中的位置（使用全局tf_listener）
        try:
            import tf
            if tf_listener is None:
                tf_listener = tf.TransformListener()

            # 短暂等待变换，避免阻塞
            if tf_listener.canTransform("map", "base_link", rospy.Time(0)):
                (trans, rot) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                position_data["map_position"] = {
                    "x": float(trans[0]),
                    "y": float(trans[1]),
                    "z": float(trans[2])
                }
                position_data["map_orientation"] = {
                    "x": float(rot[0]),
                    "y": float(rot[1]),
                    "z": float(rot[2]),
                    "w": float(rot[3])
                }

                # 直接从Odometry坐标转换为PNG坐标
                odom_x = float(pose.position.x)
                odom_y = float(pose.position.y)

                png_x, png_y = odometry_to_png(odom_x, odom_y)
                if png_x is not None and png_y is not None:
                    position_data["png_position"] = {
                        "x": png_x,  # 像素坐标
                        "y": png_y
                    }
                else:
                    position_data["png_position"] = None
            else:
                position_data["map_position"] = None
                position_data["map_orientation"] = None
                position_data["png_position"] = None
        except Exception as map_e:
            # 忽略地图位置获取失败，不影响主要功能
            position_data["map_position"] = None
            position_data["map_orientation"] = None
            position_data["png_position"] = None

        # 创建推送消息
        payload = Payload(
            cmd="robot_position_update",
            data=position_data
        )

        # 推送给所有连接的客户端
        response = Response(payload=payload, target="all")
        response_queue.put(response)

    except Exception as e:
        print(f"Error in odometry callback: {e}")

    # print(f"Received odometry: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")


def get_robot_position_for_task_point():
    """
    为任务点功能获取机器人当前位姿
    直接返回从odom转换到PNG坐标系的坐标
    """
    try:
        import tf
        import tf.transformations as tf_trans

        # 首先检查是否有缓存的里程计数据
        if latest_odometry_data:
            pose = latest_odometry_data.pose.pose
            odom_x = pose.position.x
            odom_y = pose.position.y
            odom_z = pose.position.z

            print(f"Debug: 获取Odom位置: ({odom_x:.3f}, {odom_y:.3f}, {odom_z:.3f})")

            # 从odom坐标转换到PNG坐标
            try:
                png_x, png_y = odometry_to_png(odom_x, odom_y)
                if png_x is not None and png_y is not None:
                    print(f"Debug: Odom到PNG转换成功 - PNG位置: ({png_x:.1f}px, {png_y:.1f}px)")

                    # 创建新的Pose对象，包含PNG坐标
                    from geometry_msgs.msg import Pose, Point, Quaternion
                    png_pose = Pose()
                    png_pose.position.x = png_x  # PNG坐标
                    png_pose.position.y = png_y  # PNG坐标
                    png_pose.position.z = odom_z  # 保持原始Z值
                    png_pose.orientation = pose.orientation  # 保持原始姿态

                    return True, png_pose
                else:
                    print(f"Debug: Odom到PNG转换失败 - 地图信息不可用")
                    return False, "地图信息不可用，无法转换坐标"
            except Exception as png_e:
                print(f"Debug: PNG转换失败: {png_e}")
                return False, f"PNG转换失败: {png_e}"

        else:
            # 如果没有里程计数据，尝试TF变换获取Map坐标，然后转换
            print("Debug: 无里程计数据，尝试TF变换获取机器人位置")
            global tf_listener
            if tf_listener is None:
                tf_listener = tf.TransformListener()

            # 短暂等待变换，避免长时间阻塞
            try:
                tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(0.5))
                (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

                map_x = trans[0]
                map_y = trans[1]
                map_z = trans[2]

                print(f"Debug: TF变换获取Map位置: ({map_x:.3f}, {map_y:.3f}, {map_z:.3f})")

                # 尝试将Map坐标转换为PNG坐标
                try:
                    # 使用map_to_png函数转换Map坐标
                    png_x, png_y = map_to_png(map_x, map_y)
                    if png_x is not None and png_y is not None:
                        print(f"Debug: Map到PNG转换成功 - PNG位置: ({png_x:.1f}px, {png_y:.1f}px)")

                        # 创建包含PNG坐标的Pose对象
                        from geometry_msgs.msg import Pose, Point, Quaternion
                        png_pose = Pose()
                        png_pose.position.x = png_x  # PNG坐标
                        png_pose.position.y = png_y  # PNG坐标
                        png_pose.position.z = map_z  # 保持原始Z值
                        png_pose.orientation = Quaternion(*rot)

                        return True, png_pose
                    else:
                        print(f"Debug: Map到PNG转换失败 - 地图信息不可用")
                        return False, "地图信息不可用，无法转换坐标"
                except Exception as png_e:
                    print(f"Debug: PNG转换失败: {png_e}")
                    return False, f"PNG转换失败: {png_e}"

            except tf.Exception as tf_e:
                print(f"Debug: TF变换失败: {str(tf_e)}")
                return False, f"TF变换失败: {str(tf_e)}"

    except Exception as e:
        print(f"Debug: 获取机器人位姿失败: {str(e)}")
        return False, str(e)






async def calibration_by_task_point_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    基于任务点的校准接口
    通过任务点名称自动获取位置和姿态进行校准
    """
    payload = Payload(
        cmd="calibration_by_task_point",
        data={"code": 0, "message": "基于任务点的校准成功"}
    )

    try:
        task_point_name = data.get("data", {}).get("task_point_name", "")

        if not task_point_name:
            payload.data["code"] = 1
            payload.data["message"] = "需要提供任务点名称"
            print("基于任务点的校准失败：需要提供任务点名称")
        else:
            print(f"开始基于任务点的校准: {task_point_name}")

            # 导入服务类型
            from kuavo_mapping.srv import InitialPoseWithTaskPoint, InitialPoseWithTaskPointRequest

            # 调用initialpose_with_taskpoint服务
            service_name = '/initialpose_with_taskpoint'
            rospy.wait_for_service(service_name, timeout=3.0)
            calibration_service = rospy.ServiceProxy(service_name, InitialPoseWithTaskPoint)

            # 构建请求
            req = InitialPoseWithTaskPointRequest()
            req.task_point_name = task_point_name

            print(f"调用基于任务点的校准服务: task_point_name={task_point_name}")

            # 执行校准 - 使用 asyncio 在后台线程执行，避免阻塞 WebSocket 事件循环
            loop = asyncio.get_event_loop()
            res = await loop.run_in_executor(None, calibration_service, req)
            if res.success:
                payload.data["code"] = 0
                payload.data["message"] = f"基于任务点 '{task_point_name}' 的校准成功"
                payload.data["task_point_name"] = task_point_name
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"基于任务点 '{task_point_name}' 的校准失败: 机器人不在任务点附近或建图不够精准"

    except rospy.ServiceException as e:
        payload.data["code"] = 1
        payload.data["message"] = f"校准服务调用失败: {e}"
    except rospy.ROSException as e:
        payload.data["code"] = 1
        payload.data["message"] = f"校准服务超时: {e}"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"校准异常: {e}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def set_api_key_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    设置API密钥接口
    用于设置大模型的API密钥
    """
    print('set key handler')
    llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis.json")
    payload = Payload(cmd="set_api_key", data={"code": 0, "msg": "API密钥存储成功"})
    data = data.get("data", {})

    if not data:
        payload.data["code"] = 1
        payload.data["msg"] = "需要提供API密钥"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return

    try:
        key_pairs = {}
        if not os.path.exists(llm_api_storage_path):
            os.makedirs(os.path.expanduser("~/.config/lejuconfig/"), exist_ok=True)
            with open(llm_api_storage_path, "w") as f:
                f.write("{}")
        print('读取原有文件')
        with open(llm_api_storage_path,'r') as f:
            key_pairs = json.load(f)

        for key, value in data.items():
            key_pairs[key] = str(value) if value is not None else ""


        with open(llm_api_storage_path, "w") as f:
            json.dump(key_pairs, f, indent=4)

        payload.data["code"] = 0
        payload.data["msg"] = "API密钥存储成功"

    except FileNotFoundError:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["msg"] = "API密钥文件路径不存在"

    except json.JSONDecodeError:
        print(traceback.format_exc())
        # 做处理
        with open(llm_api_storage_path, "w") as f:
            json.dump(key_pairs, f, indent=4)
        payload.data["code"] = 0
        payload.data["msg"] = "原文件格式错误,已重置为默认格式"

    except IOError as e:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["msg"] = f"API密钥存储失败: 文件操作错误 - {e}"

    except Exception as e:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["msg"] = f"API密钥存储失败: {e}"+traceback.format_exc()

    finally:
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)


    print(payload.data['msg'])


async def get_api_key_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    获取API密钥接口
    用于获取大模型的API密钥
    """
    print("获取api密钥中")
    llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis.json")
    payload = Payload(cmd="get_api_key", data={"code": 0, "msg": "API密钥获取成功"})
    valid_params = [
        "ark_X-Api-App-ID",
        "ark_X-Api-Access-Key",
        "xfyun_APPID",
        "xfyun_APISecret",
        "xfyun_APIKey",
        "ark_analysis_key",
    ]

    try:
        if not os.path.exists(llm_api_storage_path):
            payload.data["code"] = 1
            payload.data["msg"] = "API密钥文件不存在"
            return

        with open(llm_api_storage_path,'r') as f:
            content = json.load(f)

        if not content or content == {}:
            payload.data["code"] = 1
            payload.data["msg"] = "API密钥文件为空"
            return

        for key in valid_params:
            payload.data[key] = content.get(key, "")

        payload.data["code"] = 0
        payload.data["msg"] = "API密钥获取成功"

    except FileNotFoundError:
        payload.data["code"] = 1
        payload.data["msg"] = "API密钥文件不存在"

    except json.JSONDecodeError:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["msg"] = "API密钥文件格式错误"

    except IOError as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"API密钥获取失败: 文件操作错误 - {e}"

    except Exception as e:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["msg"] = f"API密钥获取失败: {e}"

    finally:
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)

        print(payload.data['msg'])


async def get_api_key_status_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    """
    获取API密钥状态接口
    用于检查特定类型模型的API密钥是否已设置
    """
    print("检查api密钥状态中")
    llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis.json")
    payload = Payload(cmd="get_api_key_status", data={"code": 0, "msg": "模型所需的key已经设置"})
    
    # 获取type参数，默认值为realtime
    api_type = data.get("data", {}).get("type", "realtime")
    
    # 根据type确定需要检查的key
    if api_type == "realtime":
        # 实时模型需要检查的key
        required_keys = [
            "ark_X-Api-App-ID",
            "ark_X-Api-Access-Key"
        ]
    elif api_type == "non-realtime":
        # 非实时模型需要检查的key
        required_keys = [
            "xfyun_APPID",
            "xfyun_APISecret",
            "xfyun_APIKey",
            "ark_analysis_key"
        ]
    else:
        # 不支持的类型
        payload.data["code"] = 1
        payload.data["type"] = api_type
        payload.data["msg"] = "不支持的模型类型"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return

    try:
        if not os.path.exists(llm_api_storage_path):
            payload.data["code"] = 1
            payload.data["type"] = api_type
            payload.data["is_empty"] = required_keys
            payload.data["msg"] = "API密钥文件不存在"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        with open(llm_api_storage_path, 'r') as f:
            content = json.load(f)

        if not content or content == {}:
            payload.data["code"] = 1
            payload.data["type"] = api_type
            payload.data["is_empty"] = required_keys
            payload.data["msg"] = "API密钥文件为空"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return

        # 检查需要的key是否存在且非空
        empty_keys = []
        for key in required_keys:
            value = content.get(key, "")
            if not value or value.strip() == "":
                empty_keys.append(key)

        payload.data["type"] = api_type
        
        if empty_keys:
            payload.data["code"] = 1
            payload.data["is_empty"] = empty_keys
            payload.data["msg"] = "存在缺失的key"
        else:
            payload.data["code"] = 0
            payload.data["is_empty"] = []
            payload.data["msg"] = "模型所需的key已经设置"

    except FileNotFoundError:
        payload.data["code"] = 1
        payload.data["type"] = api_type
        payload.data["is_empty"] = required_keys
        payload.data["msg"] = "API密钥文件不存在"

    except json.JSONDecodeError:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["type"] = api_type
        payload.data["is_empty"] = required_keys
        payload.data["msg"] = "API密钥文件格式错误"

    except IOError as e:
        payload.data["code"] = 1
        payload.data["type"] = api_type
        payload.data["is_empty"] = required_keys
        payload.data["msg"] = f"API密钥读取失败: 文件操作错误 - {e}"

    except Exception as e:
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["type"] = api_type
        payload.data["is_empty"] = required_keys
        payload.data["msg"] = f"API密钥检查失败: {e}"

    finally:
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)

        print(payload.data['msg'])

async def get_vr_status_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """
    获取VR状态接口
    返回VR连接状态、节点状态、录制状态等信息
    """
    payload = Payload(cmd="get_vr_status", data={"code": 0})

    try:
        # 导入VR状态模块
        import vr_manager

        # 获取VR状态信息
        status_info = vr_manager.get_vr_status()
        payload.data.update(status_info)

    except Exception as e:
        print(f"Get VR status error: {e}")
        import traceback
        print(traceback.format_exc())
        payload.data["code"] = 1
        payload.data["message"] = f"Failed to get VR status: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)

async def start_real_time_chat_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """开启实时对话"""
    payload = Payload(
        cmd="start_real_time_chat", 
        data={"code": 0, "message": "实时对话启动成功"}
    )

    try:
        # 获取当前脚本所在目录，用于定位realtime_client.py
        current_dir = os.path.dirname(os.path.abspath(__file__))
        realtime_client_path = os.path.join(current_dir, "realtime_client.py")
        
        # 检查文件是否存在
        if not os.path.exists(realtime_client_path):
            payload.data["code"] = 1
            payload.data["message"] = f"找不到实时对话客户端脚本: {realtime_client_path}"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return
        
        # 使用tmux启动实时对话客户端
        session_name = "realtime_session"
        cmd = f"source {KUAVO_ROS_CONTROL_WS_PATH}/devel/setup.bash && python3 {realtime_client_path}"

        # 检查当前是否存在名为realtime_session的会话
        check_result = subprocess.run(
            ["tmux", "has-session", "-t", session_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        if check_result.returncode == 0:
            payload.data["code"] = 1
            payload.data["message"] = "实时对话会话已存在"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return
        
        # 使用tmux_run_cmd函数启动会话
        success, msg = tmux_run_cmd(session_name, cmd, sudo=False)
        
        if success:
            payload.data["code"] = 0
            payload.data["message"] = msg
        else:
            payload.data["code"] = 1
            payload.data["message"] = msg
            
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"启动实时对话失败: {str(e)}"
        print(f"启动实时对话失败: {str(e)}")
        
    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def stop_real_time_chat_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """停止实时对话"""
    payload = Payload(
        cmd="stop_real_time_chat", 
        data={"code": 0, "message": "实时对话停止成功"}
    )

    try:
        session_name = "realtime_session"
        
        # 发送kill-session命令关闭对应session
        result = subprocess.run(
            ["tmux", "kill-session", "-t", session_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        if result.returncode == 0:
            payload.data["code"] = 0
            payload.data["message"] = "实时对话会话已停止"
        else:
            # 检查session是否已经不存在
            check_result = subprocess.run(
                ["tmux", "has-session", "-t", session_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            if check_result.returncode != 0:
                payload.data["code"] = 0
                payload.data["message"] = "实时对话会话已不存在"
            else:
                payload.data["code"] = 1
                payload.data["message"] = f"停止实时对话失败: {result.stderr}"
                
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"停止实时对话失败: {str(e)}"
        print(f"停止实时对话失败: {str(e)}")
        
    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def get_real_time_chat_status_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """获取实时对话状态"""
    payload = Payload(
        cmd="get_real_time_chat_status", 
        data={"code": 0, "status": False, "message": "实时对话未运行"}
    )

    try:
        session_name = "realtime_session"
        
        # 检查session是否存在
        result = subprocess.run(
            ["tmux", "has-session", "-t", session_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        if result.returncode == 0:
            payload.data["status"] = True
            payload.data["message"] = "实时对话正在运行"
        else:
            payload.data["status"] = False
            payload.data["message"] = "实时对话未运行"
            
    except Exception as e:
        payload.data["code"] = 1
        payload.data["message"] = f"获取实时对话状态失败: {str(e)}"
        print(f"获取实时对话状态失败: {str(e)}")
        
    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def start_vr_record_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """
    开始VR录制接口
    录制机器人的bag数据（文件名自动生成）
    """
    payload = Payload(cmd="start_vr_record", data={"code": 0, "message": "Recording started"})

    try:
        # 调用录制函数（自动生成文件名）
        success, message = vr_manager.start_recording()

        if success:
            payload.data["code"] = 0
            payload.data["message"] = message
        else:
            payload.data["code"] = 1
            payload.data["message"] = message

    except Exception as e:
        print(f"Start VR recording error: {e}")
        import traceback
        print(traceback.format_exc())
        payload.data["code"] = 2
        payload.data["message"] = f"Start recording failed: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def stop_vr_record_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """
    停止VR录制并转换为TACT文件接口
    """
    payload = Payload(cmd="stop_vr_record", data={"code": 0, "message": "Recording stopped"})

    try:
        # 获取tact_filename参数
        request_data = data.get("data", {}) or {}
        tact_filename = request_data.get("tact_filename")

        if not tact_filename:
            payload.data["code"] = 1
            payload.data["message"] = "tact_filename parameter is required"
        else:
            # 调用停止录制并转换函数
            success, message = vr_manager.stop_recording_and_convert(tact_filename)

            if success:
                payload.data["code"] = 0
                payload.data["message"] = message
            else:
                payload.data["code"] = 1
                payload.data["message"] = message

    except Exception as e:
        print(f"Stop VR recording error: {e}")
        import traceback
        print(traceback.format_exc())
        payload.data["code"] = 2
        payload.data["message"] = f"Stop recording failed: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)


async def cancel_vr_record_handler(websocket: websockets.WebSocketServerProtocol, data: dict):
    """
    取消VR录制接口
    """
    payload = Payload(cmd="cancel_vr_record", data={"code": 0, "message": "Recording cancelled"})

    try:
        # 调用取消录制函数
        success, message = vr_manager.cancel_recording()

        if success:
            payload.data["code"] = 0
            payload.data["message"] = message
        else:
            payload.data["code"] = 1
            payload.data["message"] = message

    except Exception as e:
        print(f"Cancel VR recording error: {e}")
        import traceback
        print(traceback.format_exc())
        payload.data["code"] = 2
        payload.data["message"] = f"Cancel recording failed: {str(e)}"

    response = Response(payload=payload, target=websocket)
    response_queue.put(response)
