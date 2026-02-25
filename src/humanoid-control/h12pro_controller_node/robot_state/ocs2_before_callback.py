#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import rospy
import os
from rich import console
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import jointBezierTrajectory, bezierCurveCubicPoint
from kuavo_msgs.srv import changeArmCtrlMode
from utils.utils import get_start_end_frame_time, frames_to_custom_action_data_ocs2
import time
import signal
import datetime
import json
from std_srvs.srv import SetBool
from std_msgs.msg import String
import threading
from h12pro_controller_node.srv import playmusic, playmusicRequest, playmusicResponse
from h12pro_controller_node.srv import ExecuteArmAction, ExecuteArmActionRequest, ExecuteArmActionResponse
from h12pro_controller_node.srv import OCS2RobotControl, OCS2RobotControlRequest, OCS2RobotControlResponse
from h12pro_controller_node.msg import RobotActionState
from h12pro_controller_node.srv import GetBarcode, GetBarcodeRequest, GetBarcodeResponse
from pytrees_actions.srv import SetMoveTarget, SetMoveTargetRequest, SetMoveTargetResponse
from pytrees_actions.srv import SetSMTTarget, SetSMTTargetRequest, SetSMTTargetResponse
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import os
import json
import hashlib
import math
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String, Bool, Int32, Float32, Int8

console = console.Console()
current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(os.path.dirname(current_dir), "config")
system_config_path = os.path.join(config_dir, "system_config.json")

music_trigger_pub = None

# 默认配置（如未提供配置文件或缺少字段则回退到这些值）
_default_settings = {
    "ACTION_FILE_FOLDER": "~/.config/lejuconfig/action_files",
    "ROS_BAG_LOG_SAVE_PATH": "~/.log/vr_remote_control/rosbag",
    "HUMANOID_ROBOT_SESSION_NAME": "humanoid_robot",
    "VR_REMOTE_CONTROL_SESSION_NAME": "vr_remote_control",
    "LAUNCH_HUMANOID_ROBOT_SIM_CMD": "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12 start_way:=auto",
    "LAUNCH_HUMANOID_ROBOT_REAL_CMD": "roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=bt2pro",
    "LAUNCH_VR_REMOTE_CONTROL_CMD": "roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch",
    "ROS_MASTER_URI": os.getenv("ROS_MASTER_URI", "http://192.168.26.1:11311"),
    "ROS_IP": os.getenv("ROS_IP", "192.168.26.1"),
    "ROS_HOSTNAME": os.getenv("ROS_HOSTNAME", "192.168.26.1"),
    "KUAVO_ROS_CONTROL_WS_PATH": os.getenv("KUAVO_ROS_CONTROL_WS_PATH", "/home/lab/lpf/kuavo-ros-control")
}

try:
    with open(system_config_path, "r") as f:
        _override_settings = json.load(f)
except Exception:
    _override_settings = {}

_settings = {**_default_settings, **_override_settings}

ACTION_FILE_FOLDER = _settings["ACTION_FILE_FOLDER"]
ROS_BAG_LOG_SAVE_PATH = _settings["ROS_BAG_LOG_SAVE_PATH"]
HUMANOID_ROBOT_SESSION_NAME = _settings["HUMANOID_ROBOT_SESSION_NAME"]
VR_REMOTE_CONTROL_SESSION_NAME = _settings["VR_REMOTE_CONTROL_SESSION_NAME"]
LAUNCH_HUMANOID_ROBOT_SIM_CMD = _settings["LAUNCH_HUMANOID_ROBOT_SIM_CMD"]
LAUNCH_HUMANOID_ROBOT_REAL_CMD = _settings["LAUNCH_HUMANOID_ROBOT_REAL_CMD"]
LAUNCH_VR_REMOTE_CONTROL_CMD = _settings["LAUNCH_VR_REMOTE_CONTROL_CMD"]
ROS_MASTER_URI = _settings["ROS_MASTER_URI"]
ROS_IP = _settings["ROS_IP"]
ROS_HOSTNAME = _settings["ROS_HOSTNAME"]
kuavo_ros_control_ws_path = _settings["KUAVO_ROS_CONTROL_WS_PATH"]

# 录制话题的格式
record_topics_path = os.path.join(config_dir, "record_topics.json")
with open(record_topics_path, "r") as f:
    record_topics = json.load(f)["record_topics"]
record_vr_rosbag_pid = None

# 自定义动作json文件
customize_config_path = os.path.join(config_dir, "customize_config.json")
# 定义质心规划类动作/步态切换类动作json文件
comGaitSwitch_config_path = os.path.join(config_dir, "com_gait_switch.json")

with open(customize_config_path, "r") as f:
    customize_config_data = json.load(f)

with open(comGaitSwitch_config_path, "r") as f:
    comGaitSwitch_config_data = json.load(f)

# 手臂状态定义
ROBOT_ACTION_STATUS = 0 # 手臂完成状态 | 0 没开始 | 1 执行中 |  2 完成

# 全局参数存储（用于call_sensor等服务）

def robot_action_state_callback(msg):
    global ROBOT_ACTION_STATUS
    ROBOT_ACTION_STATUS = msg.state

# 全局话题发布
joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
com_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)

# 控制拨杆
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7

COM_SQUAT_DATA = -0.25 # 下蹲的高度
COM_STAND_DATA = 0.0   # 站立的默认高度
COM_PITCH_DATA = 0.4   # 质心pitch角度为0.4
COM_PITCH_ZERO = 0.0   # 质心站立高度为0.0

# 服务处理函数 - 统一使用OCS2RobotControl格式
def handle_initial_pre_service(req):
    """处理initial_pre服务请求"""
    try:
        rospy.loginfo(f"Executing initial_pre service req={req}")
        launch_humanoid_robot(real_robot=True)
        return OCS2RobotControlResponse(success=True, message="Initial pre completed successfully")
    except Exception as e:
        rospy.logerr(f"Error in initial_pre service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_ready_stance_service(req):
    """处理ready_stance服务请求"""
    try:
        rospy.loginfo("Executing ready_stance service")
        call_real_initialize_srv()
        # return OCS2RobotControlResponse(success=True, message="Ready stance completed successfully")
    
        time.sleep(1)
        execute_function_by_name("call_sensor")#执行上位机请求服务
        time.sleep(1)
        execute_function_by_name("ar_control")#执行tf请求服务
        time.sleep(1)
        execute_function_by_name("switch_arm_ctrl_mode")#执行tf请求服务
        time.sleep(1)
        execute_function_by_name("start_pytrees_main")#执行主树移动服务


        # # rospy.loginfo("Executing ready_stance service")
        # # # call_real_initialize_srv()
        # # # time.sleep(3)
        # execute_function_by_name("call_sensor")#执行上位机请求服务
        time.sleep(1)

        return OCS2RobotControlResponse(success=True, message="Ready stance completed successfully")



    except Exception as e:
        rospy.logerr(f"Error in ready_stance service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_stance_service(req):
    """处理stance服务请求"""
    try:
        rospy.loginfo("Executing stance service")
        call_change_arm_ctrl_mode_service(1)
        return OCS2RobotControlResponse(success=True, message="Stance completed successfully")
    except Exception as e:
        rospy.logerr(f"Error in stance service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_walk_service(req):
    """处理walk服务请求"""
    try:
        rospy.loginfo("Executing walk service")
        call_change_arm_ctrl_mode_service(1)
        return OCS2RobotControlResponse(success=True, message="Walk started successfully")
    except Exception as e:
        rospy.logerr(f"Error in walk service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_stop_walk_service(req):
    """处理stop_walk服务请求"""
    try:
        rospy.loginfo("Executing stop_walk service")
        call_grab_box_service_stop()
        return OCS2RobotControlResponse(success=True, message="Walk stopped successfully")
    except Exception as e:
        rospy.logerr(f"Error in stop_walk service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_stop_service(req):
    """处理stop服务请求"""
    try:
        rospy.loginfo("Executing stop service")
        # kill humanoid_robot and vr_remote_control
        subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
                      stderr=subprocess.DEVNULL)
        subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME],
                      stderr=subprocess.DEVNULL)
        kill_record_vr_rosbag()

        manual_h12_init_state = rospy.get_param("manual_h12_init_state", "none")
        if "none" != manual_h12_init_state:
            subprocess.run(["rosnode", "kill", "/nodelet_manager"],
                        stderr=subprocess.DEVNULL)

        return OCS2RobotControlResponse(success=True, message="Stop completed successfully")
    except Exception as e:
        rospy.logerr(f"Error in stop service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_trot_service(req):
    """处理trot服务请求"""
    try:
        rospy.loginfo("Executing trot service")
        call_change_arm_ctrl_mode_service(1)
        return OCS2RobotControlResponse(success=True, message="Trot started successfully")
    except Exception as e:
        rospy.logerr(f"Error in trot service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_grab_service(req):
    """处理grab服务请求"""
    try:
        rospy.loginfo("Executing grab service")
        call_grab_box_service_start()
        return OCS2RobotControlResponse(success=True, message="Grab started successfully")
    except Exception as e:
        rospy.logerr(f"Error in grab service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_stop_grab_service(req):
    """处理stop_grab服务请求"""
    try:
        rospy.loginfo("Executing stop_grab service")
        call_grab_box_service_stop()
        return OCS2RobotControlResponse(success=True, message="Grab stopped successfully")
    except Exception as e:
        rospy.logerr(f"Error in stop_grab service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")




def handle_relocalize_service(req):
    """处理 relocalize 服务请求"""
    try:
        rospy.loginfo(f'Executing relocalize service req = {req}')
        RELOCALIZE_SESSION_NAME = "sps_relocalize"

        # 启动命令（上位机 relocalize）
        launch_cmd = "roslaunch h12pro_controller_node audio_control_client1.launch"
        print(f"launch_cmd: {launch_cmd}")

        # 杀掉旧的 relocalize session
        subprocess.run(["tmux", "kill-session", "-t", RELOCALIZE_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 新建 tmux session
        tmux_cmd = [
            "tmux", "new-session",
            "-s", RELOCALIZE_SESSION_NAME,
            "-d",
            f"bash -c -i 'source ~/.bashrc && \
              source {kuavo_ros_control_ws_path}/devel/setup.bash && \
              export ROS_MASTER_URI={ROS_MASTER_URI} && \
              export ROS_IP={ROS_IP} && \
              export ROS_HOSTNAME={ROS_HOSTNAME} && \
              {launch_cmd}; exec bash'"
        ]

        process = subprocess.Popen(tmux_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(3)

        # 检查是否成功
        result = subprocess.run(["tmux", "has-session", "-t", RELOCALIZE_SESSION_NAME],
                                capture_output=True)
        if result.returncode == 0:
            print(f"Started relocalize in tmux session: {RELOCALIZE_SESSION_NAME}")
            return OCS2RobotControlResponse(success=True, message="Relocalize started successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start relocalize tmux session")

    except Exception as e:
        rospy.logerr(f"Error in relocalize service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")


def handle_start_back_case_service(req):
    """处理 start_back_case 服务请求"""
    try:
        rospy.loginfo(f'Executing start_back_case service req = {req}')
        GRASP_BOX_SESSION_NAME = "claw_box_example"

        # 杀掉旧的 session
        subprocess.run(["tmux", "kill-session", "-t", GRASP_BOX_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 从请求对象中获取参数
        x = req.start_x
        y = req.start_y
        z = req.start_z

        rospy.loginfo(f"Start place_case: place_pose=({x}, {y}, {z})")

        if rospy.has_param('/task_status_back'):
            rospy.delete_param('/task_status_back')
        if rospy.has_param('/task_status'):
            rospy.delete_param('/task_status')
        if rospy.has_param('/barcode_value'):
            rospy.delete_param('/barcode_value')

        # 构建命令
        place_script_path = (
            f"{kuavo_ros_control_ws_path}/src/kuavo_humanoid_sdk/"
            f"kuavo_humanoid_sdk/kuavo_strategy_pytree/place_claw_box.py"
        )

        # 构建执行命令
        place_cmd = (
            f"python3 {place_script_path}"
            f" --place-x {x}"
            f" --place-y {y}"
            f" --place-theta {z}"
        )

        tmux_cmd3 = [
            "tmux", "new-session",
            "-s", GRASP_BOX_SESSION_NAME,
            "-d",
            f"bash -c -i 'source ~/.bashrc && "
            f"source {kuavo_ros_control_ws_path}/devel/setup.bash && "
            f"export ROS_MASTER_URI={ROS_MASTER_URI} && "
            f"export ROS_IP={ROS_IP} && "
            f"export ROS_HOSTNAME={ROS_HOSTNAME} && "
            f"cd {kuavo_ros_control_ws_path} && "
            f"sleep 2 && "
            f"{place_cmd}; exec bash'"
        ]

        process3 = subprocess.Popen(tmux_cmd3, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(3)

        # 检查 session 是否成功创建
        result3 = subprocess.run(["tmux", "has-session", "-t", GRASP_BOX_SESSION_NAME],
                                 capture_output=True)

        if result3.returncode == 0:
            rospy.loginfo(f"Started claw_box_example in tmux session: {GRASP_BOX_SESSION_NAME}")
            return OCS2RobotControlResponse(success=True, message="Start back_case completed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start back_case tmux session")

    except Exception as e:
        rospy.logerr(f"Error in start_back_case service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_start_case_service(req):
    """处理 start_case 服务请求"""
    try:
        rospy.loginfo(f"Executing start_case service req = {req}")

        GRASP_BOX_SESSION_NAME = "claw_box_example"

        # 1. 杀掉旧的 session
        subprocess.run(["tmux", "kill-session", "-t", GRASP_BOX_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 2. 获取参数
        target_id = req.id
        count = req.loop_count

        rospy.loginfo(f"Start case parameters: target_id={target_id}, grab_count={count}")

        if rospy.has_param('/task_status_back'):
            rospy.delete_param('/task_status_back')
        if rospy.has_param('/task_status'):
            rospy.delete_param('/task_status')
        if rospy.has_param('/barcode_value'):
            rospy.delete_param('/barcode_value')

        # 3. 构建执行命令
        case_cmd = (
            f"python3 {kuavo_ros_control_ws_path}/src/kuavo_humanoid_sdk/"
            f"kuavo_humanoid_sdk/kuavo_strategy_pytree/start_claw_box.py"
            f" --target-id {target_id}"
            f" --count {count}"
        )

        # 4. 创建新的 tmux 会话并执行
        tmux_cmd = [
            "tmux", "new-session",
            "-s", GRASP_BOX_SESSION_NAME,
            "-d",
            f"bash -c -i '"
            f"source ~/.bashrc && "
            f"source {kuavo_ros_control_ws_path}/devel/setup.bash && "
            f"export ROS_MASTER_URI={ROS_MASTER_URI} && "
            f"export ROS_IP={ROS_IP} && "
            f"export ROS_HOSTNAME={ROS_HOSTNAME} && "
            f"cd {kuavo_ros_control_ws_path} && "
            f"sleep 1 && "
            f"{case_cmd}; "
            f"exec bash'"
        ]

        subprocess.Popen(tmux_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 5. 等待会话启动
        time.sleep(2)

        # 检查 session 创建是否成功
        result = subprocess.run(
            ["tmux", "has-session", "-t", GRASP_BOX_SESSION_NAME],
            capture_output=True
        )

        if result.returncode == 0:
            rospy.loginfo("Started case.py in tmux session: claw_box_example")
            return OCS2RobotControlResponse(success=True, message="Start case completed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start case tmux session")

    except Exception as e:
        rospy.logerr(f"Error in start_case service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_start_pytrees_main_service(req):
    """处理 start_pytrees_main 服务请求"""
    try:
        rospy.loginfo(f"Executing start_pytrees_main service req = {req}")

        PYTREES_MAIN_SESSION_NAME = "pytrees_main"

        # 1. 杀掉旧的 session
        subprocess.run(["tmux", "kill-session", "-t", PYTREES_MAIN_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 2. 构建执行命令
        pytrees_cmd = f"python3 src/pytrees_actions/main.py"

        # 3. 创建新的 tmux 会话并执行
        tmux_cmd = [
            "tmux", "new-session",
            "-s", PYTREES_MAIN_SESSION_NAME,
            "-d",
            f"bash -c -i '"
            f"source ~/.bashrc && "
            f"source {kuavo_ros_control_ws_path}/devel/setup.bash && "
            f"export ROS_MASTER_URI={ROS_MASTER_URI} && "
            f"export ROS_IP={ROS_IP} && "
            f"export ROS_HOSTNAME={ROS_HOSTNAME} && "
            f"cd {kuavo_ros_control_ws_path} && "
            f"sleep 1 && "
            f"{pytrees_cmd}; "
            f"exec bash'"
        ]

        subprocess.Popen(tmux_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 4. 等待会话启动
        time.sleep(2)

        # 5. 检查 session 创建是否成功
        result = subprocess.run(
            ["tmux", "has-session", "-t", PYTREES_MAIN_SESSION_NAME],
            capture_output=True
        )

        if result.returncode == 0:
            rospy.loginfo("Started pytrees_actions/main.py in tmux session: pytrees_main")
            return OCS2RobotControlResponse(success=True, message="Start pytrees main completed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start pytrees main tmux session")

    except Exception as e:
        rospy.logerr(f"Error in start_pytrees_main service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")


def handle_reset_case_service(req):
    """处理 reset_case 服务请求"""
    try:
        rospy.loginfo(f"Executing reset_case service req = {req}")

        GRASP_BOX_SESSION_NAME = "claw_box_example"

        # 1. 杀掉旧的 session
        subprocess.run(["tmux", "kill-session", "-t", GRASP_BOX_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 3. 构建执行命令
        case_cmd = (
            f"python3 {kuavo_ros_control_ws_path}/src/pytrees_actions/main.py "
        )

        # 4. 创建新的 tmux 会话并执行
        tmux_cmd = [
            "tmux", "new-session",
            "-s", GRASP_BOX_SESSION_NAME,
            "-d",
            f"bash -c -i '"
            f"source ~/.bashrc && "
            f"source {kuavo_ros_control_ws_path}/devel/setup.bash && "
            f"export ROS_MASTER_URI={ROS_MASTER_URI} && "
            f"export ROS_IP={ROS_IP} && "
            f"export ROS_HOSTNAME={ROS_HOSTNAME} && "
            f"cd {kuavo_ros_control_ws_path} && "
            f"sleep 1 && "
            f"{case_cmd}; "
            f"exec bash'"
        ]

        subprocess.Popen(tmux_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 5. 等待会话启动
        time.sleep(2)

        # 检查 session 创建是否成功
        result = subprocess.run(
            ["tmux", "has-session", "-t", GRASP_BOX_SESSION_NAME],
            capture_output=True
        )

        if result.returncode == 0:
            rospy.loginfo("Started reset_case.py in tmux session: claw_box_example")
            return OCS2RobotControlResponse(success=True, message="reset case completed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to reset case tmux session")

    except Exception as e:
        rospy.logerr(f"Error in reset_case service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")


def handle_system_alarm_service(req):
    """处理系统报警服务请求"""
    global music_trigger_pub
    try:
        rospy.loginfo(f"Executing system_alarm service req = {req}")

        # 使用 loop_count 作为报警触发编码
        trigger_num = req.loop_count
        rospy.loginfo(f"system_alarm trigger_num(loop_count)={trigger_num}")

        # 边界保护（Int8 范围 -128 ~ 127）
        if trigger_num < -128 or trigger_num > 127:
            rospy.logwarn(f"trigger_num {trigger_num} out of Int8 range, clamped")
            trigger_num = max(-128, min(127, trigger_num))

        # 构造消息并发布
        msg = Int8()
        msg.data = trigger_num

        if music_trigger_pub is None:
            rospy.logerr("music_trigger_pub is not initialized!")
            return OCS2RobotControlResponse(
                success=False,
                message="Publisher not initialized"
            )

        music_trigger_pub.publish(msg)
        rospy.loginfo(f"Published /music_trigger:{trigger_num}")

        return OCS2RobotControlResponse(
            success=True,
            message=f"System alarm triggered with code {trigger_num}"
        )

    except Exception as e:
        rospy.logerr(f"Error in system_alarm service: {e}")
        return OCS2RobotControlResponse(
            success=False,
            message=f"Error: {str(e)}"
        )




def handle_stop_case_service(req):
    """处理stop_case服务请求"""
    try:
        GRASP_BOX_SESSION_NAME = "claw_box_example"

        # 杀掉会话
        subprocess.run(["tmux", "kill-session", "-t", GRASP_BOX_SESSION_NAME],
                      stderr=subprocess.DEVNULL)

        rospy.loginfo(f"Stopped case_sps.py tmux session: {GRASP_BOX_SESSION_NAME}")
        return OCS2RobotControlResponse(success=True, message="Stopped case_sps.py successfully")

    except Exception as e:
        rospy.logerr(f"Error in stop_case service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")


def handle_get_barcode_service(req):
    """处理get_barcode服务请求，从参数服务器读取条形码值并返回"""
    try:
        rospy.loginfo("Executing get_barcode service")

        # 从参数服务器读取条形码值，如果不存在则使用默认值0
        barcode_value = rospy.get_param("barcode_value", "0")

        rospy.loginfo(f"Barcode value retrieved from parameter server: {barcode_value}")
        return GetBarcodeResponse(success=True, barcode_value=str(barcode_value), message="Barcode value retrieved successfully")
    except Exception as e:
        rospy.logerr(f"Error in get_barcode service: {e}")
        return GetBarcodeResponse(success=False, barcode_value="error", message=f"Error: {str(e)}")

def handle_get_task_status_service(req):
    """处理 get_task_status 服务请求，返回当前任务执行状态"""
    try:
        rospy.loginfo("Executing get_task_status service")

        # 假设任务状态存储在参数服务器 task_status 中
        # 是字符串，例如 "running", "done"

        task_status = rospy.get_param("task_status", "none")

        rospy.loginfo(f"Current task status: {task_status}")
        return GetBarcodeResponse(success=True, barcode_value=str(task_status), message="Task status retrieved successfully")
    except Exception as e:
        rospy.logerr(f"Error in get_task_status service: {e}")
        return GetBarcodeResponse(success=False, barcode_value="error", message=f"Error: {str(e)}")


def handle_get_backtask_status_service(req):
    """处理 get_task_status 服务请求，返回当前任务执行状态"""
    try:
        rospy.loginfo("Executing get_task_status service")

        # 假设任务状态存储在参数服务器 task_status 中
        # 是字符串，例如 "running", "done"

        task_status = rospy.get_param("task_status_back", "none")

        rospy.loginfo(f"Current task status: {task_status}")
        return GetBarcodeResponse(success=True, barcode_value=str(task_status), message="Task status retrieved successfully")
    except Exception as e:
        rospy.logerr(f"Error in get_task_status service: {e}")
        return GetBarcodeResponse(success=False, barcode_value="error", message=f"Error: {str(e)}")




def handle_call_sensor_service(req):
    """处理call_sensor服务请求"""
    try:
        rospy.loginfo(f'Executing call_sensor111 service req = {req}')
        # 定义session名称
        AUDIO_CONTROL_SESSION_NAME = "audio_control_client"
        # Ar_CONTROL_SESSION_NAME = "ar_session"
        # GRASP_BOX_SESSION_NAME = "claw_box_example"

        # 构建launch命令
        launch_cmd = f"roslaunch h12pro_controller_node audio_control_client.launch"

        # print(f"launch_cmd: {launch_cmd}")

        # 先杀死已存在的sessions
        subprocess.run(["tmux", "kill-session", "-t", AUDIO_CONTROL_SESSION_NAME],
                      stderr=subprocess.DEVNULL)
        # subprocess.run(["tmux", "kill-session", "-t", Ar_CONTROL_SESSION_NAME],
        #               stderr=subprocess.DEVNULL)

        # 创建第一个tmux session来执行audio_control_client.launch
        tmux_cmd1 = [
            "tmux", "new-session",
            "-s", AUDIO_CONTROL_SESSION_NAME,
            "-d",
            f"bash -c -i 'source ~/.bashrc && \
              source {kuavo_ros_control_ws_path}/devel/setup.bash && \
              export ROS_MASTER_URI={ROS_MASTER_URI} && \
              export ROS_IP={ROS_IP} && \
              export ROS_HOSTNAME={ROS_HOSTNAME} &&\
              {launch_cmd}; exec bash'"
        ]

        #  # 创建第二个tmux session来执行audio_control_client.launch
        # tmux_cmd2 = [
        #     "tmux", "new-session",
        #     "-s", Ar_CONTROL_SESSION_NAME,
        #     "-d",
        #     f"bash -c -i 'source ~/.bashrc && \
        #       source {kuavo_ros_control_ws_path}/devel/setup.bash && \
        #       export ROS_MASTER_URI={ROS_MASTER_URI} && \
        #       export ROS_IP={ROS_IP} && \
        #       export ROS_HOSTNAME={ROS_HOSTNAME} &&\
        #       roslaunch ar_control robot_strategies.launch real:=true; exec bash'"
        # ]

        # 启动三个进程
        process1 = subprocess.Popen(tmux_cmd1, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # process2 = subprocess.Popen(tmux_cmd2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(3)

        # 检查sessions是否成功创建
        result1 = subprocess.run(["tmux", "has-session", "-t", AUDIO_CONTROL_SESSION_NAME],
                                  capture_output=True)
        # result2 = subprocess.run(["tmux", "has-session", "-t", Ar_CONTROL_SESSION_NAME],
        #                           capture_output=True)

        if result1.returncode == 0 :
            print(f"Started audio_control_client in tmux session: {AUDIO_CONTROL_SESSION_NAME}")
            # print(f"Started ar_control in tmux session: {Ar_CONTROL_SESSION_NAME}")
            return OCS2RobotControlResponse(success=True, message="Call sensor completed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start tmux sessions1")

    except Exception as e:
        rospy.logerr(f"Error in call_sensor service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def ar_control_service(req):
    """处理ar_control服务请求"""
    try:
        rospy.loginfo(f"Executing ar_control service req={req}")

        AR_CONTROL_SESSION_NAME = "ar_control_node"

        # 1. 杀掉旧的 session
        subprocess.run(["tmux", "kill-session", "-t", AR_CONTROL_SESSION_NAME],
                       stderr=subprocess.DEVNULL)

        # 2. 构建执行命令
        ar_cmd = "rosrun ar_control ar_control_node.py"

        # 3. 创建新的 tmux 会话并执行
        tmux_cmd = [
            "tmux", "new-session",
            "-s", AR_CONTROL_SESSION_NAME,
            "-d",
            f"bash -c -i '"
            f"source ~/.bashrc && "
            f"source {kuavo_ros_control_ws_path}/devel/setup.bash && "
            f"export ROS_MASTER_URI={ROS_MASTER_URI} && "
            f"export ROS_IP={ROS_IP} && "
            f"export ROS_HOSTNAME={ROS_HOSTNAME} && "
            f"cd {kuavo_ros_control_ws_path} && "
            f"sleep 1 && "
            f"{ar_cmd}; "
            f"exec bash'"
        ]

        subprocess.Popen(tmux_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 4. 等待会话启动
        time.sleep(2)

        # 5. 检查 session 创建是否成功
        result = subprocess.run(
            ["tmux", "has-session", "-t", AR_CONTROL_SESSION_NAME],
            capture_output=True
        )

        if result.returncode == 0:
            rospy.loginfo(f"Started ar_control_node in tmux session: {AR_CONTROL_SESSION_NAME}")
            return OCS2RobotControlResponse(success=True, message="AR control started successfully")
        else:
            return OCS2RobotControlResponse(success=False, message="Failed to start ar_control tmux session")

    except Exception as e:
        rospy.logerr(f"Error in ar_control service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_move_to_target_service(req):
    """处理move_to_target服务请求，调用JiBotWaitAndMoveToTarget节点的服务（节点会负责调用底盘移动服务）"""
    try:
        rospy.loginfo(f"Executing move_to_target service req={req}")

        # 从请求中获取参数
        # start_x, start_y, start_z -> target_x, target_y, target_theta
        target_x = req.start_x
        target_y = req.start_y
        target_theta = req.start_z  # 使用start_z作为theta

        rospy.loginfo(f"Move to target: x={target_x}, y={target_y}, theta={target_theta}")

        # 调用JiBotWaitAndMoveToTarget节点的移动服务
        move_service_name = '/jibot_set_move_target'
        try:
            rospy.wait_for_service(move_service_name, timeout=5.0)
            move_service_proxy = rospy.ServiceProxy(move_service_name, SetMoveTarget)

            # 创建移动请求
            move_req = SetMoveTargetRequest()
            move_req.target_pose = Float64MultiArray()
            move_req.target_pose.data = [target_x, target_y, target_theta]

            # 调用移动服务（节点会调用底盘移动服务）
            rospy.loginfo(f"调用移动服务 {move_service_name}: x={target_x}, y={target_y}, theta={target_theta}")
            move_resp = move_service_proxy(move_req)

            if move_resp.success:
                rospy.loginfo(f"移动请求发送成功: {move_resp.message}")
                return OCS2RobotControlResponse(success=True, message=f"移动请求已发送: {move_resp.message}")
            else:
                rospy.logerr(f"移动请求发送失败: {move_resp.message}")
                return OCS2RobotControlResponse(success=False, message=f"移动请求发送失败: {move_resp.message}")

        except rospy.ROSException as e:
            rospy.logerr(f"等待移动服务 {move_service_name} 超时: {e}")
            return OCS2RobotControlResponse(success=False, message=f"服务 {move_service_name} 不可用: {e}")
        except Exception as e:
            rospy.logerr(f"调用移动服务时出错: {e}")
            return OCS2RobotControlResponse(success=False, message=f"调用移动服务时出错: {e}")

    except Exception as e:
        rospy.logerr(f"Error in move_to_target service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

def handle_switch_arm_ctrl_mode_service(req):
    """处理切换手臂控制模式服务请求"""
    try:
        rospy.loginfo("Executing switch_arm_ctrl_mode service")

        service_name1 = "/enable_lb_arm_quick_mode"
        service_name2 = "/enable_vel_control"

        # 调用第一个服务：enable_lb_arm_quick_mode
        try:
            # 等待服务可用
            rospy.wait_for_service(service_name1, timeout=5.0)

            # 创建服务代理，数据类型为 std_srvs/SetBool
            arm_mode_service = rospy.ServiceProxy(service_name1, SetBool)

            # 调用服务，传递 data: true
            response1 = arm_mode_service(data=True)

            # 检查服务调用返回的结果
            if not response1.success:
                rospy.logwarn(f"Service {service_name1} call failed: {response1.message}")
                return OCS2RobotControlResponse(success=False, message=f"Service {service_name1} call failed: {response1.message}")

            rospy.loginfo(f"Service {service_name1} call successful")

        except rospy.ROSException as e:
            rospy.logerr(f"Service {service_name1} not available: {e}")
            return OCS2RobotControlResponse(success=False, message=f"Service {service_name1} not available: {e}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service {service_name1} call failed: {e}")
            return OCS2RobotControlResponse(success=False, message=f"Service {service_name1} call failed: {e}")

        # 调用第二个服务：enable_vel_control
        try:
            # 等待服务可用
            rospy.wait_for_service(service_name2, timeout=5.0)

            # 创建服务代理，数据类型为 std_srvs/SetBool
            vel_control_service = rospy.ServiceProxy(service_name2, SetBool)

            # 调用服务，传递 data: false
            response2 = vel_control_service(data=False)

            # 检查服务调用返回的结果
            if not response2.success:
                rospy.logwarn(f"Service {service_name2} call failed: {response2.message}")
                return OCS2RobotControlResponse(success=False, message=f"Service {service_name2} call failed: {response2.message}")

            rospy.loginfo(f"Service {service_name2} call successful")
            return OCS2RobotControlResponse(success=True, message="Switch arm control mode completed successfully")

        except rospy.ROSException as e:
            rospy.logerr(f"Service {service_name2} not available: {e}")
            return OCS2RobotControlResponse(success=False, message=f"Service {service_name2} not available: {e}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service {service_name2} call failed: {e}")
            return OCS2RobotControlResponse(success=False, message=f"Service {service_name2} call failed: {e}")

    except Exception as e:
        rospy.logerr(f"Error in switch_arm_ctrl_mode service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

# 服务映射字典
service_handlers = {
    "initial_pre": handle_initial_pre_service,
    "ready_stance": handle_ready_stance_service,
    # "stance": handle_stance_service,
    # "walk": handle_walk_service,
    # "stop_walk": handle_stop_walk_service,
    "stop": handle_stop_service,
    # "trot": handle_trot_service,
    "grab": handle_grab_service,
    "stop_grab": handle_stop_grab_service,
    "call_sensor": handle_call_sensor_service,
    # "relocalize": handle_relocalize_service,
    "start_case": handle_start_case_service,
    "start_pytrees_main": handle_start_pytrees_main_service,
    "ar_control":ar_control_service,
    "stop_case": handle_stop_case_service ,
    "start_back_case": handle_start_back_case_service,
    "move_to_target": handle_move_to_target_service,
    "switch_arm_ctrl_mode": handle_switch_arm_ctrl_mode_service,
    # "system_alarm": handle_system_alarm_service,
    # "reset_case":handle_reset_case_service

}

# 条形码服务处理函数（使用不同的服务类型）
barcode_service_handler = handle_get_barcode_service
#任务执行状态处理函数（使用不同的服务类型）
task_status_handler = handle_get_task_status_service
#任务执行状态处理函数（使用不同的服务类型）
backtask_status_handler = handle_get_backtask_status_service

def execute_function_by_name(function_name,
                           id="", start_x=0.0, start_y=0.0, start_z=0.0, pick_x=0.0, pick_y=0.0, pick_z=0.0, loop_count=1):
    """根据函数名执行对应的函数"""
    if function_name in service_handlers:
        try:
            # 创建虚拟的请求对象 - 使用OCS2RobotControlRequest格式
            req = OCS2RobotControlRequest()
            req.function_name = function_name
            # req.param1 = param1
            # req.param2 = param2
            # req.param3 = param3
            # req.param4 = param4
            # req.param5 = param5
            # 新增参数
            req.id = id
            req.start_x = start_x
            req.start_y = start_y
            req.start_z = start_z
            req.pick_x = pick_x
            req.pick_y = pick_y
            req.pick_z = pick_z
            req.loop_count = loop_count
            # 注意：target_x, target_y, target_theta使用start_x, start_y, start_z传递
            # target_smt_id使用loop_count传递，target_smt_id_row使用pick_x传递

            print("zouzhebian")

            response = service_handlers[function_name](req)
            rospy.loginfo(f"Function {function_name} executed111: {response.message}")
            return response.success
        except Exception as e:
            rospy.logerr(f"Error executing function {function_name}: {e}")
            return False
    else:
        rospy.logerr(f"Unknown function: {function_name}")
        return False

# 主要服务处理函数 - 使用自定义的OCS2RobotControl服务
def handle_ocs2_robot_control_service(req):
    """主要的OCS2机器人控制服务处理函数"""
    try:
        # 从请求中获取要执行的函数名和参数
        function_name = req.function_name
        # param1 = req.param1
        # param2 = req.param2
        # param3 = req.param3
        # param4 = req.param4
        # param5 = req.param5
        # 新增参数
        id = req.id
        start_x = req.start_x
        start_y = req.start_y
        start_z = req.start_z
        pick_x = req.pick_x
        pick_y = req.pick_y
        pick_z = req.pick_z
        loop_count = req.loop_count

        rospy.loginfo(f"Received request to execute function: {function_name}")
        rospy.loginfo(f"New parameters: id={id}, start_position=({start_x}, {start_y}, {start_z}), pick_position=({pick_x}, {pick_y}, {pick_z}), loop_count={loop_count}")
        if function_name == "move_to_target":
            rospy.loginfo(f"Move target: start_x={start_x}, start_y={start_y}, start_z={start_z} (作为target_x, target_y, target_theta)")
            if pick_y >= 0:
                rospy.loginfo(f"SMT parameters: pick_y={pick_y} (作为target_smt_id，将转换为int), id={id} (作为target_smt_id_row，将转换为int)")

        # 执行对应的函数
        success = execute_function_by_name(function_name,
                                         id, start_x, start_y, start_z, pick_x, pick_y, pick_z, loop_count)

        if success:
            return OCS2RobotControlResponse(success=True, message=f"Function {function_name} executed successfully")
        else:
            return OCS2RobotControlResponse(success=False, message=f"Failed to execute function {function_name}")

    except Exception as e:
        rospy.logerr(f"Error in ocs2_robot_control service: {e}")
        return OCS2RobotControlResponse(success=False, message=f"Error: {str(e)}")

# 辅助函数
def call_real_initialize_srv():
    """调用机器人初始化服务"""
    from std_srvs.srv import Trigger, TriggerRequest
    client = rospy.ServiceProxy('/humanoid_controller/real_initial_start', Trigger)
    req = TriggerRequest()

    try:
        # Call the service
        if client.call(req):
            rospy.loginfo("[JoyControl] Service call successful")
        else:
            rospy.logerr("Failed to callRealInitializeSrv service")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    """调用手臂控制模式切换服务"""
    rospy.loginfo("call_change_arm_ctrl_mode_service")
    result = True
    service_name = "/grab_box_demo/control_bt"  # 服务名称
    try:
        rospy.wait_for_service(service_name, timeout=0.5)

        # 创建服务代理，假设数据类型为 `std_srvs/SetBool`
        control_bt_service = rospy.ServiceProxy(service_name, SetBool)

        # 调用服务，传递数据
        response = control_bt_service(data=True)  # 传递 `data: true`

        # 检查服务调用返回的结果（如果有的话）
        if response.success:
            rospy.loginfo("Control BT Service call successful")
        else:
            rospy.logwarn("Control BT Service call failed with response: %s", response.message)

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result

def call_grab_box_service_start():
    """启动抓取盒子服务"""
    rospy.loginfo("call_grab_box_service_start")
    try:
        cmd = 'rosservice call /grab_box_demo/control_bt "data: true"'
        result = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, timeout=5)
        rospy.loginfo(f"系统调用成功: {result.decode()}")
        return True
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"系统调用失败[代码{e.returncode}]: {e.output.decode()}")
    except Exception as e:
        rospy.logerr(f"系统调用异常: {str(e)}")
    return False

def call_grab_box_service_stop():
    """停止抓取盒子服务"""
    rospy.loginfo("call_grab_box_service_stop")
    try:
        cmd = 'rosservice call /grab_box_demo/control_bt "data: false"'
        result = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, timeout=5)
        rospy.loginfo(f"系统调用成功: {result.decode()}")
        return True
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"系统调用失败[代码{e.returncode}]: {e.output.decode()}")
    except Exception as e:
        rospy.logerr(f"系统调用异常: {str(e)}")

    try:
        cmd = 'rosservice call /grab_box_demo/reset_bt "data: true"'
        result = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, timeout=5)
        rospy.loginfo(f"系统调用成功: {result.decode()}")
        return True
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"系统调用失败[代码{e.returncode}]: {e.output.decode()}")
    except Exception as e:
        rospy.logerr(f"系统调用异常: {str(e)}")

    return False

def launch_humanoid_robot(real_robot=True, calibrate=False):
    """启动人形机器人"""
    robot_version = os.getenv('ROBOT_VERSION')
    print(f"current robot version: {robot_version}")
    subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
                    stderr=subprocess.DEVNULL)

    if real_robot:
        launch_cmd = LAUNCH_HUMANOID_ROBOT_REAL_CMD
    else:
        launch_cmd = LAUNCH_HUMANOID_ROBOT_SIM_CMD

    if calibrate:
        launch_cmd += " cali:=true cali_arm:=true"

    # 通过读取kuavo.json文件，获取only_half_up_body参数，在launch_cmd中添加only_half_up_body:=true
    kuavo_json = os.path.join(kuavo_ros_control_ws_path, "src", "kuavo_assets", "config", f"kuavo_v{robot_version}", "kuavo.json")
    if not os.path.exists(kuavo_json):
        print(f"Error: Could not find {kuavo_json}")
        raise Exception(f"Error: Could not find {kuavo_json}")
    with open(kuavo_json, "r") as f:
        kuavo_json_data = json.load(f)
    only_half_up_body = kuavo_json_data["only_half_up_body"]
    if only_half_up_body:
        launch_cmd += " only_half_up_body:=true"

    print(f"launch_cmd: {launch_cmd}")
    print("If you want to check the session, please run 'tmux attach -t humanoid_robot'")
    tmux_cmd = [
        "sudo", "tmux", "new-session",
        "-s", HUMANOID_ROBOT_SESSION_NAME,
        "-d",
        f"source ~/.bashrc && \
            source {kuavo_ros_control_ws_path}/devel/setup.bash && \
            export ROS_MASTER_URI={ROS_MASTER_URI} && \
            export ROS_IP={ROS_IP} && \
            export ROS_HOSTNAME={ROS_HOSTNAME} &&\
            export ROBOT_VERSION={robot_version} && \
            {launch_cmd}; exec bash"
    ]

    process = subprocess.Popen(
        tmux_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )


    for i in range(10):  # 最多等 10 次
        result = subprocess.run(["sudo","tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        rospy.loginfo(f"循环次数: {i}")
        if result.returncode == 0:
            # print("tmux 会话已启动成功")
            print(f"Started humanoid_robot in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
            break
        rospy.sleep(3)
    else:
        print("Failed to start humanoid_robot")
        raise Exception("Failed to start humanoid_robot")



    # result = subprocess.run(["tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
    #                         capture_output=True)
    # rospy.loginfo(f"报错之处Test: {result}")
    # if result.returncode == 0:
    #     print(f"Started humanoid_robot in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
    # else:
    #     print("Failed to start humanoid_robot")
    #     raise Exception("Failed to start humanoid_robot")

def kill_record_vr_rosbag():
    """停止VR录制rosbag"""
    global record_vr_rosbag_pid
    if record_vr_rosbag_pid:
        os.kill(record_vr_rosbag_pid, signal.SIGINT)
        record_vr_rosbag_pid = None

def main():
    """主函数"""
    global music_trigger_pub
    rospy.init_node('ocs2_robot_control_server')

    music_trigger_pub = rospy.Publisher('/music_trigger', Int8, queue_size=10)

     # 初始化参数服务器条形码默认值
    if not rospy.has_param("barcode_value"):
        rospy.set_param("barcode_value", "0")
        rospy.loginfo("Initialized barcode_value parameter to default value: 0")


    # 订阅机器人动作状态话题
    rospy.Subscriber('/robot_action_state', RobotActionState, robot_action_state_callback)

    # 创建服务 - 统一使用OCS2RobotControl格式
    services = {}
    for service_name in service_handlers.keys():
        service_path = f"ocs2_robot_control/{service_name}"
        services[service_name] = rospy.Service(service_path, OCS2RobotControl, service_handlers[service_name])
        rospy.loginfo(f"Created service: /{service_path}")

    # 创建条形码服务 - 使用GetBarcode服务类型
    barcode_service = rospy.Service('ocs2_robot_control/get_barcode', GetBarcode, barcode_service_handler)

    # 创建任务执行状态服务 - 使用GetBarcode服务类型task_status_handle
    task_status_service = rospy.Service('ocs2_robot_control/get_task_status', GetBarcode, task_status_handler)
    # 创建任务执行状态服务 - 使用GetBarcode服务类型backtask_status_handle
    backtask_status_service = rospy.Service('ocs2_robot_control/get_backtask_status', GetBarcode, backtask_status_handler)

    rospy.loginfo("Created barcode service: /ocs2_robot_control/get_barcode")
    rospy.loginfo("Created barcode service: /ocs2_robot_control/get_task_status")
    rospy.loginfo("Created barcode service: /ocs2_robot_control/get_backtask_status")

    # 创建主要的控制服务 - 使用自定义的OCS2RobotControl服务类型
    # main_service = rospy.Service('ocs2_robot_control/control', OCS2RobotControl, handle_ocs2_robot_control_service)
    # rospy.loginfo("Created main service: /ocs2_robot_control/control")

    rospy.loginfo("OCS2 Robot Control Server started")
    rospy.spin()

if __name__ == '__main__':
    main()