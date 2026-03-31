import subprocess
import rospy
import os
import collections
from rich import console
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import jointBezierTrajectory, bezierCurveCubicPoint
from kuavo_msgs.srv import changeArmCtrlMode, switchToNextController, getControllerList, switchController, SetString
from utils.utils import get_start_end_frame_time, frames_to_custom_action_data_ocs2
import time
import signal
import datetime
import json
from std_srvs.srv import Trigger, TriggerRequest

import threading
try:
    import serial
except ImportError:
    import subprocess
    import sys
    print("pyserial 库未安装，正在尝试安装...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
from h12pro_controller_node.srv import playmusic, playmusicRequest, playmusicResponse
from h12pro_controller_node.srv import ExecuteArmAction, ExecuteArmActionRequest, ExecuteArmActionResponse
from h12pro_controller_node.msg import RobotActionState
import os
# import netifaces
import json
import hashlib
import math
import re
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

console = console.Console()

def encode_wifi_name_base64(name):
    """将 WiFi 名称编码为 base64，避免 JSON 特殊字符问题
    :param name: 原始 WiFi 名称
    :return: base64 编码后的字符串，如果输入为 None 则返回 None
    """
    if name is None:
        return None

    try:
        import base64
        # 确保是字符串类型
        if isinstance(name, bytes):
            name_bytes = name
        else:
            name_bytes = name.encode('utf-8')

        # 编码为 base64
        return base64.b64encode(name_bytes).decode('ascii')
    except Exception as e:
        print(f"WiFi 名称 base64 编码失败: {e}")
        return None

def get_wifi_password(ssid):
    """获取指定 WiFi 的密码
    :param ssid: WiFi 名称（原始名称，非 base64）
    :return: base64 编码的密码，如果获取失败返回 None
    """
    if ssid is None:
        return None

    try:
        import base64
        # 使用 nmcli 获取已保存的 WiFi 密码（需要 root 权限）
        result = subprocess.run(
            ["nmcli", "-s", "-g", "802-11-wireless-security.psk", "connection", "show", ssid],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            password = result.stdout.strip()
            # 使用 base64 编码
            return base64.b64encode(password.encode('utf-8')).decode('ascii')
    except Exception as e:
        print(f"获取 WiFi 密码失败: {e}")

    return None

def get_wifi_info():
    """获取当前 WiFi 连接信息（SSID、IP 地址和密码）
    :return: dict, 包含 wifi_name、robot_ip 和 wifi_password 的字典
    """
    wifi_info = {"wifi_name": None, "robot_ip": None, "wifi_password": None}
    wifi_interface = None
    raw_ssid = None  # 保存原始 SSID 用于获取密码

    # 获取 WiFi 接口名称和 SSID
    try:
        # 优先使用 iwgetid 获取 SSID（更可靠，不会有分隔符问题）
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            raw_ssid = result.stdout.strip()
            wifi_info["wifi_name"] = encode_wifi_name_base64(raw_ssid)
            # 获取无线接口名
            iface_result = subprocess.run(
                ["iwgetid"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if iface_result.returncode == 0 and iface_result.stdout:
                wifi_interface = iface_result.stdout.split()[0]
    except Exception as e:
        print(f"使用 iwgetid 获取 WiFi SSID 失败: {e}")

    # 如果 iwgetid 失败，回退到 nmcli
    if wifi_info["wifi_name"] is None:
        try:
            # 使用 nmcli 获取当前活动的 WiFi 连接信息
            # 注意：nmcli -t 使用 : 作为分隔符，如果 SSID 包含 : 会导致分割错误
            # 所以使用 -e yes 来转义特殊字符
            result = subprocess.run(
                ["nmcli", "-t", "-e", "yes", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    # nmcli -e yes 会将 : 转义为 \:，所以需要先处理转义
                    # 使用正则表达式分割，忽略转义的冒号
                    import re
                    # 分割时忽略 \: （转义的冒号）
                    parts = re.split(r'(?<!\\):', line)
                    # 还原转义的冒号
                    parts = [p.replace('\\:', ':') for p in parts]
                    if len(parts) >= 4 and parts[1] == 'wifi' and parts[2] == 'connected':
                        wifi_interface = parts[0]
                        raw_ssid = parts[3]
                        wifi_info["wifi_name"] = encode_wifi_name_base64(raw_ssid)
                        break
        except Exception as e:
            print(f"使用 nmcli 获取 WiFi 信息失败: {e}")

    # 获取 WiFi 密码
    if raw_ssid:
        wifi_info["wifi_password"] = get_wifi_password(raw_ssid)

    # 获取 WiFi 接口的 IP 地址
    if wifi_interface:
        try:
            import re
            result = subprocess.run(
                ["ip", "-4", "addr", "show", wifi_interface],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', result.stdout)
                if match:
                    wifi_info["robot_ip"] = match.group(1)
        except Exception as e:
            print(f"获取接口 {wifi_interface} 的 IP 地址失败: {e}")

    # 如果没找到 WiFi 接口 IP，获取第一个非 lo 接口的 IP 作为备选
    if wifi_info["robot_ip"] is None:
        try:
            import re
            result = subprocess.run(
                ["hostname", "-I"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and result.stdout.strip():
                # 取第一个 IP
                wifi_info["robot_ip"] = result.stdout.strip().split()[0]
        except Exception as e:
            print(f"获取 IP 地址失败: {e}")

    return wifi_info

def send_wifi_info_to_serial(serial_device="/dev/H12_log_channel", baudrate=57600):
    """将 WiFi 信息以 JSON 格式发送到串口（单次）
    :param serial_device: 串口设备路径
    :param baudrate: 波特率
    :return: bool, 发送是否成功
    """
    try:
        wifi_info = get_wifi_info()
        # 构造指定格式的数据
        data = {
            "cmd": "rc/send_robot_info",
            "data": {
                "wifi_name": wifi_info["wifi_name"],
                "robot_ip": wifi_info["robot_ip"]
            }
        }
        json_data = json.dumps(data, ensure_ascii=False) + "\r\n"

        with serial.Serial(serial_device, baudrate, timeout=1) as ser:
            ser.write(json_data.encode('utf-8'))
            ser.flush()

        print(f"已发送 WiFi 信息到串口: {json_data.strip()}")
        return True
    except Exception as e:
        print(f"发送 WiFi 信息到串口失败: {e}")
        return False

# WiFi 信息上报线程控制
_wifi_report_thread = None
_wifi_report_stop_event = threading.Event()

def _wifi_report_loop(serial_device, baudrate, interval=1.0):
    """WiFi 信息上报循环（在后台线程中运行）
    :param serial_device: 串口设备路径
    :param baudrate: 波特率
    :param interval: 上报间隔（秒）
    """
    global _wifi_report_stop_event
    ser = None
    try:
        ser = serial.Serial(serial_device, baudrate, timeout=1)
        print(f"WiFi 信息上报线程已启动，设备: {serial_device}, 间隔: {interval}秒")

        while not _wifi_report_stop_event.is_set():
            try:
                wifi_info = get_wifi_info()
                data = {
                    "cmd": "rc/send_robot_info",
                    "data": {
                        "wifi_name": wifi_info["wifi_name"],
                        "robot_ip": wifi_info["robot_ip"],
                        "wifi_password": wifi_info["wifi_password"]
                    }
                }
                json_data = json.dumps(data, ensure_ascii=False) + "\r\n"
                ser.write(json_data.encode('utf-8'))
                ser.flush()
            except Exception as e:
                print(f"WiFi 信息上报失败: {e}")

            # 等待间隔时间，同时检查停止事件
            _wifi_report_stop_event.wait(timeout=interval)
    except Exception as e:
        print(f"WiFi 信息上报线程异常: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print("WiFi 信息上报线程已停止")

def start_wifi_info_report(serial_device="/dev/H12_log_channel", baudrate=57600, interval=1.0):
    """启动 WiFi 信息持续上报线程
    :param serial_device: 串口设备路径
    :param baudrate: 波特率
    :param interval: 上报间隔（秒）
    """
    global _wifi_report_thread, _wifi_report_stop_event

    # 如果已有线程在运行，先停止
    stop_wifi_info_report()

    _wifi_report_stop_event.clear()
    _wifi_report_thread = threading.Thread(
        target=_wifi_report_loop,
        args=(serial_device, baudrate, interval),
        daemon=True
    )
    _wifi_report_thread.start()

def stop_wifi_info_report():
    """停止 WiFi 信息上报线程"""
    global _wifi_report_thread, _wifi_report_stop_event

    if _wifi_report_thread and _wifi_report_thread.is_alive():
        _wifi_report_stop_event.set()
        _wifi_report_thread.join(timeout=2.0)
        _wifi_report_thread = None

# switch_controller 冷却期机制
_switch_controller_lock = threading.Lock()
_switch_controller_cooling_until = 0.0  # 冷却期结束时间戳
SWITCH_CONTROLLER_COOLDOWN = 3.0  # 冷却期时长（秒）
_depth_loco_restore_controller_name = None  # 进入 depth_loco_controller 前的控制器名
_depth_history_topic_monitor = None
_depth_history_topic_monitor_lock = threading.Lock()
current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(os.path.dirname(current_dir), "config")
ACTION_FILE_FOLDER = "~/.config/lejuconfig/action_files"
ROS_BAG_LOG_SAVE_PATH = "~/.log/vr_remote_control/rosbag"
HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
VR_REMOTE_CONTROL_SESSION_NAME = "vr_remote_control"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12 start_way:=auto"
# LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12"
LAUNCH_HUMANOID_ROBOT_REAL_CMD = "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12 start_way:=auto"
LAUNCH_HUMANOID_ROBOT_REAL_WHEEL_CMD = "roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12 start_way:=auto"
LAUNCH_VR_REMOTE_CONTROL_CMD = "roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch"
ROS_MASTER_URI = os.getenv("ROS_MASTER_URI")
ROS_IP = os.getenv("ROS_IP")
ROS_HOSTNAME = os.getenv("ROS_HOSTNAME")
KUAVO_CONTROL_SCHEME = os.getenv("KUAVO_CONTROL_SCHEME", "multi")
kuavo_ros_control_ws_path = os.getenv("KUAVO_ROS_CONTROL_WS_PATH")
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

# 更新遥控器按键配置文件
def update_h12_customize_config():
    global customize_config_data
    try:
        with open(customize_config_path, "r") as f:
            customize_config_data = json.load(f)
        rospy.loginfo(f" ---------- customize_config_data ---------- : {customize_config_data}")
    except Exception as e:
        rospy.logerr(f"Error: Could not find {customize_config_path}")
        raise Exception(f"Error: Could not find {customize_config_path}")

# 手臂状态定义
ROBOT_ACTION_STATUS = 0 # 手臂完成状态 | 0 没开始 | 1 执行中 |  2 完成
robot_action_executing = False  # 用于检测动作是否正在执行（手臂模式切换完成）
def robot_action_state_callback(msg):
    global ROBOT_ACTION_STATUS
    global robot_action_executing
    ROBOT_ACTION_STATUS = msg.state
    # state: 0=失败/未执行, 1=执行中, 2=完成
    # 当state为1时，表示有动作正在执行（手臂模式切换完成）
    robot_action_executing = (msg.state == 1)
    # rospy.loginfo(f" ---------- ROBOT_ACTION_STATUS ---------- : {ROBOT_ACTION_STATUS}")
rospy.Subscriber('/robot_action_state', RobotActionState, robot_action_state_callback)

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

def call_robot_control_mode_action(action_name):
    """
        按键控制让机器人进去 原地踏步 / stance 站立模式
    """
    global joy_pub
    joy_msg = Joy()
    joy_msg.axes = [0.0] * 8  # Initialize 8 axes
    joy_msg.buttons = [0] * 11  # Initialize 11 buttons
    
    # 步态切换接口模式
    if action_name == "start_marching":
        joy_msg.buttons[BUTTON_B] = 1  # trot模式
    elif action_name == "end_marching":
        joy_msg.buttons[BUTTON_A] = 1  # stance模式
    # 发布话题
    try:
        joy_pub.publish(joy_msg)
        rospy.loginfo(f" 步态控制模式 :{action_name}")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to 步态控制模式/踏步 failed: {e}")
        return False

def call_robot_mpc_target_action(action_name):
    """
        质心发布器，控制机器人质心上下 以及前后左右走
    """
    global com_pose_pub
    global COM_SQUAT_DATA 
    global COM_STAND_DATA 
    global COM_PITCH_DATA
    global COM_PITCH_ZERO
    com_msg = Twist()
    if action_name == "squatting": # 下蹲
        data = COM_SQUAT_DATA
        pitch = COM_PITCH_DATA
    elif action_name == "stand_up": # 站起来
        data = COM_STAND_DATA
        pitch = COM_PITCH_ZERO
    com_msg.linear.x = 0.0
    com_msg.linear.y = 0.0
    com_msg.linear.z = float(data)

    com_msg.angular.x = 0.0
    com_msg.angular.y = float(pitch)
    com_msg.angular.z = 0.0
    # 发布话题
    try:
        com_pose_pub.publish(com_msg)
        rospy.loginfo(f" 质心控制模式 :{action_name} | 高度为 {data}")
        return True
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to 质心控制模式/蹲下 failed: {e}")
        return False

def call_execute_arm_action(action_name):
    """Call the /execute_arm_action service
    :param action_name 动作名字
    :return: bool， 服务调用结果
    """
    try:
        _execute_arm_action_client = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
        request = ExecuteArmActionRequest()
        request.action_name = action_name

        response = _execute_arm_action_client(request)
        rospy.loginfo(f"ExecuteArmAction service response:\nsuccess: {response.success}\nmessage: {response.message}")
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '/execute_arm_action' failed: {e}")
        return False, f"Service exception: {e}"

def call_exec_vmp_action(action_name):
    """Call the /humanoid_controllers/vmp_controller/trajectory/execute service
    :param action_name 动作名字
    :return: bool， 服务调用结果
    """
    service_name = "/humanoid_controllers/vmp_controller/trajectory/execute"
    try:
        # 确保 action_name 是字符串类型
        if not isinstance(action_name, str):
            action_name = str(action_name)
        
        # 确保是 ASCII 字符串（ROS 字符串字段要求）
        if isinstance(action_name, bytes):
            action_name = action_name.decode('utf-8')
        
        rospy.wait_for_service(service_name, timeout=1.0)
        _exec_vmp_action_client = rospy.ServiceProxy(service_name, SetString)
        
        # 使用关键字参数方式调用服务（更简单且正确）
        response = _exec_vmp_action_client(data=action_name)
        
        rospy.loginfo(f"VMP trajectory execute service response:\nsuccess: {response.success}\nmessage: {response.message}")
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '{service_name}' failed: {e}")
        return False, f"Service exception: {e}"
    except Exception as e:
        rospy.logerr(f"Error calling '{service_name}': {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        return False, f"Exception: {e}"

def set_robot_play_music(music_file_name:str, music_volume:int)->bool:
    """机器人播放指定文件的音乐
    :param music_file_name, 音乐文件名字
    :param music_volume, 音乐音量
    :return: bool, 服务调用结果 
    """
    try:
        _robot_music_play_client = rospy.ServiceProxy("/play_music", playmusic)
        request = playmusicRequest()
        request.music_number = music_file_name
        request.volume = music_volume
        # 客户端接收
        response = _robot_music_play_client(request)
        rospy.loginfo(f"Service call /play_music call: {response.success_flag}")
        return response.success_flag
    except Exception as e:
        print(f"An error occurred: {e}")
        rospy.loginfo("Service /play_music call: fail!...please check again!")
        return False

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Change arm ctrl mode Service call successful")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result

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
    req.joint_names = [
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
        "thumb1",
        "thumb2",
        "index1",
        "middle1",
        "ring1",
        "pinky1",
        "head_yaw",
        "head_pitch",
    ]
    return req

def plan_arm_trajectory_bezier_curve_client(req):
    service_name = '/bezier/plan_arm_trajectory'
    rospy.wait_for_service(service_name)
    try:
        plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
        res = plan_service(req)
        return res.success
    except rospy.ServiceException as e:
        rospy.logerr(f"PlService call failed: {e}")
        return False

def call_real_initialize_srv():
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


def print_state_transition(trigger, source, target) -> None:
    console.print(
        f"Trigger: [bold blue]{trigger}[/bold blue] From [bold green]{source}[/bold green] to [bold green]{target}[/bold green]"
    )


def launch_humanoid_robot(real_robot=True,calibrate=False):
    
    robot_version = os.getenv('ROBOT_VERSION')
    print(f"current robot version: {robot_version}")
    subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                    stderr=subprocess.DEVNULL) 
    
    # 通过读取kuavo.json文件，获取ROBOT_MODULE和only_half_up_body参数
    kuavo_json = os.path.join(kuavo_ros_control_ws_path, "src", "kuavo_assets", "config", f"kuavo_v{robot_version}", "kuavo.json")
    if not os.path.exists(kuavo_json):
        print(f"Error: Could not find {kuavo_json}")
        raise Exception(f"Error: Could not find {kuavo_json}")
    with open(kuavo_json, "r") as f:    
        kuavo_json_data = json.load(f)
    
    # 获取机器人模块类型
    robot_module = kuavo_json_data.get("ROBOT_MODULE", "")
    only_half_up_body = kuavo_json_data["only_half_up_body"]
    
    # 根据机器人模块类型选择启动命令
    if real_robot:
        # 如果是LUNBI或LUNBI_V62，使用轮臂启动命令
        if robot_module == "LUNBI" or robot_module == "LUNBI_V62":
            launch_cmd = LAUNCH_HUMANOID_ROBOT_REAL_WHEEL_CMD
        else:
            launch_cmd = LAUNCH_HUMANOID_ROBOT_REAL_CMD
    else:
        launch_cmd = LAUNCH_HUMANOID_ROBOT_SIM_CMD
    
    if calibrate:
        # LUNBI_V62 只执行 cali:=true，不执行 cali_arm
        if robot_module == "LUNBI_V62":
            launch_cmd += " cali:=true"
        else:
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

    if rospy.has_param("h12_log_channel"):
        log_channel_status = rospy.get_param("h12_log_channel")
        if log_channel_status is True:
            if os.path.exists("/dev/H12_log_channel"):
                # 启动 WiFi 信息持续上报（1秒间隔）
                start_wifi_info_report("/dev/H12_log_channel", baudrate=57600, interval=1.0)

                log_channel_cmd_start = "stdbuf -oL -eL"
                log_channel_cmd_end = "2>&1 | sed -u -e 's/\\x1b\\[[0-9;]*[mK]//g' -e 's/$/\\r/' | stdbuf -oL tee /dev/H12_log_channel"
                launch_cmd = f"{log_channel_cmd_start} {launch_cmd} {log_channel_cmd_end}"
            else:
                rospy.logerr("未检测到 /dev/H12_log_channel 设备文件，请确认已加载遥控器串口 udev 规则并连接设备。")

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
    
    rospy.sleep(5.0)
    
    result = subprocess.run(["tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                            capture_output=True)
    if result.returncode == 0:
        print(f"Started humanoid_robot in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
    else:
        print("Failed to start humanoid_robot")
        raise Exception("Failed to start humanoid_robot")
        

def start_vr_remote_control_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print(f"`launch_cmd`: {LAUNCH_VR_REMOTE_CONTROL_CMD}")
    tmux_cmd = [
        "tmux", "new-session",
        "-s", VR_REMOTE_CONTROL_SESSION_NAME, 
        "-d",  
        f"bash -c -i 'source ~/.bashrc && \
          source {kuavo_ros_control_ws_path}/devel/setup.bash && \
          export ROS_MASTER_URI={ROS_MASTER_URI} && \
          export ROS_IP={ROS_IP} && \
          export ROS_HOSTNAME={ROS_HOSTNAME} &&\
          {LAUNCH_VR_REMOTE_CONTROL_CMD}; exec bash'"
    ]
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    process = subprocess.Popen(
        tmux_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(3)
    result = subprocess.run(["tmux", "has-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                              capture_output=True)
    if result.returncode == 0:
        print(f"Started vr_remote_control in tmux session: {VR_REMOTE_CONTROL_SESSION_NAME}")
        print_state_transition(trigger, source, "vr_remote_control")
    else:
        print("Failed to start vr_remote_control")
        raise Exception("Failed to create tmux session")
    

def stop_vr_remote_control_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    print(f"Stopped {VR_REMOTE_CONTROL_SESSION_NAME} in tmux session")
    kill_record_vr_rosbag()
    time.sleep(3)
    print_state_transition(trigger, source, "stance")

def initial_pre_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("initial_pre", source, "ready_stance")
    launch_humanoid_robot(event.kwargs.get("real_robot"))
    
def calibrate_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("calibrate", source, "calibrate")
    launch_humanoid_robot(event.kwargs.get("real_robot"), calibrate=True)
    
def ready_stance_callback(event):
    source = event.kwargs.get("source")
    call_real_initialize_srv()
    print_state_transition("ready_stance", source, "stance")

def cali_to_ready_stance_callback(event):
    source = event.kwargs.get("source")
    call_real_initialize_srv()
    print_state_transition("cali_to_ready_stance", source, "ready_stance")

def stance_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("stance", source, "stance")

def walk_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("walk", source, "walk")

def trot_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("trot", source, "trot")

def stop_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("stop", source, "initial")
    # 停止 WiFi 信息上报
    stop_wifi_info_report()
    # kill humanoid_robot and vr_remote_control
    subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME],
                  stderr=subprocess.DEVNULL)
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME],
                  stderr=subprocess.DEVNULL)
    kill_record_vr_rosbag()
    
    manual_h12_init_state = rospy.get_param("manual_h12_init_state", "none")
    if "none" != manual_h12_init_state:
        # 此if分支为命令行启动机器人: joystick_type=h12，遥控器使用和服务启动相同的逻辑。
        # manual_h12_init_state为初始状态，其值为none表示当前是用服务启动的机器人。
        # manual_h12_init_state不是none表示是命令行启动的机器人，此时启动机器人程序没有使用tmux，需要额外关闭

        subprocess.run(["rosnode", "kill", "/nodelet_manager"], 
                    stderr=subprocess.DEVNULL)

def arm_pose_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    current_arm_joint_state = event.kwargs.get("current_arm_joint_state")
    print_state_transition(trigger, source, "stance")
    try:
        call_change_arm_ctrl_mode_service(2)
        action_file_path = os.path.expanduser(f"{ACTION_FILE_FOLDER}/{trigger}.tact")
        start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)
        action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
        req = create_bezier_request(action_frames, start_frame_time, end_frame_time+1)
        if plan_arm_trajectory_bezier_curve_client(req):
            rospy.loginfo("Plan arm trajectory bezier curve client call successful")
    except Exception as e:
        rospy.logerr(f"Error in arm_pose_callback: {e}")
    pass

# 等待动作完成的函数，增加超时机制
def wait_for_action_completion(timeout=10.0):
    """
    等待动作完成，直到 ROBOT_ACTION_STATUS 为 2 或超时。
    
    :param timeout: 超时时间（秒）
    """
    global ROBOT_ACTION_STATUS
    start_time = time.time()  # 记录开始时间
    while ROBOT_ACTION_STATUS != 2:
        if time.time() - start_time > timeout:  # 检查是否超时
            rospy.logwarn("等待动作完成超时，自动退出等待循环")
            break
        rospy.sleep(0.1)  # 等待0.1秒后再检查状态

# 执行动作的线程函数
def execute_arm_poses(arm_pose_names, mode_switch_event=None):
    global robot_action_executing
    for arm_pose in arm_pose_names:
        if arm_pose:  # 检查动作名称不为空
            rospy.loginfo(f"Executing arm pose: {arm_pose}")
            try:
                success, message = call_execute_arm_action(arm_pose)
                if success:
                    # 等待手臂模式切换完成（检测到动作开始执行）
                    rospy.loginfo(f"Waiting for arm mode switch to complete for action: {arm_pose}")
                    start_wait_time = time.time()
                    timeout = 5.0  # 5秒超时
                    
                    while not robot_action_executing and not rospy.is_shutdown():
                        if time.time() - start_wait_time > timeout:
                            rospy.logwarn(f"Timeout waiting for arm mode switch to complete for action: {arm_pose}")
                            # 超时未检测到模式切换完成，不通知音乐线程，音乐将不播放
                            return
                        rospy.sleep(0.01)
                    
                    if robot_action_executing:
                        rospy.loginfo(f"Arm mode switch completed for action: {arm_pose}, action is now executing")
                        # 只有在动作成功执行且模式切换完成后，才通知音乐线程可以开始播放
                        if mode_switch_event and not mode_switch_event.is_set():
                            mode_switch_event.set()
                    else:
                        rospy.logwarn(f"Arm mode switch not detected for action: {arm_pose}, music will not play")
                        # 未检测到模式切换完成，不通知音乐线程，音乐将不播放
                        return
                    
                    wait_for_action_completion(timeout=10.0)  # 等待动作完成
                else:
                    rospy.logwarn(f"Failed to execute arm pose {arm_pose}: {message}")
                    # 动作执行失败，不通知音乐线程，音乐将不播放
                    return
            except Exception as e:
                rospy.logerr(f"Failed to execute arm pose {arm_pose}: {e}")
                # 发生异常，不通知音乐线程，音乐将不播放
                return

# 播放音乐的线程函数
def play_music(music_names, mode_switch_event=None):
    # 如果有模式切换事件，等待模式切换完成后再播放音乐
    if mode_switch_event:
        rospy.loginfo("Waiting for arm mode switch to complete before playing music...")
        if not mode_switch_event.wait(timeout=10.0):  # 最多等待10秒
            rospy.logwarn("Timeout waiting for arm mode switch, action may have failed. Music will not play.")
            # 动作执行失败或超时，不播放音乐
            return
    
    # 只有在动作成功执行且模式切换完成后，才播放音乐
    for music in music_names:
        if music:  # 检查音乐名称不为空
            rospy.loginfo(f"Playing music: {music}")
            try:
                set_robot_play_music(music, 100)
            except Exception as e:
                rospy.logerr(f"Failed to play music {music}: {e}")

def customize_action_callback(event):
    global customize_config_data
    global comGaitSwitch_config_data

    # 打印动作类型
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "stance")
    try:
        # 根据 trigger 查找对应的配置
        if trigger in customize_config_data:
            action_config = customize_config_data[trigger]
            action_type = action_config.get("type", "action")  # 默认类型为action
            
            rospy.loginfo(f"Trigger: {trigger}")
            rospy.loginfo(f"Action Type: {action_type}")
            
            if action_type == "action":
                # 处理常规动作类型
                arm_pose_names = action_config.get("arm_pose_name", [])
                music_names = action_config.get("music_name", [])
                
                # 打印匹配到的动作和音乐信息
                rospy.loginfo(f"Received Arm Pose Names: {arm_pose_names}")
                rospy.loginfo(f"Received Music Names: {music_names}") 

                # 检查是否需要切换到质心规划模式或步态控制模式
                gait_control_interfaces = set(comGaitSwitch_config_data.get("gait_control_interface", []))
                com_control_interfaces = set(comGaitSwitch_config_data.get("com_control_interface", []))

                # 判断是否有匹配的接口
                matched_gait_interfaces = gait_control_interfaces.intersection(arm_pose_names)
                matched_com_interfaces = com_control_interfaces.intersection(arm_pose_names)

                # arm_pose_names移除接口
                arm_pose_names = [pose for pose in arm_pose_names if pose not in matched_gait_interfaces and pose not in matched_com_interfaces]
                rospy.loginfo(f"real Execute Arm Pose Names: {arm_pose_names}")

                if matched_gait_interfaces:
                    rospy.loginfo(f"Matched Gait Control Interfaces: {matched_gait_interfaces}")
                    # 在这里切换到步态控制模式的逻辑
                    for action_name in matched_gait_interfaces:
                        call_robot_control_mode_action(action_name)
                if matched_com_interfaces:
                    rospy.loginfo(f"Matched COM Control Interfaces: {matched_com_interfaces}")
                    # 在这里切换到质心规划模式的逻辑
                    for action_name in matched_com_interfaces:
                        call_robot_mpc_target_action(action_name)

                # 创建模式切换事件（用于同步动作执行和音乐播放）
                mode_switch_event = None
                if arm_pose_names and music_names:
                    mode_switch_event = threading.Event()

                # 创建线程执行动作和音乐
                if arm_pose_names:
                    arm_pose_thread = threading.Thread(target=execute_arm_poses, args=(arm_pose_names, mode_switch_event))
                    arm_pose_thread.start()

                if music_names:
                    music_thread = threading.Thread(target=play_music, args=(music_names, mode_switch_event))
                    music_thread.start()

                # 等待线程完成
                if arm_pose_names:
                    arm_pose_thread.join()
                if music_names:
                    music_thread.join()
                    
            elif action_type == "shell":
                # 处理shell命令类型
                command = action_config.get("command", "")
                if command:
                    rospy.loginfo(f"Executing shell command: {command}")
                    try:
                        process = subprocess.Popen(
                            command,
                            shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True
                        )
                        stdout, stderr = process.communicate()
                        if process.returncode == 0:
                            rospy.loginfo(f"Command executed successfully. Output: {stdout}")
                        else:
                            rospy.logerr(f"Command failed with error: {stderr}")
                    except Exception as e:
                        rospy.logerr(f"Failed to execute shell command: {e}")
                else:
                    rospy.logwarn("No command specified for shell action type")
            else:
                rospy.logwarn(f"Unsupported action type: {action_type}")
        else:
            rospy.logwarn(f"No configuration found for trigger: {trigger}")
    except Exception as e:
        rospy.logerr(f"Error in customize_action_callback: {e}")

def record_vr_rosbag_callback(event):
    global record_vr_rosbag_pid
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "vr_remote_control")
    try:
        base_dir = os.path.expanduser(ROS_BAG_LOG_SAVE_PATH)
        current_date = datetime.datetime.now().strftime("%Y-%m-%d")
        date_folder = os.path.join(base_dir, current_date)
        if not os.path.exists(date_folder):
            os.makedirs(date_folder)
        
        base_filename = "vr_record"
        bag_file_base = os.path.join(date_folder, base_filename)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        actual_bag_file = f"{bag_file_base}_{timestamp}.bag"  # 实际会被创建的文件路径
        
        command = [
            "rosbag",
            "record",
            "-o",
            bag_file_base
        ]

        for topic in record_topics:
            command.append(topic)

        process = subprocess.Popen(
            command,
            start_new_session=True,
        )
        record_vr_rosbag_pid = process.pid
    except Exception as e:
        rospy.logerr(f"Error in record_vr_rosbag_callback: {e}")


def kill_record_vr_rosbag():
    global record_vr_rosbag_pid
    if record_vr_rosbag_pid:
        os.kill(record_vr_rosbag_pid, signal.SIGINT)
        record_vr_rosbag_pid = None

def stop_record_vr_rosbag_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "vr_remote_control")
    kill_record_vr_rosbag()

def call_switch_controller_service(controller_name):
    """调用切换到指定控制器的服务
    注意：此服务不支持 vmp_controller，vmp_controller 需要使用 call_switch_to_vmp_controller_service()
    """
    service_name = "/humanoid_controller/switch_controller"
    try:
        current_controller_before = get_current_controller_name()
        rospy.loginfo(
            f"[ControllerSwitch] Request switch via '{service_name}': "
            f"from '{current_controller_before}' to '{controller_name}'"
        )
        rospy.wait_for_service(service_name, timeout=1.0)
        switch_client = rospy.ServiceProxy(service_name, switchController)
        response = switch_client(controller_name)
        if response.success:
            current_controller_after = get_current_controller_name()
            rospy.loginfo(
                f"[ControllerSwitch] Switch success: {response.message}. "
                f"Current controller is now '{current_controller_after}'"
            )
        else:
            current_controller_after = get_current_controller_name()
            rospy.logwarn(
                f"[ControllerSwitch] Switch failed: {response.message}. "
                f"Current controller remains '{current_controller_after}'"
            )
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"[ControllerSwitch] Service call to '{service_name}' failed: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"[ControllerSwitch] Service '{service_name}' not available: {e}")
        return False

def call_switch_to_vmp_controller_service():
    """调用切换到VMP控制器的专用服务
    使用新的服务接口：/humanoid_controller/switch_to_vmp_controller (std_srvs/Trigger)
    
    :return: bool, 服务调用结果
    """
    service_name = "/humanoid_controller/switch_to_vmp_controller"
    try:
        current_controller_before = get_current_controller_name()
        rospy.loginfo(
            f"[ControllerSwitch] Request switch via '{service_name}': "
            f"from '{current_controller_before}' to 'vmp_controller'"
        )
        rospy.wait_for_service(service_name, timeout=1.0)
        switch_client = rospy.ServiceProxy(service_name, Trigger)
        response = switch_client()
        if response.success:
            current_controller_after = get_current_controller_name()
            rospy.loginfo(
                f"[ControllerSwitch] VMP switch success: {response.message}. "
                f"Current controller is now '{current_controller_after}'"
            )
        else:
            current_controller_after = get_current_controller_name()
            rospy.logwarn(
                f"[ControllerSwitch] VMP switch failed: {response.message}. "
                f"Current controller remains '{current_controller_after}'"
            )
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"[ControllerSwitch] Service call to '{service_name}' failed: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"[ControllerSwitch] Service '{service_name}' not available: {e}")
        return False

def call_switch_to_dance_controller_service():
    """调用切换到Dance控制器的专用服务
    使用服务接口：/humanoid_controller/switch_to_dance_controller (std_srvs/Trigger)

    :return: bool, 服务调用结果
    """
    service_name = "/humanoid_controller/switch_to_dance_controller"
    try:
        current_controller_before = get_current_controller_name()
        rospy.loginfo(
            f"[ControllerSwitch] Request switch via '{service_name}': "
            f"from '{current_controller_before}' to 'dance_controller'"
        )
        rospy.wait_for_service(service_name, timeout=1.0)
        switch_client = rospy.ServiceProxy(service_name, Trigger)
        response = switch_client()
        if response.success:
            current_controller_after = get_current_controller_name()
            rospy.loginfo(
                f"[ControllerSwitch] Dance switch success: {response.message}. "
                f"Current controller is now '{current_controller_after}'"
            )
        else:
            current_controller_after = get_current_controller_name()
            rospy.logwarn(
                f"[ControllerSwitch] Dance switch failed: {response.message}. "
                f"Current controller remains '{current_controller_after}'"
            )
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"[ControllerSwitch] Service call to '{service_name}' failed: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"[ControllerSwitch] Service '{service_name}' not available: {e}")
        return False

def get_current_controller_name():
    """获取当前控制器名称
    :return: str, 当前控制器名称，如果获取失败返回 None
    """
    service_name = "/humanoid_controller/get_controller_list"
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
        get_controller_client = rospy.ServiceProxy(service_name, getControllerList)
        response = get_controller_client()
        if response.success:
            current_controller = response.current_controller
            rospy.loginfo(f"Current controller: {current_controller} (index: {response.current_index})")
            return current_controller
        else:
            rospy.logwarn(f"Get controller list failed: {response.message}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '{service_name}' failed: {e}")
        return None
    except rospy.ROSException as e:
        rospy.logerr(f"Service '{service_name}' not available: {e}")
        return None

class TopicFrequencyMonitor(object):
    """后台订阅话题并估算最近一段时间的消息频率。"""

    def __init__(self, topic, msg_type, window_size=50, stale_timeout=0.5):
        self._timestamps = collections.deque(maxlen=window_size)
        self._stale_timeout = stale_timeout
        self._lock = threading.Lock()
        self._subscriber = rospy.Subscriber(topic, msg_type, self._cb, queue_size=window_size)

    def _cb(self, _msg):
        with self._lock:
            self._timestamps.append(time.time())

    def has_recent_message(self):
        now_sec = time.time()
        with self._lock:
            if not self._timestamps:
                return False
            return (now_sec - self._timestamps[-1]) <= self._stale_timeout

    def get_hz(self):
        now_sec = time.time()
        with self._lock:
            timestamps = list(self._timestamps)

        if len(timestamps) < 2:
            return 0.0

        if (now_sec - timestamps[-1]) > self._stale_timeout:
            return 0.0

        duration = timestamps[-1] - timestamps[0]
        if duration <= 0.0:
            return 0.0

        return (len(timestamps) - 1) / duration

def _get_depth_history_topic_monitor():
    global _depth_history_topic_monitor

    with _depth_history_topic_monitor_lock:
        if _depth_history_topic_monitor is None:
            _depth_history_topic_monitor = TopicFrequencyMonitor(
                "/camera/depth/depth_history_array",
                Float64MultiArray,
                window_size=50,
                stale_timeout=0.5,
            )
        return _depth_history_topic_monitor

def is_depth_history_topic_available():
    """检查深度历史话题是否已发布且当前能收到消息。

    Returns:
        bool: True 表示话题已发布、能收到消息且频率达到最低要求，False 表示当前不可切到 depth_loco_controller
    """
    target_topic = "/camera/depth/depth_history_array"
    min_topic_frequency_hz = 50.0
    wait_timeout_sec = 2.0
    check_interval_sec = 0.05
    try:
        published_topics = rospy.get_published_topics()
        if not published_topics:
            rospy.logwarn("[DepthLocoSwitch] No published topics found while checking depth history topic.")
            return False

        if not any(topic_name == target_topic for topic_name, _topic_type in published_topics):
            rospy.logwarn(f"[DepthLocoSwitch] Required topic '{target_topic}' is not published. Refuse to switch to depth_loco_controller.")
            return False

        topic_monitor = _get_depth_history_topic_monitor()
        deadline = time.time() + wait_timeout_sec
        estimated_hz = 0.0

        while time.time() < deadline and not rospy.is_shutdown():
            estimated_hz = topic_monitor.get_hz()
            if estimated_hz >= min_topic_frequency_hz:
                rospy.loginfo(
                    f"[DepthLocoSwitch] Topic '{target_topic}' is available with estimated frequency "
                    f"{estimated_hz:.1f} Hz."
                )
                return True
            rospy.sleep(check_interval_sec)

        if not topic_monitor.has_recent_message():
            rospy.logwarn(
                f"[DepthLocoSwitch] Required topic '{target_topic}' has no recent messages. "
                f"Refuse to switch to depth_loco_controller."
            )
            return False

        if estimated_hz < min_topic_frequency_hz:
            rospy.logwarn(
                f"[DepthLocoSwitch] Topic '{target_topic}' frequency too low: "
                f"estimated {estimated_hz:.1f} Hz. Require at least {min_topic_frequency_hz:.1f} Hz."
            )
            return False
        return True
    except Exception as e:
        rospy.logerr(f"[DepthLocoSwitch] Failed to check published topics: {e}")
        return False

def _set_depth_loco_restore_controller_name(controller_name):
    """记录进入 depth_loco_controller 之前的控制器名。"""
    global _depth_loco_restore_controller_name
    with _switch_controller_lock:
        _depth_loco_restore_controller_name = controller_name

def _get_depth_loco_restore_controller_name():
    """获取 depth_loco_controller 的恢复目标，不清空。"""
    global _depth_loco_restore_controller_name
    with _switch_controller_lock:
        return _depth_loco_restore_controller_name

def _clear_depth_loco_restore_controller_name():
    """清空 depth_loco_controller 的恢复目标。"""
    global _depth_loco_restore_controller_name
    with _switch_controller_lock:
        _depth_loco_restore_controller_name = None

def _release_switch_controller_cooldown():
    """释放 switch_controller 冷却期（在后台线程中调用）"""
    global _switch_controller_cooling_until
    time.sleep(SWITCH_CONTROLLER_COOLDOWN)
    with _switch_controller_lock:
        _switch_controller_cooling_until = 0.0
        rospy.loginfo("[SwitchController] Cooldown period ended. State transitions are now allowed.")

def is_switch_controller_in_cooldown():
    """检查 switch_controller 是否在冷却期内
    
    Returns:
        bool: True 表示在冷却期内，False 表示不在冷却期
    """
    global _switch_controller_cooling_until
    with _switch_controller_lock:
        current_time = time.time()
        if _switch_controller_cooling_until > current_time:
            return True
        return False

def clear_switch_controller_cooldown():
    """清除 switch_controller 的冷却期
    用于紧急停止等需要立即执行的状态转换
    """
    global _switch_controller_cooling_until
    with _switch_controller_lock:
        _switch_controller_cooling_until = 0.0
        rospy.loginfo("[SwitchController] Cooldown cleared by emergency stop or other critical operation.")

def switch_controller_callback(event):
    """切换控制器回调函数
    - 如果当前是 mpc，切换到 amp_controller
    - 如果当前是 amp_controller，切换回 mpc
    - 执行后设置冷却期，期间不允许其他状态转换
    """
    global _switch_controller_cooling_until
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "stance")
    try:
        current_controller = get_current_controller_name()
        
        if current_controller is None:
            rospy.logerr("[SwitchController] Failed to get current controller name. Cannot switch controller.")
            return
        
        current_controller_lower = current_controller.lower()
        
        success = False
        if current_controller_lower == "mpc":
            rospy.loginfo("[SwitchController] Current controller is MPC, switching to amp_controller")
            success = call_switch_controller_service("amp_controller")
            if not success:
                rospy.logwarn("[SwitchController] Failed to switch from MPC to amp_controller. Current controller remains MPC.")
        elif current_controller_lower == "amp_controller":
            rospy.loginfo("[SwitchController] Current controller is amp_controller, switching to MPC")
            success = call_switch_controller_service("mpc")
            if not success:
                rospy.logwarn("[SwitchController] Failed to switch from amp_controller to MPC. Current controller remains amp_controller.")
        else:
            rospy.logwarn(f"[SwitchController] Unknown controller type: {current_controller}. Cannot switch.")

        with _switch_controller_lock:
            _switch_controller_cooling_until = time.time() + SWITCH_CONTROLLER_COOLDOWN
            if success:
                rospy.loginfo(f"[SwitchController] Controller switched successfully. Cooldown period started. State transitions will be blocked for {SWITCH_CONTROLLER_COOLDOWN} seconds.")
            else:
                rospy.loginfo(f"[SwitchController] Cooldown period started (service call failed). State transitions will be blocked for {SWITCH_CONTROLLER_COOLDOWN} seconds.")
        
        # 在后台线程中等待冷却期结束
        cooldown_thread = threading.Thread(target=_release_switch_controller_cooldown, daemon=True)
        cooldown_thread.start()
        
    except Exception as e:
        rospy.logerr(f"Error in switch_controller_callback: {e}")

def depth_loco_switch_callback(event):
    """切换走楼梯斜坡控制器回调函数
    - 如果当前是 mpc 或 amp_controller，切换到 depth_loco_controller
    - 如果当前是 depth_loco_controller，切换回进入前的控制器
    - 执行后设置冷却期，期间不允许其他状态转换
    """
    global _switch_controller_cooling_until
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "stance")

    try:
        current_controller = get_current_controller_name()

        if current_controller is None:
            rospy.logerr("[DepthLocoSwitch] Failed to get current controller name. Cannot switch controller.")
            return

        current_controller_lower = current_controller.lower()

        success = False
        if current_controller_lower in ["mpc", "amp_controller"]:
            if not is_depth_history_topic_available():
                rospy.logwarn("[DepthLocoSwitch] depth_loco_switch blocked because depth history topic is unavailable.")
                return
            _set_depth_loco_restore_controller_name(current_controller_lower)
            rospy.loginfo("[DepthLocoSwitch] Switching to depth_loco_controller")
            success = call_switch_controller_service("depth_loco_controller")
            if not success:
                _set_depth_loco_restore_controller_name(None)
                rospy.logwarn("[DepthLocoSwitch] Failed to switch to depth_loco_controller.")
        elif current_controller_lower == "depth_loco_controller":
            restore_controller_name = _get_depth_loco_restore_controller_name()
            if restore_controller_name not in ["mpc", "amp_controller"]:
                restore_controller_name = "amp_controller"
            rospy.loginfo(f"[DepthLocoSwitch] Switching back to {restore_controller_name}")
            success = call_switch_controller_service(restore_controller_name)
            if not success:
                rospy.logwarn(f"[DepthLocoSwitch] Failed to switch back to {restore_controller_name}.")
            else:
                _clear_depth_loco_restore_controller_name()
        else:
            rospy.logwarn(f"[DepthLocoSwitch] Unsupported current controller: {current_controller}.")

        with _switch_controller_lock:
            _switch_controller_cooling_until = time.time() + SWITCH_CONTROLLER_COOLDOWN
            if success:
                rospy.loginfo(f"[DepthLocoSwitch] Controller switched successfully. Cooldown period started for {SWITCH_CONTROLLER_COOLDOWN} seconds.")
            else:
                rospy.loginfo(f"[DepthLocoSwitch] Cooldown period started (service call failed). State transitions will be blocked for {SWITCH_CONTROLLER_COOLDOWN} seconds.")

        cooldown_thread = threading.Thread(target=_release_switch_controller_cooldown, daemon=True)
        cooldown_thread.start()

    except Exception as e:
        rospy.logerr(f"Error in depth_loco_switch_callback: {e}")

def check_can_switch_to_vmp(event):
    """检查是否可以切换到VMP控制器
    条件：当前控制器必须是 amp_controller
    :return: bool, True表示可以切换，False表示不能切换
    """
    try:
        current_controller = get_current_controller_name()
        
        if current_controller is None:
            rospy.logerr("[VMPController] Failed to get current controller name. Cannot switch to VMP controller mode.")
            return False
        
        current_controller_lower = current_controller.lower()
        
        if current_controller_lower != "amp_controller":
            rospy.logwarn(f"[VMPController] Cannot switch to VMP controller mode from '{current_controller}'. Only amp_controller is allowed.")
            return False
        
        return True
    except Exception as e:
        rospy.logerr(f"Error in check_can_switch_to_vmp: {e}")
        return False

def check_is_amp_controller(event):
    """检查当前控制器是否为 amp_controller
    用于限制某些状态转换只能在 amp_controller 下执行（mpc 不支持）
    TODO：属于临时添加的回调函数，后续 mpc 支持 trot 之后删除此函数
    :return: bool, True表示当前是 amp_controller，False表示不是
    """
    try:
        current_controller = get_current_controller_name()
        
        if current_controller is None:
            rospy.logwarn("[StanceTransition] Failed to get current controller name. Cannot switch to stance.")
            return False
        
        current_controller_lower = current_controller.lower()
        
        if current_controller_lower == "amp_controller":
            return True
        else:
            rospy.logwarn(f"[StanceTransition] Cannot switch to stance from '{current_controller}'. Only amp_controller is supported.")
            return False
    except Exception as e:
        rospy.logerr(f"Error in check_is_amp_controller: {e}")
        return False

def vmp_controller_callback(event):
    """进入VMP控制模式回调函数
    从 amp_controller 切换到 vmp_controller
    使用新的服务接口：/humanoid_controller/switch_to_vmp_controller
    """
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    
    try:
        rospy.loginfo(f"[VMPController] Switching from amp_controller to vmp_controller")
        success = call_switch_to_vmp_controller_service()
        
        if not success:
            rospy.logerr("[VMPController] Failed to switch to vmp_controller")
            return
        
        rospy.loginfo("[VMPController] Successfully entered VMP controller mode")
        print_state_transition(trigger, source, "vmp_controller")
    except Exception as e:
        rospy.logerr(f"Error in vmp_controller_callback: {e}")
        return

def exit_vmp_controller_callback(event):
    """退出VMP控制模式回调函数
    从 vmp_controller 切换回 amp_controller
    """
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    
    try:
        rospy.loginfo("[VMPController] Exiting VMP controller mode, switching back to amp_controller")
        success = call_switch_controller_service("amp_controller")
        
        if not success:
            rospy.logerr("[VMPController] Failed to switch back to amp_controller.")
            return
        
        rospy.loginfo("[VMPController] Successfully switched back to amp_controller")
        print_state_transition(trigger, source, "stance")
    except Exception as e:
        rospy.logerr(f"Error in exit_vmp_controller_callback: {e}")
        return

def dance_controller_callback(event):
    """进入Dance控制模式回调函数
    从 amp_controller 切换到 dance_controller
    使用服务接口：/humanoid_controller/switch_to_dance_controller
    """
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")

    try:
        rospy.loginfo("[DanceController] Switching from amp_controller to dance_controller")
        success = call_switch_to_dance_controller_service()

        if not success:
            rospy.logerr("[DanceController] Failed to switch to dance_controller")
            return

        rospy.loginfo("[DanceController] Successfully entered Dance controller mode")
        print_state_transition(trigger, source, "dance_controller")
    except Exception as e:
        rospy.logerr(f"Error in dance_controller_callback: {e}")
        return

def exit_dance_controller_callback(event):
    """退出Dance控制模式回调函数
    从 dance_controller 切换回 amp_controller
    """
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")

    try:
        rospy.loginfo("[DanceController] Exiting Dance controller mode, switching back to amp_controller")
        success = call_switch_controller_service("amp_controller")

        if not success:
            rospy.logerr("[DanceController] Failed to switch back to amp_controller.")
            return

        rospy.loginfo("[DanceController] Successfully switched back to amp_controller")
        print_state_transition(trigger, source, "stance")
    except Exception as e:
        rospy.logerr(f"Error in exit_dance_controller_callback: {e}")
        return

def vmp_action_callback(event):
    """VMP动作回调函数（统一处理vmp_action_RL_A/B/C/D）
    从配置文件中读取action_name并发布到话题，同时支持音乐播放
    """
    global customize_config_data

    # 打印动作类型
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "vmp_controller")
    try:
        # 根据 trigger 查找对应的配置
        if trigger in customize_config_data:
            action_config = customize_config_data[trigger]
            
            # 获取action_name和music_name
            action_names = action_config.get("action_name", [])
            music_names = action_config.get("music_name", [])
            
            rospy.loginfo(f"VMP Received Action Names: {action_names}")
            rospy.loginfo(f"VMP Received Music Names: {music_names}") 

            # 调用服务执行VMP动作
            if action_names:
                for action_name in action_names:
                    if action_name and action_name.strip():  # 检查非空
                        rospy.loginfo(f"VMP Calling /humanoid_controllers/vmp_controller/trajectory/execute service with action name: {action_name}")
                        success, message = call_exec_vmp_action(action_name)
                        if success:
                            rospy.loginfo(f"VMP Action '{action_name}' executed successfully: {message}")
                        else:
                            rospy.logwarn(f"VMP Action '{action_name}' execution failed: {message}")
                        break

            # 播放音乐
            if music_names:
                music_thread = threading.Thread(target=play_music, args=(music_names,))
                music_thread.start()
        else:
            rospy.logwarn(f"VMP No configuration found for trigger: {trigger}")
    except Exception as e:
        rospy.logerr(f"Error in vmp_action_callback: {e}")
