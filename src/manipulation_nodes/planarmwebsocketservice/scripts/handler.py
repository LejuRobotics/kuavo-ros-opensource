import rospy
import rospkg
import os
import pwd
import sys
import asyncio
import json
from queue import Queue
from dataclasses import dataclass
from typing import Any, Union, Dict
import websockets
import threading
import time
import math
import subprocess
import base64
from pathlib import Path
from utils import calculate_file_md5, frames_to_custom_action_data, get_start_end_frame_time, frames_to_custom_action_data_ocs2

from kuavo_ros_interfaces.srv import planArmTrajectoryBezierCurve, stopPlanArmTrajectory, planArmTrajectoryBezierCurveRequest, ocs2ChangeArmCtrlMode
from kuavo_ros_interfaces.msg import planArmState, jointBezierTrajectory, bezierCurveCubicPoint, robotHandPosition, robotHeadMotionData
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from kuavo_msgs.msg import sensorsData
from h12pro_controller_node.msg import UpdateH12CustomizeConfig
# Replace multiprocessing values with simple variables
plan_arm_state_progress = 0
plan_arm_state_status = False
should_stop = False
process = None
response_queue = Queue()
active_threads: Dict[websockets.WebSocketServerProtocol, threading.Event] = {}
package_name = 'planarmwebsocketservice'
package_path = rospkg.RosPack().get_path(package_name)

UPLOAD_FILES_FOLDER = package_path + "/upload_files" 

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
H12_CONFIG_PATH = h12_package_path + "/config/customize_config.json"

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
ocs2_joint_state = JointState()
ocs2_hand_state = robotHandPosition()
ocs2_head_state = robotHeadMotionData()
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
            "humanoid_change_arm_ctrl_mode", ocs2ChangeArmCtrlMode
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
    global current_arm_joint_state
    current_arm_joint_state = msg.joint_data.joint_q[12:26]
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]
    current_arm_joint_state.extend([0] * 14)

def traj_callback(msg):
    global ocs2_joint_state
    if len(msg.points) == 0:
        return
    point = msg.points[0]
    ocs2_joint_state.name = msg.joint_names
    ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
    ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
    ocs2_joint_state.effort = [0] * 14

    ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
    ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]

    ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]

kuavo_arm_traj_pub = None
control_hand_pub = None
control_head_pub = None
update_h12_config_pub = None

def timer_callback(event):
    global kuavo_arm_traj_pub, control_hand_pub, control_head_pub
    if g_robot_type == "ocs2" and len(ocs2_joint_state.position) > 0 and plan_arm_state_status is False:
        kuavo_arm_traj_pub.publish(ocs2_joint_state)
        control_hand_pub.publish(ocs2_hand_state)
        control_head_pub.publish(ocs2_head_state)

def init_publishers():
    global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, update_h12_config_pub
    kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
    control_hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
    control_head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1, tcp_nodelay=True)
    update_h12_config_pub = rospy.Publisher('/update_h12_customize_config', UpdateH12CustomizeConfig, queue_size=1, tcp_nodelay=True)

async def init_ros_node():
    print("Initializing ROS node")
    rospy.init_node("arm_action_server", anonymous=True, disable_signals=True)
    robot_plan_arm_state_topic_name = robot_settings[g_robot_type]["arm_traj_state_topic_name"]
    rospy.Subscriber(robot_plan_arm_state_topic_name, planArmState, plan_arm_state_callback)
    rospy.Subscriber('/bezier/arm_traj', JointTrajectory, traj_callback, queue_size=1, tcp_nodelay=True)
    # rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, mpc_obs_callback)
    rospy.Subscriber('/sensors_data_raw', sensorsData, sensors_data_callback, queue_size=1, tcp_nodelay=True)
    
    init_publishers()
    
    # Create a timer that calls timer_callback every 10ms (100Hz)
    rospy.Timer(rospy.Duration(0.01), timer_callback)

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
    req.joint_names = ["l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll", "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    return req

def plan_arm_trajectory_bezier_curve_client(req):
    service_name = robot_settings[g_robot_type]["plan_arm_trajectory_bezier_service_name"]
    rospy.wait_for_service(service_name)
    try:
        plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
        res = plan_service(req)
        return res.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def stop_plan_arm_trajectory_client():
    service_name = robot_settings[g_robot_type]["stop_plan_arm_trajectory_service_name"]
    rospy.wait_for_service(service_name)
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
    global g_robot_type
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

    start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)

    if g_robot_type == "ocs2":
        action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
        end_frame_time += 1
        call_change_arm_ctrl_mode_service(2)
    else:
        action_frames = frames_to_custom_action_data(action_file_path)

    req = create_bezier_request(action_frames, start_frame_time, end_frame_time)
    plan_arm_trajectory_bezier_curve_client(req)
    time.sleep(0.5)
    # If valid, create initial response
    response = Response(
        payload=payload,
        target=websocket,
    )

    # Create a new stop event for this thread
    stop_event = threading.Event()
    active_threads[websocket] = stop_event

    # Start a thread to update the progress
    thread = threading.Thread(
        target=update_preview_progress, args=(response, stop_event)
    )
    print("Starting thread to update progress")
    thread.start()


async def stop_preview_action_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
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

    robot_version = rospy.get_param('robot_version')
    payload = Payload(
        cmd="get_robot_info", 
        data={
            "code": 0, 
            "robot_type": robot_version,
            "music_folder_path": MUSIC_FILE_FOLDER,
            "h12_config_path": H12_CONFIG_PATH,
            "repo_path": REPO_PATH
        }
    )

    response = Response(
        payload=payload,
        target=websocket,
    )
    response_queue.put(response)


async def get_robot_status_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="get_robot_status", data={"code": 0, "isrun": True}
    )

    if not active_threads:
        payload.data["isrun"] = False
        print("No activate threads")
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
        target=execute_command_progress, args=(websocket, response, stop_event, execute_path)
    )
    print("Starting thread to update progress")
    thread.start()


def execute_command_progress(websocket: websockets.WebSocketServerProtocol, response: Response, stop_event: threading.Event, execute_path):
    payload = response.payload
    update_interval = 0.001 

    if not os.path.exists(execute_path):
        payload.data["code"] = 1
        payload.data["msg"] = "File not found."
        response_queue.put(response)
        return

    py_exe = sys.executable
    command_list = [py_exe, execute_path]
    print(f"Executing command: {command_list}")
    global process
    try: 
        process = subprocess.Popen(command_list)
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
    global should_stop
    process_terminated = False
    while True:
        time.sleep(1)
        if should_stop:
            print("Kill the target process")
            process.terminate()
            try:
                process.wait(timeout=3)
                process_terminated = True
            except subprocess.TimeoutExpired:
                print("Forced kill the target process")
                process.kill()
                try:
                    process.wait(timeout=2)
                    process_terminated = True
                except subprocess.TimeoutExpired:
                    process_terminated = False
            break
    return process_terminated

async def stop_run_node_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]
    
    payload = Payload(
        cmd="stop_run_node", data={"code":0}
    )

    global process, should_stop
    monitor_thread = threading.Thread(target=monitor_and_stop, args=(process,))
    monitor_thread.start()
    should_stop = True
    process_terminated = monitor_thread.join()
    
    if not process_terminated:
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
            result = upload_music_file(music_filename)
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
    global update_h12_config_pub
    payload = Payload(
        cmd="update_h12_config", data={"code": 0, "msg": "msg"}
    )

    # 更新H12遥控器按键配置
    msg = UpdateH12CustomizeConfig()
    msg.update_msg = "Update H12 Config"
    update_h12_config_pub.publish(msg)

    try:
        # 确认音乐文件在上位机还是下位机播放，并进行相应处理
        if is_player_in_body():
            payload.data["code"] = 0
            payload.data["msg"] = "Body NUC"
        else:
            result = upload_music_file("")
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
        result = download_data_pilot_to_head()
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


# Add a function to clean up when a websocket connection is closed
def cleanup_websocket(websocket: websockets.WebSocketServerProtocol):
    if websocket in active_threads:
        active_threads[websocket].set()
        del active_threads[websocket]

# Add a new function to set the robot type
def set_robot_type(robot_type: str):
    global g_robot_type
    g_robot_type = robot_type
