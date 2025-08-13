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
from typing import Any, Union, Dict
import websockets
import threading
import time
import math
import subprocess
import base64
from pathlib import Path
import yaml
from utils import calculate_file_md5, frames_to_custom_action_data, get_start_end_frame_time, frames_to_custom_action_data_ocs2
import shutil
import rosnode
from kuavo_ros_interfaces.srv import planArmTrajectoryBezierCurve, stopPlanArmTrajectory, planArmTrajectoryBezierCurveRequest, ocs2ChangeArmCtrlMode
from kuavo_ros_interfaces.msg import planArmState, jointBezierTrajectory, bezierCurveCubicPoint, robotHeadMotionData
from kuavo_msgs.msg import robotHandPosition
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from kuavo_msgs.msg import sensorsData
from h12pro_controller_node.msg import UpdateH12CustomizeConfig
from kuavo_msgs.srv import adjustZeroPoint, adjustZeroPointRequest, LoadMap, LoadMapRequest, GetAllMaps, GetAllMapsRequest,SetInitialPose, SetInitialPoseRequest
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import tf
# Replace multiprocessing values with simple variables
plan_arm_state_progress = 0
plan_arm_state_status = False
should_stop = False
terminate_process_result = False
process = None
response_queue = Queue()
active_threads: Dict[websockets.WebSocketServerProtocol, threading.Event] = {}
ACTION_FILE_FOLDER = "~/.config/lejuconfig/action_files"
g_robot_type = ""
ocs2_current_joint_state = []
robot_version = (int)(os.environ.get("ROBOT_VERSION", "45"))

current_arm_joint_state = None
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
robot_version = (int)(os.environ.get("ROBOT_VERSION", "45"))

if robot_version >= 40:
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
    global robot_version
    if robot_version >= 40:
        current_arm_joint_state = msg.joint_data.joint_q[12:26]
        current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]
        current_arm_joint_state.extend([0] * 14)
    elif robot_version >= 10 and robot_version < 30:
        current_arm_joint_state = msg.joint_data.joint_q[12:20]
        current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]
        current_arm_joint_state.extend([0] * 8)

def traj_callback(msg):
    global ocs2_joint_state
    global robot_version
    if len(msg.points) == 0:
        return
    point = msg.points[0]
    
    if robot_version >= 40:
        ocs2_joint_state.name = joint_names
        ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        ocs2_joint_state.effort = [0] * 14
        ocs2_hand_state.left_hand_position = [int(math.degrees(pos)) for pos in point.positions[14:20]]
        ocs2_hand_state.right_hand_position = [int(math.degrees(pos)) for pos in point.positions[20:26]]
        ocs2_head_state.joint_data = [math.degrees(pos) for pos in point.positions[26:]]

    elif robot_version >= 10 and robot_version < 30:
        ocs2_joint_state.name = joint_names
        ocs2_joint_state.position = [math.degrees(pos) for pos in point.positions[:8]]
        ocs2_joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:8]]
        ocs2_joint_state.effort = [0] * 8


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
    global kuavo_arm_traj_pub, control_hand_pub, control_head_pub, update_h12_config_pub,load_map_pub
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
    if robot_version >= 40:
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
    payload = Payload(
        cmd="adjust_zero_point", data={"code": 0, "message": "Zero point adjusted successfully"}
    )

    data = data["data"]
    print(f"received adjust zero point data: {data}")

    try:
        motor_index = data.get("motor_index")
        adjust_pos = data.get("adjust_pos")

        if motor_index is None or adjust_pos is None:
            payload.data["code"] = 1
            payload.data["message"] = "Missing required parameters motor_index or adjust_pos"
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
            return


        # Check if nodelet_manager is running
        running_nodes = rosnode.get_node_names()
        if '/nodelet_manager' in running_nodes:

            # Check hardware ready status first
            rospy.wait_for_service('hardware/get_hardware_ready', timeout=0.5)
            get_hardware_ready = rospy.ServiceProxy('hardware/get_hardware_ready', Trigger)

            hw_res = get_hardware_ready()
            print(hw_res.message)
            if hw_res.message == 'Hardware is ready':
            
                print(f"current cannot adjust motor zero point")
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
    payload = Payload(
        cmd="set_zero_point", data={"code": 0, "message": "Zero point set successfully"}
    )

    data = data["data"]
    print(f"received zero point data: {data}")
    
    arm_zero_file_path = f"/root/.config/lejuconfig/arms_zero.yaml"
    leg_zero_file_path = f"/root/.config/lejuconfig/offset.csv"
    
    robot_version = (int)(os.environ.get("ROBOT_VERSION", "45"))
    # Get kuavo_assets package path
    kuavo_assets_path = rospkg.RosPack().get_path('kuavo_assets')
    with open(f"{kuavo_assets_path}/config/kuavo_v{robot_version}/kuavo.json", 'r') as f:
        json_config = json.load(f)
    
    arm_joints = json_config["NUM_ARM_JOINT"]
    head_joints = json_config["NUM_HEAD_JOINT"]
    if robot_version >= 11 and robot_version <= 19:
        waist_joints = json_config["NUM_WAIST_JOINT"]
    else:
        waist_joints = 0
    num_joints = json_config["NUM_JOINT"]
    ec_joints = num_joints - arm_joints - head_joints
    
    try:
        if "zero_pos" in data:
            zero_pos = data["zero_pos"]
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
        
            response = Response(payload=payload, target=websocket)
            response_queue.put(response)
        else :
            payload.data["code"] = 1
            payload.data["message"] = "Invalid zero point data"
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
    payload = Payload(
        cmd="get_zero_point", data={"code": 0, "message": "Zero point retrieved successfully"}
    )

    robot_version = (int)(os.environ.get("ROBOT_VERSION", "45"))
    # Get kuavo_assets package path
    kuavo_assets_path = rospkg.RosPack().get_path('kuavo_assets')
    with open(f"{kuavo_assets_path}/config/kuavo_v{robot_version}/kuavo.json", 'r') as f:
        json_config = json.load(f)
    
    arm_joints = json_config["NUM_ARM_JOINT"]
    head_joints = json_config["NUM_HEAD_JOINT"]
    if robot_version >= 11 and robot_version <= 19:
        waist_joints = json_config["NUM_WAIST_JOINT"]
    else:
        waist_joints = 0
    num_joints = json_config["NUM_JOINT"]
    num_joints = json_config["NUM_JOINT"]
    ec_joints = num_joints - arm_joints - head_joints

    arm_zero_file_path = f"/root/.config/lejuconfig/arms_zero.yaml"
    leg_zero_file_path = f"/root/.config/lejuconfig/offset.csv"
    
    if not os.path.exists(arm_zero_file_path) or not os.path.exists(leg_zero_file_path):
        payload.data["code"] = 1
        payload.data["message"] = f"Zero point file not found: {arm_zero_file_path} or {leg_zero_file_path}"
        response = Response(payload=payload, target=websocket)
        response_queue.put(response)
        return

    try:
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

        payload.data["zero_pos"] = leg_zero_data + arm_zero_data
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

    robot_version = rospy.get_param('robot_version')
    payload = Payload(
        cmd="get_robot_info", 
        data={
            "code": 0, 
            "robot_type": robot_version,
            "music_folder_path": MUSIC_FILE_FOLDER,
            "maps_folder_path": MAP_FILE_FOLDER,
            "h12_config_path": H12_CONFIG_PATH,
            "repo_path": REPO_PATH
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
        rospy.wait_for_service(service_name,timeout=3.0)
        load_map_client = rospy.ServiceProxy(service_name, LoadMap)

        # request
        req = LoadMapRequest()
        req.map_name = map_name
        
        # response  
        res = load_map_client(req)
        if res.success:
            if  download_map_file(map_name):
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
        rospy.wait_for_service(service_name,timeout=3.0)
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
        
        # response
        res = init_localization_by_pose_client(req)
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

def download_map_file(map_name: str = ""):
    # 将地图文件下载到本地指定路径
    # 订阅一次/map话题，将得到的map数据保存为png格式

    """
    订阅一次/map话题，将地图数据保存为png格式图片
    """
    global MAP_FILE_FOLDER
    try:
        msg = rospy.wait_for_message('/map', OccupancyGrid)
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


async def get_robot_position_handler(
    websocket: websockets.WebSocketServerProtocol, data: dict
):
    payload = Payload(
        cmd="get_robot_position", data={"code":0}
    )
    try:
        # 获取base_link在map坐标系下的位置
        listener = tf.TransformListener()
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        # trans为(x, y, z)
        x, y, z = trans

        # 获取地图信息
        map_msg = rospy.wait_for_message('/map', OccupancyGrid, timeout=2.0)
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
    payload = Payload(
        cmd="get_all_maps", data={"code":0}
    )
    try:
        service_name = 'get_all_maps'
        rospy.wait_for_service(service_name,timeout=3.0)
        get_all_maps_client = rospy.ServiceProxy(service_name, GetAllMaps)  
        
        # request   
        req = GetAllMapsRequest()

        # response
        res = get_all_maps_client(req)
        payload.data["maps"] = res.maps
        payload.data["msg"] = "Get all maps successfully"
    except rospy.ServiceException as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Service `get_all_maps` call failed: {e}"
    except rospy.ROSException as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Service `get_all_maps` call failed: {e}"
    except Exception as e:
        payload.data["code"] = 1
        payload.data["msg"] = f"Service `get_all_maps` call failed: {e}"
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