import subprocess
import rospy
import os
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
from h12pro_controller_node.srv import playmusic, playmusicRequest, playmusicResponse
from h12pro_controller_node.srv import ExecuteArmAction, ExecuteArmActionRequest, ExecuteArmActionResponse
from h12pro_controller_node.msg import RobotActionState
import os
# import netifaces
import json
import hashlib
import math
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

console = console.Console()

# switch_controller 冷却期机制
_switch_controller_lock = threading.Lock()
_switch_controller_cooling_until = 0.0  # 冷却期结束时间戳
SWITCH_CONTROLLER_COOLDOWN = 3.0  # 冷却期时长（秒）
current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(os.path.dirname(current_dir), "config")
ACTION_FILE_FOLDER = "~/.config/lejuconfig/action_files"
ROS_BAG_LOG_SAVE_PATH = "~/.log/vr_remote_control/rosbag"
HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
VR_REMOTE_CONTROL_SESSION_NAME = "vr_remote_control"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12 start_way:=auto"
# LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12"
LAUNCH_HUMANOID_ROBOT_REAL_CMD = "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12 start_way:=auto"
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

    if rospy.has_param("h12_log_channel"):
        log_channel_status = rospy.get_param("h12_log_channel")
        if log_channel_status is True:
            if os.path.exists("/dev/H12_log_channel"):
                log_channel_cmd_start = "stdbuf -oL -eL"
                log_channel_cmd_end = "2>&1 | sed -u -e 's/\\x1b\\[[0-9;]*[mK]//g' -e 's/$/\\r/' | stdbuf -oL tee /dev/H12_log_channel"
                launch_cmd = f"{log_channel_cmd_start} {launch_cmd} {log_channel_cmd_end}"
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
        rospy.wait_for_service(service_name, timeout=1.0)
        switch_client = rospy.ServiceProxy(service_name, switchController)
        response = switch_client(controller_name)
        if response.success:
            rospy.loginfo(f"Switch controller successful: {response.message}")
            rospy.loginfo(f"Switched to controller: '{controller_name}'")
        else:
            rospy.logwarn(f"Switch controller failed: {response.message}")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '{service_name}' failed: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Service '{service_name}' not available: {e}")
        return False

def call_switch_to_vmp_controller_service():
    """调用切换到VMP控制器的专用服务
    使用新的服务接口：/humanoid_controller/switch_to_vmp_controller (std_srvs/Trigger)
    
    :return: bool, 服务调用结果
    """
    service_name = "/humanoid_controller/switch_to_vmp_controller"
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
        switch_client = rospy.ServiceProxy(service_name, Trigger)
        response = switch_client()
        if response.success:
            rospy.loginfo(f"Switch to VMP controller successful: {response.message}")
        else:
            rospy.logwarn(f"Switch to VMP controller failed: {response.message}")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '{service_name}' failed: {e}")
        return False
    except rospy.ROSException as e:
        rospy.logerr(f"Service '{service_name}' not available: {e}")
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