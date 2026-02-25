#!/usr/bin/env python3
import time
import subprocess
import os
import rospy
import psutil
import signal
import sys

from utils.console import console
from utils.utils import find_and_send, open_serial_port, send_data_to_port

from h12pro_controller_node.srv import (
    srvChangePhases,
    changeAMBACCtrlMode,
    changeArmCtrlMode,
    changeHandArmPosesByConfigName,
)

catkin_ws_path = os.environ.get("CATKIN_WS_PATH", None)
kuavo_process_name = "highlyDynamicRobot_node"
kuavo_group_id = None

vol_max = bytes([0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x1B, 0xFE, 0xDA, 0xEF])
play_11 = bytes([0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFE, 0xF7, 0xEF])
play_12 = bytes([0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x02, 0xFE, 0xF6, 0xEF])
play_13 = bytes([0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x03, 0xFE, 0xF5, 0xEF])
play_14 = bytes([0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x04, 0xFE, 0xF4, 0xEF])

arm_pose_setting = {
    "arm_pose1": {"pose_file": "arm_pose1.csv", "pose_bgm": play_11},
    "arm_pose2": {"pose_file": "arm_pose2.csv", "pose_bgm": play_12},
    "arm_pose3": {"pose_file": "arm_pose3.csv", "pose_bgm": play_13},
    "arm_pose4": {"pose_file": "arm_pose4.csv", "pose_bgm": play_14},
}

ser = None
port = find_and_send()
if port == 0:
    print("没有找到串口")
else:
    ser = open_serial_port(port)
    if ser:
        print("串口打开成功!")
        send_data_to_port(ser, vol_max)


def print_state_transition(trigger, source, target) -> None:
    console.print(
        f"Trigger: [bold blue]{trigger}[/bold blue] From [bold green]{source}[/bold green] to [bold green]{target}[/bold green]"
    )


def check_kuavo_node() -> bool:
    output = subprocess.check_output("rosnode list", shell=True)
    return "/HDrobot_node" in output.decode("utf-8")


def start_kuavo_node(*args, **kwargs) -> None:
    if catkin_ws_path is None:
        raise Exception("Please set CATKIN_WS_PATH environment variable")
    run_ros_node_command = f"whoami && source {catkin_ws_path}/devel/setup.bash && rosrun dynamic_biped highlyDynamicRobot_node --log_lcm"
    if kwargs.get("real"):
        run_ros_node_command += " --real"
    if kwargs.get("cali"):
        run_ros_node_command += " --cali"
    if kwargs.get("setzero"):
        run_ros_node_command += " --setzero"
    command = f"bash -c '{run_ros_node_command}'"

    process = subprocess.Popen(command, shell=True)
    if process.poll() is None:
        console.print("Started highlyDynamicRobot_node")
    else:
        raise Exception("Failed to start highlyDynamicRobot_node")
    global kuavo_group_id
    kuavo_group_id = os.getpgid(process.pid)
    console.print(f"kuavo_group_id: {kuavo_group_id}")


def kill_kuavo() -> None:
    global kuavo_group_id
    try:
        parent = psutil.Process(kuavo_group_id)
        kuavo_pid = None
        for child in parent.children(recursive=True):
            if child.name() == kuavo_process_name:
                kuavo_pid = child.pid
                break
        if kuavo_pid:
            os.kill(kuavo_pid, signal.SIGINT)
            time.sleep(3)  # leave time for kuavo to handle siganl
        else:
            console.print("No kuavo process found to kill")
    except Exception as e:
        print(e)


def call_change_AMBAC(control_mode):
    rospy.wait_for_service("change_AMBAC_ctrl_mode", timeout=0.5)
    change_robot_AMBAC = rospy.ServiceProxy(
        "change_AMBAC_ctrl_mode", changeAMBACCtrlMode
    )
    response = change_robot_AMBAC(control_mode=control_mode)


def call_change_phase(mainPhase, subPahse) -> None:
    rospy.wait_for_service("setPhase", timeout=0.5)
    change_robot_pahse = rospy.ServiceProxy("setPhase", srvChangePhases)
    response = change_robot_pahse(stateReq=mainPhase, subState=subPahse)


def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    result = True
    service_name = "change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "change_arm_ctrl_mode", changeArmCtrlMode
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
        if result is False:
            console.print(
                f"[red]Failed to change arm control mode to {arm_ctrl_mode}[/red]"
            )
        else:
            console.print(
                f"[green]Changed arm control mode to {arm_ctrl_mode} successfully[/green]"
            )
        return result


def call_change_hand_arm_pose_by_config_name(config_name):
    try:
        rospy.wait_for_service("change_hand_arm_poses_by_config_name", timeout=0.5)
        change_hand_arm_pose_by_config_name = rospy.ServiceProxy(
            "change_hand_arm_poses_by_config_name", changeHandArmPosesByConfigName
        )
        change_hand_arm_pose_by_config_name(config_name=config_name)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
    except rospy.ROSException:
        rospy.logerr("Service change_hand_arm_pose_by_config_name not available")
    except Exception as e:
        rospy.logerr(f"Failed to change hand arm pose by config name: {e}")


def calibrate_callback(event) -> None:
    source = event.kwargs.get("source")
    real = event.kwargs.get("real")
    start_kuavo_node(real=real, cali=True)
    print_state_transition("calibrate", source, "calibrate")
    time.sleep(5)


def calibrate_finished_callback(event) -> None:
    source = event.kwargs.get("source")
    kill_kuavo()
    time.sleep(2)
    print_state_transition("calibrate finished", source, "initial")


def start_callback(event) -> None:
    source = event.kwargs.get("source")
    real = event.kwargs.get("real")
    start_kuavo_node(real=real)
    print_state_transition("start", source, "squat")
    time.sleep(5)


def stop_callback(event) -> None:
    source = event.kwargs.get("source")
    kill_kuavo()
    time.sleep(2)
    print_state_transition("stop", source, "initial")


def squat_to_stand_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_AMBAC(True)
    call_change_phase("P_stand", "sub_phase_none")
    time.sleep(1)
    print_state_transition("squat to stand", source, "stand")


def squat_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_phase("P_squat", "sub_phase_none")
    time.sleep(1)
    print_state_transition("squat", source, "squat")


def walk_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_phase("P_walk", "sub_phase_none")
    time.sleep(1)
    rospy.loginfo("befo_walk_callbackl----------------------")
    print_state_transition("walk", source, "walk")


def stand_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_phase("P_stand", "sub_phase_none")
    time.sleep(1)
    print_state_transition("stand", source, "stand")


def jump_pre_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_phase("P_jump", "jump_pre")
    time.sleep(3)
    print_state_transition("jump pre", source, "jump")


def jump_take_off_callback(event) -> None:
    source = event.kwargs.get("source")
    call_change_phase("P_jump", "jump_take_off")
    time.sleep(3)
    print_state_transition("jump take off", source, "jump")


def emergency_stop_callback(event) -> None:
    source = event.kwargs.get("source")
    kill_kuavo()
    time.sleep(2)
    console.print("[bold red]Emergency stop!!![/bold red]")
    console.print(
        f"Trigger: [bold blue]emergency stop[/bold blue] From [bold green]{source}[/bold green] to [bold green]initial[/bold green]"
    )


def arm_pose_callback(event) -> None:
    pose_csv_file = arm_pose_setting[event.kwargs.get("trigger")]["pose_file"]
    pose_bgm = arm_pose_setting[event.kwargs.get("trigger")]["pose_bgm"]
    global ser
    if ser:
        send_data_to_port(ser, pose_bgm)
    call_change_hand_arm_pose_by_config_name(pose_csv_file)


def execute_command(command: str, cwd: str = None) -> subprocess.CompletedProcess:
    """
    Executes a shell command and returns the CompletedProcess instance.
    """
    result = subprocess.run(
        command, shell=True, capture_output=True, text=True, cwd=cwd
    )
    return result


def find_first_path(command: str) -> str:
    """
    Executes a find command and returns the first result.
    """
    result = execute_command(command)
    if result.returncode == 0:
        return result.stdout.strip().split("\n")[0]
    else:
        console.print("Error executing command:")
        console.print(result.stderr)
        return None


def set_zero_callback(event) -> None:
    source = event.kwargs.get("source")
    real = event.kwargs.get("real")
    start_kuavo_node(real=real, cali=True, setzero=True)
    time.sleep(30)
    kill_kuavo()
    print_state_transition("set zero", source, "initial")
