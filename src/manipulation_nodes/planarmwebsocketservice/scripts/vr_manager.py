#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR录制管理模块 - 定义录制状态常量和枚举
"""

from enum import Enum
import os
import signal
import subprocess
import threading
import time
import traceback

class VRState(Enum):
    """VR连接状态枚举"""
    DISCONNECTED = "disconnected"        # 未连接
    CONNECTED = "connected"              # 已连接
    RECORDING = "recording"              # 录制中


class RecordingState(Enum):
    """录制状态枚举"""
    IDLE = "idle"                        # 空闲
    RECORDING = "recording"              # 录制中
    STOPPED = "stopped"                  # 已停止
    CONVERTING = "converting"            # 转换中
    COMPLETED = "completed"              # 已完成
    CANCELLED = "cancelled"              # 已取消
    ERROR = "error"                      # 错误


# 全局状态变量
vr_connected = False
vr_nodes_running = False
recording_state = RecordingState.IDLE
recording_start_time = None
recording_process = None

# 录制文件保存路径
RECORDING_SAVE_PATH = "~/.config/lejuconfig/vr_recordings"

# 录制的ROS topics
RECORDING_TOPICS = [
    "/sensors_data_raw",
    "/dexhand/state"
]


def set_recording_state(state):
    """设置录制状态"""
    global recording_state
    recording_state = state


def set_recording_start_time(start_time):
    """设置录制开始时间"""
    global recording_start_time
    recording_start_time = start_time


def get_vr_status():
    """
    获取VR状态信息
    返回完整的VR状态字典

    VR连接状态通过读取参数服务器 /quest3/connected 参数来判断
    VR节点状态通过检查ROS节点是否存在来判断

    同时更新全局变量 vr_connected 和 vr_nodes_running
    """
    global vr_connected, vr_nodes_running

    # 检查VR连接状态（通过参数服务器）并更新全局变量
    try:
        import rospy
        vr_connected = rospy.get_param('/quest3/connected', False)
    except:
        vr_connected = False

    # 检查VR节点是否在运行并更新全局变量
    try:
        result = subprocess.run(
            ["rosnode", "list"],
            capture_output=True,
            text=True,
            timeout=2
        )
        nodes = result.stdout.strip().split('\n')

        # 检查是否有VR相关节点在运行
        vr_nodes = ["/ik_ros_uni", "/ik_ros_uni_cpp_node", "/monitor_quest3", "/monitor_quest3_cpp"]
        vr_nodes_running = any(node in nodes for node in vr_nodes)
    except:
        vr_nodes_running = False

    if not vr_nodes_running:
        vr_connected = False

    # 根据VR连接状态和录制状态确定vr_state
    if recording_state == RecordingState.RECORDING:
        actual_vr_state = VRState.RECORDING
    elif vr_connected:
        actual_vr_state = VRState.CONNECTED
    else:
        actual_vr_state = VRState.DISCONNECTED

    # 计算录制时长
    recording_duration = None
    if recording_state == RecordingState.RECORDING and recording_start_time:
        recording_duration = time.time() - recording_start_time

    return {
        "vr_connected": vr_connected,
        "vr_nodes_running": vr_nodes_running,
        "vr_state": actual_vr_state.value,
        "recording_state": recording_state.value,
        "recording_duration": recording_duration
    }


def start_recording():
    """
    开始录制VR数据到bag文件
    文件名自动生成（时间戳）

    Returns:
        (success, message): (是否成功, 消息)
    """
    global recording_state, recording_start_time, recording_process

    
    if not vr_connected:
        return False, "VR设备未连接，无法开始录制"

    # 检查是否已经在录制
    if recording_state == RecordingState.RECORDING:
        return False, "Already recording"

    try:
        # 生成文件名（时间戳）
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"vr_recording_{timestamp}"

        # 确保保存目录存在
        save_path = os.path.expanduser(RECORDING_SAVE_PATH)
        os.makedirs(save_path, exist_ok=True)

        # 构建bag文件路径
        bag_path = os.path.join(save_path, filename)

        # 构建rosbag record命令
        cmd = [
            "rosbag", "record",
            "-O", bag_path,
            *RECORDING_TOPICS
        ]

        # 启动录制进程
        recording_process = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # 更新状态
        set_recording_state(RecordingState.RECORDING)
        set_recording_start_time(time.time())

        return True, f"Recording started: {filename}.bag"

    except Exception as e:
        set_recording_state(RecordingState.ERROR)
        return False, f"Failed to start recording: {str(e)}"


def stop_recording_and_convert(tact_filename):
    """
    停止录制并转换为TACT文件

    Args:
        tact_filename: TACT文件名（不含.tact后缀，支持中英文数字）

    Returns:
        (success, message): (是否成功, 消息)
    """
    global recording_state, recording_start_time, recording_process


    if not vr_connected:
        return False, "VR设备未连接，无法停止录制"

    # 检查是否正在录制
    if recording_state != RecordingState.RECORDING:
        return False, "No recording in progress"

    try:
        # 获取bag文件路径（从recording_start_time推算）
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(recording_start_time))
        bag_filename = f"vr_recording_{timestamp}"
        save_path = os.path.expanduser(RECORDING_SAVE_PATH)
        bag_path = os.path.join(save_path, f"{bag_filename}.bag")

        # 检查目标文件是否已存在
        target_tact_path = os.path.join(save_path, f"{tact_filename}.tact")
        if os.path.exists(target_tact_path):
            return False, f"File {tact_filename}.tact already exists"

        # 停止录制进程
        if recording_process:
            os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
            recording_process.wait(timeout=5)
            recording_process = None

        # 更新状态
        set_recording_state(RecordingState.CONVERTING)

        # 记录转换前的tact文件列表（用于后续查找新生成的文件）
        tact_files_before = set()
        if os.path.exists(save_path):
            tact_files_before = set(f for f in os.listdir(save_path) if f.endswith('.tact'))

        # 转换函数（在单独线程中运行）
        def convert_bag_to_tact():
            try:
                # 从系统环境变量获取机器人版本（必须有）
                if 'ROBOT_VERSION' not in os.environ:
                    raise ValueError("ROBOT_VERSION environment variable is not set")

                # 导入转换器
                from rosbag_to_bezier_planner import RosbagToBezierPlanner

                # 创建处理器实例（会自动读取ROBOT_VERSION环境变量）
                processor = RosbagToBezierPlanner()

                # 执行转换（使用固定参数）
                output_dir = save_path
                success = processor.run(
                    bag_file_path=bag_path,
                    sampling_rate=0.1,
                    smoothing_factor=0.3,
                    output_dir=output_dir,
                    save_tact=True,
                    custom_sampling_rate=0.5,
                    get_raw_trajectory=False,
                    save_sampling_tact=True
                )

                if success:
                    # 查找新生成的tact文件
                    tact_files_after = set(f for f in os.listdir(output_dir) if f.endswith('.tact'))
                    new_tact_files = tact_files_after - tact_files_before

                    if new_tact_files:
                        # 取生成的tact文件
                        new_tact_file = list(new_tact_files)[0]
                        old_path = os.path.join(output_dir, new_tact_file)
                        new_path = target_tact_path

                        # 重命名为用户指定的文件名
                        os.rename(old_path, new_path)
                        print(f"Renamed {new_tact_file} to {tact_filename}.tact")
                    else:
                        print("Warning: No new tact file generated")

                    set_recording_state(RecordingState.COMPLETED)
                else:
                    # 转换失败，清理可能生成的文件
                    tact_files_after = set(f for f in os.listdir(output_dir) if f.endswith('.tact'))
                    new_tact_files = tact_files_after - tact_files_before
                    for new_file in new_tact_files:
                        file_path = os.path.join(output_dir, new_file)
                        try:
                            os.remove(file_path)
                            print(f"Cleaned up failed conversion file: {new_file}")
                        except:
                            pass

                    set_recording_state(RecordingState.ERROR)

            except Exception as e:
                print(f"Conversion error: {e}")
                print(traceback.format_exc())

                # 清理可能生成的文件
                try:
                    tact_files_after = set(f for f in os.listdir(output_dir) if f.endswith('.tact'))
                    new_tact_files = tact_files_after - tact_files_before
                    for new_file in new_tact_files:
                        file_path = os.path.join(output_dir, new_file)
                        try:
                            os.remove(file_path)
                            print(f"Cleaned up failed conversion file: {new_file}")
                        except:
                            pass
                except:
                    pass

                set_recording_state(RecordingState.ERROR)

        # 启动转换线程
        convert_thread = threading.Thread(target=convert_bag_to_tact, daemon=True)
        convert_thread.start()

        return True, f"Recording stopped, converting to {tact_filename}.tact"

    except Exception as e:
        set_recording_state(RecordingState.ERROR)
        return False, f"Failed to stop recording: {str(e)}"


def cancel_recording():
    """
    取消录制（停止录制并删除bag文件）

    Returns:
        (success, message): (是否成功, 消息)
    """
    global recording_state, recording_start_time, recording_process

    
    if not vr_connected:
        return False, "VR设备未连接，无法取消录制"

    # 检查是否正在录制
    if recording_state != RecordingState.RECORDING:
        return False, "No recording in progress"

    try:
        # 获取bag文件路径（从recording_start_time推算）
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(recording_start_time))
        bag_filename = f"vr_recording_{timestamp}"
        save_path = os.path.expanduser(RECORDING_SAVE_PATH)
        bag_path = os.path.join(save_path, f"{bag_filename}.bag")

        # 停止录制进程
        if recording_process:
            os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
            recording_process.wait(timeout=5)
            recording_process = None

        # 删除bag文件
        if os.path.exists(bag_path):
            os.remove(bag_path)
            print(f"Deleted bag file: {bag_filename}.bag")

        # 重置状态
        set_recording_state(RecordingState.CANCELLED)
        recording_start_time = None

        return True, f"Recording cancelled, deleted {bag_filename}.bag"

    except Exception as e:
        set_recording_state(RecordingState.ERROR)
        return False, f"Failed to cancel recording: {str(e)}"
