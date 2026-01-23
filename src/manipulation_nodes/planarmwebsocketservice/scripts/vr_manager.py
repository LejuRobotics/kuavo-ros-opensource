#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR录制管理模块 - 定义录制状态常量和枚举
"""

from enum import Enum


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

    VR连接状态通过检查ROS节点是否存在来判断
    """
    import time
    import subprocess

    # 检查VR节点是否在运行
    vr_connected = False
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
        vr_connected = any(node in nodes for node in vr_nodes)
    except:
        pass

    # 根据VR节点状态和录制状态确定vr_state
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

    # 检查是否已经在录制
    if recording_state == RecordingState.RECORDING:
        return False, "Already recording"

    try:
        import os
        import subprocess
        import time

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
