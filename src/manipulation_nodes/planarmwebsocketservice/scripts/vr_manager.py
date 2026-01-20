#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VR录制管理模块 - 定义录制状态常量和枚举
"""

from enum import Enum


class VRState(Enum):
    """VR连接状态枚举"""
    DISCONNECTED = "disconnected"        # 未连接
    CONNECTING = "connecting"            # 连接中
    CONNECTED = "connected"              # 已连接
    RECORDING = "recording"              # 录制中
    ERROR = "error"                      # 错误状态


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
vr_state = VRState.DISCONNECTED
recording_state = RecordingState.IDLE
recording_start_time = None


def set_vr_state(state):
    """设置VR连接状态"""
    global vr_state
    vr_state = state


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
    """
    import time

    # 计算录制时长
    recording_duration = None
    if recording_state == RecordingState.RECORDING and recording_start_time:
        recording_duration = time.time() - recording_start_time

    return {
        "vr_connected": vr_state == VRState.CONNECTED,
        "vr_state": vr_state.value,
        "recording_state": recording_state.value,
        "recording_duration": recording_duration
    }
