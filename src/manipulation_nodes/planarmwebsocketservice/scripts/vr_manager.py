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
current_bag_file = None
recording_start_time = None
