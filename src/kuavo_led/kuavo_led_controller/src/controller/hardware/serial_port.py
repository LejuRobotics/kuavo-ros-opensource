#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED 串口通信模块
实现单例模式的串口通信类，用于控制 LED 灯带
"""

import serial
import threading
from collections import deque
from typing import Optional, Dict


class SerialPort:
    """
    LED 串口通信类（单例模式）

    配置参数:
        - port: '/dev/ttyLED0' (默认)
        - baudrate: 115200 (默认)
        - bytesize: 8 (数据位)
        - stopbits: 1 (停止位)
        - parity: 'N' (无校验)
        - 大端序传输
        - 后台实时读取，缓冲区大小 4096 字节
    """

    _instance: Optional['SerialPort'] = None
    _lock = threading.Lock()

    # 缓冲区配置
    _MAX_BUFFER_SIZE = 4096  # 最大缓冲区大小（字节）

    def __new__(cls, *args, **kwargs) -> 'SerialPort':
        """单例模式 - 确保只存在一个实例"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, port: str = '/dev/ttyLED0', baudrate: int = 115200):
        """
        初始化串口

        Args:
            port: 串口设备路径，默认为 '/dev/ttyLED0'
            baudrate: 波特率，默认为 115200
        """
        # 防止重复初始化
        if hasattr(self, '_initialized') and self._initialized:
            return

        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self._initialized = False

        # 读取缓冲区（后台线程填充）
        self._read_buffer: deque = deque()
        self._buffer_lock = threading.Lock()
        self._overflow_count = 0

        # 后台读取线程
        self._read_thread: Optional[threading.Thread] = None
        self._stop_read = threading.Event()

        self._open()

    def _open(self) -> None:
        """打开串口并启动后台读取线程"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,      # 数据位：8
                stopbits=serial.STOPBITS_ONE,   # 停止位：1
                parity=serial.PARITY_NONE,      # 无校验
                timeout=0                       # 非阻塞读取
            )
            self._initialized = True
            print(f"[SerialPort] 串口 {self.port} 已打开")

            # 启动后台读取线程
            self._start_read_thread()
        except serial.SerialException as e:
            print(f"[SerialPort] 打开串口失败：{e}")
            self._initialized = False

    def _start_read_thread(self) -> None:
        """启动后台读取线程"""
        self._stop_read.clear()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self) -> None:
        """后台持续读取串口数据到缓冲区"""
        while not self._stop_read.is_set():
            if self.is_open():
                try:
                    # 检查有多少数据可读
                    in_waiting = self.serial.in_waiting
                    if in_waiting > 0:
                        # 一次性读取所有可用数据
                        data = self.serial.read(in_waiting)
                        self._add_to_buffer(data)
                except serial.SerialException:
                    pass
            # 短暂休眠，避免 CPU 占用过高（1ms）
            self._stop_read.wait(0.001)

    def _add_to_buffer(self, data: bytes) -> None:
        """添加数据到缓冲区，带溢出保护"""
        with self._buffer_lock:
            for byte in data:
                if len(self._read_buffer) >= self._MAX_BUFFER_SIZE:
                    # 缓冲区满，丢弃最旧的数据
                    self._read_buffer.popleft()
                    self._overflow_count += 1
                self._read_buffer.append(byte)

    def _close(self) -> None:
        """关闭串口并停止后台读取线程"""
        # 先停止读取线程
        self._stop_read.set()
        if self._read_thread:
            self._read_thread.join(timeout=0.5)

        # 关闭串口
        if self.serial and self.serial.is_open:
            self.serial.close()
            self._initialized = False
            print(f"[SerialPort] 串口 {self.port} 已关闭")

    def is_open(self) -> bool:
        """检查串口是否已打开"""
        return self._initialized and self.serial is not None and self.serial.is_open

    def send_data(self, data: bytes) -> bool:
        """
        通过串口发送数据（大端序）

        Args:
            data: 要发送的字节数据

        Returns:
            bool: 发送成功返回 True，失败返回 False
        """
        if not self.is_open():
            print("[SerialPort] 串口未打开，无法发送数据")
            return False

        try:
            # 确保数据以大端序方式发送
            self.serial.write(data)
            self.serial.flush()
            return True
        except serial.SerialException as e:
            print(f"[SerialPort] 发送数据失败：{e}")
            return False

    def send_led_control_packet(self, packet: bytes) -> bool:
        """
        发送 LED 灯控制数据包

        Args:
            packet: LED 控制数据包（字节流）

        Returns:
            bool: 发送成功返回 True，失败返回 False
        """
        return self.send_data(packet)

    def read_data(self, size: int = 1) -> bytes:
        """
        从缓冲区读取数据（非阻塞）

        Args:
            size: 要读取的字节数

        Returns:
            bytes: 读取到的数据，无数据返回空字节
        """
        if not self.is_open():
            return b''

        with self._buffer_lock:
            if len(self._read_buffer) == 0:
                return b''

            actual_size = min(size, len(self._read_buffer))
            data = bytes(list(self._read_buffer)[:actual_size])
            # 从缓冲区移除已读数据
            for _ in range(actual_size):
                self._read_buffer.popleft()
            return data

    def read_all(self) -> bytes:
        """
        读取缓冲区所有数据（非阻塞）

        Returns:
            bytes: 缓冲区中的所有数据
        """
        if not self.is_open():
            return b''

        with self._buffer_lock:
            data = bytes(self._read_buffer)
            self._read_buffer.clear()
            return data

    def get_buffer_info(self) -> Dict:
        """
        获取缓冲区状态信息

        Returns:
            dict: 包含缓冲区大小、使用率、溢出次数等信息
        """
        with self._buffer_lock:
            current_size = len(self._read_buffer)
            return {
                'current_size': current_size,
                'max_size': self._MAX_BUFFER_SIZE,
                'usage_percent': current_size / self._MAX_BUFFER_SIZE * 100,
                'overflow_count': self._overflow_count
            }

    def __del__(self):
        """析构函数，关闭串口"""
        self._close()

    def __enter__(self):
        """上下文管理器入口"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self._close()
