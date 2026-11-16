#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED 灯带控制模块
实现 24 个 RGB 彩灯的控制逻辑
"""

from typing import List, Tuple
from enum import IntEnum
from hardware.serial_port import SerialPort


class LEDMode(IntEnum):
    """LED 显示模式"""
    CONSTANT = 0x00    # 常亮
    BREATHING = 0x01   # 呼吸
    FLASH = 0x02       # 快闪
    RHYTHM = 0x03      # 律动


class LEDStrip:
    """
    LED 灯带控制类

    规格:
        - 灯数量：24 个
        - 每个灯：RGB 三色，各占 8 位 (0-255)
        - 支持模式：常亮、呼吸、快闪、律动
    """

    LED_COUNT = 24
    PACKET_HEADER = bytes([0xFF, 0xFF])
    PACKET_TYPE = 0x00
    PACKET_INSTRUCTION = 0x02
    LED_ID = 0x02  # RGB 灯的固定 ID

    def __init__(self, port: str = '/dev/ttyLED0', baudrate: int = 115200):
        """
        初始化 LED 灯带

        Args:
            port: 串口设备路径，默认为 '/dev/ttyLED0'
            baudrate: 波特率，默认为 115200
        """
        # 存储 24 个灯的颜色数据，每个灯为 (R, G, B) 元组
        self._led_colors: List[Tuple[int, int, int]] = [(0, 0, 0)] * self.LED_COUNT
        self._mode: LEDMode = LEDMode.CONSTANT
        # 初始化串口
        self._serial_port: SerialPort = SerialPort(port=port, baudrate=baudrate)

    def _calculate_checksum(self, data: bytes) -> int:
        """
        计算校验和

        Args:
            data: 除包头外的所有数据 (Type + Length + Instruction + Params)

        Returns:
            校验和（取反后的 8 位值）
        """
        total = sum(data) & 0xFF
        checksum = (~total) & 0xFF  # 简单取反，保留 8 位
        return checksum

    def build_packet(self) -> bytes:
        """
        构建 LED 控制数据包

        数据包结构:
            Header1(1) + Header2(1) + Type(1) + Length(1) + Instruction(1) +
            Param1(ID)(1) + Param2(cmd)(1) + Param3~74(DATA)(72) + Checksum(1)

        Returns:
            完整的控制数据包（80 字节）
        """
        # Param 部分：ID(1) + cmd(1) + 24 个灯的 RGB 数据 (24*3=72) = 74 字节
        params = bytearray()
        params.append(self.LED_ID)           # RGB 灯固定 ID
        params.append(self._mode.value)      # 模式：0x00~0x03

        # 添加 24 个灯的 RGB 数据
        for r, g, b in self._led_colors:
            params.append(r)
            params.append(g)
            params.append(b)

        # Length = number of Parameters + 2 = 74 + 2 = 76 = 0x4C
        length = len(params) + 2

        # 构建除校验和外的所有数据
        packet_body = bytearray()
        packet_body.append(self.PACKET_TYPE)      # Type: 0x00
        packet_body.append(length)                 # Length: 0x4C
        packet_body.append(self.PACKET_INSTRUCTION)  # Instruction: 0x02
        packet_body.extend(params)                 # Params: 74 字节

        # 计算校验和
        checksum = self._calculate_checksum(bytes(packet_body))

        # 构建完整数据包
        packet = bytearray()
        packet.extend(self.PACKET_HEADER)  # 0xFF 0xFF
        packet.extend(packet_body)
        packet.append(checksum)

        return bytes(packet)

    def _verify_response(self, serial_port: SerialPort, timeout: float = 1.0) -> bool:
        """
        验证硬件回显数据（基于事件触发方式等待响应）

        Args:
            serial_port: 串口实例
            timeout: 超时时间（秒），默认 1.0 秒

        Returns:
            bool: 校验成功返回 True，失败返回 False
        """
        import time
        
        # 应答包固定为 6 字节: FF FF C8 02 00 35
        response_size = 6
        
        # 基于事件触发方式等待硬件响应（轮询缓冲区）
        start_time = time.time()
        while time.time() - start_time < timeout:
            # 只读取固定字节数，避免清空其他数据（如电池数据）
            response = serial_port.read_data(response_size)
            if response and len(response) >= response_size:
                # 收到足够数据，立即处理
                break
            time.sleep(0.01)  # 短暂休眠，避免 CPU 占用过高
        else:
            # 超时未收到数据
            print("[LEDStrip] 未收到硬件回显（超时）")
            return False
        
        # 打印接收到的回显数据
        print(f"[LEDStrip] 收到硬件回显: {response.hex(' ').upper()}")
        
        # 只需要检查是否返回固定的应答包: FF FF C8 02 00 35
        expected_response = bytes([0xFF, 0xFF, 0xC8, 0x02, 0x00, 0x35])
        if response == expected_response:
            print(f"[LEDStrip] 硬件回显校验成功")
            return True
        else:
            print(f"[LEDStrip] 硬件回显校验失败: 期望 {expected_response.hex(' ').upper()}, 实际 {response.hex(' ').upper()}")
            return False

    def set_mode_and_color(self, mode: LEDMode, colors: List[Tuple[int, int, int]]) -> bool:
        """
        设置模式和颜色，通过串口发送并校验硬件回显

        Args:
            mode: LED显示模式 (LEDMode枚举)
            colors: 24 个灯的颜色列表，每个为 (R, G, B) 元组

        Returns:
            bool: 发送和校验成功返回 True，失败返回 False
        """
        # 检查串口是否已打开
        if not self._serial_port.is_open():
            print("[LEDStrip] 串口未打开")
            return False
        
        if len(colors) != self.LED_COUNT:
            print(f"[LEDStrip] 颜色数量错误: 期望 {self.LED_COUNT}, 实际 {len(colors)}")
            return False
        
        # 设置模式和颜色
        self._mode = mode
        
        # 律动模式不允许设置灯的颜色，必须使用全零数据
        if self._mode == LEDMode.RHYTHM:
            print("[LEDStrip] 律动模式：颜色数据不起作用，使用全零占位")
            self._led_colors = [(0, 0, 0)] * self.LED_COUNT
        else:
            self._led_colors = colors
        
        # 构建数据包
        packet = self.build_packet()
        
        # 打印发送的指令数据和长度
        print(f"[LEDStrip] 发送指令 ({len(packet)} 字节): {packet.hex(' ').upper()}")

        # 通过串口发送数据
        send_success = self._serial_port.send_led_control_packet(packet)
        if not send_success:
            print("[LEDStrip] 发送数据包失败")
            return False
        
        # 校验硬件回显
        return self._verify_response(self._serial_port)

    def close(self) -> bool:
        """
        关闭所有 LED 灯（设置为全灭状态）

        Returns:
            bool: 发送和校验成功返回 True，失败返回 False
        """
        off_colors = [(0, 0, 0)] * self.LED_COUNT
        return self.set_mode_and_color(LEDMode.CONSTANT, off_colors)
