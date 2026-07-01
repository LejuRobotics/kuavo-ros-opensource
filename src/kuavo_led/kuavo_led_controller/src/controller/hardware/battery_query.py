#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电池查询共享模块

通过 SerialPort 单例发送电池查询指令并从共享缓冲区读取响应，
供 LED 服务节点内部使用。调用方需确保互斥访问（如 threading.Lock）。
"""

import struct
import time
import threading


def query_battery(serial_port, battery_id, timeout=0.2):
    """
    通过共享串口查询电池信息

    发送 6 字节查询指令，从共享缓冲区逐字节搜索匹配的 35 字节应答包。

    Args:
        serial_port: SerialPort 单例实例
        battery_id: 电池ID (0: BAT1, 1: BAT2)
        timeout: 超时时间（秒）

    Returns:
        dict: 电池信息字典，失败返回 None
    """
    if battery_id not in [0, 1]:
        print(f"[BatteryQuery] 无效的电池ID: {battery_id}")
        return None

    instruction = 0x03 if battery_id == 0 else 0x04
    packet = bytes([0xFF, 0xFF, 0x00, 0x02, instruction])
    checksum = (~sum(packet[2:])) & 0xFF
    packet += bytes([checksum])

    if not serial_port.send_data(packet):
        print(f"[BatteryQuery] 发送电池 {battery_id} 查询指令失败")
        return None

    # 等待命令回显到达后立即排空 _read_buffer
    # 避免回显字节和硬件 UART FIFO 中可能残留的旧数据混入收包窗口
    time.sleep(0.01)
    serial_port.read_all()

    expected_header = bytes([0xFF, 0xFF, instruction])
    header_len = len(expected_header)
    accumulated = bytearray()
    start_time = time.time()

    while time.time() - start_time < timeout:
        chunk = serial_port.read_data(64)
        if chunk:
            accumulated.extend(chunk)
            if len(accumulated) >= 35:
                for i in range(len(accumulated) - header_len + 1):
                    if accumulated[i:i + header_len] == expected_header:
                        if len(accumulated) - i >= 35:
                            response = bytes(accumulated[i:i + 35])
                            if _validate_battery_response(response):
                                return _parse_response(response, battery_id)
                            # 假阳性匹配：残留数据中恰好出现了 FF FF 03/04 序列
                            # 但长度字段或校验和不通过，说明不是真正的电池应答包
                            # 跳过这个假匹配的头部，继续向后搜索
                            accumulated = accumulated[i + header_len:]
                            break  # 跳出 for，回到 while 继续收包
        else:
            time.sleep(0.005)

    # 超时前最后一次扫描，不再等待新数据
    if len(accumulated) >= 35:
        for i in range(len(accumulated) - header_len + 1):
            if accumulated[i:i + header_len] == expected_header:
                if len(accumulated) - i >= 35:
                    response = bytes(accumulated[i:i + 35])
                    if _validate_battery_response(response):
                        return _parse_response(response, battery_id)

    print(f"[BatteryQuery] 电池 {battery_id} 查询超时")
    return None


def _validate_battery_response(response):
    """二次验证电池应答包，防止误匹配 LED 残留数据中的假阳性 FF FF 03/04 序列

    验证两层：
    1. 长度字段（response[3]）：预期 0x1E（30 字节数据载荷）
    2. 校验和：response[2:35] 累加和的低 8 位应为 0xFF
       （与命令包相同的算法：checksum = (~sum(payload)) & 0xFF）
    """
    data_len = response[3]
    if data_len < 0x1C or data_len > 0x24:  # 允许 ±4 容差
        return False
    if (sum(response[2:35]) & 0xFF) != 0xFF:
        return False
    return True


def _parse_response(response, battery_id):
    """
    解析 35 字节电池应答包

    数据结构（索引4-33，30字节）:
        [0:2]   总电压（uint16, x10mV）    [8:10]  循环次数
        [2:4]   总电流（int16,  x10mA）    [10:12] 剩余百分比
        [4:6]   剩余容量（uint16, x10mAh） [12:14] 均衡状态1
        [6:8]   充满容量（uint16, x10mAh） [14:16] 均衡状态2
                                            [16:18] 保护标志
                                            [18:30] 6路温度（int16, degC）
    """
    try:
        data = response[4:34]
        if len(data) < 30:
            return None

        voltage = struct.unpack('>H', bytes(data[0:2]))[0] * 10
        current = struct.unpack('>h', bytes(data[2:4]))[0] * 10
        remaining_capacity = struct.unpack('>H', bytes(data[4:6]))[0] * 10
        full_capacity = struct.unpack('>H', bytes(data[6:8]))[0] * 10
        cycle_count = struct.unpack('>H', bytes(data[8:10]))[0]
        percentage = struct.unpack('>H', bytes(data[10:12]))[0]
        protection_flags = struct.unpack('>H', bytes(data[16:18]))[0]

        temperatures = []
        for i in range(6):
            temp = struct.unpack('>h', bytes(data[18 + i * 2:20 + i * 2]))[0]
            temperatures.append(temp)

        return {
            'battery_id': battery_id,
            'voltage': voltage,
            'current': current,
            'remaining_capacity': remaining_capacity,
            'full_capacity': full_capacity,
            'percentage': percentage,
            'cycle_count': cycle_count,
            'protection_flags': protection_flags,
            'temperatures': temperatures
        }
    except Exception as e:
        print(f"[BatteryQuery] 解析失败: {e}")
        return None


class BatteryQueryCache:
    """
    后台电池查询缓存

    在独立线程中通过 try_lock 方式查询电池硬件，缓存结果。
    handle_query_battery_hw 直接返回缓存，不获取 _op_lock，
    确保 LED 控制服务（音频呼吸灯等）永不被阻塞。

    用法:
        cache = BatteryQueryCache(serial_port, op_lock)
        info, age = cache.get(battery_id)  # 瞬时返回，(None, -1) 表示尚未缓存
        cache.stop()
    """

    def __init__(self, serial_port, op_lock, query_interval=0.5):
        """
        Args:
            serial_port: SerialPort 单例
            op_lock:     与 LED 操作共享的 threading.Lock
            query_interval: 两次完整查询周期之间的间隔（秒）
        """
        self._serial_port = serial_port
        self._op_lock = op_lock
        self._query_interval = query_interval

        self._cache = {0: None, 1: None}
        self._cache_timestamps = {0: 0.0, 1: 0.0}  # 最后成功查询的时间戳
        self._cache_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        """后台查询循环：每次只查一个电池，交替进行，降低锁占用"""
        current_batt = 0
        while not self._stop_event.is_set():
            if self._op_lock.acquire(blocking=False):
                try:
                    self._serial_port.clear_buffer()
                    info = query_battery(self._serial_port, current_batt)
                    if info is not None:
                        with self._cache_lock:
                            self._cache[current_batt] = info
                            self._cache_timestamps[current_batt] = time.time()
                except Exception:
                    pass
                finally:
                    self._op_lock.release()
                current_batt = 1 - current_batt  # 交替查询 BAT1/BAT2
            # 无论是否获得锁，都等相同间隔
            self._stop_event.wait(self._query_interval)

    def get(self, battery_id):
        """获取缓存的电池信息，瞬时返回 (info, age_seconds)
        info 为 None 且 age 为 -1 表示尚未缓存过
        """
        with self._cache_lock:
            info = self._cache.get(battery_id)
            if info is not None:
                age = time.time() - self._cache_timestamps.get(battery_id, 0)
                return info, age
            if self._cache_timestamps.get(battery_id, 0) > 0:
                # 曾成功但当前缓存为空（try_lock 未拿到锁导致未更新）
                age = time.time() - self._cache_timestamps[battery_id]
                return None, age
            return None, -1.0

    def get_age(self, battery_id):
        """获取缓存数据的新鲜度（秒），-1 表示从未缓存过"""
        with self._cache_lock:
            if self._cache_timestamps.get(battery_id, 0) == 0:
                return -1.0
            return time.time() - self._cache_timestamps[battery_id]

    def stop(self):
        """停止后台查询线程"""
        self._stop_event.set()
