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


# ==================== 协议常量 ====================
_HEADER = bytes([0xFF, 0xFF])  # 协议包头

# 电源板系统状态查询指令（0x01），内容不变，预计算
_POWER_BOARD_QUERY_PACKET = bytes([0xFF, 0xFF, 0x00, 0x02, 0x01, 0xFC])

# BatteryQueryCache 后台轮询序列：电池0→系统状态→电池1→系统状态
# 每项只查一项，持锁时间短，确保 LED 控制服务能及时抢到 _op_lock。
_SYS = 'sys'  # 系统状态查询哨兵，区别于 int 型 battery_id


def _calc_checksum(data):
    """协议校验和：(~sum(data)) & 0xFF"""
    return (~sum(data)) & 0xFF


def _build_query_packet(instruction):
    """构建 6 字节查询指令包: FF FF 00 02 <instruction> <checksum>"""
    body = bytes([0x00, 0x02, instruction])
    return _HEADER + body + bytes([_calc_checksum(body)])


def _search_response(accumulated, expected_header, response_len, validate_fn):
    """在累计数据中搜索应答包。

    返回 (matched_bytes_or_None, new_accumulated)：
    - 找到合法包：matched 为该包字节，new_accumulated 为其后的剩余数据。
    - 找到头但应答未到齐：返回 (None, accumulated) 原样，等下次数据到达再匹配（不丢头）。
    - 假阳性匹配：跳过该头，从其后继续搜索；new_accumulated 反映已跳过的部分。
    用 bytearray.find 做 C 级字节搜索，避免逐字节 Python 切片比较。
    """
    header_len = len(expected_header)
    if len(accumulated) < response_len:
        return None, accumulated
    i = accumulated.find(expected_header)
    while i != -1:
        if len(accumulated) - i < response_len:
            # 头找到但应答未到齐，保留 accumulated 等更多数据
            return None, accumulated
        candidate = bytes(accumulated[i:i + response_len])
        if validate_fn(candidate):
            return candidate, accumulated[i + response_len:]
        # 假阳性，从该头之后继续找
        i = accumulated.find(expected_header, i + header_len)
    return None, accumulated


def _query_and_parse(serial_port, packet, expected_header, response_len,
                     validate_fn, parse_fn, timeout):
    """共享的"发指令→收应答→搜索→校验→解析"骨架。

    供 query_battery / query_power_board_status 复用，避免重复的收包/搜索循环。
    """
    if not serial_port.send_data(packet):
        return None

    # 等待 RS485 半双工总线命令回显和硬件应答到达（硬件 ~5ms 返回，等 50ms 确保就绪）
    time.sleep(0.05)

    accumulated = bytearray(serial_port.read_all() or b'')

    # 先在已读取数据中搜索
    matched, accumulated = _search_response(
        accumulated, expected_header, response_len, validate_fn)
    if matched is not None:
        return parse_fn(matched)

    # 首轮未命中，继续收包
    start_time = time.time()
    while time.time() - start_time < timeout:
        chunk = serial_port.read_data(64)
        if chunk:
            accumulated.extend(chunk)
            matched, accumulated = _search_response(
                accumulated, expected_header, response_len, validate_fn)
            if matched is not None:
                return parse_fn(matched)
            # 防止缓冲无限增长
            if len(accumulated) > 255:
                accumulated = accumulated[-128:]
        else:
            time.sleep(0.005)

    return None


def query_battery(serial_port, battery_id, timeout=0.2):
    """通过共享串口查询电池信息（0x03 BAT1 / 0x04 BAT2）。

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
    return _query_and_parse(
        serial_port,
        _build_query_packet(instruction),
        _HEADER + bytes([instruction]),
        35,
        _validate_battery_response,
        lambda resp: _parse_response(resp, battery_id),
        timeout)


def _validate_battery_response(response):
    """二次验证电池应答包，防止误匹配 LED 残留数据中的假阳性 FF FF 03/04 序列

    验证两层：
    1. 长度字段（response[3]）：预期 0x1E（30 字节数据载荷），允许 ±4 容差
    2. 校验和：response[2:35] 累加和的低 8 位应为 0xFF
    """
    data_len = response[3]
    if data_len < 0x1C or data_len > 0x24:
        return False
    return (sum(response[2:35]) & 0xFF) == 0xFF


def _parse_response(response, battery_id):
    """解析 35 字节电池应答包（索引4-33，30字节）"""
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
        temperatures = [struct.unpack('>h', bytes(data[18 + i * 2:20 + i * 2]))[0]
                        for i in range(6)]

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


def query_power_board_status(serial_port, timeout=0.2):
    """通过共享串口查询电源板系统状态（协议 0x01）。

    发送 6 字节查询指令 FF FF 00 02 01 FC，搜索 12 字节应答包
    FF FF 01 08 <7字节数据> <checksum>。

    Args:
        serial_port: SerialPort 单例实例
        timeout: 超时时间（秒）。12 字节应答 ~5ms 内到达，0.2s 足够且
                 限定失败时占锁时长，避免阻塞 LED 控制服务。

    Returns:
        dict: 系统状态字典（与 BatteryInfoReader.read_system_status 返回结构一致），
              失败返回 None
    """
    return _query_and_parse(
        serial_port,
        _POWER_BOARD_QUERY_PACKET,
        _HEADER + bytes([0x01]),
        12,
        _validate_system_status_response,
        _parse_system_status_response,
        timeout)


def _validate_system_status_response(response):
    """校验系统状态应答包：length 字段 response[3] == 0x08。

    仅靠包头 + length 即可排除 LED 残留数据假阳性（LED 包为 FF FF 00 / FF FF C8，
    不会出现 FF FF 01 08）。校验和不作拒绝条件——与 serial 模式一致，
    部分电源板系统状态校验和可能不一致，仍解析数据。
    """
    return len(response) >= 12 and response[3] == 0x08


def _parse_system_status_response(response):
    """解析 12 字节系统状态应答包。

    应答结构: FF FF 01 08 <P1~P7> <checksum>（索引 4-10 为 7 字节数据）。
    返回字典的 key 与 BatteryInfoReader.read_system_status 一致。
    """
    try:
        if len(response) < 12:
            return None
        param1 = response[4]   # 中断保护状态和电池通信状态
        param2 = response[5]   # 电池和电源状态
        param3 = response[6]   # 输出控制状态
        param4 = response[7]   # NTC温度
        param5 = response[8]   # 充电电压
        param6 = response[9]   # BAT1电压
        param7 = response[10]  # BAT2电压

        return {
            # 原始字节
            'status_byte1': param1,
            'status_byte2': param2,
            'status_byte3': param3,
            'ntc_temperature': param4,
            'charge_voltage': param5,
            'bat1_voltage': param6,
            'bat2_voltage': param7,
            # Param1 拆解
            'stop_int':          bool((param1 >> 0) & 0x01),
            'rf_int':            bool((param1 >> 1) & 0x01),
            'board_is_wheel':    bool((param1 >> 2) & 0x01),
            'ideal_diode_fail':  bool((param1 >> 4) & 0x01),
            'cur_ov_protection': bool((param1 >> 5) & 0x01),
            'bat1_comm_ok':      bool((param1 >> 6) & 0x01),
            'bat2_comm_ok':      bool((param1 >> 7) & 0x01),
            # Param2 拆解
            'bat1_exists':       bool((param2 >> 0) & 0x01),
            'bat2_exists':       bool((param2 >> 1) & 0x01),
            'bat1_low_power':    bool((param2 >> 2) & 0x01),
            'bat2_low_power':    bool((param2 >> 3) & 0x01),
            'charging':          bool((param2 >> 4) & 0x01),
            'fail_12v':          bool((param2 >> 5) & 0x01),
            'fail_19v':          bool((param2 >> 6) & 0x01),
            'fail_24v':          bool((param2 >> 7) & 0x01),
            # Param3 拆解
            'arm_en':            bool((param3 >> 1) & 0x01),
            'leg_en':            bool((param3 >> 2) & 0x01),
            'out_19v_en':        bool((param3 >> 3) & 0x01),
            'out1_12v_en':       bool((param3 >> 4) & 0x01),
            'out2_12v_en':       bool((param3 >> 5) & 0x01),
            'out3_12v_en':       bool((param3 >> 6) & 0x01),
            'out_24v_en':        bool((param3 >> 7) & 0x01),
        }
    except Exception as e:
        print(f"[PowerBoardQuery] 解析失败: {e}")
        return None


class BatteryQueryCache:
    """
    后台电池查询缓存

    在独立线程中通过 try_lock 方式查询电池硬件，缓存结果。
    handle_query_battery_hw / handle_query_power_board_status 直接返回缓存，
    不获取 _op_lock，确保 LED 控制服务永不被阻塞。

    缓存两类数据：
      - 电池单体信息（0x03/0x04）：get(battery_id) -> (info|None, age)
      - 电源板系统状态（0x01）：get_system_status() -> (status|None, age)

    用法:
        cache = BatteryQueryCache(serial_port, op_lock)
        info, age = cache.get(battery_id)          # 瞬时返回，(None, -1) 表示尚未缓存
        status, age = cache.get_system_status()    # 瞬时返回
        cache.stop()
    """

    def __init__(self, serial_port, op_lock, query_interval=0.5):
        """
        Args:
            serial_port: SerialPort 单例
            op_lock:     与 LED 操作共享的 threading.Lock
            query_interval: 两次查询之间的间隔（秒）
        """
        self._serial_port = serial_port
        self._op_lock = op_lock
        self._query_interval = query_interval

        self._cache = {0: None, 1: None}
        self._cache_timestamps = {0: 0.0, 1: 0.0}
        self._system_status_cache = None
        self._system_status_timestamp = 0.0
        self._cache_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        """后台查询循环：轮流查询 电池0→系统状态→电池1→系统状态"""
        sequence = [0, _SYS, 1, _SYS]
        seq_idx = 0
        while not self._stop_event.is_set():
            target = sequence[seq_idx]
            if self._op_lock.acquire(blocking=False):
                try:
                    self._serial_port.clear_buffer()
                    if target is _SYS:
                        self._refresh_system_status()
                    else:
                        self._refresh_battery(target)
                except Exception:
                    pass
                finally:
                    self._op_lock.release()
            seq_idx = (seq_idx + 1) % len(sequence)
            self._stop_event.wait(self._query_interval)

    def _refresh_battery(self, battery_id):
        info = query_battery(self._serial_port, battery_id)
        if info is not None:
            with self._cache_lock:
                self._cache[battery_id] = info
                self._cache_timestamps[battery_id] = time.time()

    def _refresh_system_status(self):
        status = query_power_board_status(self._serial_port)
        if status is not None:
            with self._cache_lock:
                self._system_status_cache = status
                self._system_status_timestamp = time.time()

    def get(self, battery_id):
        """获取缓存的电池信息，瞬时返回 (info, age_seconds)
        info 为 None 且 age 为 -1 表示尚未缓存过
        """
        return self._get_cached(self._cache.get(battery_id),
                                self._cache_timestamps.get(battery_id, 0.0))

    def get_system_status(self):
        """获取缓存的电源板系统状态，瞬时返回 (status, age_seconds)
        status 为 None 且 age 为 -1 表示尚未缓存过
        """
        return self._get_cached(self._system_status_cache,
                                self._system_status_timestamp)

    @staticmethod
    def _get_cached(value, timestamp):
        """统一的缓存读取：返回 (value_or_None, age_seconds)
        value 非 None 返回 (value, age)；缓存过但当前为 None 返回 (None, age)；
        从未缓存（timestamp==0）返回 (None, -1.0)。
        """
        if value is not None:
            return value, time.time() - timestamp
        if timestamp > 0:
            return None, time.time() - timestamp
        return None, -1.0

    def stop(self):
        """停止后台查询线程"""
        self._stop_event.set()
