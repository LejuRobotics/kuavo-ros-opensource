#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.msg import BatteryInfo
from kuavo_msgs.msg import PowerBoardStatus
from kuavo_msgs.srv import GetBatteryInfo, GetBatteryInfoResponse
import os
import sys
try:
    import serial
except ImportError:
    import subprocess
    import sys
    print("正在安装pyserial库...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
import struct
import time
import threading
import contextlib
import fcntl

# 导入电池应答包校验函数（与 battery_query.py 共用）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'controller'))
from hardware.battery_query import _validate_battery_response


@contextlib.contextmanager
def _serial_flock(ser):
    """跨进程串口文件锁。

    led_strip_service 的 SerialPort 后台线程会不停读 /dev/ttyLED0，
    抢走本节点发给电源板的应答字节。flock(LOCK_EX) 让本节点独占串口
    直至读完应答；LED 后台线程用 LOCK_NB 非阻塞抢锁，抢不到就跳过。
    """
    fcntl.flock(ser.fileno(), fcntl.LOCK_EX)
    try:
        yield
    finally:
        fcntl.flock(ser.fileno(), fcntl.LOCK_UN)


class BatteryInfoReader:
    def __init__(self, port='/dev/ttyLED0', baudrate=115200):
        """
        初始化电池信息读取器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2
            )
            rospy.loginfo(f"成功连接到设备: {port}")
        except serial.SerialException as e:
            rospy.logerr(f"无法连接到设备: {e}")
            rospy.logerr("请检查 UDEV 规则以及硬件设备是否正常！！")
            raise

        # 添加锁，防止定时器和服务的串口访问冲突
        self.lock = threading.Lock()

    def calculate_checksum(self, data):
        """
        计算校验和
        :param data: 数据列表
        :return: 校验和
        """
        return (~sum(data)) & 0xFF


    def read_system_status(self):
        """
        读取电源板系统状态（协议 0x01 系统状态读取）
        发送: FF FF 00 02 01 FC
        应答: FF FF 01 08 <7字节数据> <checksum>
        :return: PowerBoardStatus 字典或None
        """
        with self.lock:
            return self._read_system_status_unlocked()

    def _read_system_status_unlocked(self):
        """读取电源板系统状态（无锁版本）"""
        # 构建数据包: FF FF 00 02 01 + checksum
        packet = [0xFF, 0xFF, 0x00, 0x02, 0x01]
        checksum = self.calculate_checksum(packet[2:])
        packet.append(checksum)

        # 跨进程串口锁：发指令+读应答期间独占 /dev/ttyLED0，避免被
        # led_strip_service 的后台读取线程抢走应答字节。
        with _serial_flock(self.ser):
            # 清空接收缓冲区
            self.ser.reset_input_buffer()
            timeout_orig = self.ser.timeout
            self.ser.timeout = 0.05
            while self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            self.ser.timeout = timeout_orig

            # 发送数据
            self.ser.write(bytes(packet))
            rospy.logdebug(f"[系统状态] 发送指令: {[hex(x) for x in packet]}")

            # 等待电源板应答（flock 锁保证串口独占，50ms 足够）
            time.sleep(0.05)

            # 读取应答: FF FF 01 08 <7字节> <checksum> = 12 字节
            response = bytearray()
            start_time = time.time()
            timeout = 1.0  # 加大到 1s
            while time.time() - start_time < timeout:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    response.extend(chunk)
                    # 完整应答 12 字节
                    if len(response) >= 12:
                        if response[0] == 0xFF and response[1] == 0xFF and response[2] == 0x01:
                            break
                        else:
                            response = bytearray()
                time.sleep(0.01)

            rospy.logdebug(f"[系统状态] 最终应答 ({len(response)} 字节): {[hex(x) for x in response]}")

        # 验证（锁已释放）
        if len(response) < 12:
            rospy.logwarn(f"系统状态应答数据不足: {len(response)} < 12")
            return None
        if response[0] != 0xFF or response[1] != 0xFF:
            rospy.logwarn(f"系统状态无效包头: {response[0]:02X} {response[1]:02X}")
            return None
        if response[2] != 0x01:
            rospy.logwarn(f"系统状态指令字节不匹配: 期望01, 实际{response[2]:02X}")
            return None

        # 校验和: Instruction + Length + Param1~7
        data_for_checksum = response[2:2 + 2 + 7]  # 01 08 + 7字节 = 9字节
        expected_checksum = self.calculate_checksum(data_for_checksum)
        actual_checksum = response[11]
        if expected_checksum != actual_checksum:
            rospy.logwarn(f"系统状态校验和错误: 期望{expected_checksum:02X}, 实际{actual_checksum:02X}")
            # 不返回None，仍解析数据（部分电源板校验和可能不一致）

        # 提取7字节数据
        param1 = response[4]  # 中断保护状态和电池通信状态
        param2 = response[5]  # 电池和电源状态
        param3 = response[6]  # 输出控制状态
        param4 = response[7]  # NTC温度
        param5 = response[8]  # 充电电压
        param6 = response[9]  # BAT1电压
        param7 = response[10] # BAT2电压

        status = {
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

        rospy.logdebug(f"系统状态: P1={param1:#04x} P2={param2:#04x} P3={param3:#04x} "
                       f"NTC={param4}℃ 充电V={param5}V BAT1V={param6}V BAT2V={param7}V")
        return status

    def read_battery_info(self, battery_id):
        """
        读取电池信息
        :param battery_id: 电池ID (0: BAT1/右电池, 1: BAT2/左电池)
        :return: 电池信息字典或None
        """
        # 加锁，防止定时器和服务的串口访问冲突
        with self.lock:
            return self._read_battery_info_unlocked(battery_id)

    def _read_battery_info_unlocked(self, battery_id):
        """
        读取电池信息（无锁版本）
        :param battery_id: 电池ID (0: BAT1/右电池, 1: BAT2/左电池)
        :return: 电池信息字典或None
        """
        # 验证 battery_id 范围
        if battery_id not in [0, 1]:
            rospy.logerr(f"无效的电池ID: {battery_id}，仅支持 0(右电池) 或 1(左电池)")
            return None

        # 构建数据包
        # BAT1: FF FF 00 02 03 FA
        # BAT2: FF FF 00 02 04 F9
        if battery_id == 0:
            instruction = 0x03
        else:
            instruction = 0x04

        packet = [0xFF, 0xFF, 0x00, 0x02, instruction]

        # 计算校验和
        checksum = self.calculate_checksum(packet[2:])
        packet.append(checksum)

        # 跨进程串口锁：发指令+读应答期间独占 /dev/ttyLED0，避免被
        # led_strip_service 的后台读取线程抢走应答字节。
        with _serial_flock(self.ser):
            # 清空接收缓冲区 - 使用循环彻底清空
            self.ser.reset_input_buffer()
            # 额外读取所有可能残留的数据
            timeout_orig = self.ser.timeout
            self.ser.timeout = 0.05
            while self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            self.ser.timeout = timeout_orig

            # 发送数据
            self.ser.write(bytes(packet))
            rospy.logdebug(f"发送指令: {[hex(x) for x in packet]}")

            # 等待 RS485 半双工总线上命令回显和硬件应答到达
            # 硬件在 ~5ms 内返回 35 字节应答，等待 50ms 确保数据就绪
            # 修复：不再在等待后排空数据，避免丢弃真实电池应答
            time.sleep(0.05)

            expected_instruction = 0x03 if battery_id == 0 else 0x04
            expected_header = bytes([0xFF, 0xFF, expected_instruction])
            header_len = len(expected_header)
            response = bytearray()
            matched = None
            start_time = time.time()
            timeout = 0.5  # 500ms 超时

            while time.time() - start_time < timeout:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    response.extend(chunk)
                    rospy.logdebug(f"已读取 {len(chunk)} 字节，累计 {len(response)} 字节")

                    if len(response) >= 35:
                        for i in range(len(response) - header_len + 1):
                            if response[i:i + header_len] == expected_header:
                                if len(response) - i >= 35:
                                    candidate = bytes(response[i:i + 35])
                                    if _validate_battery_response(candidate):
                                        rospy.logdebug(f"在位置 {i} 找到匹配的响应头")
                                        matched = candidate
                                        break
                                response = response[i + header_len:]
                                break
                        if matched is not None:
                            break
                    if len(response) > 255:
                        response = response[-128:]
                time.sleep(0.01)

            if matched is None and len(response) >= 35:
                for i in range(len(response) - header_len + 1):
                    if response[i:i + header_len] == expected_header:
                        if len(response) - i >= 35:
                            candidate = bytes(response[i:i + 35])
                            if _validate_battery_response(candidate):
                                rospy.logdebug(f"超时后在位置 {i} 找到匹配的响应头")
                                matched = candidate
                                break

            rospy.logdebug(f"接收到的数据长度: {len(response)}")
            rospy.logdebug(f"接收数据: {[hex(x) for x in response]}")

        if matched is None:
            if len(response) > 0:
                rospy.logdebug(f"接收到不完整数据: {len(response)} 字节，未找到匹配的响应头")
            else:
                rospy.logdebug("未收到任何响应数据")
            return None

        return self._parse_battery_data_from_response(matched, battery_id)

    def _parse_battery_data_from_response(self, response, battery_id):
        """
        从response中解析电池数据，严格按照SDK定义的30字节结构
        数据从索引4开始，跳过包头(FF FF 03 20)

        SDK定义的30字节结构:
        - Param1-2: 总电压（无符号，10mV）
        - Param3-4: 总电流（有符号，10mA）
        - Param5-6: 剩余容量（无符号，10mAh）
        - Param7-8: 充满容量（无符号，10mAh）
        - Param9-10: 放电循环次数（无符号）
        - Param11-12: 剩余容量百分比（无符号，2字节）
        - Param13-14: cell1~16均衡状态
        - Param15-16: cell17~33均衡状态
        - Param17-18: 保护标志
        - Param19-30: NTC温度（6个int16，有符号，°C）
        """
        if len(response) < 35:
            rospy.logwarn(f"数据长度不足: {len(response)} < 35")
            return None

        try:
            # 从索引4开始提取30字节电池数据
            data_start = 4
            data = response[data_start:data_start + 30]

            if len(data) < 30:
                rospy.logwarn(f"数据长度不足: {len(data)} < 30")
                return None

            # Param1-2: 总电压（无符号，10mV）
            voltage = struct.unpack('>H', bytes(data[0:2]))[0] * 10

            # Param3-4: 总电流（有符号，10mA）
            current = struct.unpack('>h', bytes(data[2:4]))[0] * 10

            # Param5-6: 剩余容量（无符号，10mAh）
            remaining_capacity = struct.unpack('>H', bytes(data[4:6]))[0] * 10

            # Param7-8: 充满容量（无符号，10mAh）注意：也需要乘以10
            full_capacity = struct.unpack('>H', bytes(data[6:8]))[0] * 10

            # Param9-10: 放电循环次数（无符号）
            cycle_count = struct.unpack('>H', bytes(data[8:10]))[0]

            # Param11-12: 剩余容量百分比（无符号，注意是2字节！）
            percentage = struct.unpack('>H', bytes(data[10:12]))[0]

            # Param13-14: cell1~16均衡状态（bit0=cell1, bit15=cell16）
            balance1 = struct.unpack('>H', bytes(data[12:14]))[0]

            # Param15-16: cell17~33均衡状态（bit0=cell17, bit16=cell33）
            balance2 = struct.unpack('>H', bytes(data[14:16]))[0]

            # Param17-18: 保护标志（独立字段）
            protection_flags = struct.unpack('>H', bytes(data[16:18]))[0]

            # Param19-30: NTC温度（6个int16，有符号，°C）
            temperatures = []
            for i in range(6):
                temp = struct.unpack('>h', bytes(data[18+i*2:20+i*2]))[0]
                temperatures.append(temp)

            # 构建返回数据
            battery_info = {
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

            rospy.logdebug(f"解析结果: 电压={voltage/1000:.2f}V, 电流={current/1000:.2f}A, "
                         f"剩余={remaining_capacity}mAh, 满充={full_capacity}mAh, "
                         f"百分比={percentage}%, 循环={cycle_count}次, 温度={temperatures}")

            return battery_info

        except Exception as e:
            rospy.logerr(f"解析电池数据失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None

    def _parse_battery_data(self, data, battery_id):
        """
        解析电池数据
        :param data: 原始数据（去掉包头后的数据）
        :param battery_id: 电池ID
        :return: 电池信息字典
        """
        if len(data) < 30:
            rospy.logwarn(f"数据长度不足: {len(data)} < 30")
            return None

        try:
            # 根据协议文档解析数据（大端序）
            # Param1-2: 电压 (uint16, ×10mV)
            voltage = struct.unpack('>H', bytes(data[0:2]))[0] * 10

            # Param3-4: 电流 (int16, ×10mA)
            current = struct.unpack('>h', bytes(data[2:4]))[0] * 10

            # Param5-6: 剩余容量 (uint16, ×10mAh)
            remaining_capacity = struct.unpack('>H', bytes(data[4:6]))[0] * 10

            # Param7-8: 满充容量 (uint16, mAh)
            full_capacity = struct.unpack('>H', bytes(data[6:8]))[0]

            # Param9-10: 循环次数 (uint16)
            cycle_count = struct.unpack('>H', bytes(data[8:10]))[0]

            # Param11: 剩余百分比 (uint8, %)
            percentage = data[10]

            # Param12-18: 均衡状态和保护标志 (7字节)
            protection_flags = 0
            for i in range(11, 18):
                protection_flags = (protection_flags << 8) | data[i]

            # Param19-30: 温度数据 (6个int16)
            temperatures = []
            for i in range(18, 30, 2):
                temp = struct.unpack('>h', bytes(data[i:i+2]))[0]
                temperatures.append(temp)

            # 构建返回数据
            battery_info = {
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

            return battery_info

        except Exception as e:
            rospy.logerr(f"解析电池数据失败: {e}")
            return None

    def close(self):
        """关闭串口连接"""
        self.ser.close()

class BatteryInfoNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('battery_info_node', anonymous=True)

        # 获取参数
        self.port = rospy.get_param('~port', '/dev/ttyLED0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # 发布频率 Hz

        # 创建电池信息读取器实例
        try:
            self.battery_reader = BatteryInfoReader(port=self.port, baudrate=self.baudrate)
        except Exception as e:
            rospy.logerr(f"初始化电池读取器失败: {e}")
            rospy.signal_shutdown("初始化失败")
            return

        # 创建ROS发布者
        self.battery1_pub = rospy.Publisher('battery_info_1', BatteryInfo, queue_size=10)
        self.battery2_pub = rospy.Publisher('battery_info_2', BatteryInfo, queue_size=10)
        # 电源板系统状态发布者（0x01 系统状态读取）
        self.power_board_status_pub = rospy.Publisher('power_board_status', PowerBoardStatus, queue_size=10)

        # 创建电池信息查询服务
        self.get_battery_service = rospy.Service('get_battery_info', GetBatteryInfo, self.handle_get_battery_info)

        # 创建定时器
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

        rospy.loginfo("电池信息节点已启动")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
        rospy.loginfo("话题: /battery_info_1, /battery_info_2, /power_board_status")
        rospy.loginfo("服务: /get_battery_info")

    def timer_callback(self, event):
        """定时器回调函数，读取并发布电池信息与电源板状态"""
        # 读取电源板系统状态（0x01）
        try:
            status = self.battery_reader.read_system_status()
            if status:
                msg = self._create_power_board_status_message(status)
                self.power_board_status_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"读取电源板系统状态失败: {e}")

        # 读取BAT1 (左电池)
        try:
            bat1_info = self.battery_reader.read_battery_info(0)
            if bat1_info:
                msg = self._create_battery_message(bat1_info)
                self.battery1_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"读取BAT1信息失败: {e}")

        # 读取BAT2 (右电池)
        try:
            bat2_info = self.battery_reader.read_battery_info(1)
            if bat2_info:
                msg = self._create_battery_message(bat2_info)
                self.battery2_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"读取BAT2信息失败: {e}")

    def _create_power_board_status_message(self, status):
        """从状态字典创建 PowerBoardStatus 消息"""
        msg = PowerBoardStatus()
        msg.timestamp = rospy.Time.now()
        # 原始字节
        msg.status_byte1 = status['status_byte1']
        msg.status_byte2 = status['status_byte2']
        msg.status_byte3 = status['status_byte3']
        msg.ntc_temperature = status['ntc_temperature']
        msg.charge_voltage = status['charge_voltage']
        msg.bat1_voltage = status['bat1_voltage']
        msg.bat2_voltage = status['bat2_voltage']
        # Param1 拆解
        msg.stop_int = status['stop_int']
        msg.rf_int = status['rf_int']
        msg.board_is_wheel = status['board_is_wheel']
        msg.ideal_diode_fail = status['ideal_diode_fail']
        msg.cur_ov_protection = status['cur_ov_protection']
        msg.bat1_comm_ok = status['bat1_comm_ok']
        msg.bat2_comm_ok = status['bat2_comm_ok']
        # Param2 拆解
        msg.bat1_exists = status['bat1_exists']
        msg.bat2_exists = status['bat2_exists']
        msg.bat1_low_power = status['bat1_low_power']
        msg.bat2_low_power = status['bat2_low_power']
        msg.charging = status['charging']
        msg.fail_12v = status['fail_12v']
        msg.fail_19v = status['fail_19v']
        msg.fail_24v = status['fail_24v']
        # Param3 拆解
        msg.arm_en = status['arm_en']
        msg.leg_en = status['leg_en']
        msg.out_19v_en = status['out_19v_en']
        msg.out1_12v_en = status['out1_12v_en']
        msg.out2_12v_en = status['out2_12v_en']
        msg.out3_12v_en = status['out3_12v_en']
        msg.out_24v_en = status['out_24v_en']
        return msg

    def _create_battery_message(self, battery_info):
        """创建ROS消息"""
        msg = BatteryInfo()
        msg.timestamp = rospy.Time.now()

        msg.battery_id = battery_info['battery_id']
        msg.voltage = battery_info['voltage']
        msg.current = battery_info['current']
        msg.remaining_capacity = battery_info['remaining_capacity']
        msg.full_capacity = battery_info['full_capacity']
        msg.percentage = battery_info['percentage']
        msg.cycle_count = battery_info['cycle_count']
        msg.protection_flags = battery_info['protection_flags']
        msg.temperatures = battery_info['temperatures']

        return msg

    def handle_get_battery_info(self, req):
        """
        处理电池信息查询服务请求
        :param req: 服务请求，包含battery_id字段
        :return: GetBatteryInfoResponse
        """
        try:
            # 读取指定电池的信息
            battery_info = self.battery_reader.read_battery_info(req.battery_id)

            if battery_info is None:
                # 读取失败
                response = GetBatteryInfoResponse()
                response.success = False
                response.message = f"Failed to read battery {req.battery_id} info"
                return response

            # 构建响应 - 直接使用请求中的 battery_id，确保返回正确的电池ID
            response = GetBatteryInfoResponse()
            response.battery_id = req.battery_id
            response.voltage = battery_info['voltage']
            response.current = battery_info['current']
            response.remaining_capacity = battery_info['remaining_capacity']
            response.full_capacity = battery_info['full_capacity']
            response.percentage = battery_info['percentage']
            response.cycle_count = battery_info['cycle_count']
            response.protection_flags = battery_info['protection_flags']
            response.temperatures = battery_info['temperatures']
            response.success = True
            response.message = f"Battery {req.battery_id} info read successfully"

            return response

        except Exception as e:
            rospy.logerr(f"处理电池信息查询服务失败: {e}")
            response = GetBatteryInfoResponse()
            response.success = False
            response.message = f"Service error: {str(e)}"
            return response

if __name__ == '__main__':
    try:
        node = BatteryInfoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals() and hasattr(node, 'battery_reader'):
            node.battery_reader.close()
