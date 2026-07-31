#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.msg import BatteryInfo
from kuavo_msgs.msg import PowerBoardStatus
from kuavo_msgs.srv import GetBatteryInfo, GetBatteryInfoResponse
from kuavo_msgs.srv import GetPowerBoardStatus
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
import time
import threading# 导入电池/电源板应答包校验与解析函数（与 battery_query.py 共用，避免重复实现）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'controller'))
from hardware.battery_query import (
    _validate_battery_response,
    _search_response,
    _parse_response,
    _parse_system_status_response,
    _validate_system_status_response,
)


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

        # 等待电源板应答（50ms 足够）
        time.sleep(0.05)

        # 读取应答: FF FF 01 08 <7字节> <checksum> = 12 字节
        response = bytearray()
        matched = None
        start_time = time.time()
        timeout = 1.0  # 加大到 1s
        expected_header = bytes([0xFF, 0xFF, 0x01])
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                response.extend(chunk)
                matched, response = _search_response(
                    response, expected_header, 12, _validate_system_status_response)
                if matched is not None:
                    break
            time.sleep(0.01)

        rospy.logdebug(f"[系统状态] 最终应答 ({len(response)} 字节): {[hex(x) for x in response]}")

        if matched is None:
            rospy.logwarn(f"系统状态应答数据不足或未匹配: {len(response)} 字节")
            return None

        # 校验和：仅警告不拒绝（部分电源板系统状态校验和可能不一致，仍解析数据）
        data_for_checksum = matched[2:2 + 2 + 7]  # 01 08 + 7字节 = 9字节
        expected_checksum = self.calculate_checksum(data_for_checksum)
        actual_checksum = matched[11]
        if expected_checksum != actual_checksum:
            rospy.logwarn(f"系统状态校验和错误: 期望{expected_checksum:02X}, 实际{actual_checksum:02X}")

        return _parse_system_status_response(matched)

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

        instruction = 0x03 if battery_id == 0 else 0x04
        packet = bytes([0xFF, 0xFF, 0x00, 0x02, instruction,
                        self.calculate_checksum([0x00, 0x02, instruction])])

        # 清空接收缓冲区
        self.ser.reset_input_buffer()
        timeout_orig = self.ser.timeout
        self.ser.timeout = 0.05
        while self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        self.ser.timeout = timeout_orig

        # 发送数据
        self.ser.write(packet)
        rospy.logdebug(f"发送指令: {[hex(x) for x in packet]}")

        # 等待 RS485 半双工总线命令回显和硬件应答到达（~5ms 返回，等 50ms 确保就绪）
        time.sleep(0.05)

        expected_header = bytes([0xFF, 0xFF, instruction])
        response = bytearray()
        matched = None
        start_time = time.time()
        timeout = 0.5  # 500ms 超时

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                response.extend(chunk)
                rospy.logdebug(f"已读取 {len(chunk)} 字节，累计 {len(response)} 字节")

                matched, response = _search_response(
                    response, expected_header, 35, _validate_battery_response)
                if matched is not None:
                    rospy.logdebug("找到匹配的响应头")
                    break
                # 防止缓冲无限增长
                if len(response) > 255:
                    response = response[-128:]
            time.sleep(0.01)

        rospy.logdebug(f"接收到的数据长度: {len(response)}")
        rospy.logdebug(f"接收数据: {[hex(x) for x in response]}")

        if matched is None:
            if len(response) > 0:
                rospy.logdebug(f"接收到不完整数据: {len(response)} 字节，未找到匹配的响应头")
            else:
                rospy.logdebug("未收到任何响应数据")
            return None

        return _parse_response(matched, battery_id)

    def close(self):
        """关闭串口连接"""
        self.ser.close()

class BatteryInfoReaderProxy:
    """
    电池/电源板信息读取器（ROS Service 代理模式）

    不直接打开串口，而是通过调用 LED 服务节点的内部 service 获取数据：
      - _query_battery_hw:        电池单体信息（0x03/0x04）
      - _query_power_board_status: 电源板系统状态（0x01）

    适用于 LED 和电池同时启用、串口已由 LED 节点独占的场景，
    避免 battery_info_node 直开 /dev/ttyLED0 与 LED 后台读取线程冲突。

    Interface 与 BatteryInfoReader 一致：
      - read_battery_info(battery_id) -> dict | None
      - read_system_status()         -> dict | None
    """

    def __init__(self,
                 battery_service='_query_battery_hw',
                 power_board_service='_query_power_board_status'):
        """初始化 ROS Service 代理"""
        try:
            rospy.loginfo(f"等待电池查询服务: {battery_service} ...")
            rospy.wait_for_service(battery_service, timeout=10.0)
            self._battery_proxy = rospy.ServiceProxy(battery_service, GetBatteryInfo)
            rospy.loginfo(f"已连接到电池查询服务: {battery_service}")
        except rospy.ROSException as e:
            rospy.logerr(f"等待电池查询服务超时: {battery_service} ({e})")
            raise

        try:
            rospy.loginfo(f"等待电源板状态服务: {power_board_service} ...")
            rospy.wait_for_service(power_board_service, timeout=10.0)
            self._power_board_proxy = rospy.ServiceProxy(power_board_service, GetPowerBoardStatus)
            rospy.loginfo(f"已连接到电源板状态服务: {power_board_service}")
        except rospy.ROSException as e:
            rospy.logerr(f"等待电源板状态服务超时: {power_board_service} ({e})")
            raise

    def read_battery_info(self, battery_id):
        """通过 ROS service 读取电池信息（与 BatteryInfoReader 接口一致）"""
        try:
            resp = self._battery_proxy(battery_id=battery_id)
            if resp.success:
                return {
                    'battery_id': battery_id,
                    'voltage': resp.voltage,
                    'current': resp.current,
                    'remaining_capacity': resp.remaining_capacity,
                    'full_capacity': resp.full_capacity,
                    'percentage': resp.percentage,
                    'cycle_count': resp.cycle_count,
                    'protection_flags': resp.protection_flags,
                    'temperatures': list(resp.temperatures)
                }
            else:
                rospy.logwarn(f"电池 {battery_id} 查询失败: {resp.message}")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"调用电池查询服务失败: {e}")
            return None

    def read_system_status(self):
        """通过 ROS service 读取电源板系统状态（与 BatteryInfoReader 接口一致）

        返回字典的 key 与 BatteryInfoReader.read_system_status 完全一致，
        供 _create_power_board_status_message 直接使用。
        """
        try:
            resp = self._power_board_proxy()
            if resp.success:
                # 从 PowerBoardStatus 消息重新组装为字典，与 serial 模式返回结构对齐
                m = resp.status
                return {
                    # 原始字节
                    'status_byte1': m.status_byte1,
                    'status_byte2': m.status_byte2,
                    'status_byte3': m.status_byte3,
                    'ntc_temperature': m.ntc_temperature,
                    'charge_voltage': m.charge_voltage,
                    'bat1_voltage': m.bat1_voltage,
                    'bat2_voltage': m.bat2_voltage,
                    # Param1 拆解
                    'stop_int':          m.stop_int,
                    'rf_int':            m.rf_int,
                    'board_is_wheel':    m.board_is_wheel,
                    'ideal_diode_fail':  m.ideal_diode_fail,
                    'cur_ov_protection': m.cur_ov_protection,
                    'bat1_comm_ok':      m.bat1_comm_ok,
                    'bat2_comm_ok':      m.bat2_comm_ok,
                    # Param2 拆解
                    'bat1_exists':       m.bat1_exists,
                    'bat2_exists':       m.bat2_exists,
                    'bat1_low_power':    m.bat1_low_power,
                    'bat2_low_power':    m.bat2_low_power,
                    'charging':          m.charging,
                    'fail_12v':          m.fail_12v,
                    'fail_19v':          m.fail_19v,
                    'fail_24v':          m.fail_24v,
                    # Param3 拆解
                    'arm_en':            m.arm_en,
                    'leg_en':            m.leg_en,
                    'out_19v_en':        m.out_19v_en,
                    'out1_12v_en':       m.out1_12v_en,
                    'out2_12v_en':       m.out2_12v_en,
                    'out3_12v_en':       m.out3_12v_en,
                    'out_24v_en':        m.out_24v_en,
                }
            else:
                rospy.logwarn(f"电源板系统状态查询失败: {resp.message}")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"调用电源板状态服务失败: {e}")
            return None

    def close(self):
        """清理代理资源"""
        self._battery_proxy = None
        self._power_board_proxy = None

class BatteryInfoNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('battery_info_node', anonymous=True)

        # 获取参数
        self.port = rospy.get_param('~port', '/dev/ttyLED0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # 发布频率 Hz
        # backend: "serial" = 直接打开串口; "ros_service" = 通过 LED 节点的
        # _query_battery_hw / _query_power_board_status 内部服务获取数据，
        # 避免与 LED 节点争抢 /dev/ttyLED0（LED 启用时自动由 launch 切到本模式）。
        self.backend = rospy.get_param('~backend', 'serial')

        # 创建电池信息读取器实例
        try:
            if self.backend == 'ros_service':
                self.battery_reader = BatteryInfoReaderProxy()
                rospy.loginfo("电池信息读取模式: ROS Service 代理（与 LED 共用串口，不直开 /dev/ttyLED0）")
            else:
                self.battery_reader = BatteryInfoReader(port=self.port, baudrate=self.baudrate)
                rospy.loginfo("电池信息读取模式: 直接串口")
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
