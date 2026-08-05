#!/usr/bin/env python3

import math
import threading

import minimalmodbus
import rospy
import serial
from kuavo_msgs.srv import RelaySelectChannel, RelaySelectChannelResponse
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse


MODBUS_RESPONSE_LENGTH = 8


def calc_modbus_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class RelayController:
    def __init__(self, port, timeout):
        self.port = port
        self.timeout = timeout
        self._lock = threading.Lock()
        self._serial = None

    def _connect(self):
        if self._serial and self._serial.is_open:
            return True

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            rospy.loginfo("Relay serial connected: %s", self.port)
            return True
        except Exception as exc:
            rospy.logerr("Failed to connect relay serial %s: %s", self.port, exc)
            self._serial = None
            return False

    def _build_command(self, channel):
        relay_bits = {
            None: 0x00,
            1: 0x01,
            2: 0x02,
        }
        if channel not in relay_bits:
            raise ValueError("channel must be 1, 2 or None")

        payload = bytes(
            [
                0xFE,
                0x0F,
                0x00,
                0x00,
                0x00,
                0x02,
                0x01,
                relay_bits[channel],
            ]
        )
        return payload + calc_modbus_crc(payload)

    def _send_command(self, channel, action_text):
        with self._lock:
            if not self._connect():
                return False, "relay serial unavailable"

            try:
                command = self._build_command(channel)
                self._serial.reset_input_buffer()
                self._serial.write(command)
                self._serial.flush()
                response = self._serial.read(MODBUS_RESPONSE_LENGTH)
                if len(response) != MODBUS_RESPONSE_LENGTH:
                    return False, "relay response timeout"
                return True, response.hex().upper()
            except Exception as exc:
                rospy.logerr("%s failed: %s", action_text, exc)
                return False, str(exc)

    def select_channel(self, channel):
        return self._send_command(channel, "select_channel")

    def all_off(self):
        return self._send_command(None, "all_off")

    def close(self):
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                rospy.loginfo("Relay serial closed: %s", self.port)


class PressureReader:
    def __init__(self, port, timeout):
        self.port = port
        self.timeout = timeout
        self._lock = threading.Lock()

    def read_kpa(self, sensor_id):
        slave_address = 1 if sensor_id == 1 else 2

        with self._lock:
            try:
                instrument = minimalmodbus.Instrument(self.port, slaveaddress=slave_address)
                instrument.serial.baudrate = 19200
                instrument.serial.bytesize = 8
                instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
                instrument.serial.stopbits = 2
                instrument.serial.timeout = self.timeout
                instrument.mode = minimalmodbus.MODE_RTU
                instrument.close_port_after_each_call = True

                raw_value = instrument.read_register(0x0001, number_of_decimals=0, signed=True)
                return raw_value * 0.1
            except minimalmodbus.NoResponseError as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Pressure sensor %s (slave %s) no response on %s: %s",
                    sensor_id,
                    slave_address,
                    self.port,
                    exc,
                )
                return None
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Pressure sensor %s (slave %s) read failed on %s: %s",
                    sensor_id,
                    slave_address,
                    self.port,
                    exc,
                )
                return None


class ModbusIoBridgeNode:
    def __init__(self):
        rospy.init_node("modbus_io_bridge_node", anonymous=False)

        relay_port = rospy.get_param("~relay_port", "/dev/kuavo_relay")
        relay_timeout = rospy.get_param("~relay_timeout", 0.5)
        pressure_port = rospy.get_param("~pressure_port", "/dev/kuavo_pressure")
        pressure_timeout = rospy.get_param("~pressure_timeout", 0.5)
        publish_rate = rospy.get_param("~pressure_publish_rate", 1.0)

        self.relay_controller = RelayController(relay_port, relay_timeout)
        self.pressure_reader = PressureReader(pressure_port, pressure_timeout)

        self.left_pressure_pub = rospy.Publisher("/pressure/left_kpa", Float32, queue_size=10)
        self.right_pressure_pub = rospy.Publisher("/pressure/right_kpa", Float32, queue_size=10)

        self.select_channel_srv = rospy.Service(
            "/relay/select_channel", RelaySelectChannel, self.handle_select_channel
        )
        self.channel_1_on_srv = rospy.Service("/relay/channel_1_on", Trigger, self.handle_channel_1_on)
        self.channel_2_on_srv = rospy.Service("/relay/channel_2_on", Trigger, self.handle_channel_2_on)
        self.all_off_srv = rospy.Service("/relay/all_off", Trigger, self.handle_all_off)

        timer_period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        self.pressure_timer = rospy.Timer(rospy.Duration(timer_period), self.publish_pressure)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Modbus IO bridge started")
        rospy.loginfo(
            "Relay services: /relay/select_channel, /relay/channel_1_on, /relay/channel_2_on, /relay/all_off"
        )
        rospy.loginfo("Pressure topics: /pressure/left_kpa, /pressure/right_kpa")

    def handle_select_channel(self, req):
        response = RelaySelectChannelResponse()

        if req.channel not in (1, 2):
            response.success = False
            response.message = "channel must be 1 or 2"
            return response

        success, detail = self.relay_controller.select_channel(req.channel)
        response.success = success
        if success:
            response.message = "channel {} selected, response={}".format(req.channel, detail)
        else:
            response.message = "failed to select channel {}: {}".format(req.channel, detail)
        return response

    def _build_trigger_response(self, channel, success, detail):
        if success:
            return TriggerResponse(
                success=True,
                message="channel {} selected, response={}".format(channel, detail),
            )
        return TriggerResponse(
            success=False,
            message="failed to select channel {}: {}".format(channel, detail),
        )

    def handle_channel_1_on(self, _req):
        success, detail = self.relay_controller.select_channel(1)
        return self._build_trigger_response(1, success, detail)

    def handle_channel_2_on(self, _req):
        success, detail = self.relay_controller.select_channel(2)
        return self._build_trigger_response(2, success, detail)

    def handle_all_off(self, _req):
        success, detail = self.relay_controller.all_off()
        if success:
            return TriggerResponse(success=True, message="all relay channels are off, response={}".format(detail))
        return TriggerResponse(success=False, message="failed to switch all relay channels off: {}".format(detail))

    def publish_pressure(self, _event):
        left_kpa = self.pressure_reader.read_kpa(sensor_id=1)
        right_kpa = self.pressure_reader.read_kpa(sensor_id=2)

        self.left_pressure_pub.publish(Float32(data=self._to_float32(left_kpa)))
        self.right_pressure_pub.publish(Float32(data=self._to_float32(right_kpa)))

    @staticmethod
    def _to_float32(value):
        return float(value) if value is not None else math.nan

    def shutdown(self):
        self.relay_controller.close()


if __name__ == "__main__":
    try:
        ModbusIoBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
