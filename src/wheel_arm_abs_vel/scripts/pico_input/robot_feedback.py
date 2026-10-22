"""Existing Pico wire format for telemetry and shared Quest wrench responses.

Currents retain the units/layout of pico-body-tracking-server. Do not reinterpret
motor current as Quest's estimated contact-force feedback.
"""
import math
import queue
import socket
import threading
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import lejuClawState
from . import body_tracking_extended_pb2 as proto


class PicoRobotFeedback:
    def __init__(self):
        self.peer = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.responses = queue.Queue(maxsize=32)
        self.lock = threading.Lock()
        self.motor = []
        self.eef = []
        self.sequence = 0
        self.subscribers = [
            rospy.Subscriber('/sensor_data_motor/motor_cur', Float64MultiArray, self.motor_callback, queue_size=1),
            rospy.Subscriber('/dexhand/state', JointState, self.hand_callback, queue_size=1),
            rospy.Subscriber('/leju_claw_state', lejuClawState, self.claw_callback, queue_size=1)]
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)

    def _store(self, channel, values):
        if all(math.isfinite(x) for x in values):
            with self.lock:
                setattr(self, channel, values)

    def motor_callback(self, msg):
        # Shared Pico server's half-body layout (14 arm currents).
        self._store('motor', [abs(x) for x in msg.data[12:26]])

    def hand_callback(self, msg):
        self._store('eef', [abs(x) * 6. / 1000 for x in msg.effort])

    def claw_callback(self, msg):
        if len(msg.data.effort) == 2:
            self._store('eef', [abs(msg.data.effort[0])] + [0.] * 5 +
                        [abs(msg.data.effort[1])] + [0.] * 5)

    def add_item_mass_force_response(self, response):
        try:
            self.responses.put_nowait(response.SerializeToString())
        except queue.Full:
            rospy.logwarn_throttle(1., 'Pico wrench response queue full')

    def publish(self, _event):
        peer = self.peer
        if peer is None:
            return
        msg = proto.VRData()
        msg.header.timestamp.GetCurrentTime()
        self.sequence += 1
        msg.header.id = self.sequence
        with self.lock:
            msg.robot_data.motor_cur.extend(self.motor)
            msg.robot_data.eef_cur.extend(self.eef)
        try:
            response = self.responses.get_nowait()
            # Quest/Pico names differ; identical field numbers carry the same response.
            msg.robot_data.item_mass_force_response.ParseFromString(response)
        except queue.Empty:
            pass
        msg.robot_data.control_mode.control_mode = 'mobile_mpc'
        try:
            self.socket.sendto(msg.SerializeToString(), peer)
        except OSError as error:
            if not rospy.is_shutdown():
                rospy.logwarn_throttle(2., 'Pico feedback send failed: %s', error)

    def close(self):
        self.timer.shutdown()
        self.peer = None
        self.socket.close()
