#!/usr/bin/env python3
import legacy_dependencies
"""Pico UDP -> Quest inputs; no hold, filter or lifecycle state machine."""
import json
import socket
import time

import rospy
from geometry_msgs.msg import Point, Quaternion
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList
from kuavo_msgs.msg import JoySticks, headCtrlMode
from kuavo_msgs.srv import SetHeadControlMode, SetHeadControlModeResponse
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_matrix

from pico_input.adapter import (
    PoseAdapter, controller_values, decode_datagram,
)


class PicoQuestReceiver:
    def __init__(self):
        self.adapter = PoseAdapter(rospy.get_param("~left_hand_offset", [0.] * 6),
                                   rospy.get_param("~right_hand_offset", [0.] * 6))
        self.peer = rospy.get_param("~peer_ip", "")
        self.connected = False
        self.pose_pub = rospy.Publisher("/leju_quest_bone_poses", PoseInfoList, queue_size=2)
        self.joy_pub = rospy.Publisher("/quest_joystick_data", JoySticks, queue_size=32, tcp_nodelay=True)
        self.mode_pub = rospy.Publisher("/quest3/head_control_mode", headCtrlMode, queue_size=1, latch=True)
        self.intent_pub = rospy.Publisher("/quest3/head_fixed_forward_user_intent", Empty, queue_size=1)
        self.service = rospy.Service("/quest3/set_head_control_mode", SetHeadControlMode, self.head_mode)
        self.mode_pub.publish(headCtrlMode(mode="fixed", fixed_main_hand="right"))
        rospy.set_param("/quest3/connected", False)
        # Explicit host discovery only; no robot commands or beta Pico control server.
        self.advertise_ip = rospy.get_param("~advertise_ip", "")
        self.broadcast_ip = rospy.get_param("~broadcast_ip", "")
        if bool(self.advertise_ip) != bool(self.broadcast_ip):
            raise ValueError("advertise_ip and broadcast_ip must both be supplied or both empty")
        self.advertiser = None
        if self.advertise_ip:
            self.advertiser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.advertiser.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.sock.bind((rospy.get_param("~bind_address", "0.0.0.0"),
                        int(rospy.get_param("~port", 12345))))
        self.sock.settimeout(0.02)
        rospy.on_shutdown(self.close)
        rospy.loginfo("Pico full_body -> Quest controller input: no adapter hold/confidence gate; "
                      "legacy timestamps are sender time; head tracking disabled.")

    def head_mode(self, request):
        # This scope has no head tracking. Report unsupported modes honestly.
        ok = request.mode == "fixed"
        if ok:
            self.intent_pub.publish(Empty())
        self.mode_pub.publish(headCtrlMode(mode="fixed", fixed_main_hand="right"))
        return SetHeadControlModeResponse(
            success=ok, message="Pico waist/hands mode: head fixed; head tracking unavailable",
            current_mode="fixed")

    def handle(self, raw, address, now):
        if self.peer and address[0] != self.peer:
            return
        msg, stamp_ns, _capture = decode_datagram(raw)
        if msg.HasField("upper_body"):
            raise ValueError("upper_body lacks SPINE3 torso; select SDK full_body mode")
        if not (msg.HasField("full_body") or msg.HasField("controller")):
            return
        body = joy = None
        if msg.HasField("full_body"):
            body = self.adapter.from_full_body(msg.full_body.full_body)
        if msg.HasField("controller"):
            joy = controller_values(msg.controller.controllers)
        if not self.peer:
            self.peer = address[0]
        # Preserve received button values/edges; no synthetic neutral samples.
        if joy is not None:
            self.joy_pub.publish(JoySticks(**joy))
        if body is not None:
            out = PoseInfoList(timestamp_ms=stamp_ns // 1000000, is_hand_tracking=False,
                               is_high_confidence=True)
            for T in body:
                q = quaternion_from_matrix(T)
                out.poses.append(PoseInfo(position=Point(*T[:3, 3]), orientation=Quaternion(*q)))
            self.pose_pub.publish(out)
        if not self.connected:
            self.connected = True
            rospy.set_param("/quest3/connected", True)

    def run(self):
        next_status = 0.
        while not rospy.is_shutdown():
            try:
                raw, address = self.sock.recvfrom(65535)
                self.handle(raw, address, time.monotonic())
            except socket.timeout:
                pass
            except OSError:
                if rospy.is_shutdown():
                    break
                raise
            except Exception as error:
                rospy.logwarn_throttle(2., "Pico malformed/unsupported packet skipped: %s", error)
            now = time.monotonic()
            if now >= next_status:
                self.mode_pub.publish(headCtrlMode(mode="fixed", fixed_main_hand="right"))
                if self.advertiser:
                    info = {"data": {"robot_name": "KUAVO", "robot_ip": self.advertise_ip,
                                     "eef_type": rospy.get_param("/end_effector_type", "qiangnao")}}
                    try:
                        self.advertiser.sendto(json.dumps(info).encode(), (self.broadcast_ip, 8443))
                    except OSError as error:
                        rospy.logwarn_throttle(5., "Pico discovery broadcast failed: %s", error)
                next_status = now + 1.

    def close(self):
        rospy.set_param("/quest3/connected", False)
        self.sock.close()
        if self.advertiser:
            self.advertiser.close()


if __name__ == "__main__":
    rospy.init_node("monitor_pico")
    PicoQuestReceiver().run()
