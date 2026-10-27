#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import sys
import os
import json
import time
import signal
import sys
from pprint import pprint
import tf
import rospy
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from UdpSenderForInfoToQuest3 import UdpSenderForInfoToQuest3
from webrtc_singaling_server import WebRTCSinglingServer 
from webrtcvideostreamclient import WebRTCVideoStreamClient

import netifaces as ni
import re
import numpy as np
import logging

# Set the logging level to DEBUG
logging.basicConfig(level=logging.DEBUG)

class WebRTCServerAndVideoStreamClient:
    def __init__(self, camera_topic_for_video_stream, quest3_ip=None):
        self.camera_topic_for_video_stream = camera_topic_for_video_stream
        self.quest3_ip = quest3_ip
        self.web_rtc_signaling_server = None
        self.webrtc_video_stream_client = None
        self.udp_sender_send_webrtc_signaling_info = None

    def start(self):
        rospy.init_node('Leju_webrtc_VideoStream', anonymous=True)

        print("Starting to create WebRTC signaling server...")
        self.web_rtc_signaling_server = WebRTCSinglingServer()
        print("WebRTC signaling server created successfully.")

        self.web_rtc_signaling_server.start()
        print("Web_rtc_signaling_server_started: WebRTC signaling server started successfully.")

        self.webrtc_video_stream_client = WebRTCVideoStreamClient(
            "127.0.0.1", self.camera_topic_for_video_stream, signaling_server=self.web_rtc_signaling_server)
        self.webrtc_video_stream_client.start()

        start_time = time.time()
        while self.webrtc_video_stream_client.width == 0 or self.webrtc_video_stream_client.height == 0:
            if time.time() - start_time > 10:
                raise TimeoutError("Failed to initialize video stream within 10 seconds")
            time.sleep(0.1)

        width = self.webrtc_video_stream_client.width
        height = self.webrtc_video_stream_client.height
        print("\033[94m" + "Start ros node loop: Starting the rosnode loop" + "\033[0m")

        # 断线重连边界（2026-09-05 实测）：
        #   ✓ App 端重开、机器端进程保持：恢复广播 + offer 重连，可自动恢复视频
        #   ✗ 机器端进程重启、App 端保持：VR App 一次性握手设计，不响应广播、不重连信令(8765)，
        #     无摄像头数据；协议(KuavoVrEvents)无重置命令，须手动重启 VR App 才能恢复
        # 持续监控 VR 客户端在线状态：在线时停广播，离线时恢复广播，保证断线后可重新发现
        last_status = None
        last_wait_log = time.time()
        while not rospy.is_shutdown():
            vr_clients_cnt = self.web_rtc_signaling_server.get_vr_clients_count()
            status = 'connected' if vr_clients_cnt > 0 else 'waiting'
            if status != last_status:
                if status == 'connected':
                    self.stop_udp_broadcast()
                    print(f"[{time.strftime('%H:%M:%S')}] VR client connected, stop UDP broadcast (vr_clients_cnt={vr_clients_cnt})")
                else:
                    self.ensure_udp_broadcast(width, height)
                    print(f"[{time.strftime('%H:%M:%S')}] No VR client, resume UDP broadcast for re-discovery (vr_clients_cnt={vr_clients_cnt})")
                last_status = status
            elif status == 'waiting' and time.time() - last_wait_log > 30:
                print(f"[{time.strftime('%H:%M:%S')}] Still waiting for VR client to connect ...")
                last_wait_log = time.time()
            time.sleep(0.5)

    def stop_udp_broadcast(self):
        if self.udp_sender_send_webrtc_signaling_info is not None:
            self.udp_sender_send_webrtc_signaling_info.stop()
            self.udp_sender_send_webrtc_signaling_info = None

    def ensure_udp_broadcast(self, width, height):
        if self.udp_sender_send_webrtc_signaling_info is None:
            self.udp_sender_send_webrtc_signaling_info = self.BroadWebRtcAndCameraInfoToQuest3(width, height)

    def BroadWebRtcAndCameraInfoToQuest3(self, width, height):
        webrtc_signaling_url = ":8765"
        ports = [10030, 10031, 10032, 10033, 10034, 10035, 10036, 10037, 10038, 10039, 10040]
        print(f"Webrtc signaling prot: {webrtc_signaling_url}")
        target_ips = [self.quest3_ip] if self.quest3_ip else None
        sender = UdpSenderForInfoToQuest3(ports, webrtc_signaling_url, width, height, target_ips=target_ips)
        sender.start()
        return sender

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 webrtc_videostream.py [<camera_topic_for_video_stream>] [quest3_ip]")
        sys.exit(1)

    camera_topic_for_video_stream = sys.argv[1]
    quest3_ip = sys.argv[2] if len(sys.argv) > 2 and "." in sys.argv[2] else None
    webrtc_server_and_video_stream_client = WebRTCServerAndVideoStreamClient(camera_topic_for_video_stream, quest3_ip)
    webrtc_server_and_video_stream_client.start()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Shutting down my_basic_node.")

    finally:
        rospy.loginfo("Leju_webrtc_VideoStream is shutting down.")
        if webrtc_server_and_video_stream_client.web_rtc_signaling_server is not None:
            webrtc_server_and_video_stream_client.web_rtc_signaling_server.stop()
        if webrtc_server_and_video_stream_client.udp_sender_send_webrtc_signaling_info is not None:
            webrtc_server_and_video_stream_client.udp_sender_send_webrtc_signaling_info.stop()

if __name__ == '__main__':
    main()
