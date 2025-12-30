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

wait_webrtc_client_connect_timeout = 180  # seconds

class WebRTCServerAndVideoStreamClient:
    def __init__(self, camera_topic_for_video_stream):
        self.camera_topic_for_video_stream = camera_topic_for_video_stream
        self.web_rtc_signaling_server = None
        self.webrtc_video_stream_client = None
        self.udp_sender_send_webrtc_signaling_info = None

    def start(self):
        rospy.init_node('Leju_webrtc_VideoStream', anonymous=True)
        rate = rospy.Rate(10)  # 10 Hz

        print("Starting to create WebRTC signaling server...")
        self.web_rtc_signaling_server = WebRTCSinglingServer()
        print("WebRTC signaling server created successfully.")

        self.web_rtc_signaling_server.start()
        print("Web_rtc_signaling_server_started: WebRTC signaling server started successfully.")

        self.webrtc_video_stream_client = WebRTCVideoStreamClient("127.0.0.1", self.camera_topic_for_video_stream)
        self.webrtc_video_stream_client.start()

        start_time = time.time()
        while self.webrtc_video_stream_client.width == 0 or self.webrtc_video_stream_client.height == 0:
            if time.time() - start_time > 10:
                raise TimeoutError("Failed to initialize video stream within 10 seconds")
            time.sleep(0.1)

        width = self.webrtc_video_stream_client.width
        height = self.webrtc_video_stream_client.height
        print("\033[94m" + "Start ros node loop: Starting the rosnode loop" + "\033[0m")
        self.udp_sender_send_webrtc_signaling_info = self.BroadWebRtcAndCameraInfoToQuest3(width, height)

        start_time = time.time()
        while not rospy.is_shutdown():
            webrtc_clients_cnt = self.web_rtc_signaling_server.get_connected_clients_count()
            if webrtc_clients_cnt > 0:
                self.webrtc_video_stream_client.start_connect_webrtc_singal = True
                self.udp_sender_send_webrtc_signaling_info.stop()
                self.udp_sender_send_webrtc_signaling_info = None
                break
            elapsed_time = time.time() - start_time
            if elapsed_time > wait_webrtc_client_connect_timeout:
                print("\033[91mcarlos_webrtc_client_connect_timeout: Wait Quest3 connect to webrtc server: Timed out after {} seconds\033[0m".format(wait_webrtc_client_connect_timeout))
                rospy.signal_shutdown("Shutting down gracefully.")
                break
            remaining_time = wait_webrtc_client_connect_timeout - int(elapsed_time)
            print("Waiting for Quest3 to connect to webrtc server... {} seconds remaining".format(remaining_time))
            time.sleep(1)  # add a 1-second sleep

    def BroadWebRtcAndCameraInfoToQuest3(self, width, height):
        webrtc_signaling_url = ":8765"
        ports = [10030, 10031, 10032, 10033, 10034, 10035, 10036, 10037, 10038, 10039, 10040]
        print(f"Webrtc signaling prot: {webrtc_signaling_url}")
        sender = UdpSenderForInfoToQuest3("255.255.255.255", ports, webrtc_signaling_url, width, height)
        sender.start()
        return sender

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 webrtc_videostream.py [<camera_topic_for_video_stream>]")
        sys.exit(1)

    camera_topic_for_video_stream = sys.argv[1]
    webrtc_server_and_video_stream_client = WebRTCServerAndVideoStreamClient(camera_topic_for_video_stream)
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
