import cv2
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.contrib.signaling import BYE
import websockets
from aiortc.contrib.media import MediaStreamTrack
from datetime import datetime
from av import VideoFrame
import av
import fractions
import sys
import subprocess
import rospy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import signal
import threading
from webrtc_singaling_server import ROBOT_SIGNALING_CLIENT_ID

class WebRTCVideoStreamClient:
    def __init__(self, server_ip, ros_topic, signaling_server=None):
        self.server_ip = server_ip
        self.ros_topic = f"{ros_topic}/image_raw/compressed"
        self.ros_camera_info_topic = f"{ros_topic}/camera_info"
        self.signaling_server = signaling_server  # 重连模式下用于感知 VR 客户端在线状态

        if not self.topic_exists(self.ros_topic):
            print(f"\033[91mcarlos_WebRTCVideoStreamClient_Topic_not_found: Topic {self.ros_topic} does not exist\033[0m")
        if not self.topic_exists(self.ros_camera_info_topic):
            print(f"\033[91mcarlos_WebRTCVideoStreamClient_Topic_not_found: Topic {self.ros_camera_info_topic} does not exist\033[0m")

        if not self.topic_exists(self.ros_topic) or not self.topic_exists(self.ros_camera_info_topic):
            print("\033[91m" + "carlos_WebRTCVideoStreamClient _init_fail: Either topic does not exist, returning None" + "\033[0m")
            raise ValueError("carlos_WebRTCVideoStreamClient _init_fail: Either topic does not exist, returning None")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.pc = None
        self.width = 0
        self.height = 0
        self.start_connect_webrtc_singal = False

    def topic_exists(self, topic_name):
        """
        Directly check if a given ROS topic exists.

        Args:
            topic_name (str): The name of the topic to check (e.g., '/camera/image_raw').

        Returns:
            bool: True if the topic exists, False otherwise.
        """
        try:
            # Retrieve the list of all published topics
            published_topics = rospy.get_published_topics()

            # Extract only the topic names for efficient searching
            topic_names = [topic for topic, _ in published_topics]

            # Check if the target topic is in the list
            return topic_name in topic_names
        except rospy.ROSException as e:
            rospy.logerr("Failed to retrieve published topics: %s", e)
            return False

    async def signaling(self, websocket, pc):
        while True:
            message = await websocket.recv()
            print(f"Received message: {message}")

            if message == BYE:
                print("Received BYE, stopping")
                await pc.close()
                break
            else:
                msg = json.loads(message)

                if 'candidate' in msg:
                    candidate_parts = msg['candidate'].split(' ')
                    print("Split candidate parts:")
                    for index, part in enumerate(candidate_parts):
                        print(f"Part {index}: {part}")

                    ice_candidate = RTCIceCandidate(
                        foundation=candidate_parts[0].split(':')[1],
                        component=int(candidate_parts[1]),
                        protocol=candidate_parts[2],
                        priority=int(candidate_parts[3]),
                        ip=candidate_parts[4],
                        port=int(candidate_parts[5]),
                        type=candidate_parts[7],
                        sdpMid=msg.get('sdpMid', None),
                        sdpMLineIndex=msg.get('sdpMLineIndex', None)
                    )

                    await pc.addIceCandidate(ice_candidate)
                    print("after add ice candidate ")
                elif 'sdp' in msg:
                    if msg['type'] == 2 or msg['type'] == '2':
                        msg['type'] = "answer"
                    
                    desc = RTCSessionDescription(sdp=msg['sdp'], type=msg['type'])
                    await pc.setRemoteDescription(desc)

                    if desc.type == "offer":
                        await pc.setLocalDescription(await pc.createAnswer())
                        await websocket.send(json.dumps({
                            'sdp': pc.localDescription.sdp,
                            'type': pc.localDescription.type
                        }))

    async def create_offer(self, websocket, pc):
        print(f"create_offer called at {datetime.now().isoformat()}")
        await pc.setLocalDescription(await pc.createOffer())
        await websocket.send(json.dumps({
            'sdp': pc.localDescription.sdp,
            'type': pc.localDescription.type
        }))

    class ImageVideoTrack(VideoStreamTrack):
        def __init__(self, client):
            super().__init__()
            self.client = client

        async def recv(self):
            pts, time_base = await self.next_timestamp()

            if self.client.latest_frame is not None:
                frame = av.VideoFrame.from_ndarray(self.client.latest_frame, format='bgr24')
                frame.pts = pts
                frame.time_base = time_base
                return frame
            else:
                return None

    def camera_info_callback(self,msg):
        self.width = msg.width
        self.height = msg.height
        # print(f"Camera resolution: {msg.width}x{msg.height}")

    def image_callback(self, ros_image):
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")

    async def main(self):
        # 断线重连循环：VR 客户端在线时发起一轮新会话，掉线后等待下次重连
        # 注意：仅覆盖「App 重开、机器端保持」场景；机器端进程重启后 VR App 不主动重连，见 webrtc_videostream.py
        while not rospy.is_shutdown():
            await self.wait_vr_client_online()
            if rospy.is_shutdown():
                break
            await self.run_session()

    async def wait_vr_client_online(self):
        if self.signaling_server is not None:
            # 重连模式：轮询信令服务器，直到有 VR 客户端在线
            while not rospy.is_shutdown() and self.signaling_server.get_vr_clients_count() == 0:
                await asyncio.sleep(0.5)
        else:
            # 兼容单机模式（无信令服务器引用）：等待外部置位一次性标志
            while not self.start_connect_webrtc_singal and not rospy.is_shutdown():
                await asyncio.sleep(1)

    async def run_session(self):
        # 发起一轮新的 WebRTC 会话；信令 WS 关闭（VR 掉线/主动断开）后结束本轮
        uri = f"ws://{self.server_ip}:8765"
        self.pc = RTCPeerConnection()
        self.pc.addTrack(self.ImageVideoTrack(self))
        print(f"[{datetime.now()}] Start new video stream session, create RTCPeerConnection")

        try:
            async with websockets.connect(uri) as websocket:
                await websocket.send(ROBOT_SIGNALING_CLIENT_ID)
                print("Video track has been added to the peer connection.")
                await self.create_offer(websocket, self.pc)
                await self.watch_session(websocket, self.pc)
        except websockets.exceptions.ConnectionClosed as e:
            print(f"[{datetime.now()}] Video stream session closed: {e}")
        except Exception as e:
            # 临时网络错误不终止重连循环，记录后进入下一轮等待
            print(f"\033[91m[{datetime.now()}] Video stream session error, will retry: {e}\033[0m")
        finally:
            await self.pc.close()
            self.pc = None

    async def watch_session(self, websocket, pc):
        # 单机模式（无信令服务器引用）：直接跑信令循环，保持一次会话行为
        if self.signaling_server is None:
            await self.signaling(websocket, pc)
            return
        # 重连模式：并发处理信令消息与 VR 在线监控，任一结束即终止本轮会话
        signaling_task = asyncio.create_task(self.signaling(websocket, pc))
        presence_task = asyncio.create_task(self.monitor_vr_presence(websocket))
        await asyncio.wait([signaling_task, presence_task], return_when=asyncio.FIRST_COMPLETED)
        for task in (signaling_task, presence_task):
            task.cancel()
        await asyncio.gather(signaling_task, presence_task, return_exceptions=True)

    async def monitor_vr_presence(self, websocket):
        # VR 客户端掉线时关闭当前信令 WS，结束本轮会话以等待下次重连
        while not rospy.is_shutdown():
            if self.signaling_server.get_vr_clients_count() == 0:
                await websocket.close()
                return
            await asyncio.sleep(0.5)

    async def handle_ros(self):
        # rospy.init_node('image_to_webrtc', anonymous=True)
        rospy.Subscriber(self.ros_topic, CompressedImage, self.image_callback)
        rospy.Subscriber(self.ros_camera_info_topic, CameraInfo, self.camera_info_callback)

        while not rospy.is_shutdown():
            await asyncio.sleep(0.03)

    def handle_signal(self, signal, frame):
        print("Received Ctrl-C, stopping...")
        sys.exit(0)


    async def run_in_background(self):
        print(f"WebRTCVideoStreamClient started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        ros_task = asyncio.create_task(self.handle_ros())
        main_task = asyncio.create_task(self.main())
        await asyncio.gather(ros_task, main_task)

    def start(self):
        def run_event_loop_in_thread():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.run_in_background())
            except KeyboardInterrupt:
                print("Received Ctrl-C, stopping...")
            finally:
                loop.close()

        thread = threading.Thread(target=run_event_loop_in_thread, daemon=True)
        thread.start()
        print("WebRTC client started in background thread")
