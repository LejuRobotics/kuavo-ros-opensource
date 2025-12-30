#!/usr/bin/env python3
import rospy
import os
import rospkg
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)


from utils import get_wifi_ip, get_hotspot_ip, get_wifi_mac_address, get_hotspot_mac_address, get_wifi, get_hotspot
import json
import websockets
import asyncio
import socket
import signal
import argparse
import pwd
from pathlib import Path
from queue import Empty
from typing import Set

from handler import (
    response_queue,
    websocket_message_handler,
    Response,
    cleanup_websocket,
    set_robot_type,
    init_ros_node,
)

from kuavo_ros_interfaces.msg import planArmState

# Set to store active WebSocket connections
active_connections: Set[websockets.WebSocketServerProtocol] = set()

ROBOT_NAME = os.getenv("ROBOT_NAME", "KUAVO")
ROBOT_USERNAME = "lab"
BROADCAST_PORT = 8443
package_name = 'planarmwebsocketservice'
package_path = rospkg.RosPack().get_path(package_name)
ROBOT_UPLOAD_FOLDER = package_path + "/upload_files"
sudo_user = os.environ.get("SUDO_USER")
if sudo_user:
    user_info = pwd.getpwnam(sudo_user)
    home_path = user_info.pw_dir
else:
    home_path = os.path.expanduser("~")
ROBOT_ACTION_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'action_files')
try:
    Path(ROBOT_ACTION_FILE_FOLDER).mkdir(parents=True, exist_ok=True)
except Exception as e:
    print(f"创建 ROBOT_ACTION_FILE_FOLDER 目录时出错: {e}")

wifi_ip = get_wifi_ip()
hotspot_ip = get_hotspot_ip()
robot_infos = {}

if wifi_ip:
    robot_infos["wifi"] = {
        "data": {
            "robot_name": ROBOT_NAME,
            "robot_ip": wifi_ip,
            "robot_connect_wifi": get_wifi(),
            "robot_ws_address": f"ws://{wifi_ip}:8888",
            "robot_ws_logger_address": f"ws://{wifi_ip}:8889",
            "robot_upload_folder": ROBOT_UPLOAD_FOLDER,
            "robot_action_file_folder": ROBOT_ACTION_FILE_FOLDER,
            "robot_username": ROBOT_USERNAME,
            "robot_mac_address": get_wifi_mac_address(),
        }
    }

if hotspot_ip:
    robot_infos["hotspot"] = {
        "data": {
            "robot_name": ROBOT_NAME,
            "robot_ip": hotspot_ip,
            "robot_connect_wifi": get_hotspot(),
            "robot_ws_address": f"ws://{hotspot_ip}:8888",
            "robot_ws_logger_address": f"ws://{hotspot_ip}:8889",
            "robot_upload_folder": ROBOT_UPLOAD_FOLDER,
            "robot_action_file_folder": ROBOT_ACTION_FILE_FOLDER,
            "robot_username": ROBOT_USERNAME,
            "robot_mac_address": get_hotspot_mac_address(),
        }
    }


async def broadcast_robot_info_wifi():
    if "wifi" not in robot_infos:
        rospy.logwarn("WiFi信息不存在，无法广播。")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_info = robot_infos["wifi"]
    broadcast_ip = f"{robot_info['data']['robot_ip'].rsplit('.', 1)[0]}.255"
    print(f"Broadcasting to {broadcast_ip}:{BROADCAST_PORT} with info: {robot_info}")
    while True:
        message = json.dumps(robot_info).encode("utf-8")
        sock.sendto(message, (broadcast_ip, BROADCAST_PORT))
        await asyncio.sleep(1)

async def broadcast_robot_info_hotspot():
    if "hotspot" not in robot_infos:
        rospy.logwarn("Hotspot信息不存在，无法广播。")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    robot_info = robot_infos["hotspot"]
    broadcast_ip = f"{robot_info['data']['robot_ip'].rsplit('.', 1)[0]}.255"
    print(f"Broadcasting to {broadcast_ip}:{BROADCAST_PORT} with info: {robot_info}")
    while True:
        message = json.dumps(robot_info).encode("utf-8")
        sock.sendto(message, (broadcast_ip, BROADCAST_PORT))
        await asyncio.sleep(1)

async def handle_websocket(websocket, path):
    try:
        active_connections.add(websocket)
        print(f"Client connected: {websocket.remote_address}")
        async for message in websocket:
            print(f"Received message from client: {message}")
            data = json.loads(message)
            await websocket_message_handler(websocket, data)
    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed for client: {websocket.remote_address}")
    finally:
        active_connections.remove(websocket)
        cleanup_websocket(websocket)
        print(f"Client disconnected: {websocket.remote_address}")

async def send_to_websockets(response: Response):
    payload = json.dumps(response.payload.__dict__)
    target = response.target
    if target == "all":
        print(f"Broadcasting message to all clients: {payload}")
        await asyncio.gather(
            *[connection.send(payload) for connection in active_connections],
            return_exceptions=True,
        )
    else:
        # print(f"Sending message to specific client: {payload}")
        if target in active_connections:
            try:
                await target.send(payload)
            except websockets.exceptions.ConnectionClosed:
                print(f"Connection closed for client: {target.remote_address}")
                active_connections.remove(target)
                cleanup_websocket(target)
        else:
            print(f"Client {target} not found in active connections")

async def process_responses():
    print("Starting to process responses")
    last_sent_message = None
    while True:
        try:
            response: Response = await asyncio.get_event_loop().run_in_executor(None, response_queue.get, True, 0.1)
            current_message = json.dumps(response.payload.__dict__)
            
            cmd = json.loads(current_message)["cmd"]
            if current_message == last_sent_message and cmd == "preview_action":
                print("Skipped sending duplicate message")
            else:
                await send_to_websockets(response)
                last_sent_message = current_message
        except Empty:
            await asyncio.sleep(0.001)


async def websocket_server():
    server = await websockets.serve(handle_websocket, "0.0.0.0", 8888)
    print("WebSocket server started on ws://0.0.0.0:8888")
    await server.wait_closed()

async def main(robot_type):

    if not robot_infos:
        rospy.logerr("WiFi和Hotspot都未连接。正在关闭...")
        return

    set_robot_type(robot_type)
    print("Starting ROS node initialization")
    ros_init_task = asyncio.create_task(init_ros_node())

    print("Starting Wi-Fi broadcast task")
    broadcast_wifi_task = asyncio.create_task(broadcast_robot_info_wifi())
    print("Starting Hotspot broadcast task")
    broadcast_hotspot_task = asyncio.create_task(broadcast_robot_info_hotspot())
    print("Starting WebSocket server task")
    websocket_server_task = asyncio.create_task(websocket_server())

    print("Starting response processing task")
    process_responses_task = asyncio.create_task(process_responses())

    # Wait for ROS node initialization to complete
    await ros_init_task

    # Start ROS spin task after initialization

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    print("All tasks started, waiting for stop event")
    try:
        await stop_event.wait()
    finally:
        print("Shutting down gracefully...")
        broadcast_wifi_task.cancel()
        broadcast_hotspot_task.cancel()
        websocket_server_task.cancel()
        process_responses_task.cancel()
        try:
            await asyncio.gather(
                broadcast_wifi_task,
                broadcast_hotspot_task,
                websocket_server_task,
                process_responses_task,
                return_exceptions=True
            )
        except asyncio.CancelledError:
            pass
        print("Shutdown complete")

def parse_args():
    parser = argparse.ArgumentParser(description='Plan arm action websocket server')
    parser.add_argument('--robot_type', type=str, default='kuavo', help='Robot type')
    
    args, unknown = parser.parse_known_args()
    
    return args

if __name__ == "__main__":
    args = parse_args()

    asyncio.run(main(args.robot_type))
