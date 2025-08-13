#!/usr/bin/env python3
import rospy
import os
import rospkg
import sys
import stat
import pwd
from pathlib import Path
import netifaces

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
from queue import Empty
from typing import Set

from handler import (
    response_queue,
    websocket_message_handler,
    Response,
    cleanup_websocket,
    set_robot_type,
    init_ros_node,
    set_folder_path,
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

# Set up folder paths with proper permissions
sudo_user = os.environ.get("SUDO_USER")
if sudo_user:
    user_info = pwd.getpwnam(sudo_user)
    home_path = user_info.pw_dir
else:
    home_path = os.path.expanduser("~")

ROBOT_ACTION_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'action_files')
ROBOT_MAP_FILE_FOLDER = os.path.join(home_path, '.config', 'lejuconfig', 'maps')

# Create directories and manage permissions
try:
    Path(ROBOT_ACTION_FILE_FOLDER).mkdir(parents=True, exist_ok=True)
    Path(ROBOT_MAP_FILE_FOLDER).mkdir(parents=True, exist_ok=True)
except Exception as e:
    print(f"创建目录时出错: {e}")

# Check and change folder permissions if needed
try:
    for folder in [ROBOT_ACTION_FILE_FOLDER, ROBOT_MAP_FILE_FOLDER]:
        folder_stat = os.stat(folder)
        folder_uid = folder_stat.st_uid
        root_uid = 0
        if folder_uid == root_uid:
            os.chown(folder, 1000, 1000)
            print(f"已将 {folder} 的所有者更改为 lab 用户")
except Exception as e:
    print(f"检查或更改目录权限时出错: {e}")

# Get network information
wifi_ip = get_wifi_ip()
hotspot_ip = get_hotspot_ip()
robot_infos = {}

# Build robot info for WiFi connection
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
            "robot_map_file_folder": ROBOT_MAP_FILE_FOLDER,
            "robot_username": ROBOT_USERNAME,
            "robot_mac_address": get_wifi_mac_address(),
        }
    }

# Build robot info for Hotspot connection
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
            "robot_map_file_folder": ROBOT_MAP_FILE_FOLDER,
            "robot_username": ROBOT_USERNAME,
            "robot_mac_address": get_hotspot_mac_address(),
        }
    }

async def broadcast_robot_info_wifi():
    """Broadcast robot info via WiFi connection"""
    if "wifi" not in robot_infos:
        rospy.logwarn("WiFi信息不存在，无法广播。")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    robot_info = robot_infos["wifi"]
    robot_ip = robot_info['data']['robot_ip']
    
    # Find the correct interface and broadcast address using netifaces
    broadcast_ip = None
    for iface_name in netifaces.interfaces():
        if_addresses = netifaces.ifaddresses(iface_name)
        if netifaces.AF_INET in if_addresses:
            for link_addr in if_addresses[netifaces.AF_INET]:
                local_ip = link_addr.get('addr')
                if local_ip == robot_ip:
                    broadcast_ip = link_addr.get('broadcast')
                    break
        if broadcast_ip:
            break
    
    if not broadcast_ip:
        # Fallback to the old method if broadcast address not found
        broadcast_ip = f"{robot_ip.rsplit('.', 1)[0]}.255"
    
    print(f"Broadcasting WiFi to {broadcast_ip}:{BROADCAST_PORT}")
    print(f"WiFi robot info: {robot_info}")
    
    try:
        while True:
            message = json.dumps(robot_info).encode("utf-8")
            sock.sendto(message, (broadcast_ip, BROADCAST_PORT))
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        sock.close()

async def broadcast_robot_info_hotspot():
    """Broadcast robot info via Hotspot connection"""
    if "hotspot" not in robot_infos:
        rospy.logwarn("Hotspot信息不存在，无法广播。")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    robot_info = robot_infos["hotspot"]
    broadcast_ip = f"{robot_info['data']['robot_ip'].rsplit('.', 1)[0]}.255"
    print(f"Broadcasting Hotspot to {broadcast_ip}:{BROADCAST_PORT}")
    print(f"Hotspot robot info: {robot_info}")
    
    try:
        while True:
            message = json.dumps(robot_info).encode("utf-8")
            sock.sendto(message, (broadcast_ip, BROADCAST_PORT))
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        sock.close()

async def handle_websocket(websocket, path):
    """Handle WebSocket connections"""
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
        active_connections.discard(websocket)
        cleanup_websocket(websocket)
        print(f"Client disconnected: {websocket.remote_address}")

async def send_to_websockets(response: Response):
    """Send responses to WebSocket clients"""
    payload = json.dumps(response.payload.__dict__)
    target = response.target
    
    if target == "all":
        print(f"Broadcasting message to all clients: {payload}")
        if active_connections:
            await asyncio.gather(
                *[connection.send(payload) for connection in active_connections.copy()],
                return_exceptions=True,
            )
    else:
        if target in active_connections:
            try:
                await target.send(payload)
            except websockets.exceptions.ConnectionClosed:
                print(f"Connection closed for client: {target.remote_address}")
                active_connections.discard(target)
                cleanup_websocket(target)
        else:
            print(f"Client {target} not found in active connections")

async def process_responses():
    """Process responses from the response queue"""
    print("Starting to process responses")
    last_sent_message = None
    
    while True:
        try:
            response: Response = await asyncio.get_event_loop().run_in_executor(
                None, response_queue.get, True, 0.1
            )
            current_message = json.dumps(response.payload.__dict__)
            
            cmd = json.loads(current_message)["cmd"]
            if current_message == last_sent_message and cmd == "preview_action":
                print("Skipped sending duplicate message")
            else:
                await send_to_websockets(response)
                last_sent_message = current_message
        except Empty:
            await asyncio.sleep(0.001)
        except asyncio.CancelledError:
            break

async def websocket_server():
    """Start the WebSocket server"""
    server = await websockets.serve(handle_websocket, "0.0.0.0", 8888)
    print("WebSocket server started on ws://0.0.0.0:8888")
    try:
        await server.wait_closed()
    except asyncio.CancelledError:
        server.close()
        await server.wait_closed()

async def main(robot_type):
    """Main function to start all services"""
    
    # Check if any network connection is available
    if not robot_infos:
        rospy.logerr("WiFi和Hotspot都未连接。正在关闭...")
        return

    # Set robot configuration
    set_robot_type(robot_type)
    set_folder_path(ROBOT_ACTION_FILE_FOLDER, ROBOT_UPLOAD_FOLDER, ROBOT_MAP_FILE_FOLDER)

    print("Starting ROS node initialization")
    await init_ros_node()

    # Start logger server if available
    logger_task = None
    try:
        from logger_server import logger
        print("Starting logger server")
        logger_task = asyncio.create_task(logger.start())
        # Wait for logger initialization
        await asyncio.sleep(2)
    except ImportError:
        print("Logger server not available, skipping")

    # Create tasks for all services
    tasks = []
    
    # Add broadcast tasks based on available connections
    if "wifi" in robot_infos:
        print("Starting WiFi broadcast task")
        tasks.append(asyncio.create_task(broadcast_robot_info_wifi()))
    
    if "hotspot" in robot_infos:
        print("Starting Hotspot broadcast task")
        tasks.append(asyncio.create_task(broadcast_robot_info_hotspot()))

    print("Starting WebSocket server task")
    tasks.append(asyncio.create_task(websocket_server()))

    print("Starting response processing task")
    tasks.append(asyncio.create_task(process_responses()))

    if logger_task:
        tasks.append(logger_task)

    # Set up signal handling for graceful shutdown
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    print("All tasks started, waiting for stop event")
    try:
        await stop_event.wait()
    finally:
        print("Shutting down gracefully...")

        # Cancel all tasks
        for task in tasks:
            task.cancel()

        try:
            await asyncio.gather(*tasks, return_exceptions=True)
        except asyncio.CancelledError:
            pass
        
        print("Shutdown complete")

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Plan arm action websocket server')
    parser.add_argument('--robot_type', type=str, default='kuavo', help='Robot type')
    
    args, unknown = parser.parse_known_args()
    
    return args

if __name__ == "__main__":
    args = parse_args()
    asyncio.run(main(args.robot_type))