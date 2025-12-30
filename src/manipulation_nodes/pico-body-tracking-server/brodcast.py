import os
from utils import get_wifi_ip, get_wifi
import socket
import json
import time

ROBOT_NAME = os.getenv("ROBOT_NAME", "KUAVO")
ROBOT_IP = get_wifi_ip()
BROADCAST_IP = f"{ROBOT_IP.rsplit('.', 1)[0]}.255"
BROADCAST_PORT = 8443

robot_info = {
    "data": {
        "robot_name": ROBOT_NAME,
        "robot_ip": ROBOT_IP,
    }
}

def broadcast_robot_info():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print(f"Broadcasting to {BROADCAST_IP}:{BROADCAST_PORT}")
    print(f"Broadcasting robot info: {robot_info}")
    while True:
        message = json.dumps(robot_info).encode("utf-8")
        sock.sendto(message, (BROADCAST_IP, BROADCAST_PORT))
        time.sleep(1)