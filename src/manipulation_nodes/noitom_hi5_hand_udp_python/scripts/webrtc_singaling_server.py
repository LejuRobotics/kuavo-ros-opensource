import asyncio
import time
import websockets
from datetime import datetime
import threading

ROBOT_SIGNALING_CLIENT_ID = "leju_robot_webrtc"  # 机器人自身信令客户端 ID，用于与 VR 端区分

class WebRTCSinglingServer:
    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()
        self.running = True
        self.connected_clients = []
        self.server_thread = None
        self.loop = None

    def get_connected_clients_count(self):
        with self.lock:
            return len(self.connected_clients)

    def get_vr_clients_count(self):
        # 统计除机器人自身信令客户端外的客户端数，用于感知 VR 端在线状态
        with self.lock:
            return len([cid for cid in self.connected_clients if cid != ROBOT_SIGNALING_CLIENT_ID])

    async def signaling(self, websocket, path):
        client_id = await websocket.recv()

        with self.lock:
            self.clients.append(websocket)
            self.connected_clients.append(f"{client_id}")

        print(f"[{datetime.now()}] Client {client_id} connected")

        try:
            async for message in websocket:
                for client in self.clients.copy():
                    if client != websocket:
                        try:
                            await client.send(message)
                            print(f"[{datetime.now()}] Broadcasting message from {client_id} to client: {message}")
                        except websockets.exceptions.ConnectionClosed as e:
                            print(f"[{datetime.now()}] Client disconnected during broadcast: {e}")
        except websockets.exceptions.ConnectionClosed as e:
            print(f"[{datetime.now()}] Client {client_id} disconnected: {e}")
        finally:
            with self.lock:
                if websocket in self.clients:
                    self.clients.remove(websocket)
                if f"{client_id}" in self.connected_clients:
                    self.connected_clients.remove(f"{client_id}")
            print(f"[{datetime.now()}] Client {client_id} removed from clients list")

    async def start_server(self):
        server = await websockets.serve(self.signaling, "0.0.0.0", 8765)
        await server.wait_closed()

    def run_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        max_bind_retries = 10
        for retry in range(max_bind_retries):
            if not self.running:
                return
            try:
                self.loop.run_until_complete(self.start_server())
                return
            except OSError as e:
                if e.errno != 98:
                    print(f"\033[91mwebrtc_singaling_server: bind failed - {e}\033[0m")
                    return
                # 快速重启时旧进程 socket 未完全释放，等待 1s 后重试绑定
                print(f"\033[91mwebrtc_singaling_server: Port 8765 is busy (Error 98), retry {retry + 1}/{max_bind_retries} in 1s\033[0m")
                time.sleep(1)
            except Exception as e:
                if not self.running:
                    return  # 主动 shutdown，静默退出
                print(f"\033[91mwebrtc_singaling_server: Except - {str(e)}\033[0m")
                return
        print(f"\033[91mwebrtc_singaling_server: Port 8765 still busy after {max_bind_retries} retries, giving up\033[0m")
        self._print_port_occupier()

    def _print_port_occupier(self):
        # 打印占用 8765 端口的进程，便于排查端口被长期占用的情况
        try:
            import subprocess
            result = subprocess.run(['lsof', '-i', ':8765'], capture_output=True, text=True)
            if result.stdout:
                print(f"\033[93mProcesses occupying port 8765:\n{result.stdout}\033[0m")
            else:
                print(f"\033[93mNo process listed for port 8765 (lsof empty)\033[0m")
        except Exception as e:
            print(f"\033[91mFailed to inspect port 8765: {e}\033[0m")
    def start(self):
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.start()
        print("WebRTC server started in background")

    def stop(self):
        print("Stopping WebRTC server...")
        self.running = False
        if self.loop:
            for client in self.clients:
                self.loop.call_soon_threadsafe(client.close)
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.server_thread:
            self.server_thread.join()
        print("WebRTC server stopped")
    def get_connected_clients(self):
        """Return the list of connected clients"""
        with self.lock:
            return self.connected_clients[:]
