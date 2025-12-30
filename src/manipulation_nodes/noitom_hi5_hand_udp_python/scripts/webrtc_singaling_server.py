import asyncio
import websockets
from datetime import datetime
import threading

class WebRTCSinglingServer:
    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()
        self.running = True
        self.connected_clients = []
        self.server_thread = None
        self.loop = None

    def get_connected_clients_count(self):
        return len(self.connected_clients)

    async def signaling(self, websocket, path):
        client_id = await websocket.recv()

        with self.lock:
            self.clients.append(websocket)
            self.connected_clients.append(f"{client_id}")

        print(f"[{datetime.now()}] Client {client_id} connected")

        try:
            async for message in websocket:
                for client in self.clients:
                    if client != websocket:
                        await client.send(message)
                        print(f"[{datetime.now()}] Broadcasting message from {client_id} to client: {message}")
        except websockets.exceptions.ConnectionClosed:
            print(f"[{datetime.now()}] Client {client_id} disconnected")
        finally:
            with self.lock:
                self.clients.remove(websocket)
                self.connected_clients.remove(f"{client_id}")
            print(f"[{datetime.now()}] Client {client_id} removed from clients list")

    async def start_server(self):
        server = await websockets.serve(self.signaling, "0.0.0.0", 8765)
        await server.wait_closed()

    def run_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.start_server())
        except Exception as e:
            print(f"webrtc_singaling_server: Except - {str(e)}")
            pass

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
