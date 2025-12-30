import socket
import sys
from concurrent.futures import ThreadPoolExecutor
import time
import kuavo_vr_events_pb2
import threading

class UdpSenderForInfoToQuest3:
    def __init__(self, ip, ports, message, width=0, height=0):
        self.ip = ip
        self.ports = ports
        event = kuavo_vr_events_pb2.KuavoVrEvents()
        event.webrtc_signaling_url = message
        event.camera_info.width = width
        event.camera_info.height = height
        self.message = event.SerializeToString()
        self.stop_event = threading.Event()

    def _send_udp_message(self, port):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.settimeout(1)  # Set a timeout of 1 second
                sock.sendto(self.message, (self.ip, port))
                data, addr = sock.recvfrom(1024)
                print(f"Response from {self.ip}:{port}: {data.decode()}")
        except socket.timeout:
            # print(f"No response from {self.ip}:{port}")
            pass
        except Exception as e:
            print(f"Error sending to {self.ip}:{port}: {str(e)}")

    def _send_udp_message_periodically(self):
        while not self.stop_event.is_set():
            with ThreadPoolExecutor(max_workers=50) as executor:
                futures = [executor.submit(self._send_udp_message, port) for port in self.ports]
                for future in futures:
                    future.result()
            if self.stop_event.is_set():
                break
            time.sleep(1)  # Wait for 3 seconds before sending again

    def start(self):
        self.thread = threading.Thread(target=self._send_udp_message_periodically)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join()  # Wait for the thread to finish
