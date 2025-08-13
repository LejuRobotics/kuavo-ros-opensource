import socket
import sys
from concurrent.futures import ThreadPoolExecutor
import time
import kuavo_vr_events_pb2
import threading
import netifaces

class UdpSenderForInfoToQuest3:
    def __init__(self, ports, message, width=0, height=0):
        self.ports = ports
        event = kuavo_vr_events_pb2.KuavoVrEvents()
        event.webrtc_signaling_url = message
        event.camera_info.width = width
        event.camera_info.height = height
        self.message = event.SerializeToString()
        self.stop_event = threading.Event()
        self.broadcast_ips = self.get_local_broadcast_ips()
        print("--------------------------------")    
        print(f"Local broadcast IPs:")
        for ip in self.broadcast_ips:
            print(f"  {ip}")
        print("--------------------------------")    

    def _send_udp_message(self, port):
        if len(self.broadcast_ips) == 0:
            print(f"No broadcast IPs found....")
            return
        for broadcast_ip in self.broadcast_ips:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    sock.settimeout(1)  # Set a timeout of 1 second
                    sock.sendto(self.message, (broadcast_ip, port))
                    # data, addr = sock.recvfrom(1024)
                    # print(f"Response from {broadcast_ip}:{port}: {data.decode()}")
            except socket.timeout:
                # print(f"No response from {broadcast_ip}:{port}")
                pass
            except Exception as e:
                print(f"Error sending to {broadcast_ip}:{port}: {str(e)}")

    def _send_udp_message_periodically(self):
        while not self.stop_event.is_set():
            with ThreadPoolExecutor(max_workers=50) as executor:
                futures = [executor.submit(self._send_udp_message, port) for port in self.ports]
                for future in futures:
                    future.result()
            if self.stop_event.is_set():
                break
            time.sleep(1)  # Wait for 3 seconds before sending again

    def get_local_broadcast_ips(self)->list:
        """
        Gets a list of local IPv4 broadcast IP addresses for all active interfaces.
        Requires the 'netifaces' library to be imported.
        """
        broadcast_ips = []
        excluded_prefixes = ("docker", "br-", "veth")
        try:
            for iface_name in netifaces.interfaces():
                if any(iface_name.startswith(prefix) for prefix in excluded_prefixes):
                    continue
                if_addresses = netifaces.ifaddresses(iface_name)
                if netifaces.AF_INET in if_addresses:
                    for link_addr in if_addresses[netifaces.AF_INET]:
                        # Ensure 'broadcast' key exists and its value is not None or empty
                        if 'broadcast' in link_addr and link_addr['broadcast']:
                            broadcast_ips.append(link_addr['broadcast'])
            # Return unique broadcast IPs, sorted for consistency
            return sorted(list(set(broadcast_ips)))
        except Exception as e: # Catch any error during netifaces operations
            print(f"Error getting broadcast IPs using 'netifaces': {e}. Ensure 'netifaces' is installed and network interfaces are configured correctly.")
            return []
    def start(self):
        self.thread = threading.Thread(target=self._send_udp_message_periodically)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join()  # Wait for the thread to finish
