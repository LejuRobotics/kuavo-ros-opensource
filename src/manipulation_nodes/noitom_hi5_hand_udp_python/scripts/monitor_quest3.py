#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import sys
import os
import json
import time
import signal
import rospy
import tf
import numpy as np
import math
import netifaces
from pprint import pprint
from kuavo_ros_interfaces.msg import robotHeadMotionData
from noitom_hi5_hand_udp_python.msg import PoseInfoList, PoseInfo
from kuavo_msgs.msg import JoySticks
from geometry_msgs.msg import Point, Quaternion
import threading
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
# Add the parent directory to the system path to allow relative imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

# Import the hand_pose_pb2 module
import protos.hand_pose_pb2 as event_pb2
import protos.robot_info_pb2 as robot_info_pb2

class Quest3BoneFramePublisher:
    def __init__(self):
        self.bone_names = [
            "LeftArmUpper", "LeftArmLower", "RightArmUpper", "RightArmLower",
            "LeftHandPalm", "RightHandPalm", "LeftHandThumbMetacarpal",
            "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
            "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
            "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
            "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
            "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip",
            "Root", "Chest", "Neck", "Head"
        ]

        self.exit_listen_thread_for_quest3_broadcast = False
        self.bone_name_to_index = {name: index for index, name in enumerate(self.bone_names)}
        self.index_to_bone_name = {index: name for index, name in enumerate(self.bone_names)}
        
        self.current_file_dir = os.path.dirname(os.path.abspath(__file__))
        self.CONFIG_FILE = os.path.join(self.current_file_dir, "config.json")
        self.calibrated_head_quat_matrix_inv = None
        self.head_motion_range = self.get_head_motion_range()
        
        self.sock = None
        self.server_address = None
        self.port = None

        self.listening_udp_ports_cnt = 0
        
        rospy.init_node('Quest3_bone_frame_publisher', anonymous=True)
        self.rate = rospy.Rate(100.0)
        
        self.br = tf.TransformBroadcaster()
        self.pose_pub = rospy.Publisher('/leju_quest_bone_poses', PoseInfoList, queue_size=2)
        self.head_data_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
        self.joysticks_pub = rospy.Publisher('quest_joystick_data', JoySticks, queue_size=2)
        
        self.listener = tf.TransformListener()
        self.hand_finger_tf_pub = rospy.Publisher('/quest_hand_finger_tf', TFMessage, queue_size=10)

        self.enable_head_control = rospy.get_param("~enable_head_control", True)
        rospy.loginfo(f"enable_head_control: {self.enable_head_control}")
        signal.signal(signal.SIGINT, self.signal_handler)
        self.broadcast_ips = []
        self.robot_info_sent_initial_broadcast = False
        self.robot_info_lock = threading.Lock()

    def update_broadcast_ips(self, ips_list):
        """Updates the list of broadcast IPs."""
        if isinstance(ips_list, list):
            self.broadcast_ips = sorted(list(set(ips_list))) # Store unique sorted IPs
            rospy.loginfo(f"Updated broadcast IPs to: {self.broadcast_ips}")
        else:
            rospy.logwarn("Failed to update broadcast IPs: input is not a list.")

    def send_robot_info_on_broadcast_ips(self, robot_name, robot_version, start_port, end_port):
        """Sends RobotDescription protobuf message to all broadcast IPs within a port range."""
        if not self.broadcast_ips:
            rospy.logwarn("No broadcast IPs configured. Cannot send robot info.")
            return
        rospy.loginfo(f"Broadcasting robot info: Name='{robot_name}', Version={robot_version}, "
                      f"Ports={start_port}-{end_port} to IPs: {self.broadcast_ips}")
        robot_desc = robot_info_pb2.RobotDescription()
        robot_desc.robot_name = robot_name
        robot_desc.robot_version = robot_version
        serialized_message = robot_desc.SerializeToString()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as send_sock:
            send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            for ip in self.broadcast_ips:
                for port in range(start_port, end_port + 1):
                    try:
                        send_sock.sendto(serialized_message, (ip, port))
                        rospy.logdebug(f"Sent robot info to {ip}:{port}")
                    except Exception as e:
                        rospy.logerr(f"Error sending robot info to {ip}:{port}: {e}")
 
    def load_config(self):
        if os.path.exists(self.CONFIG_FILE):
            with open(self.CONFIG_FILE, 'r') as f:
                return json.load(f)
        return {}

    def save_config(self, config):
        with open(self.CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)

    def get_head_motion_range(self):
        config = self.load_config()
        return config.get("head_motion_range", None)

    def signal_handler(self, sig, frame):
        print('Exiting gracefully...')
        self.exit_listen_thread_for_quest3_broadcast = True
        if self.sock:
            self.sock.close()
        sys.exit(0)

    def setup_socket(self, server_address, port):
        if self.sock is not None:
            print("Socket is already established, skip creating a new one.")
        else:
            self.server_address = (server_address, port)
            self.port = port
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(1)
        return (server_address, port)


    def send_initial_message(self):
        message = b'hi'
        max_retries = 200
        for attempt in range(max_retries):
            try:
                self.sock.sendto(message, self.server_address)
                self.sock.recvfrom(1024)
                print(f"\033[92mAcknowledgment From Quest3 received on attempt {attempt + 1}, start to receiving data...\033[0m")
                return True
            except socket.timeout:
                print(f"\033[91mQuest3_timeout: Attempt {attempt + 1} timed out. Retrying...\033[0m")
            except KeyboardInterrupt:
                print("Force quit by Ctrl-c.")
                self.signal_handler(signal.SIGINT, None)
        print("Failed to send message after 200 attempts.")
        return False

    def convert_position_to_right_hand(self, left_hand_position):
        return {
            "x": 0 - left_hand_position["z"],
            "y": 0 - left_hand_position["x"],
            "z": left_hand_position["y"]
        }

    def convert_quaternion_to_right_hand(self, left_hand_quat):
        return (
            0 - left_hand_quat[2],
            0 - left_hand_quat[0],
            left_hand_quat[1],
            left_hand_quat[3]
        )

    def updateAFrame(self, frame_name, frame_position, frame_rotation_quat, time_now):
        self.br.sendTransform((frame_position["x"], frame_position["y"], frame_position["z"]), 
                              frame_rotation_quat, time_now, frame_name, "torso")
        
    def update_quest_hand_finger_tf(self):
        tf_msg = TFMessage()
        for frame_name in [
                "LeftHandPalm", "RightHandPalm","LeftHandThumbMetacarpal",
                "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
                "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip",
                "LeftHandLittleTip", "RightHandThumbMetacarpal", "RightHandThumbProximal",
                "RightHandThumbDistal", "RightHandThumbTip", "RightHandIndexTip",
                "RightHandMiddleTip", "RightHandRingTip", "RightHandLittleTip"
            ]:
                try:
                    if "Left" in frame_name:
                        relative_position, relative_rotation = self.listener.lookupTransform("LeftHandPalm", frame_name, rospy.Time(0))
                    else:   
                        relative_position, relative_rotation = self.listener.lookupTransform("RightHandPalm", frame_name, rospy.Time(0))
                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "LeftHandPalm" if "Left" in frame_name else "RightHandPalm"
                    transform.child_frame_id = frame_name
                    transform.transform.translation.x = relative_position[0]
                    transform.transform.translation.y = relative_position[1]
                    transform.transform.translation.z = relative_position[2]
                    transform.transform.rotation.x = relative_rotation[0]
                    transform.transform.rotation.y = relative_rotation[1]
                    transform.transform.rotation.z = relative_rotation[2]
                    transform.transform.rotation.w = relative_rotation[3]
                    tf_msg.transforms.append(transform)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF lookup failed: {e}")
                    return
        self.hand_finger_tf_pub.publish(tf_msg)

    def normalize_degree_in_180(self, degree):
        if degree > 180:
            degree -= 180
        elif degree < -180:
            degree += 180
        return degree

    def pub_head_motion_data(self, cur_quat):
        try:
            # Get the transform from Chest to Head using TF listener
            (trans, rot) = self.listener.lookupTransform("Chest", "Head", rospy.Time(0))
            
            # Convert quaternion to euler angles (roll, pitch, yaw)
            rpy = tf.transformations.euler_from_quaternion(rot)
            rpy_deg = [r * 180 / math.pi for r in rpy]
            
            # Extract pitch (around X-axis) and yaw (around Y-axis)
            pitch = max(min(self.normalize_degree_in_180(round(rpy_deg[0], 2)), self.head_motion_range["pitch"][1]), self.head_motion_range["pitch"][0])
            yaw = max(min(self.normalize_degree_in_180(round(rpy_deg[1], 2)), self.head_motion_range["yaw"][1]), self.head_motion_range["yaw"][0])
            
            msg = robotHeadMotionData()
            msg.joint_data = [yaw, pitch]
            self.head_data_pub.publish(msg)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed for Chest->Head transform: {e}")
            return

    def run(self):
        loop_count = 0
        while not rospy.is_shutdown():
            try:
                loop_count += 1
                data, _ = self.sock.recvfrom(4096)
                event = event_pb2.LejuHandPoseEvent()
                event.ParseFromString(data)
                
                time_now = rospy.Time.now()
                pose_info_list = PoseInfoList()
                joysticks_msg = JoySticks()
                
                # Process joystick data
                self.process_joystick_data(event, joysticks_msg, loop_count)
                
                # Process pose data
                self.process_pose_data(event, pose_info_list, time_now)
                
                # Publish data
                pose_info_list.timestamp_ms = event.timestamp
                pose_info_list.is_high_confidence = event.IsDataHighConfidence
                pose_info_list.is_hand_tracking = event.IsHandTracking
                self.pose_pub.publish(pose_info_list)
                
                self.rate.sleep()
            except socket.timeout:
                print('Timeout occurred, no data received. Restarting socket...')
                if not self.restart_socket():
                    break
            except Exception as e:
                print(f'An error occurred: {e}')
                if not self.restart_socket():
                    break

    def process_joystick_data(self, event, joysticks_msg, loop_count):
        joysticks_msg.left_x = event.left_joystick.x
        joysticks_msg.left_y = event.left_joystick.y
        joysticks_msg.left_trigger = event.left_joystick.trigger
        joysticks_msg.left_grip = event.left_joystick.grip
        joysticks_msg.left_first_button_pressed = event.left_joystick.firstButtonPressed
        joysticks_msg.left_second_button_pressed = event.left_joystick.secondButtonPressed
        joysticks_msg.left_first_button_touched = event.left_joystick.firstButtonTouched
        joysticks_msg.left_second_button_touched = event.left_joystick.secondButtonTouched
        joysticks_msg.right_x = event.right_joystick.x
        joysticks_msg.right_y = event.right_joystick.y
        joysticks_msg.right_trigger = event.right_joystick.trigger
        joysticks_msg.right_grip = event.right_joystick.grip
        joysticks_msg.right_first_button_pressed = event.right_joystick.firstButtonPressed
        joysticks_msg.right_second_button_pressed = event.right_joystick.secondButtonPressed
        joysticks_msg.right_first_button_touched = event.right_joystick.firstButtonTouched
        joysticks_msg.right_second_button_touched = event.right_joystick.secondButtonTouched
        # if loop_count % 20 == 0:
            # rospy.loginfo(joysticks_msg)
        self.joysticks_pub.publish(joysticks_msg)

    def process_pose_data(self, event, pose_info_list, time_now):
        scale_factor = {"x": 3.0, "y": 3.0, "z": 3.0}
        for i, pose in enumerate(event.poses):
            bone_name = self.index_to_bone_name[i]
            frame_position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
            frame_rotation_quat = (pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w)
            
            right_hand_position = self.convert_position_to_right_hand(frame_position)
            right_hand_quat = self.convert_quaternion_to_right_hand(frame_rotation_quat)
            
            pose_info = PoseInfo()
            pose_info.position = Point(x=right_hand_position["x"], y=right_hand_position["y"], z=right_hand_position["z"])
            pose_info.orientation = Quaternion(x=right_hand_quat[0], y=right_hand_quat[1], z=right_hand_quat[2], w=right_hand_quat[3])
            pose_info_list.poses.append(pose_info)
            
            for axis in ["x", "y", "z"]:
                right_hand_position[axis] *= scale_factor[axis]

            self.updateAFrame(bone_name, right_hand_position, right_hand_quat, time_now)


            if bone_name == "Head" and self.enable_head_control:
                self.pub_head_motion_data(right_hand_quat)

    def restart_socket(self):
        print("Restarting socket connection...")
        self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        if not self.send_initial_message():
            print("Failed to restart socket connection.")
            return False
        print("Socket connection restarted successfully.")
        return True

    def _periodic_robot_info_broadcaster(self):
        """Periodically broadcasts robot info while waiting for Quest3."""
        rospy.loginfo("Starting periodic robot info broadcaster thread (kuavo, 0, ports 11050-11060).")
        robot_version = int(rospy.get_param('/robot_version', 45))
        while not self.exit_listen_thread_for_quest3_broadcast and not rospy.is_shutdown():
            if not self.broadcast_ips:
                rospy.logwarn_throttle(10, "_periodic_robot_info_broadcaster: broadcast_ips is empty. Robot info will not be sent until IPs are updated.")

            self.send_robot_info_on_broadcast_ips("kuavo", robot_version, 11050, 11060)
            # Sleep for 1 second, but check the exit condition more frequently
            # to allow faster shutdown if needed.
            for _ in range(10): # Check every 0.1 seconds
                if self.exit_listen_thread_for_quest3_broadcast or rospy.is_shutdown():
                    break
                time.sleep(0.1)
        rospy.loginfo("Stopping periodic robot info broadcaster thread.")

    def broadcast_robot_info_and_wait_for_quest3(self):
        """Broadcasts robot information and waits for a Quest3 device to connect."""
        start_port = 11000
        end_port = 11010
        threads = []

        # Start the periodic robot info broadcaster thread
        periodic_broadcaster_thread = threading.Thread(target=self._periodic_robot_info_broadcaster)
        periodic_broadcaster_thread.daemon = False # Ensure it completes before program exit if main threads finish
        periodic_broadcaster_thread.start()
 
        for port in range(start_port, end_port + 1):
            thread = threading.Thread(target=self.listen_for_quest3_broadcasts, args=(port,))
            thread.daemon = False  # Set as non-daemon thread to wait for threads to finish
            thread.start()
            threads.append(thread)

        import os

        if self.listening_udp_ports_cnt == 0:
            print("\033[91m" + "carlos_ All UDP broadcast ports are occupied. Please check using the command 'lsof -i :11000-11010' to see the process which occupy the ports." + "\033[0m")
            os._exit(1)

        for thread in threads:
            thread.join()

        # Wait for the periodic broadcaster thread to finish as well
        periodic_broadcaster_thread.join()

        if self.server_address and self.server_address[0]:
            print(f"\033[92mQuest3 device found at IP: {self.server_address[0]}\033[0m")
        print("\033[92m" + "Received Quest3 Broadcast, starting to connect." + "\033[0m")

    def listen_for_quest3_broadcasts(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', port))  # Listen on all interfaces
        except OSError as e:
            pass
            return
        sock.settimeout(1)  # Set timeout to 1 second
        self.listening_udp_ports_cnt += 1

        while not self.exit_listen_thread_for_quest3_broadcast:
            try:
                data, addr = sock.recvfrom(1024)
                self.exit_listen_thread_for_quest3_broadcast = True
                print(f"carlos_ Received message from Quest3: {data.decode()} from {addr[0]} on port {port} - Setting up socket connection")
                self.setup_socket(addr[0], 10019)
                break
            except socket.timeout:
                continue

def get_local_broadcast_ips():
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
        rospy.logerr(f"Error getting broadcast IPs using 'netifaces': {e}. Ensure 'netifaces' is installed and network interfaces are configured correctly.")
        return []

if __name__ == "__main__":
    publisher = Quest3BoneFramePublisher()

    broadcast_ips = get_local_broadcast_ips()
    print(f"Local broadcast IPs: {broadcast_ips}")

    publisher.update_broadcast_ips(broadcast_ips)
    if len(sys.argv) < 2 or "." not in sys.argv[1]:
        print("IP not specified. Waiting for Quest3 to connect. Please ensure Quest3 and the robot are on the same LAN and the router has broadcast mode enabled.\n未指定IP。正在等待 Quest3 主动连接。请确保 Quest3 和机器人在同一个局域网下，并且路由器已开启广播模式。")
        publisher.broadcast_robot_info_and_wait_for_quest3()
    else:
        try:

            if ':' in sys.argv[1]:
                server_address, port = sys.argv[1].split(':')
                port = int(port)
            else:
                server_address = sys.argv[1]
                port = 10019

        except ValueError:
            print("Argument must be in the format <server_address[:port]> and port must be an integer")
            sys.exit(1)


        publisher.setup_socket(server_address, port)

    if publisher.send_initial_message():
        publisher.run()
    else:
        print("Failed to establish initial connection.")
