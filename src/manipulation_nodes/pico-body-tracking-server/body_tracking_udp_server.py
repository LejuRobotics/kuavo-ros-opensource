import socket
import numpy as np
from datetime import datetime
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion, quaternion_from_euler

import tf
import rospy
import os
import subprocess
import json
import argparse
import signal
import queue
import threading
from brodcast import broadcast_robot_info

def signal_handler(sig, frame):
    rospy.signal_shutdown("Ctrl+C pressed!")

body_tracker_role = [    
    "Pelvis",
    "LEFT_HIP",
    "RIGHT_HIP",
    "SPINE1",
    "LEFT_KNEE",
    "RIGHT_KNEE",
    "SPINE2",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
    "SPINE3",
    "LEFT_FOOT",
    "RIGHT_FOOT",
    "NECK",
    "LEFT_COLLAR",
    "RIGHT_COLLAR",
    "HEAD",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "LEFT_ELBOW",
    "RIGHT_ELBOW",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_HAND",
    "RIGHT_HAND",
]

align_to_robot_urdf_joints = [
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "LEFT_ELBOW",
    "RIGHT_ELBOW",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_HAND",
    "RIGHT_HAND"
]

left_arm_idxs = [body_tracker_role.index(name) for name in [
    "LEFT_SHOULDER",
    "LEFT_ELBOW",
    "LEFT_WRIST",
    "LEFT_HAND"
]]

right_arm_idxs = [body_tracker_role.index(name) for name in [
    "RIGHT_SHOULDER",
    "RIGHT_ELBOW",
    "RIGHT_WRIST",
    "RIGHT_HAND"
]]


T_unity_2_ros = np.array([[0, 0, -1, 0],
                           [-1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]])

T_ros_2_robot_urdf = np.array([[ 0, -1,  0,  0],
                              [ 0,  0,  1,  0],
                              [-1,  0,  0,  0],
                              [ 0,  0,  0,  1]])


def homogeneous_matrix_roll(angle_degrees):
    """
    Generates a 4x4 homogeneous transformation matrix for a rotation around the X-axis (Roll).

    Args:
        angle_degrees (float): The rotation angle in degrees. Can be positive or negative.

    Returns:
        numpy.ndarray: A 4x4 homogeneous transformation matrix.
    """
    angle_radians = np.radians(angle_degrees)
    c = np.cos(angle_radians)
    s = np.sin(angle_radians)

    homogeneous_matrix = np.array([[1, 0, 0, 0],
                                     [0, c, -s, 0],
                                     [0, s, c, 0],
                                     [0, 0, 0, 1]])
    return homogeneous_matrix

T_left_arm_correction = homogeneous_matrix_roll(90)
T_right_arm_correction = homogeneous_matrix_roll(-90)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--record_bag', type=bool, default=False, help='Record bag file')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='UDP server host')
    parser.add_argument('--port', type=int, default=12345, help='UDP server port')
    return parser.parse_args()

class BodyTrackingUDPServer:
    def __init__(self, *args, **kwargs):
        self.host = kwargs.get('host', '0.0.0.0')
        self.port = kwargs.get('port', 12345)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.cached_matrices = np.tile(np.eye(4), (len(body_tracker_role), 1, 1))
        self.data_queue = queue.Queue(maxsize=5)
        self.process_data_thread = threading.Thread(target=self.process_data_thread, daemon=True)
        self.process_data_thread.start()
        self.broadcast_robot_info_thread = threading.Thread(target=broadcast_robot_info, daemon=True)
        self.broadcast_robot_info_thread.start()
        rospy.loginfo(f"Body Tracking UDP Server listening on {self.host}:{self.port}")


    def clean_up(self):
        """Cleanup when the object is destroyed."""
        self.socket.close()
        rospy.loginfo("Stopped UDP server")

    def get_pose_from_matrix(self, matrix):
        """Get pose from matrix."""
        pos = matrix[:3, 3]
        quat = quaternion_from_matrix(matrix)
        return pos, quat
    
    def parse_transform_data_to_matrix(self, data_str):
        """Parse JSON format transform data and return a [len(body_tracker_role), 4, 4] matrix."""
        transforms = self.parse_transform_data(data_str)
        if not transforms:
            return None
        
        for i, transform in enumerate(transforms):
            self.cached_matrices[i][:3, :3] = quaternion_matrix(transform['rotation'])[:3, :3]
            self.cached_matrices[i][:3, 3] = transform['position']

        return self.cached_matrices
    
    def transform_matrix_to_ros(self, matrix):
        """Transform matrix to Robot pose."""
        ros_matrices = np.matmul(T_unity_2_ros, matrix)
        ros_matrices = np.matmul(ros_matrices, T_ros_2_robot_urdf)

        # reverse x position
        ros_matrices[:,0,3] = - ros_matrices[:,0,3]
 
        # reverse y and z rotation
        ros_matrices[:, :3, :3] = np.array([
            quaternion_matrix(
                quaternion_from_euler(
                    *(np.array(euler_from_quaternion(quaternion_from_matrix(matrix))) * np.array([1, -1, -1]))
                )
            )[:3, :3] 
            for matrix in ros_matrices
        ])

        # align to robot urdf
        for idx in left_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_left_arm_correction
        for idx in right_arm_idxs:
            ros_matrices[idx] = ros_matrices[idx] @ T_right_arm_correction
        return ros_matrices

    def parse_transform_data(self, data_str):
        """Parse JSON format transform data."""
        try:
            data = json.loads(data_str)
            transforms = []
            
            for i, pose in enumerate(data['poses']):
                transform = {
                    'role': i,
                    'position': np.array(pose[0:3]),  
                    'rotation': np.array(pose[3:7])   
                }
                transforms.append(transform)
            return transforms
        except Exception as e:
            rospy.logerr(f"Error parsing JSON data: {e}")
            rospy.logerr(f"data: {data}")
            return None

    def process_data_thread(self):
        while not rospy.is_shutdown():
            try:
                data_str = self.data_queue.get(timeout=0.01)
                matrices = self.parse_transform_data_to_matrix(data_str)
                if matrices is None or matrices.size == 0:
                    rospy.logwarn("Received invalid matrices data")
                    continue
                
                robot_urdf_matrices = self.transform_matrix_to_ros(matrices)
                current_time = rospy.Time.now()

                for i, matrix in enumerate(robot_urdf_matrices):
                    role = i
                    try:
                        pos, quat = self.get_pose_from_matrix(matrix)
                    except Exception as e:
                        rospy.logerr(f"Error getting pose from matrix: {e}")
                        continue
                    self.tf_broadcaster.sendTransform(
                        pos,
                        quat,
                        current_time,
                        body_tracker_role[role],
                        "world"
                    )
                    rospy.loginfo(f"Role {body_tracker_role[role]}:")
                    rospy.loginfo(f"  Position: {pos}")
                    rospy.loginfo(f"  Rotation: {euler_from_quaternion(quat)}")
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error processing data: {e}")

    def start(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.socket.recvfrom(4096)
                data_str = data.decode('utf-8')
                self.data_queue.put_nowait(data_str)
            except queue.Full:
                try:
                    self.data_queue.get_nowait()
                    self.data_queue.put_nowait(data_str)
                except queue.Empty:
                    pass
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Error receiving data: {e}")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    args = parse_args()
    rospy.init_node('body_tracking_subscriber')
    try:
        server = BodyTrackingUDPServer(**vars(args))
        server.start()
    except rospy.ROSInterruptException:
        server.clean_up()
        rospy.loginfo("Node interrupted")
