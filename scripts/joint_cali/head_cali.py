#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pydrake
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.common import FindResourceOrThrow

from pydrake.multibody.tree import BodyIndex
from pydrake.geometry import FramePoseVector
from pydrake.common.value import Value

import numpy as np
import rospy
# print("pydrake version:", pydrake.__version__)
# arg
import argparse

from std_msgs.msg import Float64MultiArray
from kuavo_msgs.msg import sensorsData
from kuavo_msgs.srv import setTagId
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
import yaml
import base64
import struct

import os
import rospkg

from kuavo_msgs.msg import robotHeadMotionData


def get_package_path(package_name):
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None

def modify_arm_zero_yaml(yaml_file_path, delta_q_head):
    # 读取 YAML 文件
    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)

    # 备份原始文件
    yaml_backup_path = yaml_file_path + ".head_cali.bak"
    with open(yaml_backup_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False, allow_unicode=True)
        print(f"YAML backup saved to {yaml_backup_path}")

    # 检查是否存在 arms_zero_position
    if "arms_zero_position" in data:
        # 获取倒数第二和倒数第一个值
        second_last_value = data["arms_zero_position"][-2]
        last_value = data["arms_zero_position"][-1]

        # 解析 NumPy 的二进制格式（如果存在）
        def parse_numpy_scalar(value):
            if isinstance(value, dict) and "!!binary |" in value:
                binary_data = base64.b64decode(value["!!binary |"])
                return struct.unpack('<d', binary_data)[0]
            return value

        second_last_value = parse_numpy_scalar(second_last_value)
        last_value = parse_numpy_scalar(last_value)

        # 修改倒数第二和倒数第一个值
        data["arms_zero_position"][-2] = second_last_value + float(delta_q_head[0])
        data["arms_zero_position"][-1] = last_value + float(delta_q_head[1])

        # 打印修改后的值
        print("Modified values:")
        print(f"Second last value: {data['arms_zero_position'][-2]}")
        print(f"Last value: {data['arms_zero_position'][-1]}")

        # 将修改后的内容写回 YAML 文件
        with open(yaml_file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False, allow_unicode=True)
    else:
        print("arms_zero_position key not found in the YAML file.")


class HeadCali:
    def __init__(self, urdf_path):
        self.plant = MultibodyPlant(0.0)
        Parser(self.plant).AddModelFromFile(urdf_path)

        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("dummy_link"))
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()
        print("nq:", self.plant.num_positions())
        print("nv:", self.plant.num_velocities())
        print("nf:", self.plant.num_actuated_dofs())


    def get_camera_pose(self, q1, q2):
        nq = self.plant.num_positions()
        q = np.zeros(nq)
        q[-2] = q1
        q[-1] = q2
        self.plant.SetPositions(self.context, q)
        # fk
        camera_pose_in_base = self.plant.GetFrameByName("camera_base").CalcPose(
            self.context, self.plant.GetFrameByName("base_link")
        )
        return camera_pose_in_base

class HeadCaliRosNode:
    def __init__(self, urdf_path, tag_id):
        self.head_cali = HeadCali(urdf_path)
        self.q_head = np.zeros(2)
        self.odom_pos = np.zeros(3)
        self.odom_orientation = np.zeros(4)
        self.tag_pos = np.zeros(3)
        self.tag_orientation = np.zeros(4)
        self.tag_id = tag_id

        self.q_head_updated = False
        self.tag_pos_updated = False
        self.odom_updated = False

        self.q_head_sub = rospy.Subscriber("/sensor_data_motor/motor_pos", Float64MultiArray, self.sensor_data_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_pos_callback)
        self.head_motion_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

    def control_head(self, yaw, pitch):
        yaw = float(yaw)
        pitch = float(pitch)
        msg = robotHeadMotionData()
        msg.joint_data = [yaw, pitch]
        rospy.loginfo(f"Moving head to yaw={yaw}°, pitch={pitch}°")
        for _ in range(3):
            self.head_motion_pub.publish(msg)
            rospy.sleep(0.1)

    def get_camera_pos_in_world(self, q1, q2):
        camera_pose_in_base = self.head_cali.get_camera_pose(q1, q2)
        p_bc = camera_pose_in_base.translation()
        R_bc = camera_pose_in_base.rotation().matrix()
        # print("p_bc:", p_bc)
        # print("R_bc:\n", np.asarray(R_bc))
        
        p_wb = self.odom_pos
        R_wb = self.quat_to_rot(self.odom_orientation) # TODO:
        p_wc = p_wb + R_wb @ p_bc

        p_cColor = np.array([0.009, 0.032, 0.013])
        R_cColor = self.quat_to_rot([-0.500, 0.502, -0.495, 0.504])
        p_cColor_w = R_wb @ R_bc @ p_cColor
        p_wColor = p_wc + p_cColor_w
        R_wColor = R_wb @ R_bc @ R_cColor

        p_colorT = self.tag_pos
        # R_Colort = self.quat_to_rot(self.tag_orientation)
        p_colorT_w = R_wColor @ p_colorT
        p_wt = p_wColor + p_colorT_w

        return p_wt

    def get_camera_pos_in_base(self, q1, q2):
        camera_pose_in_base = self.head_cali.get_camera_pose(q1, q2)
        p_bc = camera_pose_in_base.translation()
        R_bc = camera_pose_in_base.rotation().matrix()
        # print("p_bc:", p_bc)
        # print("R_bc:\n", np.asarray(R_bc))
        p_cColor = np.array([0.009, 0.032, 0.013])
        R_cColor = self.quat_to_rot([-0.500, 0.502, -0.495, 0.504])
        p_cColor_b = R_bc @ p_cColor
        p_bColor = p_bc + p_cColor_b
        R_bColor = R_bc @ R_cColor

        p_colorT = self.tag_pos
        # R_Colort = self.quat_to_rot(self.tag_orientation)
        p_colorT_b = R_bColor @ p_colorT
        p_bt = p_bColor + p_colorT_b

        return p_bt

    def sensor_data_callback(self, msg):
        self.q_head = np.array([msg.data[-2], msg.data[-1]])
        self.q_head_updated = True
        # print("q_head:", self.q_head)
        # print("head_cali callback")

    def odom_callback(self, msg):
        # print("odom_callback")
        self.odom_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.odom_orientation = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
        # print("odom_pos:", self.odom_pos)
        self.odom_updated = True
    
    def tag_pos_callback(self, msg):
        # print("tag_pos_callback")
        # print("msg.detections: ", msg.detections)
        for detection in msg.detections:
            # print("detection.id:", detection.id[0])
            pose = detection.pose.pose.pose
            # print("detection.pose.pose.pose:\n", pose)
            # print(type(detection.id[0]))
            if detection.id[0] == self.tag_id:
                self.tag_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                self.tag_orientation = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
                # print("tag_pos:", self.tag_pos)
                self.tag_pos_updated = True


    @staticmethod
    def bin_search(func, target_value, low, high, error_threshold=0.002, max_iter=100):
        for i in range(max_iter):
            mid = (low + high) / 2.0
            value = func(mid)
            error = abs(value - target_value)
            print(f"iter:{i+1}, mid:{mid:.4f}, value:{value:.4f}, error:{error:.4f}")
            if error < error_threshold:
                return mid
            elif value < target_value:
                high = mid
            else:
                low = mid
        return mid
    @staticmethod
    def quat_to_rot(q):
        w, x, y, z = q
        R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                      [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                      [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        return R

    def cali_head(self, target_pose, base_pose: bool):
        print(f"frame: {'Base' if base_pose else 'World'}")
        print("waiting for q_head...")
        while self.q_head_updated == False:
            rospy.sleep(0.1)
        print("q_head is updated.")
        print("q_head:", self.q_head)

        print("waiting for odom...")
        while self.odom_updated == False:
            rospy.sleep(0.1)
        print("odom is updated.")
        print("odom_pos:", self.odom_pos)
        
        print(f"waiting for tag_pos(id={self.tag_id})...")
        while self.tag_pos_updated == False:
            rospy.sleep(0.1)
        print(f"tag_pos(id={self.tag_id}) is updated.")
        print("tag_pos:", self.tag_pos)

        # init tag pos in world
        print("init tag pos in world: ", self.get_camera_pos_in_world(self.q_head[0], self.q_head[1]))
        print("init tag pos in base:  ", self.get_camera_pos_in_base(self.q_head[0], self.q_head[1]))
        # return

        # find q1*
        bias = 0.5
        
        q1_low = self.q_head[0] + bias
        q1_high = self.q_head[0] - bias
        q2_low = self.q_head[1] - bias
        q2_high = self.q_head[1] + bias
        print(f"q original: {self.q_head}")
        camera_pos = None
        if not base_pose:
            q1_star = self.bin_search(lambda q1: self.get_camera_pos_in_world(q1, self.q_head[1])[1], 
                                    target_value=target_pose[1], low=q1_low, high=q1_high)
            # print(f"q1*:{q1_star:.4f}")
            # find q2*
            q2_star = self.bin_search(lambda q2: self.get_camera_pos_in_world(self.q_head[0], q2)[2],
                                    target_value=target_pose[2], low=q2_low, high=q2_high)
        else:
            q1_star = self.bin_search(lambda q1: self.get_camera_pos_in_base(q1, self.q_head[1])[1], 
                                    target_value=target_pose[1], low=q1_low, high=q1_high)
            # find q2*
            q2_star = self.bin_search(lambda q2: self.get_camera_pos_in_base(self.q_head[0], q2)[2],
                                    target_value=target_pose[2], low=q2_low, high=q2_high)
        print(f"q*: [{q1_star:.4f}, {q2_star:.4f}]")
        return -(np.array([q1_star, q2_star]) - self.q_head)


def main():
    parser = argparse.ArgumentParser(description='head_cali')
    parser.add_argument('--use_cali_tool', action='store_true', help='使用安装在躯干上的标定工具，如果为True，则使用配置文件中的参数值，如果为False, 则需要手动指定tag_id以及target_pose.')
    parser.add_argument('--target_pose', type=float, nargs='+', help='目标点在world坐标系(或者base坐标系)下的坐标, x y z,输入时候用空格隔开.如果use_cali_tool为True, 则目标点在base坐标系下, 该值将使用config/head_cali_config.yaml中的默认值.')
    parser.add_argument('--tag_id', type=int, help='Apriltag的id编号.如果use_cali_tool为True, 则tag_id将使用config/head_cali_config.yaml中的默认值.')

    use_cali_tool = parser.parse_args().use_cali_tool
    target_pose = np.array(parser.parse_args().target_pose)
    tag_id = parser.parse_args().tag_id

    if use_cali_tool:
        print("Loading config/head_cali_config.yaml")
        with open(os.path.join(os.path.dirname(__file__), "config/head_cali_config.yaml"), "r") as f:
            config = yaml.safe_load(f)
        tag_id = config["tag_id"]
        target_pose = config["tag_pose_in_base"]
    if target_pose is None:
        print("target_pose should be specified")
        return
    if tag_id is None:
        print("tag_id should be specified")
        return
    if target_pose is not None and len(target_pose)!= 3:
        print("target_pose should be 3-dimensional")        
        return
    print("target_pose:", target_pose)
    print("tag_id:", tag_id)
    print("use_cali_tool:", use_cali_tool)
    asset_path = get_package_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
    print(f"urdf_path: {urdf_path}")

    head_cali = HeadCaliRosNode(urdf_path, tag_id)
    # camera_pos_in_base = head_cali.get_camera_pos_in_world(0.0, 0.0)
    # print("p0: ", camera_pos_in_base)

    # ctrl c
    import signal
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    # ros
    rospy.init_node('head_cali', anonymous=True)
    if use_cali_tool:
        head_cali.control_head(0, 30)
    delta_head = head_cali.cali_head(target_pose, base_pose=use_cali_tool)
    print("delta_head:", delta_head)
    # 用户交互，决定是否写入文件
    # input("Press Enter to employ modified values..., or ctrl+c to exit.")
    input("按下回车键继续保存文件，或者ctrl+c退出")
    # load arm_zero.yaml file
    arm_zero_file_path = "/home/lab/.config/lejuconfig/arms_zero.yaml"
    modify_arm_zero_yaml(arm_zero_file_path, delta_head)
    print("Modified arms_zero.yaml file.")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
