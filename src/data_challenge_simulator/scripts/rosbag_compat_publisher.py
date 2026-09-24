#!/usr/bin/env python3
"""Publish passive rosbag compatibility topics from the running simulation."""

import math
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from kuavo_msgs.msg import (
    Float32MultiArrayStamped,
    sensorsData,
    twoArmHandPose,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse


ARM_COUNT = 14
TORSO_JOINT_NAMES = (
    "knee_joint", "leg_joint", "waist_pitch_joint", "waist_yaw_joint")
LEFT_ARM_JOINT_NAMES = tuple("zarm_l%d_joint" % index for index in range(1, 8))
RIGHT_ARM_JOINT_NAMES = tuple("zarm_r%d_joint" % index for index in range(1, 8))


def _vector(attribute, length, default=0.0):
    if not attribute:
        return np.full(length, default, dtype=float)
    values = [float(value) for value in attribute.split()]
    if len(values) != length:
        raise ValueError("expected %d values, got %d" % (length, len(values)))
    return np.asarray(values, dtype=float)


def _origin_matrix(element):
    matrix = np.identity(4)
    if element is None:
        return matrix
    xyz = _vector(element.get("xyz"), 3)
    rpy = _vector(element.get("rpy"), 3)
    matrix[:3, :3] = transformations.euler_matrix(*rpy)[:3, :3]
    matrix[:3, 3] = xyz
    return matrix


def _axis_rotation(axis, angle):
    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        return np.identity(4)
    return transformations.rotation_matrix(angle, axis / norm)


class UrdfForwardKinematics:
    def __init__(self, urdf_path, root_link="base_link"):
        self.root_link = root_link
        self.joints_by_child = {}
        root = ET.parse(urdf_path).getroot()
        for element in root.findall("joint"):
            parent = element.find("parent").get("link")
            child = element.find("child").get("link")
            self.joints_by_child[child] = {
                "name": element.get("name"),
                "type": element.get("type", "fixed"),
                "parent": parent,
                "origin": _origin_matrix(element.find("origin")),
                "axis": _vector(
                    None if element.find("axis") is None
                    else element.find("axis").get("xyz"),
                    3, default=0.0),
            }

    def chain(self, tip_link):
        result = []
        link = tip_link
        while link != self.root_link:
            if link not in self.joints_by_child:
                raise ValueError("no URDF chain from %s to %s" % (
                    self.root_link, tip_link))
            joint = self.joints_by_child[link]
            result.append(joint)
            link = joint["parent"]
        result.reverse()
        return tuple(result)

    @staticmethod
    def evaluate(chain, positions):
        transform = np.identity(4)
        for joint in chain:
            transform = np.dot(transform, joint["origin"])
            joint_type = joint["type"]
            value = positions.get(joint["name"], 0.0)
            if joint_type in ("revolute", "continuous"):
                transform = np.dot(transform, _axis_rotation(joint["axis"], value))
            elif joint_type == "prismatic":
                translation = np.identity(4)
                translation[:3, 3] = joint["axis"] * value
                transform = np.dot(transform, translation)
        return transform


class RosbagCompatPublisher:
    def __init__(self):
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.publish_cmd_vel_fallback = bool(
            rospy.get_param("~publish_cmd_vel_fallback", True))
        self.cmd_vel_publish_rate = float(
            rospy.get_param("~cmd_vel_publish_rate", 100.0))
        self.cmd_vel_timeout = float(
            rospy.get_param("~cmd_vel_timeout", 0.1))
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be a positive finite value")
        if (not math.isfinite(self.cmd_vel_timeout)
                or self.cmd_vel_timeout <= 0.0):
            raise ValueError("~cmd_vel_timeout must be positive and finite")
        if (not math.isfinite(self.cmd_vel_publish_rate)
                or self.cmd_vel_publish_rate <= 0.0):
            raise ValueError(
                "~cmd_vel_publish_rate must be positive and finite")
        self.lock = threading.Lock()
        self.sensor_joints = None
        self.arm_target = None
        self.last_external_cmd_vel_time = None
        self.last_external_cmd_vel_is_zero = False
        self.base_pose = None

        urdf_path = rospy.get_param("~robot_urdf")
        self.fk = UrdfForwardKinematics(urdf_path)
        self.left_chain = self.fk.chain("zarm_l7_end_effector")
        self.right_chain = self.fk.chain("zarm_r7_end_effector")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_vel_pub = None
        if self.publish_cmd_vel_fallback:
            self.cmd_vel_pub = rospy.Publisher(
                "/cmd_vel", Twist, queue_size=10)
        self.eef_pose_pub = rospy.Publisher(
            "/ik_fk_result/eef_pose", twoArmHandPose, queue_size=10)
        self.input_pos_pub = rospy.Publisher(
            "/ik_fk_result/input_pos", Float32MultiArrayStamped, queue_size=10)
        self.cmd_pose_world_pub = rospy.Publisher(
            "/cmd_pose_world", Twist, queue_size=1)

        rospy.Subscriber(
            "/sensors_data_raw", sensorsData,
            self._sensor_callback, queue_size=1)
        rospy.Subscriber(
            "/kuavo_arm_traj", JointState,
            self._arm_target_callback, queue_size=1)
        rospy.Subscriber(
            "/cmd_vel", Twist, self._cmd_vel_callback, queue_size=1)
        rospy.Subscriber(
            "/mujoco/base_link/pose", PoseStamped,
            self._base_pose_callback, queue_size=1)
        self.final_cmd_pose_service = rospy.Service(
            "/rosbag_compat/publish_current_base_command",
            Trigger, self._publish_current_base_command)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._publish)
        self.cmd_vel_timer = None
        if self.publish_cmd_vel_fallback:
            self.cmd_vel_timer = rospy.Timer(
                rospy.Duration(1.0 / self.cmd_vel_publish_rate),
                self._publish_cmd_vel)
        rospy.loginfo(
            "Rosbag compatibility publisher started: IK/FK topics "
            "at %.1f Hz; cmd_vel fallback at %.1f Hz",
            self.publish_rate, self.cmd_vel_publish_rate)

    def _sensor_callback(self, message):
        joints = list(message.joint_data.joint_q)
        if len(joints) < 18:
            rospy.logwarn_throttle(2.0, "sensors_data_raw has fewer than 18 joints")
            return
        with self.lock:
            self.sensor_joints = joints

    def _arm_target_callback(self, message):
        if len(message.position) < ARM_COUNT:
            rospy.logwarn_throttle(2.0, "kuavo_arm_traj has fewer than 14 positions")
            return
        with self.lock:
            self.arm_target = [
                math.radians(float(value)) for value in message.position[:ARM_COUNT]]

    @staticmethod
    def _is_zero_cmd_vel(message):
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        return all(
            math.isfinite(float(value)) and abs(float(value)) <= 1e-9
            for value in values)

    def _cmd_vel_callback(self, message):
        caller_id = getattr(message, "_connection_header", {}).get(
            "callerid", "")
        if (self.publish_cmd_vel_fallback
                and caller_id == rospy.get_name()):
            return
        with self.lock:
            self.last_external_cmd_vel_time = time.monotonic()
            self.last_external_cmd_vel_is_zero = self._is_zero_cmd_vel(message)

    def _base_pose_callback(self, message):
        with self.lock:
            self.base_pose = message.pose

    def _publish_current_base_command(self, _request):
        """Publish a no-motion world target after task completion for rosbag."""
        with self.lock:
            pose = self.base_pose
        if pose is None:
            return TriggerResponse(
                success=False,
                message="/mujoco/base_link/pose has not been received")

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        roll, pitch, yaw = transformations.euler_from_quaternion(quaternion)
        command = Twist()
        command.linear.x = pose.position.x
        command.linear.y = pose.position.y
        command.linear.z = pose.position.z
        command.angular.x = roll
        command.angular.y = pitch
        command.angular.z = yaw
        self.cmd_pose_world_pub.publish(command)
        return TriggerResponse(
            success=True,
            message="published current base pose as a no-motion world target")

    @staticmethod
    def _fill_pose(target, translation, rotation, joint_angles):
        target.pos_xyz = [
            translation.x, translation.y, translation.z]
        target.quat_xyzw = [
            rotation.x, rotation.y, rotation.z, rotation.w]
        target.joint_angles = joint_angles

    def _publish_eef_pose(self, stamp, sensor_joints):
        if sensor_joints is None:
            return
        try:
            left = self.tf_buffer.lookup_transform(
                "base_link", "zarm_l7_end_effector", rospy.Time(0),
                rospy.Duration(0.01)).transform
            right = self.tf_buffer.lookup_transform(
                "base_link", "zarm_r7_end_effector", rospy.Time(0),
                rospy.Duration(0.01)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        message = twoArmHandPose()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        self._fill_pose(
            message.left_pose, left.translation, left.rotation,
            sensor_joints[4:11])
        self._fill_pose(
            message.right_pose, right.translation, right.rotation,
            sensor_joints[11:18])
        self.eef_pose_pub.publish(message)

    @staticmethod
    def _matrix_pose(matrix):
        quaternion = transformations.quaternion_from_matrix(matrix)
        return list(matrix[:3, 3]) + list(quaternion)

    def _publish_input_pose(self, stamp, sensor_joints, arm_target):
        if sensor_joints is None or arm_target is None:
            return
        positions = dict(zip(TORSO_JOINT_NAMES, sensor_joints[:4]))
        positions.update(zip(LEFT_ARM_JOINT_NAMES, arm_target[:7]))
        positions.update(zip(RIGHT_ARM_JOINT_NAMES, arm_target[7:14]))
        left = self.fk.evaluate(self.left_chain, positions)
        right = self.fk.evaluate(self.right_chain, positions)

        message = Float32MultiArrayStamped()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.data.data = self._matrix_pose(left) + self._matrix_pose(right)
        self.input_pos_pub.publish(message)

    def _cmd_vel_fallback_command(self):
        """Fill external-command silence with a safe stationary command.

        Repeating the last non-zero velocity would keep driving the chassis
        after a task publisher stops.  The task motion helper already ends
        each segment with zero commands, so zero is the only safe value to
        synthesize before the first command and throughout silent periods.
        """
        if self.cmd_vel_pub is None:
            return None

        with self.lock:
            last_time = self.last_external_cmd_vel_time
            last_is_zero = self.last_external_cmd_vel_is_zero
        if (last_time is not None and not last_is_zero
                and time.monotonic() - last_time < self.cmd_vel_timeout):
            return None

        message = Twist()
        self.cmd_vel_pub.publish(message)
        return message

    def _publish(self, _event):
        with self.lock:
            sensor_joints = None if self.sensor_joints is None else list(self.sensor_joints)
            arm_target = None if self.arm_target is None else list(self.arm_target)
        stamp = rospy.Time.now()
        self._publish_eef_pose(stamp, sensor_joints)
        self._publish_input_pose(stamp, sensor_joints, arm_target)

    def _publish_cmd_vel(self, _event):
        self._cmd_vel_fallback_command()


def main():
    rospy.init_node("rosbag_compat_publisher")
    RosbagCompatPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
