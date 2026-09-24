#!/usr/bin/env python3
"""Latch and animate the Scene 1 V2 source bin conveyor handoff."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from kuavo_msgs.srv import (
    SetJointPosition,
    SetJointPositionRequest,
    SetObjectPosition,
    SetObjectPositionRequest,
)
from std_msgs.msg import Bool


SOURCE_BIN_SLIDE_JOINT = "source_bin_slide"
SOURCE_CONVEYOR_TOP_Z = 0.62 + 0.03 * math.cos(math.radians(30.0))
# The rigid-support bin origin is its floor reference.  Release when that
# reference reaches its designed height at the 30-degree handoff pose.
CONTACT_TOLERANCE = 0.00002
LATCH_RATE_HZ = 50.0
CONVEYOR_SPEED_MPS = 0.10
# The white cradle ends at local X=0.21 m and the bin half-length is 0.20 m.
# Moving the bin centre to 0.45 m puts its complete footprint beyond the
# cradle while staying inside the declared 0.60 m slide range.
CONVEYOR_STOP_POSITION_M = 0.45
CYLINDER_NAMES = ("cylinder_1", "cylinder_2", "cylinder_3")
# Source-bin wall inner faces in the bin frame.  The Z window only rejects
# objects which have already left the bin; the rigid follower preserves the
# complete captured pose once the conveyor animation begins.
SOURCE_BIN_INNER_HALF_X_M = 0.18
SOURCE_BIN_INNER_HALF_Y_M = 0.20
SOURCE_BIN_MIN_CENTER_Z_M = -0.03
SOURCE_BIN_MAX_CENTER_Z_M = 0.12


def _quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    result = (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1e-12:
        raise ValueError("cannot normalize a zero quaternion")
    return tuple(value / norm for value in result)


def _quaternion_conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _rotate_vector(quaternion, vector):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    # Expanded q * (0, v) * conjugate(q), avoiding normalization of the
    # intermediate pure-vector quaternion.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _relative_pose(parent_pose, child_pose):
    parent_position, parent_quaternion = parent_pose
    child_position, child_quaternion = child_pose
    inverse_parent = _quaternion_conjugate(parent_quaternion)
    relative_position = _rotate_vector(
        inverse_parent,
        tuple(child - parent for child, parent in zip(
            child_position, parent_position)),
    )
    relative_quaternion = _quaternion_multiply(
        inverse_parent, child_quaternion)
    return relative_position, relative_quaternion


def _compose_pose(parent_pose, relative_pose):
    parent_position, parent_quaternion = parent_pose
    relative_position, relative_quaternion = relative_pose
    rotated_position = _rotate_vector(
        parent_quaternion, relative_position)
    child_position = tuple(
        parent + offset
        for parent, offset in zip(parent_position, rotated_position)
    )
    child_quaternion = _quaternion_multiply(
        parent_quaternion, relative_quaternion)
    return child_position, child_quaternion


def _inside_source_bin(relative_position):
    x, y, z = relative_position
    return (
        abs(x) <= SOURCE_BIN_INNER_HALF_X_M
        and abs(y) <= SOURCE_BIN_INNER_HALF_Y_M
        and SOURCE_BIN_MIN_CENTER_Z_M <= z <= SOURCE_BIN_MAX_CENTER_Z_M
    )


def _pose_tuple(message):
    pose = message.pose
    quaternion = (
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise ValueError("pose contains a zero quaternion")
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        tuple(value / norm for value in quaternion),
    )


class SourceBinLatch:
    def __init__(self):
        self._lock = threading.Lock()
        self._source_bin_pose = None
        self._object_poses = {}
        self._released = False
        self._release_publisher = rospy.Publisher(
            "/mujoco/source_bin_latch_released", Bool,
            queue_size=1, latch=True)
        self._release_publisher.publish(Bool(data=False))
        self._complete_publisher = rospy.Publisher(
            "/mujoco/source_bin_conveyor_complete", Bool,
            queue_size=1, latch=True)
        self._pose_subscriber = rospy.Subscriber(
            "/mujoco/source_bin/pose", PoseStamped,
            self._source_bin_pose_callback, queue_size=1)
        self._object_pose_subscribers = [
            rospy.Subscriber(
                "/mujoco/{}/pose".format(name), PoseStamped,
                self._object_pose_callback, callback_args=name, queue_size=1)
            for name in CYLINDER_NAMES
        ]
        rospy.wait_for_service("/set_joint_position")
        rospy.wait_for_service("/set_object_position")
        self._set_joint_position = rospy.ServiceProxy(
            "/set_joint_position", SetJointPosition, persistent=True)
        self._set_object_position = rospy.ServiceProxy(
            "/set_object_position", SetObjectPosition, persistent=True)

    def _source_bin_pose_callback(self, message):
        with self._lock:
            self._source_bin_pose = _pose_tuple(message)

    def _object_pose_callback(self, message, object_name):
        with self._lock:
            self._object_poses[object_name] = _pose_tuple(message)

    def _snapshot_poses(self):
        with self._lock:
            return self._source_bin_pose, dict(self._object_poses)

    def _capture_bin_followers(self):
        source_bin_pose, object_poses = self._snapshot_poses()
        if source_bin_pose is None:
            raise RuntimeError("source-bin pose is unavailable at release")
        missing = [
            name for name in CYLINDER_NAMES if name not in object_poses]
        if missing:
            raise RuntimeError(
                "object poses are unavailable at release: {}".format(missing))

        followers = {}
        for name in CYLINDER_NAMES:
            relative_pose = _relative_pose(
                source_bin_pose, object_poses[name])
            if _inside_source_bin(relative_pose[0]):
                followers[name] = relative_pose
        rospy.loginfo(
            "[Scene1V2Latch] captured source-bin followers: %s",
            sorted(followers))
        return followers

    def _sync_bin_followers(self, followers):
        if not followers:
            return
        source_bin_pose, _ = self._snapshot_poses()
        if source_bin_pose is None:
            raise RuntimeError("source-bin pose is unavailable during motion")
        for name, relative_pose in followers.items():
            position, quaternion = _compose_pose(
                source_bin_pose, relative_pose)
            request = SetObjectPositionRequest(object_name=name)
            request.position.x, request.position.y, request.position.z = position
            request.orientation.w = quaternion[0]
            request.orientation.x = quaternion[1]
            request.orientation.y = quaternion[2]
            request.orientation.z = quaternion[3]
            response = self._set_object_position(request)
            if not response.success:
                raise RuntimeError(
                    "failed to move {} with source bin: {}".format(
                        name, response.message))

    def run(self):
        rate = rospy.Rate(LATCH_RATE_HZ)
        hold_request = SetJointPositionRequest(
            joint_name=SOURCE_BIN_SLIDE_JOINT, position=0.0)
        while not rospy.is_shutdown() and not self._released:
            with self._lock:
                bottom_z = (
                    None if self._source_bin_pose is None
                    else self._source_bin_pose[0][2])

            if (bottom_z is not None and
                    bottom_z <= SOURCE_CONVEYOR_TOP_Z + CONTACT_TOLERANCE):
                try:
                    followers = self._capture_bin_followers()
                except RuntimeError as error:
                    rospy.logerr("[Scene1V2Latch] %s", error)
                    return
                self._released = True
                self._release_publisher.publish(Bool(data=True))
                rospy.loginfo(
                    "[Scene1V2Latch] source bin reached conveyor at z=%.6f; "
                    "white-cradle latch released", bottom_z)
                break

            try:
                response = self._set_joint_position(hold_request)
                if not response.success:
                    rospy.logwarn_throttle(
                        2.0, "[Scene1V2Latch] failed to hold source bin: %s",
                        response.message)
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(
                    2.0, "[Scene1V2Latch] set_joint_position failed: %s",
                    error)
            rate.sleep()

        if rospy.is_shutdown():
            return

        step_m = CONVEYOR_SPEED_MPS / LATCH_RATE_HZ
        step_count = int(math.ceil(CONVEYOR_STOP_POSITION_M / step_m))
        for step_index in range(1, step_count + 1):
            if rospy.is_shutdown():
                return
            position = min(
                step_index * step_m, CONVEYOR_STOP_POSITION_M)
            move_request = SetJointPositionRequest(
                joint_name=SOURCE_BIN_SLIDE_JOINT,
                position=position)
            try:
                response = self._set_joint_position(move_request)
                if not response.success:
                    raise RuntimeError(response.message)
                # Let the simulator publish the just-updated bin pose, then
                # apply the same rigid transform to objects captured inside.
                rate.sleep()
                self._sync_bin_followers(followers)
            except (rospy.ServiceException, RuntimeError) as error:
                rospy.logerr(
                    "[Scene1V2Latch] conveyor motion failed at %.3f m: %s",
                    position, error)
                return

        rospy.loginfo(
            "[Scene1V2Latch] source bin left the white cradle: "
            "slide=%.3f m speed=%.3f m/s",
            CONVEYOR_STOP_POSITION_M, CONVEYOR_SPEED_MPS)
        self._complete_publisher.publish(Bool(data=True))
        rospy.spin()


def main():
    rospy.init_node("source_bin_latch_v2")
    SourceBinLatch().run()


if __name__ == "__main__":
    main()
