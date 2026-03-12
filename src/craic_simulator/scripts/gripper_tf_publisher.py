#!/usr/bin/env python3
"""Publish gripper and wrist-camera TFs directly via tf2, without robot_state_publisher.

Subscribes to /gripper/state for driver joint angles and publishes:
  - Static TFs for fixed gripper links (base, camera, end_effector, outer_finger, inner_finger)
  - Dynamic TFs for outer_knuckle and inner_knuckle joints (X-axis rotation matching MuJoCo)

The simplified model clamps the driver angle to [0, MAX_ANGLE] to prevent
mesh penetration (the full 4-bar linkage is not modeled in URDF).

This avoids TF_REPEATED_DATA warnings caused by running robot_state_publisher
with the full robot URDF alongside the body controller.
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

PI = math.pi
# Clamp driver angle to prevent mesh penetration in simplified model.
# MuJoCo driver range is [0, 0.8] rad, but without 4-bar linkage the
# outer_knuckle clips through the base/inner_knuckle at large angles.
MIN_ANGLE = 0.0
MAX_ANGLE = 0.5


def rpy_to_quat(r, p, y):
    """Convert roll-pitch-yaw to quaternion (x, y, z, w)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_mul(a, b):
    """Hamilton product of two quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def axis_angle_quat(axis, angle):
    """Quaternion (x, y, z, w) for rotation of *angle* rad about *axis*."""
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def make_tf(parent, child, xyz, quat, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
    return t


# ---------- TF definitions (from URDF / MuJoCo) ----------
# Static TFs: (parent, child, xyz, rpy)
STATIC_TFS = []
# Dynamic TFs: joint_name -> [(parent, child, xyz, base_rpy), ...]
DYNAMIC_JOINTS = {}

# MuJoCo driver joint axis
ROTATION_AXIS = (1, 0, 0)  # X-axis

for side, wrist, y_sign in [("left", "zarm_l7_link", 1), ("right", "zarm_r7_link", -1)]:
    pfx = side + "_gripper"

    # ---- Wrist D405 camera (fixed to wrist) ----
    cam_rpy = (PI, 0, PI) if y_sign > 0 else (PI, 0, 0)
    STATIC_TFS.append((wrist, side + "_wrist_camera_link",
                        (0, 0.06 * y_sign, 0), cam_rpy))

    # ---- Gripper base (fixed to wrist) ----
    STATIC_TFS.append((wrist, pfx + "_base",
                        (0, 0, -0.0038), (PI, 0, PI / 2)))

    # ---- End effector (fixed to base) ----
    STATIC_TFS.append((pfx + "_base", "zarm_%s7_end_effector" % side[0],
                        (0, 0, 0.145), (0, 0, 0)))

    # ---- Outer finger & inner finger (fixed to their outer knuckle) ----
    for fs in ("right", "left"):
        STATIC_TFS.append((pfx + "_%s_outer_knuckle" % fs,
                           pfx + "_%s_outer_finger" % fs,
                           (0, 0.0315, -0.0041), (0, 0, 0)))
        STATIC_TFS.append((pfx + "_%s_outer_finger" % fs,
                           pfx + "_%s_inner_finger" % fs,
                           (0, 0.0061, 0.0471), (0, 0, 0)))

    # ---- Dynamic joints (driven by /gripper/state) ----
    joint_name = side + "_gripper_joint"
    DYNAMIC_JOINTS[joint_name] = [
        # Outer knuckle - right side (no base rotation)
        (pfx + "_base", pfx + "_right_outer_knuckle",
         (0, 0.0306011, 0.054904), (0, 0, 0)),
        # Outer knuckle - left side (mirrored 180 deg around Z)
        (pfx + "_base", pfx + "_left_outer_knuckle",
         (0, -0.0306011, 0.054904), (0, 0, PI)),
        # Inner knuckle - right side (follows driver angle, like MuJoCo spring_link)
        (pfx + "_base", pfx + "_right_inner_knuckle",
         (0, 0.0127, 0.06142), (0, 0, 0)),
        # Inner knuckle - left side (mirrored)
        (pfx + "_base", pfx + "_left_inner_knuckle",
         (0, -0.0127, 0.06142), (0, 0, PI)),
    ]


class GripperTFPublisher:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        # Publish all static TFs once
        stamp = rospy.Time.now()
        static_msgs = []
        for parent, child, xyz, rpy in STATIC_TFS:
            q = rpy_to_quat(*rpy)
            static_msgs.append(make_tf(parent, child, xyz, q, stamp))
        self.static_br.sendTransform(static_msgs)
        rospy.loginfo("gripper_tf_publisher: published %d static TFs", len(static_msgs))

        # Publish initial dynamic TFs (angle = 0)
        self._publish_dynamic({}, rospy.Time.now())

        # Subscribe to gripper state
        rospy.Subscriber("/gripper/state", JointState, self._gripper_cb)

    def _gripper_cb(self, msg):
        angles = {}
        for name, pos in zip(msg.name, msg.position):
            if name in DYNAMIC_JOINTS:
                angles[name] = pos
        if angles:
            self._publish_dynamic(angles, rospy.Time.now())

    def _publish_dynamic(self, angles, stamp):
        tfs = []
        for joint_name, knuckle_list in DYNAMIC_JOINTS.items():
            theta = angles.get(joint_name, 0.0)
            # Clamp to valid range to prevent mesh penetration
            theta = max(MIN_ANGLE, min(MAX_ANGLE, theta))
            rot_q = axis_angle_quat(ROTATION_AXIS, theta)

            for parent, child, xyz, base_rpy in knuckle_list:
                base_q = rpy_to_quat(*base_rpy)
                combined_q = quat_mul(base_q, rot_q)
                tfs.append(make_tf(parent, child, xyz, combined_q, stamp))

        if tfs:
            self.br.sendTransform(tfs)


if __name__ == "__main__":
    rospy.init_node("gripper_tf_publisher")
    GripperTFPublisher()
    rospy.spin()
