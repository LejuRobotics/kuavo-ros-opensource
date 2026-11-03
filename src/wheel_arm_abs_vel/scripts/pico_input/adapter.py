"""Pico full-body SDK poses -> the existing Quest ROS input contract.

Only upper-body indices are read. No IK, retarget lengths, control modes or filters.
The matrix convention below matches beta pico_utils.transform_matrix_to_ros.
The resulting chest/palm local axes are then expressed in Quest's source convention.
"""
import math
import struct

import numpy as np
from tf.transformations import quaternion_matrix, euler_matrix

from . import body_tracking_extended_pb2 as proto

HEADER = struct.Struct("!4sHHHHI16sQQQQIIQ")
# Quest slot -> PICO BodyTrackerRole. Finger slots remain identity in controller mode.
BONES = {0: 16, 1: 18, 2: 17, 3: 19, 4: 22, 5: 23, 22: 0, 23: 9}
HEAD_BONES = {24: 12, 25: 15}
REQUIRED = (0, 1, 2, 3, 4, 5, 23)
A = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
B = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
D = np.diag([1., -1., -1.])
P = np.array([[0., 0., 1.], [-1., 0., 0.], [0., 1., 0.]])
# SDK full_body chest axes become physical forward/left/up after raw_to_robot.
# The beta retargeter's historical chest neutral matrix is NOT the SDK bone basis.
# Verified against shoulder left-right and pelvis-to-chest vectors in 2026-09-28 bag.
PICO_CHEST_REFERENCE = np.eye(3)
QUEST_CHEST_REFERENCE = euler_matrix(math.pi / 2, 0, math.pi / 2)[:3, :3]


def decode_datagram(raw):
    """Return protobuf and optional capture identity; reject truncated/unknown framing."""
    capture = None
    if raw.startswith(b"PDT1"):
        if len(raw) < HEADER.size:
            raise ValueError("truncated Pico header")
        h = HEADER.unpack(raw[:HEADER.size])
        if h[1] != HEADER.size or h[2] != 1 or h[3] != 1:
            raise ValueError("unsupported Pico transport header/type")
        if h[12] != len(raw) - HEADER.size:
            raise ValueError("Pico payload size mismatch")
        capture = (h[6], h[9])
        raw = raw[HEADER.size:]
    msg = proto.VRData()
    msg.ParseFromString(raw)
    stamp = msg.header.timestamp
    if stamp.seconds <= 0 or not 0 <= stamp.nanos < 1000000000:
        raise ValueError("missing/invalid sender timestamp")
    return msg, stamp.seconds * 1000000000 + stamp.nanos, capture


def pose_matrix(pose):
    p = np.array([pose.pos_x, pose.pos_y, pose.pos_z], dtype=float)
    q = np.array([pose.rot_qx, pose.rot_qy, pose.rot_qz, pose.rot_qw], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(p).all() or not np.isfinite(q).all() or not 0.5 <= norm <= 1.5:
        raise ValueError("invalid body position/quaternion")
    # Reject gross unit/protocol errors, not normal human motion.
    if np.linalg.norm(p) > 100:
        raise ValueError("body position outside metre-scale tracking space")
    out = quaternion_matrix(q / norm)
    out[:3, 3] = p
    return out


def raw_to_robot(pose, source_index):
    raw = pose_matrix(pose)
    out = np.eye(4)
    out[:3, 3] = P @ raw[:3, 3]
    out[:3, :3] = D @ A @ raw[:3, :3] @ B @ D
    if source_index in (16, 18, 20, 22):
        out[:3, :3] = out[:3, :3] @ euler_matrix(math.pi/2, 0, 0)[:3, :3]
    elif source_index in (17, 19, 21, 23):
        out[:3, :3] = out[:3, :3] @ euler_matrix(-math.pi/2, 0, 0)[:3, :3]
    return out


class PoseAdapter:
    def __init__(self, left_source_to_quest=None, right_source_to_quest=None):
        self.offsets = {}
        for side, value in (("Left", left_source_to_quest), ("Right", right_source_to_quest)):
            value = [0.] * 6 if value is None else value
            if len(value) != 6 or not np.isfinite(value).all():
                raise ValueError("hand calibration must be finite [x,y,z,roll,pitch,yaw]")
            T = euler_matrix(*value[3:])
            T[:3, 3] = value[:3]
            self.offsets[side] = T

    def from_full_body(self, poses):
        if len(poses) not in (24, 25):
            raise ValueError("expected full_body BodyTrackerRole[24] (+ optional foot state)")
        matrices = {}
        for target, source in BONES.items():
            # Root is optional diagnostic data. Never require feet or leg tracking.
            if target == 22:
                try:
                    matrices[target] = raw_to_robot(poses[source], source)
                except ValueError:
                    matrices[target] = np.eye(4)
            else:
                matrices[target] = raw_to_robot(poses[source], source)
        # Missing head tracking must not discard otherwise valid arm input.
        try:
            head = {target: raw_to_robot(poses[source], source)
                    for target, source in HEAD_BONES.items()}
        except ValueError:
            head = {}
        matrices.update(head)
        return self.from_robot_matrices(matrices)

    def from_robot_matrices(self, matrices):
        """Also usable for offline beta /robot_body_matrices evidence."""
        slots = dict(BONES)
        if all(slot in matrices for slot in HEAD_BONES):
            slots.update(HEAD_BONES)
        out = np.tile(np.eye(4), (26 if 25 in slots else 24, 1, 1))
        for slot in slots:
            if slot not in matrices:
                raise ValueError("missing upper-body matrix")
            T = np.asarray(matrices[slot], dtype=float)
            if (T.shape != (4, 4) or not np.isfinite(T).all()
                    or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
                    or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
                    or not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-5)):
                raise ValueError("invalid rigid body matrix")
            out[slot] = T
        for slot in (23, 24, 25):
            if slot < len(out):
                out[slot, :3, :3] = out[slot, :3, :3] @ PICO_CHEST_REFERENCE.T @ QUEST_CHEST_REFERENCE
        for side, slot in (("Left", 4), ("Right", 5)):
            out[slot] = out[slot] @ self.offsets[side]
            # Inverse of lightweight_transforms.vr_quat2robot_quat(..., bias=0).
            fixed = euler_matrix(-math.pi/2, 0, -math.pi if side == "Left" else 0)[:3, :3]
            out[slot, :3, :3] = out[slot, :3, :3] @ fixed.T
        for shoulder, elbow, hand in ((0, 1, 4), (2, 3, 5)):
            for i, j in ((shoulder, elbow), (elbow, hand)):
                length = np.linalg.norm(out[i, :3, 3] - out[j, :3, 3])
                if not 0.03 < length < 1.5:
                    raise ValueError("degenerate shoulder/elbow/hand geometry")
        return out


def controller_values(controllers):
    out = {}
    for side in ("left", "right"):
        if not controllers.HasField(side):
            raise ValueError("missing controller side")
        c = getattr(controllers, side)
        if len(c.thumbstick) != 2:
            raise ValueError("controller thumbstick must have two axes")
        values = list(c.thumbstick) + [c.trigger_value, c.grip_value]
        if not np.isfinite(values).all():
            raise ValueError("invalid controller values")
        if any(abs(x) > 1.001 for x in values[:2]) or any(x < 0 or x > 1.001 for x in values[2:]):
            raise ValueError("controller value out of range")
        for key, value in zip(("x", "y", "trigger", "grip"), values):
            out[side + "_" + key] = float(value)
        out[side + "_first_button_pressed"] = c.primary_button
        out[side + "_second_button_pressed"] = c.secondary_button
        # Source protocol has no touch fields. Do not invent touch observations.
        out[side + "_first_button_touched"] = False
        out[side + "_second_button_touched"] = False
    return out
