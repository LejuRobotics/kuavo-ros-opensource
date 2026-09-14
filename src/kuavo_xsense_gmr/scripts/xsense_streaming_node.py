#!/usr/bin/env python3


import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque
from datetime import datetime
from typing import Dict, List, Tuple, Optional

# Import custom messages from kuavo_msgs

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from kuavo_msgs.msg import (
    ControllerSwitchEvent,
    JoySticks,
    RigidBodyDescription,
    xsensePoseInfoList,
    xsensePoseRetarget,
)

from kuavo_gmr import RobotMotionViewer, VMR_bvh
from kuavo_gmr.utils.xsense import cali_pose_from_seg_desc
import kuavo_gmr.utils.lafan_vendor.utils as utils

# ------------------------------------------------------------
# Coordinate canonicalization (Xsens UDP pose stream -> GMR canonical)
XS_NAME_MAPPING: Dict[str, str]= {
    # torso
    "Pelvis": "Hips", 
    "Chest": "Spine",  
    "Neck": "Neck",
    "Head": "Head",

    # arms (Motive chain: Collar -> Shoulder -> Elbow -> Wrist)
    "Left Shoulder": "LeftShoulder",
    "Left Upper Arm": "LeftArm",
    "Left Forearm": "LeftForeArm",
    "Left Hand": "LeftHand",

    "Right Shoulder": "RightShoulder",
    "Right Upper Arm": "RightArm",
    "Right Forearm": "RightForeArm",
    "Right Hand": "RightHand",

    # legs
    "Left Upper Leg": "LeftUpLeg",
    "Left Lower Leg": "LeftLeg",
    "Left Foot": "LeftFoot",
    "Left Toe": "LeftToeBase",

    "Right Upper Leg": "RightUpLeg",
    "Right Lower Leg": "RightLeg",
    "Right Foot": "RightFoot",
    "Right Toe": "RightToeBase",
}
xsense_body_names = list(XS_NAME_MAPPING.keys())
ARM_JOINT_NAMES = [
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
    "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
]
FrameDict = Dict[str, Tuple[np.ndarray, np.ndarray]]

# ------------------------------------------------------------
R_STREAM2CAN = np.array([
    [0.0,  0.0, 1.0],
    [1.0,  0.0, 0.0],
    [0.0,  1.0, 0.0],
], dtype=float)

try:
    Q_STREAM2CAN_WXYZ = R.from_matrix(R_STREAM2CAN).as_quat(scalar_first=True)  # wxyz
except TypeError:
    q_xyzw = R.from_matrix(R_STREAM2CAN).as_quat()
    Q_STREAM2CAN_WXYZ = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)

def _quat_normalize_wxyz(q, eps=1e-12):
    q = np.asarray(q, dtype=float).reshape(4,)
    n = np.linalg.norm(q)
    if n < eps:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def stream_to_bvhloader_canonical(pos_xyz, quat_wxyz):
    pos_xyz = np.asarray(pos_xyz, dtype=float).reshape(3,)
    quat_wxyz = np.asarray(quat_wxyz, dtype=float).reshape(4,)

    pos_can = pos_xyz @ R_STREAM2CAN.T
    quat_can = utils.quat_mul(Q_STREAM2CAN_WXYZ, quat_wxyz)  
    quat_can = _quat_normalize_wxyz(quat_can)
    return pos_can, quat_can

def canonicalize_frame(frame_dict):
    """Return a NEW dict with canonicalized (pos, quat_wxyz)."""
    out = {}
    for name, (p, q) in frame_dict.items():
        p2, q2 = stream_to_bvhloader_canonical(p, q)
        out[name] = [p2, q2]
    return out

def map_xsense_frame_to_gmr(frame_data: FrameDict) -> FrameDict:
    """Map canonicalized xsense frame into the source skeleton expected by GMR."""
    out: FrameDict = {}
    for xsense_name, lafan_name in XS_NAME_MAPPING.items():
        if xsense_name not in frame_data:
            raise ValueError(f"Missing expected PICO joint '{xsense_name}' in frame data")
        pos, quat_wxyz = frame_data[xsense_name]
        out[lafan_name] = [np.asarray(pos, dtype=np.float64), quat_wxyz]
    return out


# ---------------------------------------------------------------------------
# Pure-numpy angular velocity from two quaternions (wxyz convention)
# ---------------------------------------------------------------------------
def _angular_velocity_wxyz(q_prev_wxyz: np.ndarray, q_curr_wxyz: np.ndarray, inv_dt: float) -> np.ndarray:
    """Compute angular velocity from two unit quaternions (wxyz)."""
    qp = np.asarray(q_prev_wxyz, dtype=np.float64)
    qc = np.asarray(q_curr_wxyz, dtype=np.float64)
    qp = qp / max(np.linalg.norm(qp), 1e-12)
    qc = qc / max(np.linalg.norm(qc), 1e-12)

    pw, px, py, pz = qp[0], -qp[1], -qp[2], -qp[3]
    cw, cx, cy, cz = qc[0], qc[1], qc[2], qc[3]
    dw = cw * pw - cx * px - cy * py - cz * pz
    dx = cw * px + cx * pw + cy * pz - cz * py
    dy = cw * py - cx * pz + cy * pw + cz * px
    dz = cw * pz + cx * py - cy * px + cz * pw

    if dw < 0.0:
        dw, dx, dy, dz = -dw, -dx, -dy, -dz

    sin_half = np.sqrt(dx * dx + dy * dy + dz * dz)
    if sin_half < 1e-10:
        return np.zeros(3, dtype=np.float64)

    half_angle = np.arctan2(sin_half, dw)
    angle = 2.0 * half_angle
    axis = np.array([dx, dy, dz], dtype=np.float64) / sin_half
    return axis * angle * inv_dt


def _wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _yaw_from_quat_wxyz(q_wxyz: np.ndarray) -> float:
    q_wxyz = _quat_normalize_wxyz(q_wxyz)
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    fwd_xy = R.from_quat(q_xyzw).apply([1.0, 0.0, 0.0])[:2]
    n = np.linalg.norm(fwd_xy)
    if n < 1e-8 or not np.all(np.isfinite(fwd_xy)):
        return 0.0
    return _wrap_to_pi(np.arctan2(fwd_xy[1], fwd_xy[0]))


def _left_apply_yaw_to_quat_wxyz(q_wxyz: np.ndarray, yaw: float) -> np.ndarray:
    q_wxyz = _quat_normalize_wxyz(q_wxyz)
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    q_corr_xyzw = (R.from_euler("z", yaw) * R.from_quat(q_xyzw)).as_quat()
    q_corr_xyzw /= np.clip(np.linalg.norm(q_corr_xyzw), 1e-12, None)
    return np.array(
        [q_corr_xyzw[3], q_corr_xyzw[0], q_corr_xyzw[1], q_corr_xyzw[2]],
        dtype=np.float64,
    )


class GMRStreamingNode:
    """ROS node for GMR motion retargeting from OptiTrack."""

    def __init__(self):
        rospy.init_node('gmr_streaming_node', anonymous=False)

        # CPU affinity: bind process to specific cores for deterministic scheduling
        self._setup_cpu_affinity()

        # Parameters
        # NOTE: GMR IK config for "bvh_motive_streaming" only supports "kuavo_s52".
        # Do NOT read /robot_type here — that is for the physical robot, not the GMR target model.
        self.robot = rospy.get_param("~robot", "kuavo_s54")
        self.format = rospy.get_param("~format", "xsense")
        self.enable_viewer = bool(rospy.get_param("~enable_viewer", True))
        self.publish_kuavo_arm_traj = bool(rospy.get_param("~publish_kuavo_arm_traj", False))
        self.topic_name = rospy.get_param("~topic", "/xsense/world_bone_poses")
        self.topic_name_cali = rospy.get_param("~topic_cali", "/xsense/bvh_seg_desc")
        self.pico_joy_topic = rospy.get_param("~pico_joy_topic", "/pico/joy")
        self.pico_trigger_threshold = float(rospy.get_param("~pico_trigger_threshold", 0.5))
        self.vmp_controller_name = rospy.get_param("~vmp_controller_name", "vmp_controller")
        self.fps = float(rospy.get_param("~fps", 100.0))
        self.cali_arm_pose_deg = rospy.get_param('~cali_arm_pose_deg', 90.0)

        # Producer (get_frame) rate + buffer
        self.frame_buffer_len = int(rospy.get_param('~frame_buffer_len', 3)) # ring buffer length

        # Publishers
        self.vmp_pub = rospy.Publisher('/xsense/retargeted_pose', xsensePoseRetarget, queue_size=10)
        self.arm_traj_pub = (
            rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
            if self.publish_kuavo_arm_traj else None
        )
        # Subscribers
        self.xsense_pose_subscriber = rospy.Subscriber(self.topic_name, xsensePoseInfoList, self.bone_poses_callback, queue_size=10)
        self.xsense_seg_desc = None
        self.xsense_seg_desc_subscriber = rospy.Subscriber(self.topic_name_cali, RigidBodyDescription, self.xsense_seg_desc_callback, queue_size=1)
        # State variables
        self.retarget = None
        self.viewer = None
        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None

        self._heading_R_delta = None  # scipy Rotation
        self._heading_p0 = None       # np.ndarray(3,)
        self._heading_init_qpos_buf = deque(maxlen=6)

        # RT+Y pauses VMP; RT+X resumes it. Keep published root yaw
        # continuous across that resume by left-applying a yaw-only correction.
        self._vmp_yaw_lock = threading.Lock()
        self._vmp_yaw_correction = 0.0
        self._vmp_yaw_freeze_yaw = None
        self._vmp_yaw_resume_pending = False
        self._vmp_yaw_auto_resume_pending = False
        self._vmp_yaw_ready_frames_remaining = 0
        self._vmp_yaw_last_pause_combo = False
        self._vmp_yaw_last_resume_combo = False
        self._latest_published_root_rot_wxyz = None
        self.vmp_yaw_resume_ready_pub = rospy.Publisher(
            '/xsense/vmp_yaw_resume_ready', Header, queue_size=1
        )
        self.pico_joy_subscriber = rospy.Subscriber(self.pico_joy_topic, JoySticks, self.vmp_yaw_joy_callback, queue_size=10)
        self.controller_switch_subscriber = rospy.Subscriber(
            "/humanoid_controller/controller_switch_event",
            ControllerSwitchEvent,
            self.vmp_yaw_controller_switch_callback,
            queue_size=10,
        )

        # dt based on ROS stamp (publish-time 100Hz)
        self.prev_pub_stamp = None
        self.prev_ee_world_pub = None

        # Thread safety for retarget/config access
        self.retarget_lock = threading.Lock()

        # End effector link names for kuavo_s5x
        self.ee_links = {
            'left_foot': 'leg_l6_link',
            'right_foot': 'leg_r6_link',
            'left_hand': 'zarm_l7_link',
            'right_hand': 'zarm_r7_link',
        }

        # -----------------------------
        # Frame ring buffer (producer/consumer)
        # -----------------------------
        self.frame_buf = deque(maxlen=max(1, self.frame_buffer_len))  # stores (rospy.Time stamp, canonical_frame)
        self.frame_buf_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._frame_thread = None

       
        self.prev_ee_world_pub = None   # aligned + ground-dragged EE world positions from previous published frame

        self.contact_height_threshold = rospy.get_param('~contact_height_threshold', 0.12)
        self.contact_trans_threshold = rospy.get_param('~contact_trans_threshold', 0.02)

        self._last_frame = None
        self._last_capture_stamp = None

        rospy.loginfo(f"GMR Streaming Node initialized")
        rospy.loginfo(f"  Robot: {self.robot}")
        rospy.loginfo(f"  Format: {self.format}")
        rospy.loginfo(f"  FPS: {self.fps:.1f}")
        rospy.loginfo(f"  Publish /kuavo_arm_traj: {'Enabled' if self.publish_kuavo_arm_traj else 'Disabled'}")

    # ------------------------------------------------------------
    # CPU affinity: bind process to specific cores
    # ------------------------------------------------------------
    def _setup_cpu_affinity(self):
        """Bind this process (and all its threads) to specific CPU cores.

        Reads '~cpu_cores' ROS parameter (list of ints). Empty list = no binding.
        This must be called early, before spawning background threads, so that
        child threads inherit the affinity mask.

        On the target Intel Core Ultra 7 255H (16 cores, no HT):
            P-cores: CPU 0-5  (up to 5.1 GHz)  ← preferred for retarget()
            E-cores: CPU 6-13 (up to 4.4 GHz)
            LP E-cores: CPU 14-15 (2.5 GHz)    ← avoid
        Already occupied: 2,3 (WBC+MPC), 5 (IMU), 7 (Motor/CAN)
        Recommended: [4] or [0,4] for GMR
        """
        cpu_cores = rospy.get_param('~cpu_cores', [])
        if not cpu_cores:
            rospy.loginfo("[GMR] CPU affinity: not set (OS free scheduling)")
            return

        # Validate: must be list of non-negative ints
        try:
            cpu_cores = [int(c) for c in cpu_cores]
        except (TypeError, ValueError):
            rospy.logwarn(f"[GMR] CPU affinity: invalid cpu_cores={cpu_cores}, skipping")
            return

        n_cpus = os.cpu_count() or 1
        invalid = [c for c in cpu_cores if c < 0 or c >= n_cpus]
        if invalid:
            rospy.logwarn(f"[GMR] CPU affinity: cores {invalid} out of range [0, {n_cpus-1}], skipping")
            return

        try:
            os.sched_setaffinity(0, cpu_cores)  # 0 = current process (all threads)
            actual = sorted(os.sched_getaffinity(0))
            rospy.loginfo(f"[GMR] CPU affinity: bound to cores {actual} (requested {cpu_cores})")
        except OSError as e:
            rospy.logwarn(f"[GMR] CPU affinity: failed to set cores {cpu_cores}: {e}")

    # ------------------------------------------------------------------
    # Callback / latest-frame ring buffer
    # ------------------------------------------------------------------
    def bone_poses_callback(self, msg: xsensePoseInfoList):
        try:
            if hasattr(msg, "header") and msg.header.stamp != rospy.Time():
                capture_stamp = msg.header.stamp
            else:
                capture_stamp = rospy.Time.now()

            frame_data: FrameDict = {}
            for bone_idx, xsense_name in enumerate(xsense_body_names):
                pose = msg.poses[bone_idx]
                pos_xyz = np.array(
                    [pose.position.x, pose.position.y, pose.position.z],
                    dtype=np.float64,
                )
                quat_xyzw = np.array(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                    dtype=np.float64,
                )
                
                quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)

                frame_data[xsense_name] = [pos_xyz, quat_wxyz]

            frame_data_map = map_xsense_frame_to_gmr(frame_data)
            frame_gmr = canonicalize_frame(frame_data_map)

            with self.frame_buf_lock:
                self.frame_buf.append((capture_stamp, frame_gmr))

        except Exception as e:
            rospy.logerr(f"[XSense] Error in callback: {e}")
            import traceback; traceback.print_exc()

    def _seg_desc_msg_to_dict(self, msg) -> dict:
        if isinstance(msg, dict):
            return msg

        def get_field(obj, names, default=None):
            for name in names:
                if hasattr(obj, name):
                    return getattr(obj, name)
            return default

        def xyz(value):
            if value is None:
                return [0.0, 0.0, 0.0]
            if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
                return [float(value.x), float(value.y), float(value.z)]
            return [float(value[0]), float(value[1]), float(value[2])]

        ids = get_field(msg, ["ids"], None)
        names = get_field(msg, ["names"], None)
        parent_ids = get_field(msg, ["parent_ids"], None)
        offsets = get_field(msg, ["offsets"], None)
        if ids is not None and names is not None and parent_ids is not None and offsets is not None:
            rb_desc = {}
            n = min(len(ids), len(names), len(parent_ids), len(offsets))
            for idx in range(n):
                rb_desc[int(ids[idx])] = {
                    "name": names[idx],
                    "parent_id": int(parent_ids[idx]),
                    "offset": xyz(offsets[idx]),
                }
            return rb_desc

        entries = get_field(msg, ["segments", "segment_desc", "descriptions", "poses"], [])
        rb_desc = {}
        fallback_names = globals().get("xsense_body_names", [])

        for idx, entry in enumerate(entries):
            rid = get_field(entry, ["id", "segment_id", "body_id", "index"], idx + 1)
            parent_id = get_field(entry, ["parent_id", "parent", "parent_index"], 0)
            name = get_field(entry, ["name", "segment_name", "body_name"], None)
            if name is None and idx < len(fallback_names):
                name = fallback_names[idx]
            if name is None:
                name = f"segment_{rid}"

            offset = get_field(entry, ["offset", "position", "pos"], None)
            if offset is None and hasattr(entry, "pose"):
                offset = get_field(entry.pose, ["position"], None)

            rb_desc[int(rid)] = {
                "name": name,
                "parent_id": int(parent_id),
                "offset": xyz(offset),
            }

        return rb_desc

    def xsense_seg_desc_callback(self, msg: xsensePoseInfoList):
        if self.xsense_seg_desc is not None:
            return

        try:
            seg_desc = self._seg_desc_msg_to_dict(msg)
            if len(seg_desc) == 0:
                rospy.logwarn_throttle(1.0, "[XSense] Received empty segment description")
                return

            self.xsense_seg_desc = seg_desc
            seg_desc_subscriber = getattr(self, "xsense_seg_desc_subscriber", None)
            if seg_desc_subscriber is not None:
                seg_desc_subscriber.unregister()
                self.xsense_seg_desc_subscriber = None
            rospy.loginfo(f"[XSense] Received segment description once: {len(seg_desc)} segments")
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[XSense] Failed to parse segment description: {e}")


    def _pop_latest_frame(self):
        """Consumer: return latest (stamp, frame) and drop older ones."""
        with self.frame_buf_lock:
            if not self.frame_buf:
                return None, None
            stamp, frame = self.frame_buf[-1]
            self.frame_buf.clear()
            return stamp, frame

    # ------------------------------------------------------------------
    # VMP yaw invariant across pause/resume
    # ------------------------------------------------------------------
    def _mark_vmp_yaw_freeze(self, preserve_existing=False):
        with self._vmp_yaw_lock:
            # A new pause/exit invalidates any older resume request or retry.
            self._vmp_yaw_resume_pending = False
            self._vmp_yaw_auto_resume_pending = False
            self._vmp_yaw_ready_frames_remaining = 0
            if preserve_existing and self._vmp_yaw_freeze_yaw is not None:
                return
            latest = None if self._latest_published_root_rot_wxyz is None else self._latest_published_root_rot_wxyz.copy()

        if latest is None:
            rospy.logwarn_throttle(1.0, "[VMPYawInvariant] freeze ignored: no published root orientation yet")
            return

        freeze_yaw = _yaw_from_quat_wxyz(latest)
        with self._vmp_yaw_lock:
            self._vmp_yaw_freeze_yaw = freeze_yaw

        rospy.loginfo(
            f"[VMPYawInvariant] freeze yaw captured: {freeze_yaw:.6f} rad "
            f"({np.degrees(freeze_yaw):.2f} deg)"
        )

    def _request_vmp_yaw_resume(self, automatic=False):
        with self._vmp_yaw_lock:
            has_freeze_yaw = self._vmp_yaw_freeze_yaw is not None
            if has_freeze_yaw:
                self._vmp_yaw_resume_pending = True
                self._vmp_yaw_auto_resume_pending = automatic
                self._vmp_yaw_ready_frames_remaining = 0

        if not has_freeze_yaw:
            rospy.logwarn_throttle(1.0, "[VMPYawInvariant] resume ignored: freeze yaw was not captured")

    def vmp_yaw_joy_callback(self, joy: JoySticks):
        try:
            rt_pressed = float(joy.right_trigger) >= self.pico_trigger_threshold
            y_pressed = bool(joy.left_second_button_pressed)
            x_pressed = bool(joy.left_first_button_pressed)
            if rt_pressed and y_pressed and not self._vmp_yaw_last_y_pressed:
                self._mark_vmp_yaw_freeze()
            if rt_pressed and x_pressed and not self._vmp_yaw_last_x_pressed:
                self._request_vmp_yaw_resume()

            self._vmp_yaw_last_y_pressed = y_pressed
            self._vmp_yaw_last_x_pressed = x_pressed
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[VMPYawInvariant] joy callback failed: {e}")

    def vmp_yaw_controller_switch_callback(self, event: ControllerSwitchEvent):
        """Apply the same yaw handling as RT+Y/X on VMP controller transitions."""
        try:
            if event.from_controller == self.vmp_controller_name:
                # Do not replace an earlier explicit RT+Y reference.
                self._mark_vmp_yaw_freeze(preserve_existing=True)
            if event.to_controller == self.vmp_controller_name:
                self._request_vmp_yaw_resume(automatic=True)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[VMPYawInvariant] controller switch callback failed: {e}")

    def _complete_vmp_yaw_resume_if_needed(self, root_rot_wxyz: np.ndarray):
        with self._vmp_yaw_lock:
            if not self._vmp_yaw_resume_pending or self._vmp_yaw_freeze_yaw is None:
                return False

            resume_yaw = _yaw_from_quat_wxyz(root_rot_wxyz)
            yaw_correction = _wrap_to_pi(self._vmp_yaw_freeze_yaw - resume_yaw)
            automatic_resume = self._vmp_yaw_auto_resume_pending
            self._vmp_yaw_correction = yaw_correction
            self._vmp_yaw_resume_pending = False
            self._vmp_yaw_auto_resume_pending = False
            self._vmp_yaw_freeze_yaw = None

        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None
        self.prev_pub_stamp = None
        self.prev_ee_world_pub = None

        rospy.loginfo(
            f"[VMPYawInvariant] resume yaw={resume_yaw:.6f} rad, "
            f"correction={yaw_correction:.6f} rad ({np.degrees(yaw_correction):.2f} deg)"
        )
        return automatic_resume

    def _apply_vmp_yaw_correction(self, root_rot_wxyz: np.ndarray) -> np.ndarray:
        root_rot_wxyz = _quat_normalize_wxyz(root_rot_wxyz)

        with self._vmp_yaw_lock:
            yaw_correction = self._vmp_yaw_correction

        corrected = _left_apply_yaw_to_quat_wxyz(root_rot_wxyz, yaw_correction)

        with self._vmp_yaw_lock:
            self._latest_published_root_rot_wxyz = corrected.copy()

        return corrected
    
    def _compute_dt_sec_from_pub_stamp(self, pub_stamp):
        """dt based on publish-time ROS stamp (stable with consumer 100Hz)."""
        if pub_stamp is None:
            return None
        if self.prev_pub_stamp is None:
            self.prev_pub_stamp = pub_stamp
            return None
        dt = (pub_stamp - self.prev_pub_stamp).to_sec()
        self.prev_pub_stamp = pub_stamp
        if (not np.isfinite(dt)) or (dt <= 0.0):
            return None
        return float(np.clip(dt, 1e-4, 0.05))
    
    # ------------------------------------------------------------------
    # Root heading alignment (ported from OptiTrack node)
    # ------------------------------------------------------------------
    def align_root_heading_only(self, qpos_arr: np.ndarray):
        qpos_arr = np.asarray(qpos_arr, dtype=np.float64).copy()

        try:
            if self._heading_R_delta is None:
                self._heading_init_qpos_buf.append(qpos_arr.copy())
                if len(self._heading_init_qpos_buf) < self._heading_init_qpos_buf.maxlen:
                    return None

                pos_xy = []
                fwd_xy_list = []

                for qk in self._heading_init_qpos_buf:
                    pos_xy.append(np.asarray(qk[:2], dtype=np.float64))
                    q_wxyz = np.asarray(qk[3:7], dtype=np.float64)
                    q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
                    q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

                    Rk = R.from_quat(q_xyzw)
                    fwd = Rk.apply([1.0, 0.0, 0.0])[:2]
                    n = np.linalg.norm(fwd)
                    if n > 1e-8 and np.all(np.isfinite(fwd)):
                        fwd_xy_list.append(fwd / n)

                body_samples = fwd_xy_list[1:]
                if len(body_samples) == 0:
                    body_vec = np.array([1.0, 0.0], dtype=np.float64)
                else:
                    seed = np.sum(np.asarray(body_samples), axis=0)
                    seed_n = np.linalg.norm(seed)
                    seed = np.array([1.0, 0.0], dtype=np.float64) if seed_n < 1e-8 else seed / seed_n
                    keep = [v for v in body_samples if np.dot(v, seed) > 0.5]
                    body_vec = np.sum(np.asarray(keep if len(keep) >= 3 else body_samples), axis=0)
                    body_n = np.linalg.norm(body_vec)
                    body_vec = seed if body_n < 1e-8 else body_vec / body_n

                d_xy = pos_xy[-1] - pos_xy[0]
                d_norm = np.linalg.norm(d_xy)
                move_thresh = 0.05
                full_traj_thresh = 0.1

                if d_norm > move_thresh and np.all(np.isfinite(d_xy)):
                    traj_vec = d_xy / d_norm
                    alpha = np.clip((d_norm - move_thresh) / (full_traj_thresh - move_thresh), 0.0, 1.0)
                    v = (1.0 - alpha) * body_vec + alpha * traj_vec
                    v_n = np.linalg.norm(v)
                    v = traj_vec if v_n < 1e-8 else v / v_n
                else:
                    v = body_vec

                yaw0 = np.arctan2(v[1], v[0])
                yaw0 = (yaw0 + np.pi) % (2.0 * np.pi) - np.pi

                self._heading_R_delta = R.from_euler("z", -yaw0)
                self._heading_p0 = self._heading_init_qpos_buf[0][:3].copy()
                self._heading_init_qpos_buf.clear()

                rospy.loginfo(
                    f"[HeadingAlign] initialized: yaw0={yaw0:.6f} rad, deg={np.degrees(yaw0):.3f}"
                )

            root_loc = qpos_arr[:3]
            q_wxyz = qpos_arr[3:7]
            q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
            q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

            root_loc_aligned = self._heading_R_delta.apply(root_loc - self._heading_p0) + self._heading_p0
            root_rot_xyzw_aligned = (self._heading_R_delta * R.from_quat(q_xyzw)).as_quat()
            root_rot_xyzw_aligned /= np.clip(np.linalg.norm(root_rot_xyzw_aligned), 1e-12, None)
            root_rot_wxyz_aligned = np.array(
                [
                    root_rot_xyzw_aligned[3],
                    root_rot_xyzw_aligned[0],
                    root_rot_xyzw_aligned[1],
                    root_rot_xyzw_aligned[2],
                ],
                dtype=np.float64,
            )

            qpos_arr[:3] = root_loc_aligned
            qpos_arr[3:7] = root_rot_wxyz_aligned
            return qpos_arr

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[HeadingAlign] failed: {e}")
            return None if self._heading_R_delta is None else qpos_arr

    # ------------------------------------------------------------------
    # VMP publish with height-drift suppression (ported from OptiTrack node)
    # ------------------------------------------------------------------
    def publish_vmp_input(self, qpos_raw, root_loc, root_rot_wxyz, dof_pos, dt_sec, stamp=None):
        msg = xsensePoseRetarget()
        msg.header = Header()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = "world"

        root_loc_pub = np.asarray(root_loc, dtype=np.float64).copy()
        msg.base_link_pose = Pose()
        msg.base_link_pose.position = Point(x=float(root_loc_pub[0]), y=float(root_loc_pub[1]), z=float(root_loc_pub[2]))
        msg.base_link_pose.orientation = Quaternion(
            x=float(root_rot_wxyz[1]),
            y=float(root_rot_wxyz[2]),
            z=float(root_rot_wxyz[3]),
            w=float(root_rot_wxyz[0]),
        )

        if (
            (dt_sec is None)
            or (self.prev_root_loc is None)
            or (self.prev_root_rot_wxyz is None)
            or (self.prev_dof_pos is None)
        ):
            v_lin = np.zeros(3)
            w_ang = np.zeros(3)
            dof_vel = np.zeros_like(dof_pos)
        else:
            inv_dt = 1.0 / dt_sec
            v_lin = (root_loc - self.prev_root_loc) * inv_dt
            w_ang = _angular_velocity_wxyz(self.prev_root_rot_wxyz, root_rot_wxyz, inv_dt)
            dof_vel = (dof_pos - self.prev_dof_pos) * inv_dt

        msg.base_velocity = np.concatenate([v_lin, w_ang]).tolist()

        # picoPoseRetarget.msg defines float64[27] — 12 leg + 1 waist + 14 arm.
        # dof_pos layout (27): legs(0-11) + waist(12) + arms(13-26).
        # 与 MocapPoseRetarget.msg 格式一致，直接发布完整 27-DOF。
        msg.joint_position = dof_pos.tolist()
        msg.joint_velocity = dof_vel.tolist()

        with self._vmp_yaw_lock:
            yaw_correction = self._vmp_yaw_correction
        yaw_correction_R = R.from_euler("z", yaw_correction)

        with self.retarget_lock:
            self.retarget.configuration.update(q=qpos_raw)

            ee_world_pub = {}
            for ee_name in ["left_foot", "right_foot", "left_hand", "right_hand"]:
                link_name = self.ee_links[ee_name]
                se = self.retarget.configuration.get_transform_frame_to_world(link_name, frame_type="body")
                pos = np.asarray(se.translation(), dtype=np.float64)

                if self._heading_R_delta is not None and self._heading_p0 is not None:
                    pos = self._heading_R_delta.apply(pos - self._heading_p0) + self._heading_p0
                pos = root_loc_pub + yaw_correction_R.apply(pos - root_loc_pub)
                ee_world_pub[ee_name] = pos

            # l_now = ee_world_pub["left_foot"]
            # r_now = ee_world_pub["right_foot"]

            # if self.prev_ee_world_pub is None:
            #     l_contact = bool(l_now[2] < self.contact_height_threshold)
            #     r_contact = bool(r_now[2] < self.contact_height_threshold)
            # else:
            #     l_prev = self.prev_ee_world_pub["left_foot"]
            #     r_prev = self.prev_ee_world_pub["right_foot"]
            #     l_disp_xy = np.max(np.abs(l_now[:2] - l_prev[:2]))
            #     r_disp_xy = np.max(np.abs(r_now[:2] - r_prev[:2]))

            #     l_contact = bool(
            #         (l_disp_xy < self.contact_trans_threshold)
            #         and (l_now[2] < self.contact_height_threshold)
            #     )
            #     r_contact = bool(
            #         (r_disp_xy < self.contact_trans_threshold)
            #         and (r_now[2] < self.contact_height_threshold)
            #     )

            # contact_z = []
            # if l_contact:
            #     contact_z.append(float(l_now[2]))
            # if r_contact:
            #     contact_z.append(float(r_now[2]))

            # if len(contact_z) > 0:
            #     dz_ground = -float(np.mean(contact_z)) + 0.055
            #     root_loc_pub[2] += dz_ground
            #     for k in ee_world_pub.keys():
            #         ee_world_pub[k] = ee_world_pub[k].copy()
            #         ee_world_pub[k][2] += dz_ground

            # msg.base_link_pose.position.z = float(root_loc_pub[2])

            ee_positions = []
            for ee_name in ["left_foot", "right_foot", "left_hand", "right_hand"]:
                pos = ee_world_pub[ee_name]
                ee_positions.append(Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])))
            msg.end_effector_poses = ee_positions

            self.prev_ee_world_pub = {k: v.copy() for k, v in ee_world_pub.items()}

        self.vmp_pub.publish(msg)

        if self.arm_traj_pub is not None:
            if len(dof_pos) < 27 or len(dof_vel) < 27:
                rospy.logwarn_throttle(
                    1.0,
                    f"[XSense] Cannot publish /kuavo_arm_traj: expected 27 DOF, "
                    f"got position={len(dof_pos)}, velocity={len(dof_vel)}",
                )
            else:
                arm_traj = JointState()
                arm_traj.header.stamp = msg.header.stamp
                arm_traj.name = list(ARM_JOINT_NAMES)
                arm_traj.position = (np.asarray(dof_pos[13:27], dtype=np.float64) * 180.0 / np.pi).tolist()
                arm_traj.velocity = (np.asarray(dof_vel[13:27], dtype=np.float64) * 180.0 / np.pi).tolist()
                self.arm_traj_pub.publish(arm_traj)

        return root_loc_pub


    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def run(self):
        """Main loop."""
        rospy.loginfo(f"Subscribing to {self.topic_name} ...")
        
        rate = rospy.Rate(self.fps)

        # Wait for calibration
        rospy.loginfo("Waiting for rigid body descriptions...")
        cali_pose = None
        actual_human_height = None

        while cali_pose is None and not rospy.is_shutdown():
            seg_desc = self.xsense_seg_desc
            if seg_desc is None:
                rospy.loginfo_throttle(2.0, f"Waiting for segment description on {self.topic_name_cali} ...")
                time.sleep(0.001)
                continue

            try:
                cali_pose, actual_human_height = cali_pose_from_seg_desc(
                    rb_desc=seg_desc,
                    arm_pose_deg=self.cali_arm_pose_deg
                )

                if actual_human_height is None:
                    actual_human_height = 1.75  # fallback
                rospy.loginfo(f"Calibration ready. height={actual_human_height:.3f} m")
                break
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Failed to build calibration from segment description: {e}")

            time.sleep(0.001)
        
        if rospy.is_shutdown():
            return


        # Initialize retargeting
        rospy.loginfo("Initializing motion retargeting...")
        self.retarget = VMR_bvh(
            src_human=f"bvh_{self.format}",
            tgt_robot=self.robot,
            actual_human_height=actual_human_height,
            cali_pose=cali_pose,
        )

        # Initialize viewer if enabled
        if self.enable_viewer:
            self.viewer = RobotMotionViewer(robot_type=self.robot, motion_fps=60)

        rospy.loginfo("Starting motion retargeting loop...")
        rate = rospy.Rate(self.fps)

        while not rospy.is_shutdown():

            # Consumer tick stamp (this is what we use for dt to keep it smooth at 100Hz)
            pub_stamp = rospy.Time.now()

            # Take the newest buffered frame; if none available, skip this tick
            # avoid to publish duplicate data
            capture_stamp, frame_gmr = self._pop_latest_frame()

            if frame_gmr is not None:
                self._last_frame = frame_gmr
                self._last_capture_stamp = capture_stamp
            else:
                # No new frame from producer — sleep and retry next tick
                rate.sleep()
                continue

            if frame_gmr is None:
                rate.sleep()
                continue
            
            # Retarget motion 
            try:
                with self.retarget_lock:
                    qpos = self.retarget.retarget(frame_gmr)
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"retarget() failed: {e}")
                rate.sleep()
                continue

            if not hasattr(self.retarget, "scaled_human_data"):
                rate.sleep()
                continue

            # Keep raw solver qpos for GMR/Mink continuity
            qpos_raw = np.asarray(qpos, dtype=np.float64).copy()

            # Create a separate aligned copy only for publishing / viewing / saving
            qpos_pub = self.align_root_heading_only(qpos_raw.copy())

            # Wait until heading reference is initialized from the first 10 frames
            if qpos_pub is None:
                rate.sleep()
                continue

            root_loc = qpos_pub[:3]           # aligned root position
            root_rot_wxyz = qpos_pub[3:7]     # aligned root orientation (wxyz)
            yaw_resume_completed = self._complete_vmp_yaw_resume_if_needed(root_rot_wxyz)
            root_rot_wxyz = self._apply_vmp_yaw_correction(root_rot_wxyz)
            dof_pos = qpos_raw[7:-2]          # same as qpos_pub[7:-2], alignment only changes root

            # dt from publish stamp -> smooth/regular at consumer rate
            dt_sec = self._compute_dt_sec_from_pub_stamp(pub_stamp)

            # Publish VMP using raw qpos for GMR/FK state, then align/correct FK positions for VMP.
            root_loc_pub = self.publish_vmp_input(
                                    qpos_raw=qpos_raw,
                                    root_loc=root_loc,
                                    root_rot_wxyz=root_rot_wxyz,
                                    dof_pos=dof_pos,
                                    dt_sec=dt_sec,
                                    stamp=pub_stamp,
                                )

            # Publish readiness only after the corrected retarget pose. VMP
            # matches this stamp to the pose before releasing its frozen frame.
            with self._vmp_yaw_lock:
                if yaw_resume_completed:
                    self._vmp_yaw_ready_frames_remaining = 10
                publish_yaw_ready = self._vmp_yaw_ready_frames_remaining > 0
                if publish_yaw_ready:
                    self._vmp_yaw_ready_frames_remaining -= 1
            if publish_yaw_ready:
                ready = Header()
                ready.stamp = pub_stamp
                ready.frame_id = "xsense"
                self.vmp_yaw_resume_ready_pub.publish(ready)

            # Update viewer if enabled
            if self.viewer is not None:
                try:
                    qpos_view = qpos_pub.copy()
                    # qpos_view[2] = root_loc_pub[2]
                    # qpos_view[3:7] = root_rot_wxyz
                    self.viewer.step(
                        root_pos=qpos_view[:3],
                        root_rot=qpos_view[3:7],
                        dof_pos=qpos_view[7:],
                        human_motion_data=self.retarget.scaled_human_data,
                        rate_limit=False,
                    )
                except Exception as e:
                    rospy.logwarn_throttle(1.0, f"viewer.step failed: {e}")

            # Update previous state
            self.prev_root_loc = root_loc.copy()
            self.prev_root_rot_wxyz = root_rot_wxyz.copy()
            self.prev_dof_pos = dof_pos.copy()

            # enforce consumer loop at ~100Hz
            rate.sleep()
        

def main():
    try:
        node = GMRStreamingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
