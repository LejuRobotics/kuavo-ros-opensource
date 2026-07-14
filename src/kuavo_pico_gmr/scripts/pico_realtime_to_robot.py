import os
import sys
import time
import pickle
import threading
from collections import deque
from typing import Dict, List, Tuple, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------
# Setup paths for GMR core modules
# -----------------------------------------------------------------------------
def setup_gmr_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    gmr_core_path = os.path.join(pkg_dir, "src", "kuavo_mocap_gmr", "gmr_core")
    if os.path.isdir(gmr_core_path) and gmr_core_path not in sys.path:
        sys.path.insert(0, gmr_core_path)
        print(f"[GMR] Added to path: {gmr_core_path}")


setup_gmr_path()

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse

from kuavo_msgs.msg import (
    JoySticks, picoPoseInfoList, picoPoseRetarget
)

from kuavo_gmr import RobotMotionViewer, VMR_bvh


# -----------------------------------------------------------------------------
# Incoming PICO body order (fixed)
# -----------------------------------------------------------------------------
pico_body_names: List[str] = [
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
]


# -----------------------------------------------------------------------------
# PICO -> LAFAN/GMR mapping
# Only the mapped joints are sent into the GMR retargeter/calibration.
# -----------------------------------------------------------------------------
PICO_TO_LAFAN: Dict[str, str] = {
    # torso
    "pelvis": "Hips",
    "spine1": "Spine",
    "neck": "Neck",
    "head": "Head",
    # arms
    "left_collar": "LeftShoulder",
    "left_shoulder": "LeftArm",
    "left_elbow": "LeftForeArm",
    "left_wrist": "LeftHand",
    "right_collar": "RightShoulder",
    "right_shoulder": "RightArm",
    "right_elbow": "RightForeArm",
    "right_wrist": "RightHand",
    # legs
    "left_hip": "LeftUpLeg",
    "left_knee": "LeftLeg",
    "left_ankle": "LeftFoot",
    "left_foot": "LeftToeBase",
    "right_hip": "RightUpLeg",
    "right_knee": "RightLeg",
    "right_ankle": "RightFoot",
    "right_foot": "RightToeBase",
}

upper_body_cali_part = ["LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "RightShoulder", "RightArm", "RightForeArm", "RightHand", "Neck", "Head"]

FrameDict = Dict[str, Tuple[np.ndarray, np.ndarray]]

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def _quat_normalize_wxyz(q: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4,)
    n = np.linalg.norm(q)
    if n < eps:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def average_quat_wxyz(quats: np.ndarray) -> np.ndarray:
    quats = np.asarray(quats, dtype=np.float64)
    if quats.ndim != 2 or quats.shape[1] != 4:
        raise ValueError(f"Expected (N,4), got {quats.shape}")

    ref = quats[0].copy()
    aligned = []
    for q in quats:
        qn = _quat_normalize_wxyz(q)
        if np.dot(qn, ref) < 0.0:
            qn = -qn
        aligned.append(qn)

    q_mean = np.mean(np.asarray(aligned), axis=0)
    return _quat_normalize_wxyz(q_mean)


def build_cali_pose_from_frames(frames: List[FrameDict]) -> FrameDict:
    if len(frames) == 0:
        raise ValueError("No frames provided for calibration")

    keys = list(frames[0].keys())
    cali_pose: FrameDict = {}

    for k in keys:
        pos_list = []
        quat_list = []
        for f in frames:
            if k not in f:
                continue
            p, q = f[k]
            pos_list.append(np.asarray(p, dtype=np.float64))
            quat_list.append(np.asarray(q, dtype=np.float64))

        if len(pos_list) == 0:
            continue

        pos_mean = np.mean(np.stack(pos_list, axis=0), axis=0)
        quat_mean = average_quat_wxyz(np.stack(quat_list, axis=0))
        cali_pose[k] = [pos_mean, quat_mean]

    return cali_pose


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


def map_pico_frame_to_gmr(frame_data: FrameDict) -> FrameDict:
    """Map canonicalized PICO frame into the source skeleton expected by GMR."""
    out: FrameDict = {}
    for pico_name, lafan_name in PICO_TO_LAFAN.items():
        if pico_name not in frame_data:
            raise ValueError(f"Missing expected PICO joint '{pico_name}' in frame data")
        pos, quat_wxyz = frame_data[pico_name]
        out[lafan_name] = [np.asarray(pos, dtype=np.float64), quat_wxyz]
    return out


def estimate_human_height_from_frame(frame_gmr: FrameDict) -> float:
    try:
        head_z = float(frame_gmr["Head"][0][2])
        l_foot_z = float(frame_gmr["LeftToeBase"][0][2])
        r_foot_z = float(frame_gmr["RightToeBase"][0][2])
        height = head_z - 0.5 * (l_foot_z + r_foot_z)
        return float(np.clip(height, 1.2, 2.2))
    except Exception:
        return 1.75



class PicoStreamingNode:
    """ROS node for GMR motion retargeting from PICO topic frames."""
    def __init__(self):
        rospy.init_node("pico_realtime_retargeter", anonymous=False)
        self._setup_cpu_affinity()

        # Parameters
        # NOTE: GMR IK config for "bvh_motive_streaming" only supports "kuavo_s52".
        # Do NOT read /robot_type here — that is for the physical robot, not the GMR target model.
        self.robot = rospy.get_param("~robot", "kuavo_s52")
        self.format = rospy.get_param("~format", "motive_streaming")
        self.enable_viewer = bool(rospy.get_param("~enable_viewer", True))
        self.topic_name = rospy.get_param("~topic", "/pico/world_bone_poses")
        self.fps = float(rospy.get_param("~fps", 100.0))
        self.frame_buffer_len = int(rospy.get_param("~frame_buffer_len", 3))

        # Calibration trigger / collection
        self.cali_pose = None
        self.cali_frame_count = int(rospy.get_param("~cali_frame_count", 10))
        self.cali_timeout_sec = float(rospy.get_param("~cali_timeout_sec", 3.0))
        # Warining: point towards the forward direction of pico to calibrate, and stand still
        self._cali_option = rospy.get_param("~cali_option", "upper_body") # default cali option from param
        self._cali_pending_option = None   # set by service handler: "upper_body" or "whole_body"
        self._cali_request_pending = False
        self._cali_running = False
        # RT+B → 半身校准,  LT+B → 全身校准
        self._calibrate_srv = rospy.Service(
            '/pico/gmr_calibrate', Trigger,
            lambda req: self._handle_calibrate_request(req, "upper_body"))
        self._calibrate_whole_srv = rospy.Service(
            '/pico/gmr_calibrate_whole', Trigger,
            lambda req: self._handle_calibrate_request(req, "whole_body"))

        self.contact_height_threshold = float(rospy.get_param("~contact_height_threshold", 0.12))
        self.contact_trans_threshold = float(rospy.get_param("~contact_trans_threshold", 0.02))

        self.retarget = None
        self.viewer = RobotMotionViewer(robot_type=self.robot, motion_fps=60) if self.enable_viewer else None
        self.retarget_lock = threading.Lock()

        # End effector link names for Kuavo
        self.ee_links = {
            "left_foot": "leg_l6_link",
            "right_foot": "leg_r6_link",
            "left_hand": "zarm_l7_link",
            "right_hand": "zarm_r7_link",
        }

        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None
        self.prev_pub_stamp = None
        self.prev_ee_world_pub = None

        # RT+Y freezes the stream; RT+X resumes it. Keep published root yaw
        # continuous across that resume by left-applying a yaw-only correction.
        self._vmp_yaw_lock = threading.Lock()
        self._vmp_yaw_correction = 0.0
        self._vmp_yaw_freeze_yaw = None
        self._vmp_yaw_resume_pending = False
        self._vmp_yaw_last_x = False
        self._vmp_yaw_last_y = False
        self._latest_published_root_rot_wxyz = None

        # Heading alignment state
        self._heading_R_delta = None
        self._heading_p0 = None
        self._heading_init_qpos_buf = deque(maxlen=6)

        # Producer/consumer buffer
        self.frame_buf = deque(maxlen=max(1, self.frame_buffer_len))
        self.frame_buf_lock = threading.Lock()

        self._last_frame = None
        self._last_capture_stamp = None

        # Statistics / saving
        self.total_frames_received = 0
        self.total_frames_processed = 0
        self.saved_qpos = []

        # ROS interfaces must come after internal state init to avoid callback races
        self.pico_pose_subscriber = rospy.Subscriber(self.topic_name, picoPoseInfoList, self.bone_poses_callback, queue_size=10)
        self.pico_joy_subscriber = rospy.Subscriber("/pico/joy", JoySticks, self.vmp_yaw_joy_callback, queue_size=10)

        self.vmp_pub = rospy.Publisher('/pico/retargeted_pose', picoPoseRetarget, queue_size=10)
  

        rospy.loginfo("=" * 80)
        rospy.loginfo("PICO Real-time Motion Retargeting")
        rospy.loginfo(f"Robot: {self.robot}")
        rospy.loginfo(f"Topic: {self.topic_name}")
        rospy.loginfo(f"Format: {self.format}")
        rospy.loginfo(f"FPS: {self.fps:.1f}")
        rospy.loginfo(f"Viewer: {'Enabled' if self.enable_viewer else 'Disabled'}")
        rospy.loginfo("VMP yaw invariant: RT+Y=freeze, RT+X=resume")
        rospy.loginfo("=" * 80)

    # ------------------------------------------------------------------
    # CPU affinity
    # ------------------------------------------------------------------
    def _setup_cpu_affinity(self):
        cpu_cores = rospy.get_param("~cpu_cores", [])
        if not cpu_cores:
            rospy.loginfo("[GMR] CPU affinity: not set (OS free scheduling)")
            return

        try:
            cpu_cores = [int(c) for c in cpu_cores]
        except (TypeError, ValueError):
            rospy.logwarn(f"[GMR] CPU affinity: invalid cpu_cores={cpu_cores}, skipping")
            return

        n_cpus = os.cpu_count() or 1
        invalid = [c for c in cpu_cores if c < 0 or c >= n_cpus]
        if invalid:
            rospy.logwarn(f"[GMR] CPU affinity: cores {invalid} out of range [0, {n_cpus - 1}], skipping")
            return

        try:
            os.sched_setaffinity(0, cpu_cores)
            actual = sorted(os.sched_getaffinity(0))
            rospy.loginfo(f"[GMR] CPU affinity: bound to cores {actual} (requested {cpu_cores})")
        except OSError as e:
            rospy.logwarn(f"[GMR] CPU affinity: failed to set cores {cpu_cores}: {e}")


    # ------------------------------------------------------------------
    # Callback / latest-frame ring buffer
    # ------------------------------------------------------------------
    def bone_poses_callback(self, msg: picoPoseInfoList):
        try:
            self.total_frames_received += 1
            if self.total_frames_received <= 3 or self.total_frames_received % 500 == 0:
                rospy.loginfo(f"[PICO] callback frame#{self.total_frames_received}, poses={len(msg.poses) if hasattr(msg,'poses') else '?'}")

            if not hasattr(msg, "poses"):
                rospy.logwarn_throttle(1.0, "[PICO] incoming message has no 'poses' field")
                return

            if len(msg.poses) < len(pico_body_names):
                rospy.logwarn_throttle(
                    2.0,
                    f"[PICO] expected {len(pico_body_names)} poses, got {len(msg.poses)}",
                )
                return

            if hasattr(msg, "header") and msg.header.stamp != rospy.Time():
                capture_stamp = msg.header.stamp
            else:
                capture_stamp = rospy.Time.now()

            frame_data: FrameDict = {}
            for bone_idx, pico_name in enumerate(pico_body_names):
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

                frame_data[pico_name] = [pos_xyz, quat_wxyz]

            # if 'spine1' in frame_data and 'pelvis' in frame_data:
            #     # PICO pelvis orientation is unreliable; use spine1 orientation instead.
            #     frame_data['pelvis'][1] = frame_data['spine1'][1]

            frame_gmr = map_pico_frame_to_gmr(frame_data)

            with self.frame_buf_lock:
                self.frame_buf.append((capture_stamp, frame_gmr))

        except Exception as e:
            rospy.logerr(f"[PICO] Error in callback: {e}")
            import traceback; traceback.print_exc()

    def offset_pico_height_drift(self, pico_frame: FrameDict) -> FrameDict:

        left_z = float(pico_frame["LeftToeBase"][0][2])
        right_z = float(pico_frame["RightToeBase"][0][2])
        lowest_z = min(left_z, right_z)
        return lowest_z
    
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
    def _mark_vmp_yaw_freeze(self):
        with self._vmp_yaw_lock:
            latest = None if self._latest_published_root_rot_wxyz is None else self._latest_published_root_rot_wxyz.copy()

        if latest is None:
            rospy.logwarn_throttle(1.0, "[VMPYawInvariant] freeze ignored: no published root orientation yet")
            return

        freeze_yaw = _yaw_from_quat_wxyz(latest)
        with self._vmp_yaw_lock:
            self._vmp_yaw_freeze_yaw = freeze_yaw
            self._vmp_yaw_resume_pending = False

        rospy.loginfo(
            f"[VMPYawInvariant] freeze yaw captured: {freeze_yaw:.6f} rad "
            f"({np.degrees(freeze_yaw):.2f} deg)"
        )

    def _request_vmp_yaw_resume(self):
        with self._vmp_yaw_lock:
            has_freeze_yaw = self._vmp_yaw_freeze_yaw is not None
            if has_freeze_yaw:
                self._vmp_yaw_resume_pending = True

        if not has_freeze_yaw:
            rospy.logwarn_throttle(1.0, "[VMPYawInvariant] resume ignored: freeze yaw was not captured")

    def vmp_yaw_joy_callback(self, joy: JoySticks):
        try:
            rt_pressed = float(joy.right_trigger) >= 0.5
            y_pressed = bool(joy.left_second_button_pressed)
            x_pressed = bool(joy.left_first_button_pressed)

            if rt_pressed and y_pressed and not self._vmp_yaw_last_y:
                self._mark_vmp_yaw_freeze()
            if rt_pressed and x_pressed and not self._vmp_yaw_last_x:
                self._request_vmp_yaw_resume()

            self._vmp_yaw_last_x = x_pressed
            self._vmp_yaw_last_y = y_pressed
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[VMPYawInvariant] joy callback failed: {e}")

    def _complete_vmp_yaw_resume_if_needed(self, root_rot_wxyz: np.ndarray):
        with self._vmp_yaw_lock:
            pending = self._vmp_yaw_resume_pending
            freeze_yaw = self._vmp_yaw_freeze_yaw

        if not pending or freeze_yaw is None:
            return

        resume_yaw = _yaw_from_quat_wxyz(root_rot_wxyz)
        yaw_correction = _wrap_to_pi(freeze_yaw - resume_yaw)

        with self._vmp_yaw_lock:
            self._vmp_yaw_correction = yaw_correction
            self._vmp_yaw_resume_pending = False
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

    def _apply_vmp_yaw_correction(self, root_rot_wxyz: np.ndarray) -> np.ndarray:
        root_rot_wxyz = _quat_normalize_wxyz(root_rot_wxyz)

        with self._vmp_yaw_lock:
            yaw_correction = self._vmp_yaw_correction

        corrected = _left_apply_yaw_to_quat_wxyz(root_rot_wxyz, yaw_correction)

        with self._vmp_yaw_lock:
            self._latest_published_root_rot_wxyz = corrected.copy()

        return corrected
    

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------
    def _handle_calibrate_request(self, req, cali_option: str = "upper_body"):
        if self._cali_running or self._cali_request_pending:
            return TriggerResponse(success=False, message="Calibration already running or pending.")

        self._cali_pending_option = cali_option
        self._cali_request_pending = True
        label = "半身(upper_body)" if cali_option == "upper_body" else "全身(whole_body)"
        rospy.loginfo(f"[PICO] Calibration requested: {label}")
        return TriggerResponse(success=True, message=f"Calibration request accepted: {cali_option}.")
    
    def _reset_runtime_state_after_calibration(self):
        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None
        self.prev_pub_stamp = None
        self.prev_ee_world_pub = None

        self._heading_R_delta = None
        self._heading_p0 = None
        self._heading_init_qpos_buf.clear()

        with self._vmp_yaw_lock:
            self._vmp_yaw_correction = 0.0
            self._vmp_yaw_freeze_yaw = None
            self._vmp_yaw_resume_pending = False
            self._vmp_yaw_last_x = False
            self._vmp_yaw_last_y = False
            self._latest_published_root_rot_wxyz = None

    def _calibration_callback(self, rate):
        # Use the option set by the service handler for this calibration round
        cali_option = self._cali_pending_option or self._cali_option
        self._cali_pending_option = None
        self._cali_request_pending = False
        self._cali_running = True

        label = "半身(upper_body)" if cali_option == "upper_body" else "全身(whole_body)"
        rospy.loginfo(f"[PICO] Starting {label} calibration...")

        try:
            with self.frame_buf_lock:
                self.frame_buf.clear()
            rospy.loginfo(
                f"[PICO] Start collecting calibration frames: "
                f"target={self.cali_frame_count}, timeout={self.cali_timeout_sec:.2f}s"
            )

            frames_for_cali: List[FrameDict] = []
            t0 = time.time()

            while not rospy.is_shutdown():
                capture_stamp, frame_gmr = self._pop_latest_frame()
                if frame_gmr is not None:
                    frames_for_cali.append(frame_gmr)
                    if len(frames_for_cali) >= self.cali_frame_count:
                        break

                if (time.time() - t0) > self.cali_timeout_sec:
                    break

                rate.sleep()

            if len(frames_for_cali) == 0:
                rospy.logwarn("[PICO] Calibration failed: collected 0 frames.")
                return

            rospy.loginfo(f"[PICO] Collected {len(frames_for_cali)} calibration frames.")

            self.cali_pose = build_cali_pose_from_frames(frames_for_cali)

            height_list = []
            for f in frames_for_cali:
                try:
                    height_list.append(estimate_human_height_from_frame(f))
                except Exception:
                    pass
            actual_human_height = float(np.mean(height_list)) if len(height_list) > 0 else None

            rospy.loginfo(
                f"[PICO] Calibration pose built. "
                f"estimated_human_height={actual_human_height if actual_human_height is not None else 'None'}"
            )

            with self.retarget_lock:
                if self.retarget is None:
                    rospy.loginfo(f"[PICO] Creating retargeter with calibration pose (cali_option={cali_option})...")
                    self.retarget = VMR_bvh(
                        src_human=f"bvh_{self.format}",
                        tgt_robot=self.robot,
                        actual_human_height=actual_human_height,
                        cali_pose=self.cali_pose,
                        cali_option=cali_option,
                        cali_part=upper_body_cali_part,
                    )
                else:
                    rospy.loginfo(f"[PICO] Applying calibration to existing retargeter (cali_option={cali_option})...")
                    self.retarget.autocalibrate_offsets_from_bvh_default_pose(
                        self.cali_pose,
                        align_feet=True,
                        cali_option=cali_option,
                        cali_part=upper_body_cali_part,
                    )
                    if hasattr(self.retarget, "_build_offset_cache"):
                        self.retarget._build_offset_cache()

            self._reset_runtime_state_after_calibration()
            rospy.loginfo(f"\033[92m[PICO] ✅ {label} calibration finished.\033[0m")

        except Exception as e:
            rospy.logerr(f"[PICO] Calibration failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._cali_running = False


    # ------------------------------------------------------------------
    # ROS publishing
    # ------------------------------------------------------------------
    def _compute_dt_sec_from_pub_stamp(self, pub_stamp):
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
        msg = picoPoseRetarget()
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
        return root_loc_pub


    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def run(self):
        """Main loop."""
        rospy.loginfo(f"Subscribing to {self.topic_name} ...")
        
        rate = rospy.Rate(self.fps)

        # Initialize retargeting
        rospy.loginfo("Initializing motion retargeting...")
        self.retarget = VMR_bvh(
            src_human=f"bvh_{self.format}",
            tgt_robot=self.robot,
            actual_human_height=None,
            cali_pose=None,
        )

        rospy.loginfo("Starting motion retargeting loop...")
        rate = rospy.Rate(self.fps)

        while not rospy.is_shutdown():
            # Service handler requested a calibration — run it here in the
            # main loop so that rate.sleep() keeps rospy callbacks alive.
            if self._cali_request_pending:
                self._calibration_callback(rate)
                continue

            # Consumer tick stamp (this is what we use for dt to keep it smooth at 100Hz)
            pub_stamp = rospy.Time.now()

            # Take the newest buffered frame; if none available, skip this tick
            # avoid to publish duplicate data
            capture_stamp, frame_gmr = self._pop_latest_frame()

            if frame_gmr is not None:
                self._last_frame = frame_gmr
                self._last_capture_stamp = capture_stamp
            else:
                if self.total_frames_processed == 0 and self.total_frames_received == 0:
                    rospy.loginfo_throttle(
                        2.0,
                        "[PICO] No frames yet. Waiting for Pico pairing / IP / stream..."
                    )
                # No new frame from producer — sleep and retry next tick
                rate.sleep()
                continue

            # delta_z = self.offset_pico_height_drift(frame_gmr)

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
            self._complete_vmp_yaw_resume_if_needed(root_rot_wxyz)
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

            # Update viewer if enabled
            if self.viewer is not None:
                try:
                    qpos_view = qpos_pub.copy()
                    qpos_view[2] = root_loc_pub[2]
                    qpos_view[3:7] = root_rot_wxyz
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

            self.total_frames_processed += 1

            # enforce consumer loop at ~100Hz
            rate.sleep()
        

def main():
    try:
        node = PicoStreamingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
