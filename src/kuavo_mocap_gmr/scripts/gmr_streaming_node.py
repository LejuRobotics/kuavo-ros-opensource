#!/usr/bin/env python3
"""
GMR Streaming ROS Node

Real-time motion retargeting from OptiTrack to Kuavo S52 robot.
Publishes VMP input data, skeleton frames, and rigid body descriptions.

Usage:
    roslaunch kuavo_mocap_gmr gmr_streaming.launch
"""

import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque
from datetime import datetime



# Setup paths for GMR core modules
# This handles both direct execution and roslaunch scenarios
def setup_gmr_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    gmr_core_path = os.path.join(pkg_dir, 'src', 'kuavo_mocap_gmr', 'gmr_core')

    # Add to path if not already there
    if os.path.isdir(gmr_core_path) and gmr_core_path not in sys.path:
        sys.path.insert(0, gmr_core_path)
        print(f"[GMR] Added to path: {gmr_core_path}")

setup_gmr_path()

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

# Import custom messages from kuavo_msgs
from kuavo_msgs.msg import (
    MocapPoseRetarget,
    RigidBodyFrame,
    SkeletonFrame,
    RigidBodyDescription,
)

# Import GMR modules
from GMRmodule_bvh import VMR_bvh
from robot_motion_viewer import RobotMotionViewer
from utils.motivestreaming import cali_pose_from_rb_desc_
import utils.lafan_vendor.utils as utils
from optitrack_streaming import (
    start_streaming,
    get_frame,
    get_latest_rigid_body_descriptions,
)


# ------------------------------------------------------------
# Coordinate canonicalization (OptiTrack streaming -> BVH loader canonical)
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


# ---------------------------------------------------------------------------
# Pure-numpy angular velocity from two quaternions (wxyz convention)
# ---------------------------------------------------------------------------
def _angular_velocity_wxyz(q_prev_wxyz, q_curr_wxyz, inv_dt):
    """Compute angular velocity from two unit quaternions (wxyz).

    Equivalent to:  dR = R_curr * R_prev.inv();  w = dR.as_rotvec() * inv_dt
    but avoids creating scipy Rotation objects.
    """
    # Normalize
    qp = np.asarray(q_prev_wxyz, dtype=np.float64)
    qc = np.asarray(q_curr_wxyz, dtype=np.float64)
    qp = qp / max(np.linalg.norm(qp), 1e-12)
    qc = qc / max(np.linalg.norm(qc), 1e-12)

    # dq = q_curr * conj(q_prev)    (Hamilton product, wxyz)
    # conj(q) = (w, -x, -y, -z)
    pw, px, py, pz = qp[0], -qp[1], -qp[2], -qp[3]   # conjugate
    cw, cx, cy, cz = qc[0], qc[1], qc[2], qc[3]
    dw = cw*pw - cx*px - cy*py - cz*pz
    dx = cw*px + cx*pw + cy*pz - cz*py
    dy = cw*py - cx*pz + cy*pw + cz*px
    dz = cw*pz + cx*py - cy*px + cz*pw

    # Ensure shortest path (dw >= 0)
    if dw < 0.0:
        dw, dx, dy, dz = -dw, -dx, -dy, -dz

    # rotvec = axis * angle;  for unit quaternion: q = (cos(a/2), sin(a/2)*axis)
    sin_half = np.sqrt(dx*dx + dy*dy + dz*dz)
    if sin_half < 1e-10:
        return np.zeros(3, dtype=np.float64)

    half_angle = np.arctan2(sin_half, dw)
    angle = 2.0 * half_angle
    axis = np.array([dx, dy, dz], dtype=np.float64) / sin_half

    return axis * angle * inv_dt


class GMRStreamingNode:
    """ROS node for GMR motion retargeting from OptiTrack."""

    def __init__(self):
        rospy.init_node('gmr_streaming_node', anonymous=False)

        # CPU affinity: bind process to specific cores for deterministic scheduling
        self._setup_cpu_affinity()

        # Load parameters
        self.server_ip = rospy.get_param('~server_ip', '192.168.200.160')
        self.client_ip = rospy.get_param('~client_ip', '192.168.200.117')
        self.use_multicast = rospy.get_param('~use_multicast', False)
        self.robot = rospy.get_param('~robot', 'kuavo_s52')
        self.format = rospy.get_param('~format', 'motive_streaming')
        self.cali_arm_pose_deg = rospy.get_param('~cali_arm_pose_deg', 0.0)
        self.fps = float(rospy.get_param('~fps', 100.0))  # consumer rate
        self.enable_viewer = rospy.get_param('~enable_viewer', False)

        # Producer (get_frame) rate + buffer
        self.frame_buffer_len = int(rospy.get_param('~frame_buffer_len', 3)) # ring buffer length

        # Publishers
        self.vmp_pub = rospy.Publisher('/gmr/vmp_input', MocapPoseRetarget, queue_size=10)
        self.frame_pub = rospy.Publisher('/gmr/skeleton_frame', SkeletonFrame, queue_size=10)
        self.rb_desc_pub = rospy.Publisher('/gmr/rigid_body_desc', RigidBodyDescription, queue_size=1, latch=True)

        # State variables
        self.retarget = None
        self.viewer = None
        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None
        self.rb_desc_published = False

        self._heading_R_delta = None  # scipy Rotation
        self._heading_p0 = None       # np.ndarray(3,)

        # dt based on ROS stamp (publish-time 100Hz)
        self.prev_pub_stamp = None

        # Thread safety for retarget/config access
        self.retarget_lock = threading.Lock()

        # Retarget timing (ms) for profiling (bounded to avoid unbounded memory growth)
        self.timing_max_samples = int(rospy.get_param('~timing_max_samples', 200000))

        # EE/FK timing (ms) for profiling
        self.ee_solve_ms = deque(maxlen=max(1, int(rospy.get_param('~ee_timing_max_samples', self.timing_max_samples))))

        # End effector link names for kuavo_s52
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

        self.saved_root_pos = []
        self.saved_root_rot = []   # save as xyzw
        self.saved_dof_pos = []
        self.saved_qpos = []       #
        self._heading_init_qpos_buf = deque(maxlen=6)   
        self.prev_ee_world_pub = None   # aligned + ground-dragged EE world positions from previous published frame

        self.contact_height_threshold = rospy.get_param('~contact_height_threshold', 0.12)
        self.contact_trans_threshold = rospy.get_param('~contact_trans_threshold', 0.02)

        self._last_frame = None
        self._last_capture_stamp = None

        rospy.loginfo(f"GMR Streaming Node initialized")
        rospy.loginfo(f"  Server IP: {self.server_ip}")
        rospy.loginfo(f"  Client IP: {self.client_ip}")
        rospy.loginfo(f"  Robot: {self.robot}")
        rospy.loginfo(f"  Format: {self.format}")
        rospy.loginfo(f"  FPS: {self.fps:.1f}")

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

    # ------------------------------------------------------------
    # Background producer: get_frame() -> ring buffer
    # ------------------------------------------------------------
    def _start_frame_thread(self):
        if self._frame_thread is not None:
            return
        self._stop_event.clear()
        self._frame_thread = threading.Thread(target=self._frame_producer_loop, daemon=True)
        self._frame_thread.start()
        rospy.on_shutdown(self._stop_background)

    def _stop_background(self):
        self._stop_event.set()
        try:
            if self._frame_thread is not None:
                self._frame_thread.join(timeout=0.5)
        except Exception:
            pass

    def _frame_producer_loop(self):
        """
        Producer: call get_frame() as fast as it can provide frames.
        Push newest into deque. No 100Hz restriction here.
        """
        while (not rospy.is_shutdown()) and (not self._stop_event.is_set()):
            frame = get_frame()

            if frame is None:
                
                time.sleep(0.00001)  # avoid burning CPU if upstream has no data momentarily
                continue

            # "capture stamp" (best-effort). If you can get NatNet timestamp, use it here.
            capture_stamp = rospy.Time.now()

            frame_can = canonicalize_frame(frame)
            with self.frame_buf_lock:
                self.frame_buf.append((capture_stamp, frame_can))

    def _pop_latest_frame(self):
        """Consumer: return latest (stamp, frame) and drop older ones."""
        with self.frame_buf_lock:
            if not self.frame_buf:
                return None, None
            stamp, frame = self.frame_buf[-1]
            self.frame_buf.clear()
            return stamp, frame
    
    # ------------------------------------------------------------
    # ROS publishing
    # ------------------------------------------------------------
    def publish_rb_desc(self, rb_desc):
        """Publish rigid body descriptions (T-pose static data)."""
        msg = RigidBodyDescription()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "optitrack"

        ids = sorted(rb_desc.keys())
        for rid in ids:
            info = rb_desc[rid]
            msg.ids.append(rid)

            name = info.get('name', b'')
            if isinstance(name, bytes):
                name = name.decode('utf-8', errors='ignore')
            msg.names.append(name)

            msg.parent_ids.append(int(info.get('parent_id', 0)))

            offset = info.get('offset', [0, 0, 0])
            msg.offsets.append(Point(x=offset[0], y=offset[1], z=offset[2]))

        self.rb_desc_pub.publish(msg)
        rospy.loginfo("Published rigid body descriptions")

    def publish_skeleton_frame(self, frame, stamp=None):
        """Publish skeleton frame (canonicalized OptiTrack data)."""
        msg = SkeletonFrame()
        msg.header = Header()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = "optitrack"

        for name, (pos, quat_wxyz) in frame.items():
            rb = RigidBodyFrame()
            rb.name = name
            rb.position = Point(x=pos[0], y=pos[1], z=pos[2])
            # Convert wxyz to xyzw for ROS
            rb.orientation = Quaternion(
                x=quat_wxyz[1], y=quat_wxyz[2], z=quat_wxyz[3], w=quat_wxyz[0]
            )
            msg.rigid_bodies.append(rb)

        self.frame_pub.publish(msg)

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
    
    def align_root_heading_only(self, qpos_arr):
   
        qpos_arr = np.asarray(qpos_arr, dtype=np.float64).copy()

        try:
            # ------------------------------------------------------------
            # Initialization stage: wait until we have 6 frames
            # ------------------------------------------------------------
            if self._heading_R_delta is None:
                self._heading_init_qpos_buf.append(qpos_arr.copy())

                if len(self._heading_init_qpos_buf) < 6:
                    return None

                pos_xy = []
                fwd_xy_list = []

                for qk in self._heading_init_qpos_buf:
                    # root position
                    pos_xy.append(np.asarray(qk[:2], dtype=np.float64))

                    # root orientation -> horizontal forward vector
                    q_wxyz = np.asarray(qk[3:7], dtype=np.float64)
                    q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
                    q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

                    Rk = R.from_quat(q_xyzw)
                    fwd = Rk.apply([1.0, 0.0, 0.0])[:2]   # x-forward convention
                    n = np.linalg.norm(fwd)
                    if n > 1e-8 and np.all(np.isfinite(fwd)):
                        fwd_xy_list.append(fwd / n)

                # -------------------------
                # Body-facing estimate
                # -------------------------
                body_samples = fwd_xy_list[1:]

                if len(body_samples) == 0:
                    body_vec = np.array([1.0, 0.0], dtype=np.float64)
                else:
                    seed = np.sum(np.asarray(body_samples), axis=0)
                    seed_n = np.linalg.norm(seed)
                    if seed_n < 1e-8:
                        seed = np.array([1.0, 0.0], dtype=np.float64)
                    else:
                        seed = seed / seed_n

                    # Reject strong outliers
                    keep = [v for v in body_samples if np.dot(v, seed) > 0.5]  # within ~60 deg

                    if len(keep) >= 3:
                        body_vec = np.sum(np.asarray(keep), axis=0)
                    else:
                        body_vec = np.sum(np.asarray(body_samples), axis=0)

                    body_n = np.linalg.norm(body_vec)
                    if body_n < 1e-8:
                        body_vec = seed
                    else:
                        body_vec = body_vec / body_n

                # -------------------------
                # Trajectory estimate
                # -------------------------
                p0_xy = pos_xy[0]
                p4_xy = pos_xy[-1]
                d_xy = p4_xy - p0_xy
                d_norm = np.linalg.norm(d_xy)

                # If there is motion, blend trajectory direction with body direction.
                # If almost standing, rely on body-facing only.
                move_thresh = 0.05   
                full_traj_thresh = 0.1  

                if d_norm > move_thresh and np.all(np.isfinite(d_xy)):
                    traj_vec = d_xy / d_norm
                    alpha = np.clip((d_norm - move_thresh) / (full_traj_thresh - move_thresh), 0.0, 1.0)

                    v = (1.0 - alpha) * body_vec + alpha * traj_vec
                    v_n = np.linalg.norm(v)
                    if v_n < 1e-8:
                        v = traj_vec
                    else:
                        v = v / v_n
                else:
                    v = body_vec

                yaw0 = np.arctan2(v[1], v[0])
                yaw0 = (yaw0 + np.pi) % (2.0 * np.pi) - np.pi   # wrap to [-pi, pi)

                self._heading_R_delta = R.from_euler("z", -yaw0)
                self._heading_p0 = self._heading_init_qpos_buf[0][:3].copy()

                rospy.loginfo(
                    f"[HeadingAlign] initialized from first 5 frames: "
                    f"yaw0={yaw0:.6f} rad, deg={np.degrees(yaw0):.3f}, "
                    f"d_xy=({d_xy[0]:.6f}, {d_xy[1]:.6f}), d_norm={d_norm:.6f}"
                )

                self._heading_init_qpos_buf.clear()

            # ------------------------------------------------------------
            # Apply fixed heading alignment
            # ------------------------------------------------------------
            root_loc = qpos_arr[:3]
            q_wxyz = qpos_arr[3:7]

            q_xyzw = q_wxyz[[1, 2, 3, 0]].copy()
            q_xyzw /= np.clip(np.linalg.norm(q_xyzw), 1e-12, None)

            R_delta = self._heading_R_delta
            p0 = self._heading_p0

            root_loc_aligned = R_delta.apply(root_loc - p0) + p0
            root_rot_xyzw_aligned = (R_delta * R.from_quat(q_xyzw)).as_quat()
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
        
    def publish_vmp_input(self, qpos_raw, root_loc, root_rot_wxyz, dof_pos, dt_sec, stamp=None):
        """Publish VMP input data."""
        msg = MocapPoseRetarget()
        msg.header = Header()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = "world"

        # Base link pose uses aligned/root-published frame
        root_loc_pub = np.asarray(root_loc, dtype=np.float64).copy()
        msg.base_link_pose = Pose()
        msg.base_link_pose.position = Point(x=root_loc_pub[0], y=root_loc_pub[1], z=root_loc_pub[2])
        msg.base_link_pose.orientation = Quaternion(
            x=root_rot_wxyz[1], y=root_rot_wxyz[2], z=root_rot_wxyz[3], w=root_rot_wxyz[0]
        )

        if (dt_sec is None) or (self.prev_root_loc is None) or (self.prev_root_rot_wxyz is None) or (self.prev_dof_pos is None):
            v_lin = np.zeros(3)
            w_ang = np.zeros(3)
            dof_vel = np.zeros_like(dof_pos)
        else:
            inv_dt = 1.0 / dt_sec
            v_lin = (root_loc - self.prev_root_loc) * inv_dt
            w_ang = _angular_velocity_wxyz(self.prev_root_rot_wxyz, root_rot_wxyz, inv_dt)
            dof_vel = (dof_pos - self.prev_dof_pos) * inv_dt

        # Base velocity: [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
        msg.base_velocity = np.concatenate([v_lin, w_ang]).tolist()

        # Joint positions and velocities (27 DOF)
        msg.joint_position = dof_pos.tolist()
        msg.joint_velocity = dof_vel.tolist()

        # End effector positions in published/aligned world frame
        contact_mask = np.zeros(2, dtype=np.float32)  # [L, R]
        _t0 = time.perf_counter()
        try:
            with self.retarget_lock:
                self.retarget.configuration.update(q=qpos_raw)

                ee_world_pub = {}
                for ee_name in ['left_foot', 'right_foot', 'left_hand', 'right_hand']:
                    link_name = self.ee_links[ee_name]
                    se = self.retarget.configuration.get_transform_frame_to_world(link_name, frame_type="body")
                    pos = np.asarray(se.translation(), dtype=np.float64)

                    # apply the same global heading alignment as qpos_pub
                    if self._heading_R_delta is not None and self._heading_p0 is not None:
                        pos = self._heading_R_delta.apply(pos - self._heading_p0) + self._heading_p0

                    ee_world_pub[ee_name] = pos

                # # ------------------------------------------------------------
                # # Streaming foot contact detection
                # # current vs previous published EE positions
                # # ------------------------------------------------------------
                # l_now = ee_world_pub['left_foot']
                # r_now = ee_world_pub['right_foot']

                # if self.prev_ee_world_pub is None:
                #     l_contact = bool(l_now[2] < self.contact_height_threshold)
                #     r_contact = bool(r_now[2] < self.contact_height_threshold)
                # else:
                #     l_prev = self.prev_ee_world_pub['left_foot']
                #     r_prev = self.prev_ee_world_pub['right_foot']

                #     l_disp_xy = np.max(np.abs(l_now[:2] - l_prev[:2]))
                #     r_disp_xy = np.max(np.abs(r_now[:2] - r_prev[:2]))

                #     l_contact = bool(
                #         (l_disp_xy < self.contact_trans_threshold) and
                #         (l_now[2] < self.contact_height_threshold)
                #     )
                #     r_contact = bool(
                #         (r_disp_xy < self.contact_trans_threshold) and
                #         (r_now[2] < self.contact_height_threshold)
                #     )

                # contact_mask[0] = float(l_contact)
                # contact_mask[1] = float(r_contact)

                # contact_z = []
                # if l_contact:
                #     contact_z.append(float(l_now[2]))
                # if r_contact:
                #     contact_z.append(float(r_now[2]))

                # dz_ground = 0.0
                # if len(contact_z) > 0:
                #     dz_ground = -float(np.mean(contact_z)) + 0.055   # smoother for double support
                #     root_loc_pub[2] += dz_ground

                #     for k in ee_world_pub.keys():
                #         ee_world_pub[k] = ee_world_pub[k].copy()
                #         ee_world_pub[k][2] += dz_ground

                # # update published base pose z only
                # msg.base_link_pose.position.z = float(root_loc_pub[2])

                ee_positions = []
                for ee_name in ['left_foot', 'right_foot', 'left_hand', 'right_hand']:
                    pos = ee_world_pub[ee_name]
                    ee_positions.append(Point(
                        x=float(pos[0]),
                        y=float(pos[1]),
                        z=float(pos[2]),
                    ))

                msg.end_effector_poses = ee_positions

                # store EE positions after published z-drag for next contact check
                self.prev_ee_world_pub = {k: v.copy() for k, v in ee_world_pub.items()}

        finally:
            self.ee_solve_ms.append((time.perf_counter() - _t0) * 1000.0)

        self.vmp_pub.publish(msg)
        return root_loc_pub
    
    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def run(self):
        """Main loop."""
        # Start OptiTrack streaming
        rospy.loginfo("Starting OptiTrack streaming...")
        client = start_streaming(
            server_ip=self.server_ip,
            client_ip=self.client_ip,
            use_multicast=self.use_multicast,
            data_queue_maxsize=10,
            data_queue_policy="drop_oldest",
        )

        if not client:
            rospy.logerr("Failed to setup OptiTrack client")
            return

        rospy.loginfo("OptiTrack streaming started")

        # Start background get_frame producer (ring buffer)
        self._start_frame_thread()

        # Wait for calibration
        rospy.loginfo("Waiting for rigid body descriptions...")
        cali_pose = None
        actual_human_height = None

        while cali_pose is None and not rospy.is_shutdown():
            rb_desc = get_latest_rigid_body_descriptions()
            if rb_desc:
                rospy.loginfo("Received rigid body descriptions")

                # Publish rb_desc (latched, only once)
                if not self.rb_desc_published:
                    self.publish_rb_desc(rb_desc)
                    self.rb_desc_published = True

                cali_pose, actual_human_height = cali_pose_from_rb_desc_(
                    rb_desc=rb_desc,
                    arm_pose_deg=self.cali_arm_pose_deg
                )

                if actual_human_height is None:
                    actual_human_height = 1.75  # fallback
                rospy.loginfo(f"Calibration ready. height={actual_human_height:.3f} m")
                break

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
            # (do NOT reuse the old frame — that would publish duplicate data)
            capture_stamp, frame_ = self._pop_latest_frame()
            if frame_ is not None:
                self._last_frame = frame_
                self._last_capture_stamp = capture_stamp
            else:
                # No new frame from producer — sleep and retry next tick
                rate.sleep()
                continue

            if frame_ is None:
                rate.sleep()
                continue

            # Publish skeleton frame using capture stamp
            self.publish_skeleton_frame(frame_, stamp=capture_stamp if capture_stamp is not None else pub_stamp)

            # Retarget motion 
            try:
                with self.retarget_lock:
                    qpos = self.retarget.retarget(frame_)
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
            dof_pos = qpos_raw[7:-2]          # same as qpos_pub[7:-2], alignment only changes root

            # dt from publish stamp -> smooth/regular at consumer rate
            dt_sec = self._compute_dt_sec_from_pub_stamp(pub_stamp)

            # Publish VMP using aligned base pose, but keep raw qpos for internal FK
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


if __name__ == '__main__':
    main()
