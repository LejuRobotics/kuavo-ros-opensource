"""Sensor-axis, fingertip and hand-frame input transformations."""

import numpy as np
from scipy.spatial.transform import Rotation as R

from .quaternion import (
    normalize_quat,
    quat_multiply,
    quat_wxyz_to_matrix,
    rpy_to_quat,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)

_SWAP_XZ = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])


_SWAP_YZ = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])


_SWAP_XY = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


_INITIAL_ROTATION = R.from_quat([0.0, 0.0, -0.7071067811865475, 0.7071067811865477])


_ROBOT_FRAME_ROTATION = R.from_euler("z", -90, degrees=True)


def to_robot_frame_quaternions(quat_wxyz):
    q = np.asarray(quat_wxyz, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"quat_wxyz must have shape (N, 4), got {q.shape}")
    q = q / np.clip(np.linalg.norm(q, axis=1, keepdims=True), 1e-12, None)
    input_rotations = R.from_quat(q[:, [1, 2, 3, 0]])
    xyzw = (_ROBOT_FRAME_ROTATION * input_rotations).as_quat()
    return xyzw[:, [3, 0, 1, 2]]


def mcp_abduction_angles(quat_wxyz, anchor_wxyz, finger_names):
    quat = np.asarray(quat_wxyz, dtype=float)
    quat = quat / np.clip(np.linalg.norm(quat, axis=1, keepdims=True), 1e-12, None)
    anchor = np.asarray(anchor_wxyz, dtype=float)
    anchor_rotation = R.from_quat(anchor[[1, 2, 3, 0]])
    input_rotations = R.from_quat(quat[:, [1, 2, 3, 0]])
    canonical_x = (anchor_rotation.inv() * input_rotations).as_euler("XYZ", degrees=False)[:, 0]
    x_angle = 0.5 * np.arctan2(np.sin(2.0 * canonical_x), np.cos(2.0 * canonical_x))
    return {
        name: float(value)
        for name, value in zip(finger_names, x_angle)
        if name in {"index", "little"}
    }


def swap_xz_quaternion(q_wxyz):
    rot = R.from_quat(wxyz_to_xyzw(q_wxyz)).as_matrix()
    return xyzw_to_wxyz(R.from_matrix(_SWAP_XZ @ rot @ _SWAP_XZ.T).as_quat())


def _relative_swap(q_ref_wxyz, q_cur_wxyz, permutation):
    r_ref = R.from_quat(wxyz_to_xyzw(q_ref_wxyz))
    r_cur = R.from_quat(wxyz_to_xyzw(q_cur_wxyz))
    r_rel = (r_ref.inv() * r_cur).as_matrix()
    r_new = r_ref * R.from_matrix(permutation @ r_rel @ permutation.T)
    return xyzw_to_wxyz(r_new.as_quat())


def swap_relative_yz(q_ref_wxyz, q_cur_wxyz):
    return _relative_swap(q_ref_wxyz, q_cur_wxyz, _SWAP_YZ)


def swap_relative_xz(q_ref_wxyz, q_cur_wxyz):
    return _relative_swap(q_ref_wxyz, q_cur_wxyz, _SWAP_XZ)


def swap_relative_xy(q_ref_wxyz, q_cur_wxyz):
    return _relative_swap(q_ref_wxyz, q_cur_wxyz, _SWAP_XY)


def swap_relative_xy_batch(quat_wxyz, quat_ref_wxyz):
    q = np.asarray(quat_wxyz, dtype=float)
    refs = np.asarray(quat_ref_wxyz, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"quat_wxyz must have shape (N, 4), got {q.shape}")
    if refs.shape != q.shape:
        raise ValueError(f"quat_ref_wxyz shape {refs.shape} != quat_wxyz shape {q.shape}")
    return np.array([swap_relative_xy(refs[i], q[i]) for i in range(q.shape[0])], dtype=float)


def apply_initial_rotation(q_wxyz):
    rot = _INITIAL_ROTATION * R.from_quat(wxyz_to_xyzw(q_wxyz))
    return xyzw_to_wxyz(rot.as_quat())


def rotate_finger_y180(q_wxyz, finger_name):
    rot = R.from_euler("y", 180, degrees=True) * R.from_quat(wxyz_to_xyzw(q_wxyz))
    return xyzw_to_wxyz(rot.as_quat())


def rotate_finger_y90(q_wxyz, finger_name):
    rot = R.from_euler("y", 90, degrees=True) * R.from_quat(wxyz_to_xyzw(q_wxyz))
    return xyzw_to_wxyz(rot.as_quat())


def rotate_finger_z_minus90(q_wxyz, finger_name):
    rot = R.from_euler("z", -90, degrees=True) * R.from_quat(wxyz_to_xyzw(q_wxyz))
    return xyzw_to_wxyz(rot.as_quat())


def align_quaternion_hemisphere(q, q_ref):
    q = normalize_quat(q)
    if q_ref is not None and np.dot(q, q_ref) < 0:
        return -q
    return q


def flip_relative_axes(q_ref_wxyz, q_cur_wxyz, plane="xy"):
    r_ref = R.from_quat(wxyz_to_xyzw(q_ref_wxyz))
    r_cur = R.from_quat(wxyz_to_xyzw(q_cur_wxyz))
    r_rel = (r_ref.inv() * r_cur).as_matrix()
    signs = {"x": 1.0, "y": 1.0, "z": 1.0}
    for axis in str(plane).lower():
        if axis in signs:
            signs[axis] = -1.0
    d = np.diag([signs["x"], signs["y"], signs["z"]])
    r_new = r_ref * R.from_matrix(d @ r_rel @ d)
    return xyzw_to_wxyz(r_new.as_quat())


def flip_relative_xy(q_ref_wxyz, q_cur_wxyz):
    r_ref = R.from_quat(wxyz_to_xyzw(q_ref_wxyz))
    r_cur = R.from_quat(wxyz_to_xyzw(q_cur_wxyz))
    r_rel = (r_ref.inv() * r_cur).as_matrix()
    d = np.diag([-1, -1, 1])
    r_new = r_ref * R.from_matrix(d @ r_rel @ d)
    return xyzw_to_wxyz(r_new.as_quat())


def finger_vector(cfg, finger_name, default_key, per_finger_key):
    default = cfg.get(default_key, [0.0, 0.0, 0.0])
    per_finger = cfg.get(per_finger_key, {})
    return np.asarray(per_finger.get(finger_name, default), dtype=float).reshape(3)


def module_to_tip(pos, quat_wxyz, cfg, finger_order=None):
    pos = np.asarray(pos, dtype=float)
    quat_wxyz = np.asarray(quat_wxyz, dtype=float)
    if pos.ndim != 2 or pos.shape[1] != 3:
        raise ValueError(f"pos must have shape (N, 3), got {pos.shape}")
    if quat_wxyz.ndim != 2 or quat_wxyz.shape[1] != 4:
        raise ValueError(f"quat_wxyz must have shape (N, 4), got {quat_wxyz.shape}")
    if len(pos) != len(quat_wxyz):
        raise ValueError(f"pos rows {len(pos)} != quat rows {len(quat_wxyz)}")
    if finger_order is None:
        finger_order = ["thumb", "index", "middle", "ring", "pinky"][: len(pos)]
    if len(finger_order) != len(pos):
        raise ValueError(f"finger_order length {len(finger_order)} != rows {len(pos)}")
    if not cfg.get("enabled", False):
        return pos.copy(), np.array([normalize_quat(q) for q in quat_wxyz], dtype=float)
    out_pos = pos.copy()
    out_quat = np.zeros_like(quat_wxyz, dtype=float)
    for i, name in enumerate(finger_order):
        q_module = normalize_quat(quat_wxyz[i])
        rot_module = quat_wxyz_to_matrix(q_module)
        offset_local = finger_vector(
            cfg, name, "default_offset_local_xyz", "per_finger_offset_local_xyz"
        )
        rpy_local = finger_vector(cfg, name, "default_rpy_local", "per_finger_rpy_local")
        offset_world = rot_module @ offset_local
        if cfg.get("offset_is_module_from_tip", True):
            out_pos[i] = pos[i] - offset_world
        else:
            out_pos[i] = pos[i] + offset_world
        out_quat[i] = quat_multiply(q_module, rpy_to_quat(rpy_local))
    return out_pos, out_quat


def module_to_tip_from_config(pos, quat_wxyz, config, finger_order=None):
    tcfg = config.get("input_transform", {})
    mtcfg = tcfg.get("module_to_tip", {})
    if finger_order is None:
        finger_order = config.get("finger_order", ["thumb", "index", "middle", "ring", "pinky"])
    return module_to_tip(pos, quat_wxyz, mtcfg, finger_order)


def transform_sensor_axes(pos, quat_wxyz, tcfg):
    p = np.asarray(pos, dtype=float).copy()
    q = np.asarray(quat_wxyz, dtype=float)
    if tcfg.get("negate_raw_z", True):
        p[:, 2] = -p[:, 2]
    axis_order = tcfg.get("axis_order")
    swap_xz = bool(tcfg.get("swap_xz", True))
    if axis_order is None:
        p = p[:, [2, 1, 0]].copy() if swap_xz else p.copy()
    else:
        order = np.asarray(axis_order, dtype=int).reshape(3)
        p = p[:, order].copy()
    axis_sign = np.asarray(tcfg.get("axis_sign", [1.0, 1.0, 1.0]), dtype=float).reshape(3)
    p = p * axis_sign
    q = np.array([swap_xz_quaternion(v) if swap_xz else normalize_quat(v) for v in q], dtype=float)
    if swap_xz:
        p[:, 0] = -p[:, 0]
    return p, q


def transform_positions(pos, tcfg, finger_order):
    p = np.asarray(pos, dtype=float)

    def apply_scale(v):
        return v * float(tcfg.get("scale", 1.0))

    def apply_offset(v):
        return v + np.asarray(tcfg.get("offset_xyz", [0.0, 0.0, 0.0]), dtype=float)

    def apply_per_finger_scale(v):
        out = v.copy()
        scales = tcfg.get("per_finger_scale", {})
        for i, name in enumerate(finger_order):
            out[i] = out[i] * np.asarray(scales.get(name, [1.0, 1.0, 1.0]), dtype=float)
        return out

    def apply_per_finger_offset(v):
        out = v.copy()
        offsets = tcfg.get("per_finger_offset_xyz", {})
        for i, name in enumerate(finger_order):
            out[i] = out[i] + np.asarray(offsets.get(name, [0.0, 0.0, 0.0]), dtype=float)
        return out

    if tcfg.get("scale_before_offset", True):
        return apply_per_finger_offset(apply_offset(apply_per_finger_scale(apply_scale(p))))
    return apply_per_finger_scale(apply_per_finger_offset(apply_scale(apply_offset(p))))


def transform_input(pos, quat_wxyz, tcfg, finger_order):
    p, q = transform_sensor_axes(pos, quat_wxyz, tcfg)
    return transform_positions(p, tcfg, finger_order), q


def transform_orientations(quat, quat_prev, quat_ref, tcfg, anchor_quat=None, finger_order=None):
    q = np.asarray(quat, dtype=float)
    if tcfg.get("apply_init_rotation", False):
        q = np.array([apply_initial_rotation(v) for v in q], dtype=float)
    q = np.array(
        [
            align_quaternion_hemisphere(v, None if quat_prev is None else quat_prev[i])
            for i, v in enumerate(q)
        ],
        dtype=float,
    )
    q_prev_out = q.copy()
    q_ref_out = q.copy() if quat_ref is None else np.asarray(quat_ref, dtype=float).copy()
    if anchor_quat is not None:
        q_ref_out = np.tile(normalize_quat(anchor_quat), (q.shape[0], 1))
    if tcfg.get("swap_xy_relative_rotation_all", False):
        q = swap_relative_xy_batch(q, q_ref_out)
    if tcfg.get("swap_yz_relative_rotation", False):
        q = np.array([swap_relative_yz(q_ref_out[i], q[i]) for i in range(q.shape[0])], dtype=float)
    names = finger_order or ["thumb", "index", "middle", "ring", "pinky"]
    if tcfg.get("apply_finger_init_rotation", False):
        q = np.array([rotate_finger_y180(q[i], names[i]) for i in range(q.shape[0])], dtype=float)
    if tcfg.get("flip_relative_rotation", False):
        default_plane = tcfg.get("flip_relative_rotation_plane", "xy")
        plane_per_finger = tcfg.get("flip_relative_rotation_plane_per_finger", {})
        q = np.array(
            [
                flip_relative_axes(
                    q_ref_out[i], q[i], plane=plane_per_finger.get(names[i], default_plane)
                )
                for i in range(q.shape[0])
            ],
            dtype=float,
        )
    if tcfg.get("swap_xy_relative_rotation", False):
        q = np.array(
            [
                swap_relative_xy(q_ref_out[i], q[i]) if names[i] in {"middle", "ring"} else q[i]
                for i in range(q.shape[0])
            ],
            dtype=float,
        )
    if tcfg.get("apply_finger_y90_init_rotation", False):
        q = np.array([rotate_finger_y90(q[i], names[i]) for i in range(q.shape[0])], dtype=float)
    if tcfg.get("swap_xz_relative_rotation", False):
        q = np.array([swap_relative_xz(q_ref_out[i], q[i]) for i in range(q.shape[0])], dtype=float)
    return q, q_prev_out, q_ref_out
