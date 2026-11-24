"""Angular decompositions and scalar optimization helpers."""

import numpy as np


def quat_xyz_angles_wxyz(quat_wxyz):
    q = np.asarray(quat_wxyz, dtype=float).reshape(4)
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    w, x, y, z = q
    return np.array(
        [
            np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)),
            np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        ],
        dtype=float,
    )


def quat_yz_plane_x_angle_wxyz(quat_wxyz):
    return float(quat_xyz_angles_wxyz(quat_wxyz)[0])


def golden_section_minimize(function, lower, upper, iterations):
    if upper <= lower:
        return float(lower)
    ratio = 0.5 * (np.sqrt(5.0) - 1.0)
    left = float(lower)
    right = float(upper)
    x1 = right - ratio * (right - left)
    x2 = left + ratio * (right - left)
    f1 = float(function(x1))
    f2 = float(function(x2))
    for _ in range(max(int(iterations), 0)):
        if f1 <= f2:
            right = x2
            x2 = x1
            f2 = f1
            x1 = right - ratio * (right - left)
            f1 = float(function(x1))
        else:
            left = x1
            x1 = x2
            f1 = f2
            x2 = left + ratio * (right - left)
            f2 = float(function(x2))
    return float(x1 if f1 <= f2 else x2)


def root_flexion_weight(self, position_q, finger_name, component):
    cfg = self.cfg.root_flexion_weight or {}
    roots = {"index": "index_j2", "middle": "middle_j1", "little": "little_j2"}
    if not cfg.get("enabled", False) or finger_name not in roots:
        return 1.0
    settings = cfg.get(component, {}) or {}
    start = float(settings.get("start_deg", 0.0))
    end = float(settings.get("end_deg", 120.0))
    minimum = float(settings.get("min_weight", 0.0))
    if not (
        np.isfinite([start, end, minimum]).all() and 0.0 <= start < end and 0.0 <= minimum <= 1.0
    ):
        raise ValueError(
            f"root_flexion_weight.{component}: require 0 <= start_deg < end_deg and 0 <= min_weight <= 1"
        )
    angle = float(np.degrees(position_q[self.qidx(roots[finger_name])]))
    fraction = float(np.clip((angle - start) / (end - start), 0.0, 1.0))
    return 1.0 - fraction * (1.0 - minimum)
