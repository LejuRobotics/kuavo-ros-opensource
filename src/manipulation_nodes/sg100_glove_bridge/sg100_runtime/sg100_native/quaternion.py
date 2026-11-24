"""Quaternion operations using scalar-first (w, x, y, z) arrays."""

import numpy as np
from scipy.spatial.transform import Rotation as R


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_multiply(a, b):
    aw, ax, ay, az = normalize_quat(a)
    bw, bx, by, bz = normalize_quat(b)
    return normalize_quat(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ]
        )
    )


def quat_wxyz_to_matrix(q):
    w, x, y, z = normalize_quat(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_to_quat(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return normalize_quat(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        )
    )


def wxyz_to_xyzw(q):
    return normalize_quat(q)[[1, 2, 3, 0]]


def xyzw_to_wxyz(q):
    return normalize_quat(np.asarray(q, dtype=float)[[3, 0, 1, 2]])


def quat_conjugate(q):
    q = np.asarray(q, dtype=float).copy()
    q[1:] *= -1
    return q


def matrix_to_quat_wxyz(matrix):
    return xyzw_to_wxyz(R.from_matrix(matrix).as_quat())
