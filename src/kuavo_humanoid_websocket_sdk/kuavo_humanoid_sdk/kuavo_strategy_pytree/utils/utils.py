import numpy as np

import sys
import os
from datetime import datetime

class Logger:
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, "w", encoding='utf-8')

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

def normalize_angle(angle: float) -> float:
    """
    将角度归一化到[-pi, pi]范围内

    Args:
        angle: 输入角度，单位弧度

    Returns:
        归一化后的角度，单位弧度
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi

    return angle

def quaternion_inverse(q):
    q = np.array(q)
    return np.array([ -q[0], -q[1], -q[2], q[3] ]) / np.dot(q, q)

def quaternion_multiply(q1, q2):
    """
    四元数乘法，用于组合旋转
    q1, q2: 两个四元数 [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return [x, y, z, w]


def quaternion_rotate(q, v):
    """
    使用四元数旋转向量
    q: 四元数 [x, y, z, w]
    v: 三维向量 [x, y, z]
    """
    # 提取四元数分量
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # 计算 q × v（四元数和向量的叉积）
    cross_qv_x = qy * vz - qz * vy
    cross_qv_y = qz * vx - qx * vz
    cross_qv_z = qx * vy - qy * vx

    # 计算 q × v + qw·v
    term_x = cross_qv_x + qw * vx
    term_y = cross_qv_y + qw * vy
    term_z = cross_qv_z + qw * vz

    # 计算 q × (q × v + qw·v)
    cross_q_term_x = qy * term_z - qz * term_y
    cross_q_term_y = qz * term_x - qx * term_z
    cross_q_term_z = qx * term_y - qy * term_x

    # 最终结果 v' = v + 2·(q × (q × v + qw·v))
    result_x = vx + 2 * cross_q_term_x
    result_y = vy + 2 * cross_q_term_y
    result_z = vz + 2 * cross_q_term_z

    return [result_x, result_y, result_z]

