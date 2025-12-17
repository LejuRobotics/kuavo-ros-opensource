#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

- 包含：
- 四元数插值基础：quat_normalize, quat_slerp, clamp
- 时间网格：generate_time_grid(keyframes, rate_hz)
- 末端位姿线性插值：precompute_eef_sequence(keyframes, rate_hz)
- 关节角插值：interpolate_q_linear(q_keyframes, t)

"""

import math
import numpy as np


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_lerp(q0, q1, t):
    """四元数量线性插值（Lerp）后归一化，q=[x,y,z,w]；做同向半球对齐。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
    q = (1.0 - t) * q0 + t * q1
    return quat_normalize(q)

def quat_slerp(q0, q1, t):
    """四元数球面线性插值（SLERP），q=[x,y,z,w]，带半球对齐与归一化。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:
        # 角度很小，退化为 LERP
        return quat_lerp(q0, q1, t)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) < 1e-8:
        return quat_normalize(q0)
    theta = theta_0 * t
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    q = s0 * q0 + s1 * q1
    return quat_normalize(q)




def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def generate_time_grid(keyframes, rate_hz):
    t_start = float(keyframes[0]['time'])
    t_end = float(keyframes[-1]['time'])
    dt = 1.0 / max(rate_hz, 1e-6)
    t = t_start
    times = []
    while t < t_end - 1e-9:
        times.append(round(t, 6))
        t += dt
    times.append(round(t_end, 6))
    return times

def precompute_eef_sequence(keyframes, rate_hz):
    """按固定采样率对关键帧进行线性插值，返回末端位姿序列。"""
    times = generate_time_grid(keyframes, rate_hz)
    seq = []

    def lerp(a, b, u):
        return (1.0 - u) * np.asarray(a, dtype=float) + u * np.asarray(b, dtype=float)

    n = len(keyframes)
    for t in times:
        if t <= keyframes[0]['time']:
            lp = keyframes[0]['left']['pos']; lq = keyframes[0]['left']['quat']
            rp = keyframes[0]['right']['pos']; rq = keyframes[0]['right']['quat']
        elif t >= keyframes[-1]['time']:
            lp = keyframes[-1]['left']['pos']; lq = keyframes[-1]['left']['quat']
            rp = keyframes[-1]['right']['pos']; rq = keyframes[-1]['right']['quat']
        else:
            i = 0
            for j in range(n - 1):
                if keyframes[j]['time'] <= t <= keyframes[j + 1]['time']:
                    i = j
                    break
            t0 = keyframes[i]['time']; t1 = keyframes[i + 1]['time']
            u = clamp((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0)
            lp0 = keyframes[i]['left']['pos']; lp1 = keyframes[i + 1]['left']['pos']
            rp0 = keyframes[i]['right']['pos']; rp1 = keyframes[i + 1]['right']['pos']
            lq0 = keyframes[i]['left']['quat']; lq1 = keyframes[i + 1]['left']['quat']
            rq0 = keyframes[i]['right']['quat']; rq1 = keyframes[i + 1]['right']['quat']
            lp = lerp(lp0, lp1, u)
            rp = lerp(rp0, rp1, u)
            # 姿态使用 SLERP
            lq = quat_slerp(lq0, lq1, u)
            rq = quat_slerp(rq0, rq1, u)
        seq.append({
            't': t,
            'left_pos': np.asarray(lp, dtype=float),
            'left_quat': np.asarray(lq, dtype=float),
            'right_pos': np.asarray(rp, dtype=float),
            'right_quat': np.asarray(rq, dtype=float),
        })
    return seq


def interpolate_q_linear(q_keyframes, t):
    """线性插值 14 维关节角：q(t) between nearest keyframes."""
    n = len(q_keyframes)
    if n == 0:
        return [0.0] * 14
    if t <= q_keyframes[0]['time']:
        return list(q_keyframes[0]['q'])
    if t >= q_keyframes[-1]['time']:
        return list(q_keyframes[-1]['q'])
    i = 0
    for j in range(n - 1):
        if q_keyframes[j]['time'] <= t <= q_keyframes[j + 1]['time']:
            i = j
            break
    t0 = q_keyframes[i]['time']
    t1 = q_keyframes[i + 1]['time']
    u = 0.0 if abs(t1 - t0) < 1e-9 else (t - t0) / (t1 - t0)
    q0 = np.asarray(q_keyframes[i]['q'], dtype=float)
    q1 = np.asarray(q_keyframes[i + 1]['q'], dtype=float)
    q = (1.0 - u) * q0 + u * q1
    return q.tolist()
