#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- default_config_path(): 返回默认配置文件路径
- get_current_eef(): 从 /sensors_data_raw 获取关节角并调用 /ik/fk_srv 得到当前左右末端位姿
- load_keyframes(config_path): 读取 YAML 关键帧，必要时用当前末端位姿补 t=0

YAML 关键帧格式至少包含：
    - time: float
    - left: { pos_xyz: [x,y,z], quat_xyzw: [x,y,z,w] }
    - right: { pos_xyz: [x,y,z], quat_xyzw: [x,y,z,w] }

可选标签字段（任意其一存在即视为被标记）：
    - mark | flag | label | tag | marker | signal | emit_flag

加载后每个 keyframe 为：
{
    'time': float,
    'left': {'pos': np.ndarray(3), 'quat': np.ndarray(4)},
    'right': {'pos': np.ndarray(3), 'quat': np.ndarray(4)},
    'mark': bool  # 若在 YAML 中提供了任一标签字段，则为 True，否则 False
}
"""
import os
import math
import yaml
import rospy
import numpy as np
from kuavo_msgs.srv import fkSrv
from kuavo_msgs.msg import sensorsData


def default_config_path():
    return os.path.join(os.path.dirname(__file__), "../config/target_pose.yaml")


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def _quat_angle(q1, q2):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
    dot = abs(dot)  # q 与 -q 等价
    return 2.0 * math.acos(dot)


def _get_current_joint_angles(timeout=2.0):
    try:
        msg = rospy.wait_for_message('/sensors_data_raw', sensorsData, timeout=timeout)
        if hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_q'):
            return list(msg.joint_data.joint_q)
    except rospy.ROSException:
        pass
    return None


def get_current_eef(timeout_sec: float = 1.0):
    """
    获取当前手臂关节角度并调用 FK 服务，返回左右末端位姿（位置+四元数）。
    返回: (left_pos, left_quat, right_pos, right_quat) 或 None
    """
    joint_angles = _get_current_joint_angles(timeout=timeout_sec)
    if joint_angles is None or len(joint_angles) < 26:
        return None
    arm_joint_angles = joint_angles[12:26]
    if len(arm_joint_angles) != 14:
        return None
    try:
        rospy.wait_for_service('/ik/fk_srv', timeout=1.0)
        fk_srv = rospy.ServiceProxy('/ik/fk_srv', fkSrv)
        resp = fk_srv(list(arm_joint_angles))
        left_pose = resp.hand_poses.left_pose
        right_pose = resp.hand_poses.right_pose
        left_pos = np.array(left_pose.pos_xyz, dtype=float)
        left_quat = np.array(left_pose.quat_xyzw, dtype=float)
        right_pos = np.array(right_pose.pos_xyz, dtype=float)
        right_quat = np.array(right_pose.quat_xyzw, dtype=float)
        return left_pos, left_quat, right_pos, right_quat
    except Exception:
        return None


def load_keyframes(config_path=None):
    """读取关键帧并进行基本整理，详见模块说明。"""
    if config_path is None:
        config_path = default_config_path()
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    traj = cfg.get('trajectory', [])
    if not traj:
        raise ValueError('配置文件中 trajectory 为空')

    # 校验 time
    for idx, kf in enumerate(traj):
        if 'time' not in kf or kf['time'] is None or (isinstance(kf['time'], str) and kf['time'] == ''):
            rospy.logerr(f'第 {idx} 个关键帧缺少有效的 time 字段')
            raise ValueError('关键帧缺少 time 或 time 无效')
        kf['time'] = float(kf['time'])

    # 若没有 time=0 的关键帧，则用当前末端位姿补帧
    try:
        has_t0 = any(float(kf.get('time', 0.0)) == 0.0 for kf in traj)
    except Exception:
        has_t0 = False
    if not has_t0 and len(traj) > 0:
        seed = get_current_eef(timeout_sec=float(rospy.get_param('~wait_current_eef_timeout', 1.0)))
        if seed is not None:
            lp, lq, rp, rq = seed
            traj.insert(0, {
                'time': 0.0,
                'left': {'pos_xyz': lp.tolist(), 'quat_xyzw': lq.tolist()},
                'right': {'pos_xyz': rp.tolist(), 'quat_xyzw': rq.tolist()},
            })
            rospy.loginfo('已在 load_keyframes 中使用当前末端位姿补充 t=0 关键帧')
        else:
            rospy.logerr('未能获取当前末端位姿，且配置缺少 time=0 关键帧，无法继续。')
            raise RuntimeError('缺少 time=0 关键帧，且当前位姿获取失败')

    
    # 规范化为 numpy
    keyframes = []
    for kf in traj:
        t = float(kf['time'])
        lp = np.array(kf['left']['pos_xyz'], dtype=float)
        lq = np.array(kf['left']['quat_xyzw'], dtype=float)
        rp = np.array(kf['right']['pos_xyz'], dtype=float)
        rq = np.array(kf['right']['quat_xyzw'], dtype=float)
        # 可选标签字段
        mark_fields = ['mark', 'flag', 'label', 'tag', 'marker', 'signal', 'emit_flag']
        mark_val = False
        for mf in mark_fields:
            if mf in kf and kf[mf] not in (None, ''):
                try:
                    # 支持 bool / 数字 / 字符串("true"/"1")
                    v = kf[mf]
                    if isinstance(v, str):
                        vnorm = v.strip().lower()
                        mark_val = vnorm in ('1', 'true', 'yes', 'y', 'on')
                    else:
                        mark_val = bool(v)
                except Exception:
                    mark_val = False
                break
        keyframes.append({
            'time': t,
            'left': {'pos': lp, 'quat': lq},
            'right': {'pos': rp, 'quat': rq},
            'mark': bool(mark_val),
        })

    keyframes.sort(key=lambda x: x['time'])

    
    return keyframes
