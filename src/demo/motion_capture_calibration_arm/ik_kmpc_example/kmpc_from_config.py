#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
from kuavo_msgs.msg import twoArmHandPoseCmd
from std_msgs.msg import Int32, Float64

import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.load_keyframes import load_keyframes, default_config_path
from utils.interpolation_utils import precompute_eef_sequence, quat_slerp, clamp
from utils.ros_control_utils import (
    call_change_arm_ctrl_mode_service,
    call_mm_mpc_control_service,
    fk_srv_client,
    enable_mm_wbc_arm_trajectory_control,
)

 
def _get_cfg():
    """集中读取必须参数"""
    rate_hz = float(rospy.get_param('~rate_hz', rospy.get_param('rate_hz', 100.0)))
    config_path = rospy.get_param('~config_path', rospy.get_param('config_path', default_config_path()))
    wait_for_state = float(rospy.get_param('~wait_for_state', rospy.get_param('wait_for_state', 2.0)))
    after_ctrl_mode_sleep = float(rospy.get_param('~after_ctrl_mode_sleep', 0.2))
    play_loop_count = int(rospy.get_param('~play_loop_count', 10))
    keyframe_flag_topic = rospy.get_param('~keyframe_flag_topic', rospy.get_param('keyframe_flag_topic', '/keyframe_flag'))
    publish_zero_outside = bool(rospy.get_param('~publish_zero_outside_keyframe', rospy.get_param('publish_zero_outside_keyframe', True)))
    return {
        'rate_hz': rate_hz,
        'config_path': config_path,
        'wait_for_state': wait_for_state,
        'after_ctrl_mode_sleep': after_ctrl_mode_sleep,
        'play_loop_count': play_loop_count,
        'keyframe_flag_topic': keyframe_flag_topic,
        'publish_zero_outside': publish_zero_outside,
    }


def _setup_pub_and_modes(after_sleep: float):
    """初始化发布器并切换控制模式，留一点缓冲时间。"""
    pub = rospy.Publisher('/mm/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
    # 切换手臂控制模式(2) & 启动MM MPC控制(ArmOnly=1)
    call_change_arm_ctrl_mode_service(2)
    call_mm_mpc_control_service(1)
    # KMPC: 是否启用 MM WBC 轨迹控制（/enable_mm_wbc_arm_trajectory_control），默认开启，可通过参数关闭
    if bool(rospy.get_param('~enable_mm_wbc_arm_trajectory_control', rospy.get_param('enable_mm_wbc_arm_trajectory_control', True))):
        enable_mm_wbc_arm_trajectory_control()
    else:
        rospy.loginfo('Skip enabling /enable_mm_wbc_arm_trajectory_control per parameter setting.')
    rospy.sleep(after_sleep)
    return pub


 
def main():
    rospy.init_node('kf_pose_to_mm_topic', anonymous=True)
    cfg = _get_cfg()

    try:
        keyframes = load_keyframes(cfg['config_path'])
    except Exception as e:
        rospy.logerr(f'加载配置失败: {e}')
        return

    cycle_seq = precompute_eef_sequence(keyframes, cfg['rate_hz'])
    
    # 添加闭环插值：从最后一帧到第一帧，使用与 precompute_eef_sequence 相同的插值方式
    if len(keyframes) > 1:
        # 定义与 precompute_eef_sequence 相同的 lerp 函数
        def lerp(a, b, u):
            return (1.0 - u) * np.asarray(a, dtype=float) + u * np.asarray(b, dtype=float)
        
        t_end = float(keyframes[-1]['time'])
        t_start_next = t_end + 4.0  # 闭环过渡时间4秒
        dt = 1.0 / max(cfg['rate_hz'], 1e-6)
        t = t_end + dt
        last_item = cycle_seq[-1]
        first_item = cycle_seq[0]
        while t < t_start_next - 1e-9:
            t0 = t_end
            t1 = t_start_next
            u = clamp((t - t0) / max(t1 - t0, 1e-9), 0.0, 1.0)  # 使用 clamp 确保在 [0, 1] 范围内
            # 位置线性插值（与 precompute_eef_sequence 相同）
            lp0 = last_item['left_pos']; lp1 = first_item['left_pos']
            rp0 = last_item['right_pos']; rp1 = first_item['right_pos']
            lp = lerp(lp0, lp1, u)
            rp = lerp(rp0, rp1, u)
            # 姿态SLERP插值（与 precompute_eef_sequence 相同）
            lq0 = last_item['left_quat']; lq1 = first_item['left_quat']
            rq0 = last_item['right_quat']; rq1 = first_item['right_quat']
            lq = quat_slerp(lq0, lq1, u)
            rq = quat_slerp(rq0, rq1, u)
            cycle_seq.append({
                't': round(t, 6),
                'left_pos': np.asarray(lp, dtype=float),
                'left_quat': np.asarray(lq, dtype=float),
                'right_pos': np.asarray(rp, dtype=float),
                'right_quat': np.asarray(rq, dtype=float),
            })
            t += dt
        # 添加第一帧作为闭环终点
        cycle_seq.append({
            't': round(t_start_next, 6),
            'left_pos': np.asarray(first_item['left_pos'], dtype=float),
            'left_quat': np.asarray(first_item['left_quat'], dtype=float),
            'right_pos': np.asarray(first_item['right_pos'], dtype=float),
            'right_quat': np.asarray(first_item['right_quat'], dtype=float),
        })
    
    rate = rospy.Rate(cfg['rate_hz'])
    pub = _setup_pub_and_modes(cfg['after_ctrl_mode_sleep'])

    times = [item['t'] for item in cycle_seq]
    trigger_indices = []
    for kf in keyframes:
        if not bool(kf.get('mark', False)):
            continue
        tk = float(kf['time'])
        for i, t in enumerate(times):
            if t >= tk - 1e-9:
                trigger_indices.append(i)
                break
    trigger_indices_set = set(trigger_indices)
    # 建立索引到关键帧时间(秒)的映射，供发布时使用
    index_to_kftime = {}
    for kf in keyframes:
        if not bool(kf.get('mark', False)):
            continue
        tk = float(kf['time'])
        for i, t in enumerate(times):
            if t >= tk - 1e-9:
                index_to_kftime[i] = tk
                break
    # 关键帧话题发布 Float64：关键帧时间（秒），非关键帧按需发0.0
    kf_flag_pub = rospy.Publisher(cfg['keyframe_flag_topic'], Float64, queue_size=10)

    for loop_idx in range(cfg['play_loop_count']):
        for idx, item in enumerate(cycle_seq):
            if rospy.is_shutdown():
                break
            lpos = item['left_pos']; lquat = item['left_quat']
            rpos = item['right_pos']; rquat = item['right_quat']

            msg = twoArmHandPoseCmd()
            msg.hand_poses.left_pose.pos_xyz = lpos.tolist()
            msg.hand_poses.left_pose.quat_xyzw = lquat.tolist()
            msg.hand_poses.right_pose.pos_xyz = rpos.tolist()
            msg.hand_poses.right_pose.quat_xyzw = rquat.tolist()
            msg.frame = 0  # LocalFrame，本地(体)坐标
            pub.publish(msg)
            # 关键帧标志
            if idx in trigger_indices_set:
                m = Float64(); m.data = float(index_to_kftime.get(idx, 0.0))
                kf_flag_pub.publish(m)
            elif cfg['publish_zero_outside']:
                m = Float64(); m.data = 0.0
                kf_flag_pub.publish(m)
            rate.sleep()
        if loop_idx < cfg['play_loop_count'] - 1:
            rospy.sleep(1.0)

    
if __name__ == '__main__':
    main()