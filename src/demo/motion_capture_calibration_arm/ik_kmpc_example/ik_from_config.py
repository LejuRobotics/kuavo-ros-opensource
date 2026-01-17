#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取关键帧(包含末端位姿和到达时间)，首帧/尾帧补齐；
仅在关键帧时刻调用双臂末端IK得到关节角关键帧，然后对关节角按时间线性插值，
按频率发布到 /kuavo_arm_traj。
"""

import math
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float64
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.load_keyframes import load_keyframes, default_config_path
from utils.interpolation_utils import generate_time_grid, interpolate_q_linear
from utils.ros_control_utils import call_ik_srv, call_change_arm_ctrl_mode_service, enable_wbc_arm_trajectory_control
from utils.analysis_utils import KeyframeJointAngleRecorder

mirring_mode = False

def deg(rad):
    return rad * 180.0 / math.pi

def apply_mirror_processing(q_arm):
    """
    对14维关节角进行镜像处理：
    - 当左臂第2个关节角 > 0 时，镜像左臂到右臂
    - 否则，镜像右臂到左臂
    
    Args:
        q_arm: 14维关节角列表（弧度），前7个是左臂，后7个是右臂
    
    Returns:
        处理后的14维关节角列表
    """
    if len(q_arm) < 14:
        rospy.logwarn(f"关节角维度不足14，跳过镜像处理: {len(q_arm)}")
        return q_arm
    
    # 分离左右臂关节角
    left_joint_pose = list(q_arm[:7])
    right_joint_pose = list(q_arm[7:])
    
    rospy.logdebug(f"left_joint: {left_joint_pose}")
    rospy.logdebug(f"right_joint: {right_joint_pose}")
    
    if left_joint_pose[1] > 0:
        rospy.logdebug("镜像左->右")
        right_joint_pose = [
            left_joint_pose[0], -left_joint_pose[1],
            -left_joint_pose[2], left_joint_pose[3],
            -left_joint_pose[4], -left_joint_pose[5], left_joint_pose[6]
        ]
    else:
        rospy.logdebug("镜像右->左")
        left_joint_pose = [
            right_joint_pose[0], -right_joint_pose[1],
            -right_joint_pose[2], right_joint_pose[3],
            -right_joint_pose[4], -right_joint_pose[5], right_joint_pose[6]
        ]
    
    target_joint_positions = left_joint_pose + right_joint_pose
    rospy.logdebug(f"镜像处理后关节角：{target_joint_positions}")
    
    return target_joint_positions

def build_ik_param():
    # ik solver param
    ik_solve_param = ikSolveParam()
    # snopt params
    ik_solve_param.major_optimality_tol = 1e-3
    ik_solve_param.major_feasibility_tol = 1e-3
    ik_solve_param.minor_feasibility_tol = 1e-3
    ik_solve_param.major_iterations_limit = 100
    # constraint and cost params
    ik_solve_param.oritation_constraint_tol= 1e-3
    ik_solve_param.pos_constraint_tol = 1e-3 # 0.001m, work when pos_cost_weight==0.0
    ik_solve_param.pos_cost_weight = 0.0 # If U need high accuracy, set this to 0.0 !!!
    return ik_solve_param
 
 

def build_eef_pose_msg(left_pose, right_pose, ik_param):
    msg = twoArmHandPoseCmd()
    msg.ik_param = ik_param
    msg.use_custom_ik_param = True
    msg.joint_angles_as_q0 = False
    # left
    msg.hand_poses.left_pose.pos_xyz = left_pose['pos_xyz']
    msg.hand_poses.left_pose.quat_xyzw = left_pose['quat_xyzw']
    msg.hand_poses.left_pose.elbow_pos_xyz = left_pose['elbow_pos_xyz']
    msg.hand_poses.left_pose.joint_angles = left_pose['joint_angles']
    # right
    msg.hand_poses.right_pose.pos_xyz = right_pose['pos_xyz']
    msg.hand_poses.right_pose.quat_xyzw = right_pose['quat_xyzw']
    msg.hand_poses.right_pose.elbow_pos_xyz = right_pose['elbow_pos_xyz']
    msg.hand_poses.right_pose.joint_angles = right_pose['joint_angles']
    return msg

def build_joint_state_msg_from_q(q_arm):
    """由 14 维关节角（弧度）构造 JointState 消息（角度单位：度）。"""
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = [
        "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
        "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
        "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
        "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
    ]
    js.position = [deg(x) for x in q_arm]
    js.velocity = [0.0] * len(js.position)
    js.effort = [0.0] * len(js.position)
    return js

def publish_joint_state(q_arm, ctrl_mode_ok):
    """发布 JointState（从弧度角度列表）。"""
    global mirring_mode

    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10, latch=True)
    # 应用镜像处理
    if mirring_mode:
        q_arm = apply_mirror_processing(q_arm)
    
    js = build_joint_state_msg_from_q(q_arm)
    if ctrl_mode_ok:
        pub.publish(js)
    else:
        rospy.logwarn('未切换到预期的手臂控制模式，跳过发布关节数据')

def solve_ik_for_keyframe(left_pos, left_quat,
                          right_pos, right_quat,
                          ik_param: ikSolveParam):
    """对单个关键帧调用双臂 IK，返回 14 维关节角（弧度）或 None。"""
    zero7 = [0.0] * 7
    left_pose = {
        'pos_xyz': left_pos.tolist(),
        'quat_xyzw': left_quat.tolist(),
        'elbow_pos_xyz': [0.0, float(left_pos[1]), 0.0],
        'joint_angles': list(zero7),
    }
    right_pose = {
        'pos_xyz': right_pos.tolist(),
        'quat_xyzw': right_quat.tolist(),
        'elbow_pos_xyz': [0.0, float(right_pos[1]), 0.0],
        'joint_angles': list(zero7),
    }
    eef_msg = build_eef_pose_msg(left_pose, right_pose, ik_param)
    res = call_ik_srv(eef_msg)
    if res and getattr(res, 'success', False) and hasattr(res, 'q_arm') and len(res.q_arm) >= 14:
        return list(res.q_arm)
    return None

def compute_q_keyframes_with_ik(keyframes, ik_param):
    """
    仅在关键帧时刻调用 IK，返回关节角关键帧列表：
    [{'time': t, 'q': [14 floats]}, ...]
    - 所有关节的 IK 初值都固定为 7 个 0（左右臂相同）。
    """
    qkfs = []
    for i, kf in enumerate(keyframes):
        if rospy.is_shutdown():
            break
        lpos = kf['left']['pos']; lquat = kf['left']['quat']
        rpos = kf['right']['pos']; rquat = kf['right']['quat']
        q = solve_ik_for_keyframe(lpos, lquat, rpos, rquat, ik_param)
        if q is not None:
            qkfs.append({'time': float(kf['time']), 'q': q})
        else:
            rospy.logwarn(f"IK@t={kf['time']:.3f} 失败，使用上一成功解或零向量兜底")
            if qkfs:
                fallback = list(qkfs[-1]['q'])
            else:
                fallback = [0.0] * 14
            qkfs.append({'time': float(kf['time']), 'q': fallback})
    return qkfs

def _get_config():
    rate_hz = float(rospy.get_param('~rate_hz', 100.0))
    config_path = rospy.get_param('~config_path', default_config_path())
    play_loop_count = 10  # 设置循环次数
    keyframe_flag_topic = rospy.get_param('~keyframe_flag_topic', rospy.get_param('keyframe_flag_topic', '/keyframe_flag'))
    publish_zero_outside = bool(rospy.get_param('~publish_zero_outside_keyframe', rospy.get_param('publish_zero_outside_keyframe', True)))

    return {
        'rate_hz': rate_hz,
        'config_path': config_path,
        'play_loop_count': play_loop_count,
        'keyframe_flag_topic': keyframe_flag_topic,
        'publish_zero_outside': publish_zero_outside,
    }

# 插值所有关节角（包含闭环插值）
def interpolate_q_with_loop(q_keyframes, t, keyframes):
    """插值关节角，支持从最后一帧到第一帧的闭环插值"""
    t_end = float(keyframes[-1]['time'])
    if t > t_end:
        # 闭环段：从最后一帧到第一帧
        t_start_next = t_end + 2.0
        if t >= t_start_next:
            return list(q_keyframes[0]['q'])  # 已到达第一帧
        # 在闭环段内插值
        u = (t - t_end) / 2.0  # 归一化到 [0, 1]
        q_last = np.asarray(q_keyframes[-1]['q'], dtype=float)
        q_first = np.asarray(q_keyframes[0]['q'], dtype=float)
        q = (1.0 - u) * q_last + u * q_first
        return q.tolist()
    else:
        # 正常段：使用原有插值
        return interpolate_q_linear(q_keyframes, t)

def main():
    global mirring_mode

    rospy.init_node('ik_from_keyframes', anonymous=True)
    cfg = _get_config()

    # 输出镜像模式状态
    if mirring_mode:
        rospy.loginfo('========== 镜像模式：已启用 (mirring_mode=True) ==========')
    else:
        rospy.loginfo('========== 镜像模式：已禁用 (mirring_mode=False) ==========')

    # 读取关键帧
    try:
        keyframes = load_keyframes(cfg['config_path'])
    except Exception as e:
        rospy.logerr(f'加载配置失败: {e}')
        return

    # 准备 IK 参数
    ik_param = build_ik_param()
    rate = rospy.Rate(cfg['rate_hz'])

    # 创建关键帧关节角度记录器
    output_file = rospy.get_param('~joint_angle_record_file', None)
    recorder = KeyframeJointAngleRecorder(output_file=output_file)
    # 等待传感器数据订阅建立
    rospy.sleep(0.5)

    # 切换模式并等待
    ctrl_mode_ok = call_change_arm_ctrl_mode_service(2)
    rospy.sleep(0.2)
    # IK: 是否启用 WBC 轨迹控制（/enable_wbc_arm_trajectory_control），默认开启，可通过参数关闭
    if bool(rospy.get_param('~enable_wbc_arm_trajectory_control', rospy.get_param('enable_wbc_arm_trajectory_control', True))):
        enable_wbc_arm_trajectory_control()
    else:
        rospy.loginfo('Skip enabling /enable_wbc_arm_trajectory_control per parameter setting.')

    # 关键帧求 IK，得到关节角关键帧
    q_keyframes = compute_q_keyframes_with_ik(keyframes, ik_param)

    # 构建时间网格（包含从最后一帧到第一帧的闭环插值）
    times = generate_time_grid(keyframes, cfg['rate_hz'])
    
    # 添加闭环时间段：从最后一帧到第一帧（额外2秒）
    if len(keyframes) > 1:
        t_end = float(keyframes[-1]['time'])
        t_start_next = t_end + 4.0  # 闭环过渡时间2秒
        dt = 1.0 / max(cfg['rate_hz'], 1e-6)
        t = t_end + dt
        while t < t_start_next - 1e-9:
            times.append(round(t, 6))
            t += dt
        times.append(round(t_start_next, 6))

    # 记录关键帧触发的时间网格索引 -> 关键帧时间（秒）
    index_to_kftime = {}
    for kf in keyframes:
        if not bool(kf.get('mark', False)):
            continue
        tk = float(kf['time'])
        for i, t in enumerate(times):
            if t >= tk - 1e-9:  # 包含浮点容差
                index_to_kftime[i] = tk
                break
    trigger_indices_set = set(index_to_kftime.keys())

    # 关键帧标志发布器：发送关键帧"时间（秒）"
    kf_flag_pub = rospy.Publisher(cfg['keyframe_flag_topic'], Float64, queue_size=10)
    q_traj = [interpolate_q_with_loop(q_keyframes, t, keyframes) for t in times]

    # 循环发布插值结果
    for loop_idx in range(cfg['play_loop_count']):
        for idx, q in enumerate(q_traj):
            if rospy.is_shutdown():
                break
            # 发布关节角
            publish_joint_state(q, ctrl_mode_ok)
            if mirring_mode:
                q = apply_mirror_processing(q)
            # 发布关键帧标志：到达关键帧起点时发送该关键帧的时间（秒）
            if idx in trigger_indices_set:
                m = Float64(); m.data = float(index_to_kftime[idx])
                kf_flag_pub.publish(m)
                # 记录关键帧的关节角度（下发角度，q已经是弧度）
                # record_keyframe 内部会等待一段时间以获取传感器反馈
                recorder.record_keyframe(index_to_kftime[idx], q, wait_for_feedback=0.15, loop_index=loop_idx + 1)
            elif cfg['publish_zero_outside']:
                m = Float64(); m.data = 0.0
                kf_flag_pub.publish(m)
            rate.sleep()
        if loop_idx < cfg['play_loop_count'] - 1:
            rospy.sleep(1.0)
    
    # 程序结束前写入文件
    recorder.write_to_file()

if __name__ == '__main__':
    main()