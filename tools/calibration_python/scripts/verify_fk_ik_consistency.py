#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人手臂正逆运动学一致性验证脚本

脚本作用：
    该脚本用于验证机器人手臂正逆运动学的一致性，主要功能包括：
    1. 自动生成标定位形：根据关节限位随机生成多组目标关节角，覆盖工作空间
    2. 工作空间验证：通过正运动学服务检查每个目标点的末端位置是否在安全工作空间内
    3. 平滑轨迹执行：使用本地贝塞尔曲线插值生成平滑轨迹，控制机器人依次运动到各个目标点
    4. 正逆运动学一致性验证：在每个目标点到位后，进行以下验证：
       a. 用正运动学计算当前末端的姿态1（位置+旋转）
       b. 用这个姿态求逆解
       c. 用逆解的关节角再正运动学计算姿态2
       d. 比较姿态1和姿态2的位置和旋转误差

主要设计点：
    1. 本地关节空间贝塞尔插值：不依赖外部轨迹规划服务，脚本内部实现二次贝塞尔曲线插值
    2. 工作空间自动过滤：生成关节角后，调用正运动学服务检查末端位置，仅执行在工作空间内的目标点
    3. 正逆运动学一致性验证：验证FK->IK->FK的闭环一致性
    4. 双臂独立控制：支持左臂、右臂或双臂同时运动

使用方法:
    1. 启动机器人控制节点：
       cd kuavo-ros-opensource
       sudo su
       source devel/setup.bash
       roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim
       # 或实物模式: roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12
    
    2. 运行脚本：
       cd tools/calibration_python/scripts
       python3 verify_fk_ik_consistency.py

输出文件（保存在 config 目录）：
    - fk_ik_pos_compare_<arm_side>_arm_paramX.txt：
        每行保存同一目标点在“期望姿态1”和“用参数X求逆、再FK得到的姿态2”之间的末端位置对比：
        点索引, [期望x(mm),  期望y(mm),  期望z(mm)], [解算x(mm),  解算y(mm),  解算z(mm)]
    - fk_ik_error_detailed_<arm_side>_arm_paramX.txt：
        每行保存同一目标点在“期望姿态1”和“用参数X求逆、再FK得到的姿态2”之间的详细误差：
        位置误差模、xyz 分量误差 (mm) 以及 roll/pitch/yaw 角度误差 (deg)
"""

import os
import sys
import time
import rospy
import rospkg
import json
import numpy as np
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData

# 添加 motion_capture_ik 包的路径以导入 Quest3ArmInfoTransformer
Quest3ArmInfoTransformer = None
try:
    rospack = rospkg.RosPack()
    motion_capture_ik_path = rospack.get_path("motion_capture_ik")
    sys.path.insert(0, os.path.join(motion_capture_ik_path, "scripts"))
    from tools.quest3_utils import Quest3ArmInfoTransformer
except Exception as e:
    rospy.logwarn(f"Warning: Could not import Quest3ArmInfoTransformer: {e}")
    Quest3ArmInfoTransformer = None

from function.motion_utils import (
    generate_joint_angles_with_margin,
    build_jointspace_bezier_trajectory,
    check_workspace,
    quaternion_to_euler,
    quaternion_angle_error,
    pose_difference,
    generate_filtered_joint_targets,
    euler_to_quaternion,
    map_left_to_right_joints,
    mirror_quaternion_for_right_arm,
    mirror_position_for_right_arm,
)
from function.file_utils import get_config_dir
from function.data_utils import (
    call_fk_service_both_arms,
    call_change_arm_ctrl_mode_service,
    build_dual_arm,
    call_ik_service,
    call_fk_service_with_pose,
)
from function.marker_utils import (
    publish_marker,
    init_marker_publishers,
    publish_trajectory_markers_both_arms,
    publish_trajectory_markers_single_arm,
    DEFAULT_COLOR_RED,
    DEFAULT_COLOR_BLUE,
)
from function.verification_utils import (
    save_pos_compare,
    save_detailed_error,
    print_statistics,
)


# ==================== 配置参数 ====================

# 左臂关节限位矩阵 (7×2, rad)
LEFT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],    # l1: l_arm_pitch
    [-0.349,  1.46],   # l2: l_arm_roll
    [-1.57,   0.46],   # l3: l_arm_yaw
    [-2.618,  0.0],    # l4: l_forearm_pitch
    [-1.57,   1.57],   # l5: l_hand_yaw
    [-1.309,  0.698],  # l6: l_hand_pitch
    [-0.698,  0.698]   # l7: l_hand_roll
])

# 右臂关节限位矩阵 (7×2, rad)
RIGHT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],    # r1: r_arm_pitch
    [-1.46,   0.349],  # r2: r_arm_roll (左右对称)
    [-0.46,   1.57],   # r3: r_arm_yaw (左右对称)
    [-2.618,  0.0],    # r4: r_forearm_pitch
    [-1.57,   1.57],   # r5: r_hand_yaw
    [-0.698,  1.309],  # r6: r_hand_pitch (左右对称)
    [-0.698,  0.698]   # r7: r_hand_roll
])

# 生成调试用关节角的数量
NUM_DEBUG_PTS = 200

# 每个目标点之间的运动时间 (秒)
TIME_PER_POINT = 3.0

# 每段路径末尾的停顿时间 (秒)
DWELL_TIME_PER_POINT = 2.0

# 验证循环中的延时时间 (秒)
VERIFICATION_DELAY = 5.0

# 是否执行关节控制（True: 实际控制机器人运动; False: 仅验证，不执行轨迹）
EXECUTE_JOINT_CONTROL = False

# 选择生成和发布的手臂
ARM_SIDE = 'both'

# 初始手臂位置 (度)
INIT_ARM_POS_DEG = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]

# 发布频率（Hz）
PUBLISH_HZ = 100

# 右手工作空间边界（单位：米）
RIGHT_ARM_WORKSPACE = {
    'x_min': 0.2, 'x_max': 0.6,
    'y_min': -0.62, 'y_max': -0.05,
    'z_min': 0.0, 'z_max': 0.7
}

# 左手工作空间边界（单位：米）
LEFT_ARM_WORKSPACE = {
    'x_min': 0.2, 'x_max': 0.6,
    'y_min': 0.05, 'y_max': 0.62,
    'z_min': 0.0, 'z_max': 0.7
}

# IK求解参数 - 第一套参数
IK_SOLVE_PARAM_1 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 1e-3,
    'major_iterations_limit': 20000,
    'oritation_constraint_tol': 10e-3,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10,
}

# IK求解参数 - 第二套参数
IK_SOLVE_PARAM_2 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 1e-3,
    'major_iterations_limit': 20000,
    'oritation_constraint_tol': 10e-3,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10.0,
}

# IK参数集名称（用于结果文件命名）
IK_PARAM_NAMES = ['param1', 'param2']

# Marker命名空间和颜色配置
MARKER_NAMESPACE_TARGET = "arm_target_pose"      # 期望位姿marker的命名空间
MARKER_NAMESPACE_RESULT_PARAM1 = "arm_result_param1"  # param1验证结果marker的命名空间
MARKER_NAMESPACE_RESULT_PARAM2 = "arm_result_param2"  # param2验证结果marker的命名空间

MARKER_COLOR_TARGET = [1.0, 0.0, 0.0, 0.9]      # 期望位姿颜色：红色
MARKER_COLOR_RESULT_PARAM1 = [0.0, 0.0, 1.0, 0.9]  # param1验证结果颜色：蓝色
MARKER_COLOR_RESULT_PARAM2 = [0.0, 1.0, 1.0, 0.9]  # param2验证结果颜色：青色


# ==================== 全局变量 ====================
COLOR_PURPLE = "\033[95m"
COLOR_RESET = "\033[0m"

joint_state = JointState()
current_arm_joint_state = []
frozen_arm_joint_state = None
config_dir = None  # 配置目录路径，在主函数中初始化


# ==================== 回调函数 ====================

def sensors_data_callback(msg):
    """
    传感器数据回调函数,获取当前手臂关节状态
    """
    global current_arm_joint_state
    current_arm_joint_state = msg.joint_data.joint_q[12:26]
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]


# ==================== 服务调用函数 ====================


def verify_fk_ik_consistency(arm_side, ik_param_dict, param_name="param", target_joints_rad=None):
    """
    验证正逆运动学一致性
    
    Args:
        arm_side: 手臂侧，"left" 或 "right"
        ik_param_dict: IK参数字典
        param_name: 参数集名称（用于日志）
        target_joints_rad: 目标关节角（14个关节，弧度），如果为None则使用实时关节角
    
    Returns:
        dict: 包含验证结果的字典
            - pos_error: 位置误差（米）
            - rot_error: 旋转误差（弧度）
            - success: 是否成功
            - param_name: 参数集名称
            - pos1: 期望姿态的位置（目标关节角FK得到）
            - quat1: 期望姿态的四元数
            - pos2: 实际姿态的位置（IK逆解关节角FK得到）
            - quat2: 实际姿态的四元数
    """
    global current_arm_joint_state, frozen_arm_joint_state
    
    # 使用目标关节角（期望关节角）计算期望姿态
    if target_joints_rad is not None:
        target_joints_14 = np.array(target_joints_rad)
    else:
        # 如果没有提供目标关节角，使用实时关节角（向后兼容）
        if frozen_arm_joint_state is not None and len(frozen_arm_joint_state) >= 14:
            target_joints_14 = np.array(frozen_arm_joint_state[:14])
        elif current_arm_joint_state and len(current_arm_joint_state) >= 14:
            target_joints_14 = np.array(current_arm_joint_state[:14])
        else:
            rospy.logwarn("跳过验证：关节状态为空")
            return None
    
    # 步骤1: 用目标关节角做FK得到期望姿态（位置和姿态）
    pos1, quat1 = call_fk_service_with_pose(target_joints_14, arm_side)
    if pos1 is None or quat1 is None:
        rospy.logwarn(f"[{param_name}] FK服务调用失败（期望姿态）")
        return None
    
    # 另一只手的期望位置（用于全身IK约束）
    other_arm_side = "left" if arm_side == "right" else "right"
    other_pos, other_quat = call_fk_service_with_pose(target_joints_14, other_arm_side)
    
    # 步骤2: 用这个姿态求逆解（使用指定的IK参数），并告知另一只手的当前位置
    # 说明：
    #   - param1：使用默认的 IK 服务 "/ik/two_arm_hand_pose_cmd_srv"
    #   - param2：使用多参考的 IK 服务 "/ik/two_arm_hand_pose_cmd_srv_muli_refer"
    if param_name == "param2":
        service_name = "/ik/two_arm_hand_pose_cmd_srv_muli_refer"
    else:
        service_name = None  # 使用 call_ik_service 内部的默认服务
    ik_result = call_ik_service(
        pos1,
        quat1,
        arm_side,
        ik_param_dict=ik_param_dict,
        other_arm_pos=other_pos,
        other_arm_quat=other_quat,
        service_name=service_name,
    )
    if ik_result is None or ik_result[0] is None:
        ik_joints = None
    else:
        ik_joints, _, _ = ik_result  # 忽略时间和错误原因，在线脚本不需要
    if ik_joints is None:
        rospy.logwarn(f"[{param_name}] IK服务调用失败")
        return {
            'pos1': pos1,
            'quat1': quat1,
            'pos2': None,
            'quat2': None,
            'param_name': param_name,
            'success': False
        }
    
    # 步骤3: 用逆解的关节角再正运动学计算姿态2
    # 需要将单臂7关节扩展为14关节
    if arm_side == "left":
        ik_joints_14 = build_dual_arm(ik_joints, "left", np.radians(INIT_ARM_POS_DEG[7:]))
    elif arm_side == "right":
        ik_joints_14 = build_dual_arm(ik_joints, "right", np.radians(INIT_ARM_POS_DEG[:7]))
    else:
        rospy.logwarn(f"未知的手臂侧: {arm_side}")
        return None
    
    pos2, quat2 = call_fk_service_with_pose(ik_joints_14, arm_side)
    if pos2 is None or quat2 is None:
        rospy.logwarn(f"[{param_name}] FK服务调用失败（姿态2）")
        return {
            'pos1': pos1,
            'quat1': quat1,
            'pos2': None,
            'quat2': None,
            'param_name': param_name,
            'success': False
        }
    
    # 调试：打印实际使用的姿态
    roll1, pitch1, yaw1 = quaternion_to_euler(quat1)
    roll2, pitch2, yaw2 = quaternion_to_euler(quat2)
    rospy.loginfo(
        f"[{param_name}] 期望姿态(quat1): roll={np.degrees(roll1):.2f}°, "
        f"pitch={np.degrees(pitch1):.2f}°, yaw={np.degrees(yaw1):.2f}°"
    )
    rospy.loginfo(
        f"[{param_name}] 实际姿态(quat2): roll={np.degrees(roll2):.2f}°, "
        f"pitch={np.degrees(pitch2):.2f}°, yaw={np.degrees(yaw2):.2f}°"
    )
    
    # 步骤4: 比较姿态1和姿态2的位置和旋转
    pos_error = np.linalg.norm(pos1 - pos2)
    rot_error = quaternion_angle_error(quat1, quat2)
    
    result = {
        'pos_error': pos_error,
        'rot_error': rot_error,
        'pos1': pos1,
        'pos2': pos2,
        'quat1': quat1,
        'quat2': quat2,
        'param_name': param_name,
        'success': True
    }
    
    return result


# 全局变量：存储所有验证结果
# 结构：{arm_side: {param_name: {'pos_errors': [], 'rot_errors': []}}}
verification_results = {
    'left': {'param1': {'pos_errors': [], 'rot_errors': []}, 
             'param2': {'pos_errors': [], 'rot_errors': []}},
    'right': {'param1': {'pos_errors': [], 'rot_errors': []}, 
              'param2': {'pos_errors': [], 'rot_errors': []}},
}


def save_detailed_difference(result1, result2, arm_side, point_idx):
    """
    保存姿态1与参数1的姿态2、姿态1与参数2的姿态3之间的详细差值
    
    Args:
        result1: 参数1的验证结果（可以为None）
        result2: 参数2的验证结果（可以为None）
        arm_side: 手臂侧
        point_idx: 点索引
    """
    # 获取姿态1（优先从result1获取，如果result1为None则从result2获取）
    pos1 = None
    quat1 = None
    if result1 is not None and result1.get('pos1') is not None:
        pos1 = result1['pos1']
        quat1 = result1.get('quat1')
    elif result2 is not None and result2.get('pos1') is not None:
        pos1 = result2['pos1']
        quat1 = result2.get('quat1')
    
    # 如果两个结果都没有姿态1，则无法保存，记录警告
    if pos1 is None:
        rospy.logwarn(f"[点 {point_idx}] 无法获取姿态1，跳过详细误差保存")
        return
    
    # 使用通用函数保存参数1的详细误差（即使失败也会保存nan值）
    save_detailed_error(
        point_idx, arm_side, 'param1',
        pos1, quat1,
        result1.get('pos2') if result1 else None,
        result1.get('quat2') if result1 else None,
        result1.get('success', False) if result1 else False,
        file_prefix='fk_ik',
        config_dir=config_dir
    )
    
    # 使用通用函数保存参数2的详细误差（即使失败也会保存nan值）
    save_detailed_error(
        point_idx, arm_side, 'param2',
        pos1, quat1,
        result2.get('pos2') if result2 else None,
        result2.get('quat2') if result2 else None,
        result2.get('success', False) if result2 else False,
        file_prefix='fk_ik',
        config_dir=config_dir
    )


def publish_expected_pose_marker(arm_side, target_joints_rad, point_idx, progress_info,
                                quest3_arm_info_transformer=None,
                                marker_pub_target_left=None,
                                marker_pub_target_right=None,
                                is_execution_mode=True):
    """
    发布期望位姿的marker，支持left/right/both模式
    
    Args:
        arm_side: 手臂侧，"left"、"right" 或 "both"
        target_joints_rad: 目标关节角（14个关节，弧度）
        point_idx: 点索引
        progress_info: 进度信息字符串（用于日志）
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_target_left: 左手期望位姿marker发布器（可选）
        marker_pub_target_right: 右手期望位姿marker发布器（可选）
        is_execution_mode: 是否为执行模式（True: 轨迹执行模式, False: 仅验证模式）
    """
    # 姿态固定为 pitch = -89 度（避免万向锁）
    expected_roll = 0.0
    expected_pitch = -89.0
    expected_yaw = 0.0
    
    if arm_side == 'both':
        # 双臂模式：计算左手期望位置，右手位置使用左手位置的镜像
        expected_pos_left, _ = call_fk_service_with_pose(target_joints_rad, "left")
        if expected_pos_left is not None:
            expected_pos_right = mirror_position_for_right_arm(expected_pos_left)
        else:
            expected_pos_right = None
        
        if expected_pos_left is not None and expected_pos_right is not None:
            pos_left_mm = expected_pos_left * 1000.0
            pos_right_mm = expected_pos_right * 1000.0
            mode_prefix = "[轨迹进度]" if is_execution_mode else "[验证进度]"
            rospy.loginfo(
                f"{mode_prefix} {progress_info}, "
                f"期望位置-左(mm)=[{pos_left_mm[0]:.1f}, {pos_left_mm[1]:.1f}, {pos_left_mm[2]:.1f}], "
                f"期望位置-右(mm)=[{pos_right_mm[0]:.1f}, {pos_right_mm[1]:.1f}, {pos_right_mm[2]:.1f}], "
                f"期望姿态(欧拉角)=roll={expected_roll:.2f}°, pitch={expected_pitch:.2f}°, yaw={expected_yaw:.2f}°"
            )
            
            # 发布期望位姿的marker
            quat_left_expected = euler_to_quaternion(roll_deg=expected_roll, pitch_deg=expected_pitch, yaw_deg=expected_yaw)
            quat_right_expected = mirror_quaternion_for_right_arm(quat_left_expected)
            publish_trajectory_markers_both_arms(
                quest3_arm_info_transformer,
                marker_pub_target_left, marker_pub_target_right,
                None, None,  # result marker在验证时发布
                expected_pos_left, expected_pos_right,
                quat_left_expected, quat_right_expected,
                target_color=MARKER_COLOR_TARGET,
                point_idx=point_idx,
                target_namespace=MARKER_NAMESPACE_TARGET
            )
        else:
            mode_prefix = "[轨迹进度]" if is_execution_mode else "[验证进度]"
            rospy.loginfo(f"{mode_prefix} {progress_info}, FK计算失败")
    else:
        # 单臂模式
        expected_pos, _ = call_fk_service_with_pose(target_joints_rad, arm_side)
        if expected_pos is not None:
            pos_mm = expected_pos * 1000.0
            mode_prefix = "[轨迹进度]" if is_execution_mode else "[验证进度]"
            rospy.loginfo(
                f"{mode_prefix} {progress_info}, "
                f"期望位置(mm)=[{pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}], "
                f"期望姿态(欧拉角)=roll={expected_roll:.2f}°, pitch={expected_pitch:.2f}°, yaw={expected_yaw:.2f}°"
            )
            
            # 发布期望位姿的marker
            marker_pub_target = marker_pub_target_left if arm_side == "left" else marker_pub_target_right
            if arm_side == "left":
                quat_expected = euler_to_quaternion(roll_deg=expected_roll, pitch_deg=expected_pitch, yaw_deg=expected_yaw)
            else:
                quat_left_expected = euler_to_quaternion(roll_deg=expected_roll, pitch_deg=expected_pitch, yaw_deg=expected_yaw)
                quat_expected = mirror_quaternion_for_right_arm(quat_left_expected)
            publish_trajectory_markers_single_arm(
                quest3_arm_info_transformer,
                marker_pub_target, None,  # result marker在验证时发布
                expected_pos, quat_expected,
                arm_side,
                target_color=MARKER_COLOR_TARGET,
                point_idx=point_idx,
                target_namespace=MARKER_NAMESPACE_TARGET
            )
        else:
            mode_prefix = "[轨迹进度]" if is_execution_mode else "[验证进度]"
            rospy.loginfo(f"{mode_prefix} {progress_info}, FK计算失败")


def execute_trajectory_and_verify(filtered_targets, arm_side, kuavo_arm_traj_pub,
                                 quest3_arm_info_transformer=None,
                                 marker_pub_target_left=None,
                                 marker_pub_target_right=None,
                                 marker_pub_result_param1_left=None,
                                 marker_pub_result_param1_right=None,
                                 marker_pub_result_param2_left=None,
                                 marker_pub_result_param2_right=None):
    """
    执行轨迹并验证正逆运动学一致性（执行模式）
    
    Args:
        filtered_targets: 过滤后的目标点列表
        arm_side: 手臂侧，"left"、"right" 或 "both"
        kuavo_arm_traj_pub: 轨迹发布器
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_target_left: 左手期望位姿marker发布器（可选）
        marker_pub_target_right: 右手期望位姿marker发布器（可选）
        marker_pub_result_param1_left: 左手param1验证结果marker发布器（可选）
        marker_pub_result_param1_right: 右手param1验证结果marker发布器（可选）
        marker_pub_result_param2_left: 左手param2验证结果marker发布器（可选）
        marker_pub_result_param2_right: 右手param2验证结果marker发布器（可选）
    """
    num_valid_targets = len(filtered_targets)
    points_per_segment = max(5, int(TIME_PER_POINT * PUBLISH_HZ))
    full_traj_deg = build_jointspace_bezier_trajectory(
        filtered_targets,
        INIT_ARM_POS_DEG,
        TIME_PER_POINT,
        PUBLISH_HZ,
        num_points_per_segment=points_per_segment,
        dwell_time_per_point=0.0,
    )
    
    segment_lengths = [points_per_segment] * num_valid_targets
    seg_idx = 0
    seg_step = 0
    seg_len = segment_lengths[seg_idx] if segment_lengths else 0
    rate = rospy.Rate(PUBLISH_HZ)
    
    for i, q in enumerate(full_traj_deg):
        if rospy.is_shutdown():
            break
        
        # 每个目标段开始时打印一次，并发布期望位姿的marker
        if seg_step == 0 and seg_idx < len(filtered_targets):
            current_target = filtered_targets[seg_idx]
            if arm_side == 'left':
                target_view = current_target[:7]
            elif arm_side == 'right':
                target_view = current_target[7:]
            else:
                target_view = current_target
            progress = i / float(len(full_traj_deg)) * 100.0 if len(full_traj_deg) > 0 else 0.0
            
            target_rad = np.radians(current_target)
            progress_info = f"段 {seg_idx + 1}/{num_valid_targets}, 段内步 {seg_step + 1}/{seg_len}, 整体 {progress:.1f}%, 目标(度)={np.round(target_view, 2)}"
            publish_expected_pose_marker(
                arm_side, target_rad, seg_idx,
                progress_info,
                quest3_arm_info_transformer=quest3_arm_info_transformer,
                marker_pub_target_left=marker_pub_target_left,
                marker_pub_target_right=marker_pub_target_right,
                is_execution_mode=True
            )
        
        # 发布轨迹
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
            "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
            "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        ]
        msg.position = q
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        kuavo_arm_traj_pub.publish(msg)
        
        # 段计数
        seg_step += 1
        finished_segment = seg_step >= seg_len
        last_segment = seg_idx >= len(segment_lengths) - 1
        
        # 段结束后执行验证
        if finished_segment:
            # 等待机器人运动到位并采样关节角
            if DWELL_TIME_PER_POINT > 0:
                rospy.sleep(DWELL_TIME_PER_POINT)
            
            # 等待稳定（采样关节角）
            global current_arm_joint_state, frozen_arm_joint_state
            rospy.sleep(1.0)
            joint_samples = []
            for _ in range(10):
                if current_arm_joint_state and len(current_arm_joint_state) >= 14:
                    joint_samples.append(current_arm_joint_state[:14])
                rospy.sleep(0.1)
            
            if len(joint_samples) > 0:
                # 计算关节角均值（弧度）
                samples_np = np.array(joint_samples)
                avg_joints_rad = np.mean(samples_np, axis=0)
                current_arm_joint_state = avg_joints_rad.tolist()
                # 冻结本段的关节角，供本段内两套参数共享使用
                frozen_arm_joint_state = avg_joints_rad.tolist()
            
            # 获取当前段的目标关节角（期望关节角）
            current_target = filtered_targets[seg_idx]
            target_rad = np.radians(current_target)
            
            # 验证单点
            verify_single_point(
                arm_side, target_rad, seg_idx + 1,
                quest3_arm_info_transformer=quest3_arm_info_transformer,
                marker_pub_result_param1_left=marker_pub_result_param1_left,
                marker_pub_result_param1_right=marker_pub_result_param1_right,
                marker_pub_result_param2_left=marker_pub_result_param2_left,
                marker_pub_result_param2_right=marker_pub_result_param2_right
            )
        
        if finished_segment and not last_segment:
            seg_idx += 1
            seg_step = 0
            seg_len = segment_lengths[seg_idx]
        
        rate.sleep()


def verify_points_only(filtered_targets, arm_side,
                      quest3_arm_info_transformer=None,
                      marker_pub_target_left=None,
                      marker_pub_target_right=None,
                      marker_pub_result_param1_left=None,
                      marker_pub_result_param1_right=None,
                      marker_pub_result_param2_left=None,
                      marker_pub_result_param2_right=None):
    """
    直接验证目标点，不执行轨迹（仅验证模式）
    
    Args:
        filtered_targets: 过滤后的目标点列表
        arm_side: 手臂侧，"left"、"right" 或 "both"
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_target_left: 左手期望位姿marker发布器（可选）
        marker_pub_target_right: 右手期望位姿marker发布器（可选）
        marker_pub_result_param1_left: 左手param1验证结果marker发布器（可选）
        marker_pub_result_param1_right: 右手param1验证结果marker发布器（可选）
        marker_pub_result_param2_left: 左手param2验证结果marker发布器（可选）
        marker_pub_result_param2_right: 右手param2验证结果marker发布器（可选）
    """
    num_valid_targets = len(filtered_targets)
    rospy.loginfo(f"{COLOR_PURPLE}开始验证 {num_valid_targets} 个目标点...{COLOR_RESET}")
    
    for seg_idx, current_target in enumerate(filtered_targets):
        if rospy.is_shutdown():
            break
        
        target_rad = np.radians(current_target)
        
        # 准备进度信息
        if arm_side == 'left':
            target_view = current_target[:7]
        elif arm_side == 'right':
            target_view = current_target[7:]
        else:
            target_view = current_target
        progress = (seg_idx + 1) / float(num_valid_targets) * 100.0
        progress_info = f"点 {seg_idx + 1}/{num_valid_targets}, 整体 {progress:.1f}%, 目标(度)={np.round(target_view, 2)}"
        
        # 发布期望位姿的marker
        publish_expected_pose_marker(
            arm_side, target_rad, seg_idx,
            progress_info,
            quest3_arm_info_transformer=quest3_arm_info_transformer,
            marker_pub_target_left=marker_pub_target_left,
            marker_pub_target_right=marker_pub_target_right,
            is_execution_mode=False
        )
        
        # 验证单点
        verify_single_point(
            arm_side, target_rad, seg_idx + 1,
            quest3_arm_info_transformer=quest3_arm_info_transformer,
            marker_pub_result_param1_left=marker_pub_result_param1_left,
            marker_pub_result_param1_right=marker_pub_result_param1_right,
            marker_pub_result_param2_left=marker_pub_result_param2_left,
            marker_pub_result_param2_right=marker_pub_result_param2_right
        )
        
        # 短暂延迟，避免过快处理
        rospy.sleep(VERIFICATION_DELAY)


def verify_single_point(arm_side, target_joints_rad, point_idx,
                       quest3_arm_info_transformer=None,
                       marker_pub_result_param1_left=None,
                       marker_pub_result_param1_right=None,
                       marker_pub_result_param2_left=None,
                       marker_pub_result_param2_right=None):
    """
    验证单个目标点的正逆运动学一致性，支持left/right/both模式
    
    Args:
        arm_side: 手臂侧，"left"、"right" 或 "both"
        target_joints_rad: 目标关节角（14个关节，弧度）
        point_idx: 点索引
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_result_param1_left: 左手param1验证结果marker发布器（可选）
        marker_pub_result_param1_right: 右手param1验证结果marker发布器（可选）
        marker_pub_result_param2_left: 左手param2验证结果marker发布器（可选）
        marker_pub_result_param2_right: 右手param2验证结果marker发布器（可选）
    
    Returns:
        dict: {arm_side: [result1, result2]} 验证结果字典
    """
    ik_params = [IK_SOLVE_PARAM_1, IK_SOLVE_PARAM_2]
    all_results = {}
    
    # 确定要验证的手臂列表
    if arm_side == 'both':
        arms_to_verify = ['left', 'right']
    else:
        arms_to_verify = [arm_side]
    
    # 对每个手臂分别验证
    for arm in arms_to_verify:
        results = []
        for param_idx, ik_param in enumerate(ik_params):
            param_name = IK_PARAM_NAMES[param_idx]
            result = verify_fk_ik_consistency(arm, ik_param, param_name, target_joints_rad=target_joints_rad)
            save_verification_result_with_markers(
                result, arm, point_idx, param_name,
                quest3_arm_info_transformer=quest3_arm_info_transformer,
                marker_pub_result_param1_left=marker_pub_result_param1_left,
                marker_pub_result_param1_right=marker_pub_result_param1_right,
                marker_pub_result_param2_left=marker_pub_result_param2_left,
                marker_pub_result_param2_right=marker_pub_result_param2_right
            )
            results.append(result)
        
        # 保存详细差值（姿态1与参数1的姿态2、姿态1与参数2的姿态3）
        if len(results) == 2:
            save_detailed_difference(results[0], results[1], arm, point_idx)
        
        all_results[arm] = results
    
    return all_results


def save_verification_result_with_markers(result, arm_side, point_idx, param_name,
                                         quest3_arm_info_transformer=None,
                                         marker_pub_result_param1_left=None,
                                         marker_pub_result_param1_right=None,
                                         marker_pub_result_param2_left=None,
                                         marker_pub_result_param2_right=None):
    """
    统一的验证结果保存调用函数，自动选择合适的marker发布器
    
    Args:
        result: 验证结果字典
        arm_side: 手臂侧 ("left" 或 "right")
        point_idx: 点索引
        param_name: 参数集名称（'param1' 或 'param2'）
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_result_param1_left: 左手param1验证结果marker发布器（可选）
        marker_pub_result_param1_right: 右手param1验证结果marker发布器（可选）
        marker_pub_result_param2_left: 左手param2验证结果marker发布器（可选）
        marker_pub_result_param2_right: 右手param2验证结果marker发布器（可选）
    """
    save_verification_result(
        result, arm_side, point_idx, param_name,
        quest3_arm_info_transformer=quest3_arm_info_transformer,
        marker_pub_target_left=None,  # 期望位姿marker已在轨迹执行前发布
        marker_pub_target_right=None,
        marker_pub_result_param1_left=marker_pub_result_param1_left,
        marker_pub_result_param1_right=marker_pub_result_param1_right,
        marker_pub_result_param2_left=marker_pub_result_param2_left,
        marker_pub_result_param2_right=marker_pub_result_param2_right
    )


def save_verification_result(result, arm_side, point_idx, param_name,
                            quest3_arm_info_transformer=None,
                            marker_pub_target_left=None, marker_pub_target_right=None,
                            marker_pub_result_param1_left=None, marker_pub_result_param1_right=None,
                            marker_pub_result_param2_left=None, marker_pub_result_param2_right=None):
    """
    保存验证结果到文件，并发布marker到rviz
    
    Args:
        result: 验证结果字典
        arm_side: 手臂侧
        point_idx: 点索引
        param_name: 参数集名称（'param1' 或 'param2'）
        quest3_arm_info_transformer: Quest3ArmInfoTransformer 实例（可选）
        marker_pub_target_left: 左手期望位姿marker发布器（可选）
        marker_pub_target_right: 右手期望位姿marker发布器（可选）
        marker_pub_result_param1_left: 左手param1验证结果marker发布器（可选）
        marker_pub_result_param1_right: 右手param1验证结果marker发布器（可选）
        marker_pub_result_param2_left: 左手param2验证结果marker发布器（可选）
        marker_pub_result_param2_right: 右手param2验证结果marker发布器（可选）
    """
    # 即使验证失败，也保存数据（失败时保存nan值）
    if result is None:
        rospy.logwarn(f"[点 {point_idx}][{param_name}] 验证结果为空，保存失败标记")
        # 即使result为None，也尝试保存（save_pos_compare和save_detailed_error会处理None值）
        save_pos_compare(
            point_idx, arm_side, param_name,
            None, None,  # pos1和pos2都为None
            quat1=None, quat2=None,
            file_prefix='fk_ik',
            config_dir=config_dir
        )
        return
    
    success = result.get('success', False)
    if not success:
        rospy.logwarn(f"[点 {point_idx}][{param_name}] 验证失败，保存失败标记")
    
    # 使用通用函数保存位置对比（即使失败也会保存，pos2为None时会保存nan）
    save_pos_compare(
        point_idx, arm_side, param_name,
        result.get('pos1'), result.get('pos2'),
        quat1=result.get('quat1'), quat2=result.get('quat2'),
        file_prefix='fk_ik',
        config_dir=config_dir
    )

    # 更新统计用的误差（仅成功时更新）
    if success:
        pos_error_mm = result.get('pos_error', 0) * 1000.0
        rot_error_deg = np.degrees(result.get('rot_error', 0))
        if arm_side in verification_results and param_name in verification_results[arm_side]:
            verification_results[arm_side][param_name]['pos_errors'].append(pos_error_mm)
            verification_results[arm_side][param_name]['rot_errors'].append(rot_error_deg)
        
        rospy.loginfo(
            f"{COLOR_PURPLE}[点 {point_idx}][{param_name}] 位置误差: {pos_error_mm:.4f} mm, "
            f"旋转误差: {rot_error_deg:.4f} deg{COLOR_RESET}"
        )
    else:
        # 失败时也记录到统计中（使用nan值）
        if arm_side in verification_results and param_name in verification_results[arm_side]:
            verification_results[arm_side][param_name]['pos_errors'].append(np.nan)
            verification_results[arm_side][param_name]['rot_errors'].append(np.nan)
    
    # 发布实际位姿的marker到rviz（期望位姿已在轨迹执行前发布）
    # 为param1和param2分别发布不同颜色和命名空间的marker
    if quest3_arm_info_transformer is not None:
        if param_name == 'param1':
            # param1使用蓝色和对应的命名空间
            marker_pub_result_left = marker_pub_result_param1_left
            marker_pub_result_right = marker_pub_result_param1_right
            rgba = MARKER_COLOR_RESULT_PARAM1
            namespace = MARKER_NAMESPACE_RESULT_PARAM1
            marker_id_offset = 0  # param1的marker_id偏移
        elif param_name == 'param2':
            # param2使用青色和对应的命名空间
            marker_pub_result_left = marker_pub_result_param2_left
            marker_pub_result_right = marker_pub_result_param2_right
            rgba = MARKER_COLOR_RESULT_PARAM2
            namespace = MARKER_NAMESPACE_RESULT_PARAM2
            marker_id_offset = 4  # param2的marker_id偏移（避免与param1冲突）
        else:
            # 未知参数，不发布
            return
        
        # 发布实际位姿marker
        if arm_side == "left" and marker_pub_result_left is not None:
            # marker_id: param1左手=4, param2左手=8
            marker_id = 4 + marker_id_offset if arm_side == "left" else 5 + marker_id_offset
            publish_marker(quest3_arm_info_transformer, marker_pub_result_left,
                         result.get('pos2'), result.get('quat2'), "left", 
                         rgba=rgba, 
                         marker_id=marker_id, 
                         point_idx=point_idx,
                         namespace=namespace)
        elif arm_side == "right" and marker_pub_result_right is not None:
            # marker_id: param1右手=5, param2右手=9
            marker_id = 5 + marker_id_offset if arm_side == "right" else 6 + marker_id_offset
            publish_marker(quest3_arm_info_transformer, marker_pub_result_right,
                         result.get('pos2'), result.get('quat2'), "right", 
                         rgba=rgba, 
                         marker_id=marker_id, 
                         point_idx=point_idx,
                         namespace=namespace)




# ==================== 主函数 ====================

def main():
    """
    主函数: 生成随机关节角并通过本地贝塞尔插值发送到机器人，然后验证正逆运动学一致性
    """
    rospy.init_node('arm_fk_ik_consistency_verifier')
    
    # 订阅传感器数据（仅在执行关节控制时需要）
    if EXECUTE_JOINT_CONTROL:
        rospy.Subscriber('/sensors_data_raw', sensorsData, sensors_data_callback, queue_size=1, tcp_nodelay=True)
        kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        
        # 切换到手臂控制模式
        rospy.loginfo(f"{COLOR_PURPLE}切换手臂控制模式...{COLOR_RESET}")
        call_change_arm_ctrl_mode_service(2)
        rospy.sleep(0.5)
    else:
        # 仅验证模式：不需要订阅传感器数据和发布轨迹
        kuavo_arm_traj_pub = None
        rospy.loginfo(f"{COLOR_PURPLE}仅验证模式：跳过手臂控制模式切换和传感器数据订阅{COLOR_RESET}")
    
    # 初始化可视化marker发布器
    (quest3_arm_info_transformer, marker_pub_target_left, marker_pub_target_right,
     marker_pub_result_param1_left, marker_pub_result_param1_right,
     marker_pub_result_param2_left, marker_pub_result_param2_right) = init_marker_publishers(Quest3ArmInfoTransformer)
    
    # 获取配置文件目录（统一路径，确保初始化和保存使用相同路径）
    global config_dir
    config_dir = get_config_dir(__file__)
    rospy.loginfo(f"{COLOR_PURPLE}配置文件目录: {config_dir}{COLOR_RESET}")
    
    # 初始化结果文件（删除旧文件并创建新文件）- 为两套参数分别创建文件
    for param_name in IK_PARAM_NAMES:
        if ARM_SIDE == 'both':
            # 双臂模式：为左右臂分别创建文件
            for arm in ['left', 'right']:
                result_file = f'fk_ik_pos_compare_{arm}_arm_{param_name}.txt'
                result_path = os.path.join(config_dir, result_file)
                # 删除旧文件（如果存在）
                if os.path.exists(result_path):
                    os.remove(result_path)
                    rospy.loginfo(f"删除旧文件: {result_path}")
                # 创建新文件并写入表头
                with open(result_path, "w") as f:
                    f.write("# 点索引,  [期望x(mm),  期望y(mm),  期望z(mm)],  [解算x(mm),  解算y(mm),  解算z(mm)],  [期望roll(度), pitch(度), yaw(度)],  [解算roll(度), pitch(度), yaw(度)]\n")
                rospy.loginfo(f"{COLOR_PURPLE}结果文件已初始化: {result_path}{COLOR_RESET}")
        else:
            # 单臂模式
            result_file = f'fk_ik_pos_compare_{ARM_SIDE}_arm_{param_name}.txt'
            result_path = os.path.join(config_dir, result_file)
            # 删除旧文件（如果存在）
            if os.path.exists(result_path):
                os.remove(result_path)
                rospy.loginfo(f"删除旧文件: {result_path}")
            # 创建新文件并写入表头
            with open(result_path, "w") as f:
                f.write("# 点索引,  [期望x(mm),  期望y(mm),  期望z(mm)],  [解算x(mm),  解算y(mm),  解算z(mm)],  [期望roll(度), pitch(度), yaw(度)],  [解算roll(度), pitch(度), yaw(度)]\n")
            rospy.loginfo(f"{COLOR_PURPLE}结果文件已初始化: {result_path}{COLOR_RESET}")
    
    # 初始化详细误差文件（删除旧文件并创建新文件）
    if ARM_SIDE == 'both':
        for arm in ['left', 'right']:
            for param_name in IK_PARAM_NAMES:
                diff_file = f'fk_ik_error_detailed_{arm}_arm_{param_name}.txt'
                diff_path = os.path.join(config_dir, diff_file)
                # 删除旧文件（如果存在）
                if os.path.exists(diff_path):
                    os.remove(diff_path)
                    rospy.loginfo(f"删除旧文件: {diff_path}")
                # 创建新文件并写入表头
                with open(diff_path, "w") as f:
                    f.write("# 点索引,位置误差模(mm),x差值(mm),y差值(mm),z差值(mm),roll差值(度),pitch差值(度),yaw差值(度)\n")
                rospy.loginfo(f"{COLOR_PURPLE}详细差值文件已初始化: {diff_path}{COLOR_RESET}")
    else:
        for param_name in IK_PARAM_NAMES:
            diff_file = f'fk_ik_error_detailed_{ARM_SIDE}_arm_{param_name}.txt'
            diff_path = os.path.join(config_dir, diff_file)
            # 删除旧文件（如果存在）
            if os.path.exists(diff_path):
                os.remove(diff_path)
                rospy.loginfo(f"删除旧文件: {diff_path}")
            # 创建新文件并写入表头
            with open(diff_path, "w") as f:
                f.write("# 点索引,位置误差模(mm),x差值(mm),y差值(mm),z差值(mm),roll差值(度),pitch差值(度),yaw差值(度)\n")
            rospy.loginfo(f"{COLOR_PURPLE}详细差值文件已初始化: {diff_path}{COLOR_RESET}")
    
    # =============== 生成随机关节角 ===============
    rospy.loginfo(f"{COLOR_PURPLE}目标有效点数 = {NUM_DEBUG_PTS} ... (ARM_SIDE={ARM_SIDE}){COLOR_RESET}")
    
    if ARM_SIDE == 'both':
        # 双臂模式：只生成左手的随机关节角，然后对称映射到右手
        rospy.loginfo(f"{COLOR_PURPLE}双臂模式：生成左手随机关节角，对称映射到右手{COLOR_RESET}")
        try:
            left_targets = generate_filtered_joint_targets(
                num_points=NUM_DEBUG_PTS,
                left_limit=LEFT_ARM_JOINT_LIMIT,
                right_limit=RIGHT_ARM_JOINT_LIMIT,
                init_arm_pos_deg=INIT_ARM_POS_DEG,
                left_workspace=LEFT_ARM_WORKSPACE,
                right_workspace=RIGHT_ARM_WORKSPACE,
                arm_side='left',  # 只生成左手的
                margin_ratio=0.1,
                timeout_s=5.0,
                fk_service_callback=call_fk_service_both_arms,
            )
        except TimeoutError as e:
            rospy.logerr(str(e))
            return
        
        # 将左手的关节角映射到右手（对称映射）
        filtered_targets = []
        for left_target in left_targets:
            left_joints_7 = left_target[:7]  # 提取左手的7个关节角
            right_joints_7 = map_left_to_right_joints(left_joints_7)  # 对称映射
            # 组合成14个关节角：左手7个 + 右手7个
            both_target = np.concatenate([left_joints_7, right_joints_7])
            filtered_targets.append(both_target)
        
        rospy.loginfo(f"{COLOR_PURPLE}工作空间过滤完成: {len(filtered_targets)}/{NUM_DEBUG_PTS} 个有效点（左手生成，对称映射到右手）{COLOR_RESET}")
    else:
        # 单臂模式：使用原有逻辑
        try:
            filtered_targets = generate_filtered_joint_targets(
                num_points=NUM_DEBUG_PTS,
                left_limit=LEFT_ARM_JOINT_LIMIT,
                right_limit=RIGHT_ARM_JOINT_LIMIT,
                init_arm_pos_deg=INIT_ARM_POS_DEG,
                left_workspace=LEFT_ARM_WORKSPACE,
                right_workspace=RIGHT_ARM_WORKSPACE,
                arm_side=ARM_SIDE,
                margin_ratio=0.1,
                timeout_s=5.0,
                fk_service_callback=call_fk_service_both_arms,
            )
        except TimeoutError as e:
            rospy.logerr(str(e))
            return
        rospy.loginfo(f"{COLOR_PURPLE}工作空间过滤完成: {len(filtered_targets)}/{NUM_DEBUG_PTS} 个有效点{COLOR_RESET}")
    
    # =============== 执行轨迹或直接验证 ===============
    try:
        if EXECUTE_JOINT_CONTROL:
            # 等待订阅者
            rospy.loginfo("本地关节空间贝塞尔插值生成完整轨迹...")
            while kuavo_arm_traj_pub.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.loginfo("等待 kuavo_arm_traj 订阅者连接...")
                rospy.sleep(0.5)
            
            rospy.loginfo("开始发布轨迹到 /kuavo_arm_traj 并验证正逆运动学一致性...")
            execute_trajectory_and_verify(
                filtered_targets, ARM_SIDE, kuavo_arm_traj_pub,
                quest3_arm_info_transformer=quest3_arm_info_transformer,
                marker_pub_target_left=marker_pub_target_left,
                marker_pub_target_right=marker_pub_target_right,
                marker_pub_result_param1_left=marker_pub_result_param1_left,
                marker_pub_result_param1_right=marker_pub_result_param1_right,
                marker_pub_result_param2_left=marker_pub_result_param2_left,
                marker_pub_result_param2_right=marker_pub_result_param2_right
            )
        else:
            rospy.loginfo(f"{COLOR_PURPLE}仅验证模式：不执行关节控制，直接使用目标关节角进行验证{COLOR_RESET}")
            verify_points_only(
                filtered_targets, ARM_SIDE,
                quest3_arm_info_transformer=quest3_arm_info_transformer,
                marker_pub_target_left=marker_pub_target_left,
                marker_pub_target_right=marker_pub_target_right,
                marker_pub_result_param1_left=marker_pub_result_param1_left,
                marker_pub_result_param1_right=marker_pub_result_param1_right,
                marker_pub_result_param2_left=marker_pub_result_param2_left,
                marker_pub_result_param2_right=marker_pub_result_param2_right
            )
        
        # 打印统计信息
        rospy.loginfo("轨迹发布和验证完成")
        if ARM_SIDE == 'both':
            for param_name in IK_PARAM_NAMES:
                print_statistics('left', param_name, verification_results, COLOR_PURPLE, COLOR_RESET)
                print_statistics('right', param_name, verification_results, COLOR_PURPLE, COLOR_RESET)
        else:
            for param_name in IK_PARAM_NAMES:
                print_statistics(ARM_SIDE, param_name, verification_results, COLOR_PURPLE, COLOR_RESET)
    except KeyboardInterrupt:
        rospy.loginfo("用户中断,退出程序")
        # 即使中断也打印已完成的统计信息
        if ARM_SIDE == 'both':
            for param_name in IK_PARAM_NAMES:
                print_statistics('left', param_name, verification_results, COLOR_PURPLE, COLOR_RESET)
                print_statistics('right', param_name, verification_results, COLOR_PURPLE, COLOR_RESET)
        else:
            for param_name in IK_PARAM_NAMES:
                print_statistics(ARM_SIDE, param_name, verification_results, COLOR_PURPLE, COLOR_RESET)


if __name__ == "__main__":
    main()