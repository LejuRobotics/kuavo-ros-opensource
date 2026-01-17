#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线正逆运动学一致性验证脚本（不执行真实轨迹）

作用：
    - 使用与在线脚本相同的关节限位和工作空间，生成一批“期望关节角”
    - 直接对这些期望关节角做：
        1) FK 得到末端期望姿态 (pos1, quat1)
        2) 用两套 IK 参数分别对该期望姿态求逆解
        3) 再用 IK 解做 FK 得到姿态2
        4) 对比姿态1 与 姿态2 之间的位置 / 旋转误差
    - 全程不发布 /kuavo_arm_traj，不用 /sensors_data_raw，仅调用 FK/IK 服务

输出（保存在 config 目录）：
    - fk_ik_offline_pos_compare_<arm_side>_arm_paramX.txt
        点索引,  [期望x(mm),  期望y(mm),  期望z(mm)],  [解算x(mm),  解算y(mm),  解算z(mm)]
    - fk_ik_offline_error_detailed_<arm_side>_arm_paramX.txt
        点索引, 位置误差模(mm), x差值(mm), y差值(mm), z差值(mm), roll差值(度), pitch差值(度), yaw差值(度), 求解时间(ms)
"""

import os
import sys
import rospy
import rospkg
import numpy as np
from kuavo_msgs.srv import fkSrv

from function.motion_utils import (
    generate_filtered_joint_targets,
    check_workspace,
    quaternion_angle_error,
    pose_difference,
    euler_to_quaternion,
    quaternion_relative_rotation,
    mirror_quaternion_for_right_arm,
    mirror_position_for_right_arm,
)
from function.file_utils import get_config_dir
from function.data_utils import (
    call_fk_service_both_arms,
    build_dual_arm,
    call_ik_service,
    call_ik_service_both_arms,
)
from function.marker_utils import (
    publish_marker,
    init_marker_publishers,
)


# 添加 motion_capture_ik 包的路径以导入 Quest3ArmInfoTransformer（用于 marker 可视化）
Quest3ArmInfoTransformer = None
try:
    rospack = rospkg.RosPack()
    motion_capture_ik_path = rospack.get_path("motion_capture_ik")
    sys.path.insert(0, os.path.join(motion_capture_ik_path, "scripts"))
    from tools.quest3_utils import Quest3ArmInfoTransformer
except Exception as e:
    rospy.logwarn(f"Warning: Could not import Quest3ArmInfoTransformer for offline script: {e}")
    Quest3ArmInfoTransformer = None


# ==================== 配置参数（与在线脚本保持一致） ====================

LEFT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],
    [-0.349,  1.46],
    [-1.57,   0.46],
    [-2.618,  0.0],
    [-1.57,   1.57],
    [-1.309,  0.698],
    [-0.698,  0.698],
])

RIGHT_ARM_JOINT_LIMIT = np.array([
    [-2.4,    0.6],
    [-1.46,   0.349],
    [-0.46,   1.57],
    [-2.618,  0.0],
    [-1.57,   1.57],
    [-0.698,  1.309],
    [-0.698,  0.698],
])

NUM_DEBUG_PTS = 200
TIME_PER_POINT = 3.0  # 仅用于与在线脚本保持一致，这里不实际用到轨迹
ARM_SIDE = "both"    # "left" / "right" / "both"(双臂对称)
# 固定随机种子（默认启用，保证每次运行生成相同的“随机”目标点，便于复现对比）
# - 设为 None：不固定，每次运行都会不同
RANDOM_SEED = 0
# 参考关节：始终使用初始关节角作为 IK 的参考关节
USE_PREVIOUS_TARGET_AS_REFERENCE = False
USE_RANDOM_ROTATION = False  # True: 期望姿态采用随机姿态; False: 采用默认姿态(只有pitch旋转-90度的四元数姿态)
# 每个验证点之间的延时（秒），用于观察 RViz 中的 marker 变化
OFFLINE_VERIFICATION_DELAY = 0.0

# ==================== 单点测试模式（用于对齐 test_ik_fk.py 的关键点） ====================
# True：只跑下面指定的这一个点（不再随机生成 NUM_DEBUG_PTS 个点）
SINGLE_TEST_ENABLE = True
# 来自 tools/calibration_python/scripts/test_ik_fk.py 关键点4：
# left_pos=(0.359, 0.1400, 0.5500), right_pos=(0.359, -0.1400, 0.5500)
# quat = euler_to_quaternion(roll=0, pitch=-89.99, yaw=0)
SINGLE_TEST_LEFT_POS = np.array([0.3590, 0.1400, 0.5500])
SINGLE_TEST_RIGHT_POS = np.array([0.3590, -0.1400, 0.5500])
SINGLE_TEST_EULER_DEG = (0.0, -89.99, 0.0)  # (roll, pitch, yaw)

INIT_ARM_POS_DEG = [20, 0, 0, -30, 0, 0, 0,
                    20, 0, 0, -30, 0, 0, 0]

RIGHT_ARM_WORKSPACE = {
    'x_min': 0.2, 'x_max': 0.6,
    'y_min': -0.62, 'y_max': -0.05,
    'z_min': 0.0, 'z_max': 0.6
}

# 左手工作空间边界（单位：米）
LEFT_ARM_WORKSPACE = {
    'x_min': 0.2, 'x_max': 0.6,
    'y_min': 0.05, 'y_max': 0.62,
    'z_min': 0.0, 'z_max': 0.6
}

# 和在线脚本保持同一套参数

# 位软 + 姿软
IK_SOLVE_PARAM_1 = {
    "major_optimality_tol": 9e-3,
    "major_feasibility_tol": 9e-3,
    "minor_feasibility_tol": 9e-3,
    "major_iterations_limit": 200,
    "oritation_constraint_tol": 19e-3,
    "pos_constraint_tol": 9e-3,
    "pos_cost_weight": 10.0,
    "constraint_mode": 0,
}

# 位软 + 姿软
IK_SOLVE_PARAM_2 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 3e-3,
    'major_iterations_limit': 200,
    'oritation_constraint_tol': 0.0000001,
    'pos_constraint_tol': 2e-3,
    'pos_cost_weight': 10,
    "constraint_mode": 1,
}

#三点约束(一硬两软)
IK_SOLVE_PARAM_3 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 3e-3,
    'major_iterations_limit': 100,
    'oritation_constraint_tol': 0.05,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10.0,
    "constraint_mode": 6,
}

IK_PARAM_NAMES = ["param1", "param2", "param3"]


# ==================== Marker 可视化配置 ====================

# 命名空间（namespace），用于在 RViz 中区分不同类型的marker
MARKER_NAMESPACE_TARGET = "offline_arm_target_pose"
MARKER_NAMESPACE_RESULT_PARAM1 = "offline_arm_result_param1"
MARKER_NAMESPACE_RESULT_PARAM2 = "offline_arm_result_param2"
MARKER_NAMESPACE_RESULT_PARAM3 = "offline_arm_result_param3"

# 颜色 [r, g, b, a]
MARKER_COLOR_TARGET = [1.0, 0.0, 0.0, 0.9]        # 红色：期望位姿
MARKER_COLOR_RESULT_PARAM1 = [0.0, 0.0, 1.0, 0.9] # 蓝色：param1 结果
MARKER_COLOR_RESULT_PARAM2 = [0.0, 1.0, 1.0, 0.9] # 青色：param2 结果
MARKER_COLOR_RESULT_PARAM3 = [0.0, 1.0, 0.0, 0.9] # 绿色：param3 结果

# 全局变量：marker 可视化相关
quest3_arm_info_transformer = None
marker_pub_target_left = None
marker_pub_target_right = None
marker_pub_result_param1_left = None
marker_pub_result_param1_right = None
marker_pub_result_param2_left = None
marker_pub_result_param2_right = None


# ==================== FK 封装 ====================

def call_fk_service_with_pose(joint_angles_rad, arm_side="right", timeout=2.0):
    """
    调用正运动学服务，返回指定手臂的末端位置和四元数

    Args:
        joint_angles_rad: 14个关节角（弧度）
        arm_side: "left" 或 "right"
        timeout: 服务等待超时时间
    """
    service_name = "/ik/fk_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        fk_client = rospy.ServiceProxy(service_name, fkSrv)

        if isinstance(joint_angles_rad, np.ndarray):
            joints_list = joint_angles_rad.tolist()
        else:
            joints_list = joint_angles_rad

        res = fk_client(joints_list)
        if not res.success:
            return None, None

        if arm_side == "left":
            pos = np.array(res.hand_poses.left_pose.pos_xyz)
            quat = np.array(res.hand_poses.left_pose.quat_xyzw)
        elif arm_side == "right":
            pos = np.array(res.hand_poses.right_pose.pos_xyz)
            quat = np.array(res.hand_poses.right_pose.quat_xyzw)
        else:
            rospy.logwarn(f"未知的手臂侧: {arm_side}")
            return None, None

        return pos, quat
    except Exception as e:
        rospy.logwarn(f"FK服务调用失败: {e}")
        return None, None


# ==================== 随机位置和姿态生成 ====================

def generate_random_position_in_workspace(arm_side):
    """
    在工作空间内随机生成位置
    
    Args:
        arm_side: 手臂侧，"left" 或 "right"
    
    Returns:
        np.ndarray: 随机位置 (3,) 米，在工作空间内
    """
    if arm_side == "left":
        workspace = LEFT_ARM_WORKSPACE
    elif arm_side == "right":
        workspace = RIGHT_ARM_WORKSPACE
    else:
        rospy.logwarn(f"未知的手臂侧: {arm_side}")
        return None
    
    # 在工作空间内随机生成位置
    pos = np.array([
        np.random.uniform(workspace['x_min'], workspace['x_max']),
        np.random.uniform(workspace['y_min'], workspace['y_max']),
        np.random.uniform(workspace['z_min'], workspace['z_max'])
    ])
    
    return pos


def generate_random_euler_deg(arm_side):
    """
    按照约束随机生成欧拉角 (roll, pitch, yaw)（单位：度）

    约束：
        1) 左手 roll ∈ [0, 90]，右手 roll ∈ [-90, 0]
        2) 左右手 pitch ∈ [-40, 40]
        3) 左手 yaw ∈ [-75, 40]，右手 yaw ∈ [40, 75]
    """
    # pitch：左右手相同范围
    pitch = np.random.uniform(-40.0, 40.0)

    if arm_side == "left":
        roll = np.random.uniform(0.0, 90.0)
        yaw = np.random.uniform(-75.0, 20.0)
    elif arm_side == "right":
        roll = np.random.uniform(-90.0, 0.0)
        yaw = np.random.uniform(-20.0, 75.0)
    else:
        # 未知手臂侧，退回安全默认值
        roll, pitch, yaw = 0.0, -90.0, 0.0

    return roll, pitch, yaw


def generate_orientation_quat(arm_side, use_random_rotation=True):
    """
    根据配置生成姿态四元数：
        - use_random_rotation=True  : 使用满足约束的随机姿态
        - use_random_rotation=False : 使用默认姿态 (roll=0, pitch=-90, yaw=0)
    """
    if use_random_rotation:
        roll_deg, pitch_deg, yaw_deg = generate_random_euler_deg(arm_side)
    else:
        roll_deg, pitch_deg, yaw_deg = 0.0, -90.0, 0.0

    quat = euler_to_quaternion(roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg)
    return quat


def generate_random_pose_in_workspace(arm_side, use_random_rotation=True):
    """
    在工作空间内生成随机位置和姿态
    
    Args:
        arm_side: 手臂侧，"left" 或 "right"
        use_random_rotation: True: 使用随机旋转姿态; False: 使用默认姿态(pitch=-90°)
    
    Returns:
        tuple: (pos, quat) 或 (None, None) 如果失败
            - pos: 随机位置 (3,) 米，在工作空间内
            - quat: 四元数 (4,) [x, y, z, w]
    """
    pos = generate_random_position_in_workspace(arm_side)
    if pos is None:
        return None, None
    
    quat = generate_orientation_quat(arm_side, use_random_rotation)
    
    return pos, quat


def generate_symmetric_both_arm_pose(use_random_rotation=True):
    """
    生成双臂对称的目标位姿：
      - 先在 LEFT_ARM_WORKSPACE 内生成左手 (pos_left, quat_left)
      - 再通过镜像得到右手 (pos_right, quat_right)
    这样可模仿 verify_fk_ik_consistency.py 的 both 模式。
    """
    pos_left, quat_left = generate_random_pose_in_workspace("left", use_random_rotation)
    if pos_left is None or quat_left is None:
        return None
    pos_right = mirror_position_for_right_arm(pos_left)
    quat_right = mirror_quaternion_for_right_arm(quat_left)
    return pos_left, quat_left, pos_right, quat_right


# ==================== 结果保存 ====================

def format_with_sign(value, precision=1):
    """
    格式化数值，保留符号位（正数前面加空格，负数用负号）
    """
    if value < 0:
        return f"{value:.{precision}f}"
    else:
        return f" {value:.{precision}f}"


def save_pos_compare(point_idx, arm_side, param_name, pos1, pos2, expected_joints=None, ik_joints=None):
    """
    保存期望/解算位置对比（单位 mm），以及关节角误差（单位 度）
    
    Args:
        point_idx: 点索引
        arm_side: 手臂侧（"left" 或 "right"）
        param_name: 参数名称
        pos1: 期望位置（不能为None）
        pos2: 解算位置（可以为None，表示失败）
        expected_joints: 期望关节角（弧度，7个关节）
        ik_joints: 逆解关节角（弧度，7个关节）
    """
    if pos1 is None:
        return

    pos1_mm = np.array(pos1) * 1000.0
    
    # 如果解算位置为None，使用nan替代
    if pos2 is None:
        pos2_str = "[nan,  nan,  nan]"
    else:
        pos2_mm = np.array(pos2) * 1000.0
        pos2_str = f"[{format_with_sign(pos2_mm[0])},  {format_with_sign(pos2_mm[1])},  {format_with_sign(pos2_mm[2])}]"

    file_name = {
        "left": f"fk_ik_offline_pos_compare_left_arm_{param_name}.txt",
        "right": f"fk_ik_offline_pos_compare_right_arm_{param_name}.txt",
        "both": f"fk_ik_offline_pos_compare_both_arms_{param_name}.txt",
    }.get(arm_side, f"fk_ik_offline_pos_compare_{param_name}.txt")

    path = os.path.join(get_config_dir(), file_name)
    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}"
        
        # 格式化位置，使用固定宽度对齐（每个数值7个字符宽度）
        pos1_x = format_with_sign(pos1_mm[0], precision=1)
        pos1_y = format_with_sign(pos1_mm[1], precision=1)
        pos1_z = format_with_sign(pos1_mm[2], precision=1)
        pos1_str = f"[{pos1_x:>7},  {pos1_y:>7},  {pos1_z:>7}]"
        
        # 格式化解算位置，使用固定宽度对齐
        if pos2 is None:
            pos2_str = "[   nan,     nan,     nan]"
        else:
            pos2_x = format_with_sign(pos2_mm[0], precision=1)
            pos2_y = format_with_sign(pos2_mm[1], precision=1)
            pos2_z = format_with_sign(pos2_mm[2], precision=1)
            pos2_str = f"[{pos2_x:>7},  {pos2_y:>7},  {pos2_z:>7}]"
        
        line = f"{idx_str},  {pos1_str},  {pos2_str}"
        
        # 如果有关节角信息，添加关节角误差（只输出数值，不输出"各关节误差(度)="前缀）
        if expected_joints is not None and ik_joints is not None:
            expected_joints = np.array(expected_joints)
            ik_joints = np.array(ik_joints)
            
            # 计算每个关节的误差（弧度转度）
            joint_errors_rad = ik_joints - expected_joints
            joint_errors_deg = np.degrees(joint_errors_rad)
            
            # 格式化每个关节的误差，使用固定宽度对齐（每个数值7个字符宽度）
            joint_error_parts = [f"{format_with_sign(err, precision=2):>7}" for err in joint_errors_deg]
            joint_error_str = ",  ".join(joint_error_parts)
            
            line += f",  [{joint_error_str}]"
        elif expected_joints is not None:
            # 如果只有期望关节角，没有逆解关节角，关节误差也用nan
            line += ",  [   nan,     nan,     nan,     nan,     nan,     nan,     nan]"
        
        line += "\n"
        f.write(line)


def save_error_reason(point_idx, arm_side, param_name, error_reason, status="fail"):
    """
    保存逆解结果与原因
    格式：点索引, 状态, 错误原因
    """
    file_name = f"fk_ik_offline_error_reason_{arm_side}_arm_{param_name}.txt"
    path = os.path.join(get_config_dir(), file_name)
    
    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}"
        f.write(f"{idx_str}, {status}, {error_reason}\n")


def save_detailed_error(point_idx, arm_side, param_name, pos1, quat1, pos2, quat2, success, time_cost_ms=None):
    """
    保存详细误差（位置模 / xyz / 欧拉角差 / 求解时间）
    格式：001,    0.1,    [ 0.0,  -0.0,  -0.0],   2.1  [-0.4,   0.3,   1.2],    time=1.32
    """
    file_name = f"fk_ik_offline_error_detailed_{arm_side}_arm_{param_name}.txt"
    path = os.path.join(get_config_dir(), file_name)

    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}"
        if (not success) or pos2 is None or quat2 is None:
            time_str = "nan" if time_cost_ms is None else f"{time_cost_ms:.1f}"
            f.write(
                f"{idx_str},    "
                f"nan,    [nan,  nan,  nan],    nan,    [nan,  nan,  nan],    time={time_str}\n"
            )
        else:
            diff = pose_difference(pos1, quat1, pos2, quat2)
            time_str = f"{time_cost_ms:.1f}" if time_cost_ms is not None else "nan"
            
            # 使用四元数方法计算总角度误差（度）
            total_angle_error_rad = diff['total_angle_error']
            total_angle_error = np.degrees(total_angle_error_rad)
            
            # 进一步将总角度误差分解为绕 x/y/z 轴的等效旋转分量（度）
            # 旋转向量 = 旋转轴(unit) * 总角度(弧度)，这里输出的是各分量对应的角度(度)
            axis_x_deg = 0.0
            axis_y_deg = 0.0
            axis_z_deg = 0.0
            if total_angle_error_rad > 1e-8:
                q_rel = quaternion_relative_rotation(quat1, quat2)  # [x, y, z, w]
                sin_half = np.sin(total_angle_error_rad / 2.0)
                if abs(sin_half) > 1e-8:
                    axis = q_rel[:3] / sin_half  # 单位旋转轴
                    rot_vec_deg = axis * total_angle_error  # 各轴上的等效角度分量（度）
                    axis_x_deg, axis_y_deg, axis_z_deg = rot_vec_deg.tolist()
            
            # 格式化数值，保留符号位
            pos_norm_mm = diff['pos_norm'] * 1000.0
            pos_diff_x_mm = diff['pos_diff_x'] * 1000.0
            pos_diff_y_mm = diff['pos_diff_y'] * 1000.0
            pos_diff_z_mm = diff['pos_diff_z'] * 1000.0
            
            f.write(
                f"{idx_str},    "
                f"{format_with_sign(pos_norm_mm)},    "
                f"[{format_with_sign(pos_diff_x_mm)},  {format_with_sign(pos_diff_y_mm)},  {format_with_sign(pos_diff_z_mm)}],    "
                f"{total_angle_error:.2f},    "
            f"[{format_with_sign(axis_x_deg)},  {format_with_sign(axis_y_deg)},  {format_with_sign(axis_z_deg)}],    "
                f"time={time_str}\n"
            )


# ==================== Marker辅助函数 ====================

def publish_failed_marker(idx, param_name, arm_side):
    """
    发布失败时的marker（位置设为(0, 0, 0)）
    
    Args:
        idx: 点索引
        param_name: 参数名称 ("param1", "param2", "param3")
        arm_side: 手臂侧 ("left" 或 "right")
    """
    global quest3_arm_info_transformer
    global marker_pub_result_param1_left, marker_pub_result_param1_right
    global marker_pub_result_param2_left, marker_pub_result_param2_right
    
    if quest3_arm_info_transformer is None:
        return
    
    # 获取对应的发布器、颜色、命名空间和base_id
    if param_name == "param1":
        result_pub = (
            marker_pub_result_param1_left
            if arm_side == "left"
            else marker_pub_result_param1_right
        )
        rgba = MARKER_COLOR_RESULT_PARAM1
        namespace = MARKER_NAMESPACE_RESULT_PARAM1
        base_id = 10
    elif param_name == "param2":
        result_pub = (
            marker_pub_result_param2_left
            if arm_side == "left"
            else marker_pub_result_param2_right
        )
        rgba = MARKER_COLOR_RESULT_PARAM2
        namespace = MARKER_NAMESPACE_RESULT_PARAM2
        base_id = 12
    elif param_name == "param3":
        # param3 复用 param2 的发布器
        result_pub = (
            marker_pub_result_param2_left
            if arm_side == "left"
            else marker_pub_result_param2_right
        )
        rgba = MARKER_COLOR_RESULT_PARAM3
        namespace = MARKER_NAMESPACE_RESULT_PARAM3
        base_id = 14
    else:
        result_pub = None
    
    if result_pub is not None:
        try:
            failed_pos = np.array([0.0, 0.0, 0.0])
            failed_quat = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数
            marker_id = base_id if arm_side == "left" else base_id + 1
            publish_marker(
                quest3_arm_info_transformer,
                result_pub,
                failed_pos,
                failed_quat,
                arm_side,
                rgba=rgba,
                marker_id=marker_id,
                point_idx=idx,
                namespace=namespace,
            )
        except Exception as e:
            rospy.logwarn(f"[offline][点 {idx}][{param_name}] 发布失败marker失败: {e}")


# ==================== 主流程 ====================

def main():
    rospy.init_node("arm_fk_ik_offline_verifier")

    # 固定随机种子（保证伪随机可复现）
    if RANDOM_SEED is not None:
        np.random.seed(int(RANDOM_SEED))
        rospy.loginfo(f"[offline] RANDOM_SEED 固定为 {RANDOM_SEED}")
    else:
        rospy.loginfo("[offline] RANDOM_SEED=None（不固定随机种子，每次运行随机点会不同）")

    # 初始化可视化 marker 发布器
    global quest3_arm_info_transformer
    global marker_pub_target_left, marker_pub_target_right
    global marker_pub_result_param1_left, marker_pub_result_param1_right
    global marker_pub_result_param2_left, marker_pub_result_param2_right

    (
        quest3_arm_info_transformer,
        marker_pub_target_left,
        marker_pub_target_right,
        marker_pub_result_param1_left,
        marker_pub_result_param1_right,
        marker_pub_result_param2_left,
        marker_pub_result_param2_right,
    ) = init_marker_publishers(Quest3ArmInfoTransformer)

    # 初始化结果文件
    for param_name in IK_PARAM_NAMES:
        if ARM_SIDE == "both":
            arms = ["left", "right"]
        else:
            arms = [ARM_SIDE]

        for arm in arms:
            pos_file = f"fk_ik_offline_pos_compare_{arm}_arm_{param_name}.txt"
            pos_path = os.path.join(get_config_dir(), pos_file)
            with open(pos_path, "w") as f:
                f.write(
                    "# 点索引,  [期望x(mm),  期望y(mm),  期望z(mm)],  "
                    "[解算x(mm),  解算y(mm),  解算z(mm)],  各关节误差(度)=[j1, j2, j3, j4, j5, j6, j7]\n"
                )

            err_file = f"fk_ik_offline_error_detailed_{arm}_arm_{param_name}.txt"
            err_path = os.path.join(get_config_dir(), err_file)
            with open(err_path, "w") as f:
                f.write(
                    "# 点索引,位置误差模(mm),[x差值(mm), y差值(mm), z差值(mm)],总角度误差(度) [绕x/y/z轴的等效旋转分量(度)],time=求解时间(ms)\n"
                )
            
            # 为param3初始化错误原因文件
            if param_name == "param3":
                error_reason_file = f"fk_ik_offline_error_reason_{arm}_arm_{param_name}.txt"
                error_reason_path = os.path.join(get_config_dir(), error_reason_file)
                with open(error_reason_path, "w") as f:
                    f.write("# 点索引, 状态, 错误原因\n")

    # 直接生成随机位置和姿态（在工作空间内）
    rospy.loginfo(
        f"[offline] 目标有效点数 = {NUM_DEBUG_PTS} ... (ARM_SIDE={ARM_SIDE}, USE_RANDOM_ROTATION={USE_RANDOM_ROTATION})"
    )
    
    # 生成随机位置和姿态列表
    # - 单臂: [(pos, quat), ...]
    # - 双臂对称(both): [(pos_left, quat_left, pos_right, quat_right), ...]
    random_poses = []
    if SINGLE_TEST_ENABLE:
        quat_single = euler_to_quaternion(
            roll_deg=SINGLE_TEST_EULER_DEG[0],
            pitch_deg=SINGLE_TEST_EULER_DEG[1],
            yaw_deg=SINGLE_TEST_EULER_DEG[2],
        )
        if ARM_SIDE == "both":
            # 注意：这里按 test_ik_fk.py 的写法，左右手 quat 相同（不做镜像）
            random_poses = [
                (SINGLE_TEST_LEFT_POS, quat_single, SINGLE_TEST_RIGHT_POS, quat_single)
            ]
        elif ARM_SIDE == "left":
            random_poses = [(SINGLE_TEST_LEFT_POS, quat_single)]
        elif ARM_SIDE == "right":
            random_poses = [(SINGLE_TEST_RIGHT_POS, quat_single)]
        else:
            rospy.logwarn(f"[offline] 未知 ARM_SIDE={ARM_SIDE}，单点测试无法继续")
            return
        rospy.loginfo(
            f"[offline][single] 启用单点测试：ARM_SIDE={ARM_SIDE}, "
            f"left_pos={SINGLE_TEST_LEFT_POS.tolist()}, right_pos={SINGLE_TEST_RIGHT_POS.tolist()}, "
            f"euler(deg)={SINGLE_TEST_EULER_DEG}"
        )
    else:
        for _ in range(NUM_DEBUG_PTS):
            if ARM_SIDE == "both":
                both_pose = generate_symmetric_both_arm_pose(USE_RANDOM_ROTATION)
                if both_pose is not None:
                    random_poses.append(both_pose)
            else:
                pos, quat = generate_random_pose_in_workspace(ARM_SIDE, USE_RANDOM_ROTATION)
                if pos is not None and quat is not None:
                    random_poses.append((pos, quat))
    
    rospy.loginfo(
        f"[offline] 随机位置和姿态生成完成: {len(random_poses)}/{NUM_DEBUG_PTS} 个有效点"
    )
    
    if len(random_poses) == 0:
        rospy.logerr("[offline] 未能生成任何有效的随机位置和姿态，退出")
        return

    # 逐点做 IK(param1/param2/param3) -> FK 一致性验证
    ik_params_list = [IK_SOLVE_PARAM_1, IK_SOLVE_PARAM_2]

    # 统计用：位置误差(mm)、旋转误差(度) 和 求解时间(ms)
    verification_results = {
        "right": {
            "param1": {"pos_errors": [], "rot_errors": [], "time_costs": []},
            "param2": {"pos_errors": [], "rot_errors": [], "time_costs": []},
            "param3": {"pos_errors": [], "rot_errors": [], "time_costs": []},
        },
        "left": {
            "param1": {"pos_errors": [], "rot_errors": [], "time_costs": []},
            "param2": {"pos_errors": [], "rot_errors": [], "time_costs": []},
            "param3": {"pos_errors": [], "rot_errors": [], "time_costs": []},
        },
    }

    # 参考关节：单臂时使用对应臂 INIT_ARM_POS_DEG；双臂时左右分别取各自初始
    init_q0_joints_left = np.radians(INIT_ARM_POS_DEG[:7])
    init_q0_joints_right = np.radians(INIT_ARM_POS_DEG[7:])

    for idx, pose_item in enumerate(random_poses, start=1):
        rotation_mode = "随机旋转" if USE_RANDOM_ROTATION else "默认旋转(pitch=-90°)"

        # 统一构造本点要验证的臂列表，以及每只臂的 (pos, quat, other_pos, other_quat, q0_joints)
        if ARM_SIDE == "both":
            pos_l, quat_l, pos_r, quat_r = pose_item
            rospy.loginfo(
                f"[offline][点 {idx}] 双臂对称目标生成完成，姿态模式={rotation_mode}，开始三套参数IK验证"
            )
            # 发布双臂期望 marker
            if quest3_arm_info_transformer is not None:
                try:
                    if marker_pub_target_left is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_target_left,
                            pos_l,
                            quat_l,
                            "left",
                            rgba=MARKER_COLOR_TARGET,
                            marker_id=0,
                            point_idx=idx,
                            namespace=MARKER_NAMESPACE_TARGET,
                        )
                    if marker_pub_target_right is not None:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_target_right,
                            pos_r,
                            quat_r,
                            "right",
                            rgba=MARKER_COLOR_TARGET,
                            marker_id=1,
                            point_idx=idx,
                            namespace=MARKER_NAMESPACE_TARGET,
                        )
                except Exception as e:
                    rospy.logwarn(f"[offline][点 {idx}] 发布双臂期望marker失败: {e}")

            # ==================== both：直接双臂一起求解（不再“先左后右”） ====================
            q0_14_init = np.concatenate([init_q0_joints_left, init_q0_joints_right])
            # 单点模式：希望 param3 使用 param1 的结果作为关节参考（q0）
            q0_14_for_param3 = q0_14_init.copy()

            # 每套参数可选指定不同 service；None 则走默认 "/ik/two_arm_hand_pose_cmd_srv"
            param_specs = [
                ("param1", IK_SOLVE_PARAM_1, None),
                ("param2", IK_SOLVE_PARAM_2, "/ik/two_arm_hand_pose_cmd_srv_muli_refer"),
                ("param3", IK_SOLVE_PARAM_3, "/ik/two_arm_hand_pose_cmd_srv_muli_refer"),
            ]

            for param_name, ik_param, service_name in param_specs:
                # 单点模式：param3 以 param1 结果作为 q0（参考关节）
                q0_for_this_param = q0_14_init
                if SINGLE_TEST_ENABLE and param_name == "param3":
                    # 按需求：param3 的参考关节固定使用初始化关节 q0_14_init（不参考 param1 解）
                    # 在这里设置 param3 的 q0 偏置（单位：度）
                    # - 方式1：标量（所有14个关节同偏置），例如 5.0
                    # - 方式2：长度为14的列表（每个关节单独偏置），例如 [1, -2, 0, ...]（单位：度）
                    bias_deg = 0.2
                    # bias_deg = [0.0] * 14

                    if isinstance(bias_deg, (int, float)):
                        bias_deg = [float(bias_deg)] * int(q0_14_init.shape[0])
                    if len(bias_deg) != int(q0_14_init.shape[0]):
                        rospy.logwarn(
                            f"[offline][both][param3] bias_deg 长度应为14，当前为 {len(bias_deg)}，将不加偏置"
                        )
                        q0_for_this_param = q0_14_init
                    else:
                        q0_for_this_param = q0_14_init + np.deg2rad(np.array(bias_deg, dtype=float))

                l_j, r_j, time_ms, reason = call_ik_service_both_arms(
                    pos_l,
                    quat_l,
                    pos_r,
                    quat_r,
                    ik_param_dict=ik_param,
                    q0_joints_14=q0_for_this_param,
                    service_name=service_name,
                )

                if l_j is None or r_j is None:
                    rospy.logwarn(f"[offline][点 {idx}][both][{param_name}] 双臂IK失败: {reason}")
                    # 失败原因：左右手各写一条（便于你按手统计/排查）
                    save_error_reason(idx, "left", param_name, reason, status="fail")
                    save_error_reason(idx, "right", param_name, reason, status="fail")
                    save_pos_compare(idx, "left", param_name, pos_l, None, expected_joints=None, ik_joints=None)
                    save_pos_compare(idx, "right", param_name, pos_r, None, expected_joints=None, ik_joints=None)
                    save_detailed_error(idx, "left", param_name, pos_l, quat_l, None, None, False, time_ms)
                    save_detailed_error(idx, "right", param_name, pos_r, quat_r, None, None, False, time_ms)
                    publish_failed_marker(idx, param_name, "left")
                    publish_failed_marker(idx, param_name, "right")
                    verification_results["left"][param_name]["pos_errors"].append(np.nan)
                    verification_results["left"][param_name]["rot_errors"].append(np.nan)
                    verification_results["right"][param_name]["pos_errors"].append(np.nan)
                    verification_results["right"][param_name]["rot_errors"].append(np.nan)
                    continue

                q14_final = np.concatenate([l_j, r_j])

                # 单点模式：把 param1 的解作为 param3 的参考关节（q0）
                if SINGLE_TEST_ENABLE and param_name == "param1":
                    q0_14_for_param3 = q14_final.copy()

                pos2_l, quat2_l = call_fk_service_with_pose(q14_final, "left")
                pos2_r, quat2_r = call_fk_service_with_pose(q14_final, "right")
                if pos2_l is None or quat2_l is None or pos2_r is None or quat2_r is None:
                    rospy.logwarn(f"[offline][点 {idx}][both][{param_name}] FK失败")
                    save_error_reason(idx, "left", param_name, "FK failed after IK success", status="fail")
                    save_error_reason(idx, "right", param_name, "FK failed after IK success", status="fail")
                    save_pos_compare(idx, "left", param_name, pos_l, None, expected_joints=None, ik_joints=l_j)
                    save_pos_compare(idx, "right", param_name, pos_r, None, expected_joints=None, ik_joints=r_j)
                    save_detailed_error(idx, "left", param_name, pos_l, quat_l, None, None, False, time_ms)
                    save_detailed_error(idx, "right", param_name, pos_r, quat_r, None, None, False, time_ms)
                    publish_failed_marker(idx, param_name, "left")
                    publish_failed_marker(idx, param_name, "right")
                    verification_results["left"][param_name]["pos_errors"].append(np.nan)
                    verification_results["left"][param_name]["rot_errors"].append(np.nan)
                    verification_results["right"][param_name]["pos_errors"].append(np.nan)
                    verification_results["right"][param_name]["rot_errors"].append(np.nan)
                    continue

                # 误差（对齐原始目标：左对 (pos_l,quat_l)，右对 (pos_r,quat_r)）
                pos_err_l = np.linalg.norm(pos_l - pos2_l)
                rot_err_l = quaternion_angle_error(quat_l, quat2_l)
                pos_err_r = np.linalg.norm(pos_r - pos2_r)
                rot_err_r = quaternion_angle_error(quat_r, quat2_r)

                # 成功也记录 reason（如果服务端没填，就写 "success"）
                save_error_reason(idx, "left", param_name, reason if reason else "success", status="success")
                save_error_reason(idx, "right", param_name, reason if reason else "success", status="success")

                save_pos_compare(idx, "left", param_name, pos_l, pos2_l, expected_joints=None, ik_joints=l_j)
                save_pos_compare(idx, "right", param_name, pos_r, pos2_r, expected_joints=None, ik_joints=r_j)
                save_detailed_error(idx, "left", param_name, pos_l, quat_l, pos2_l, quat2_l, True, time_ms)
                save_detailed_error(idx, "right", param_name, pos_r, quat_r, pos2_r, quat2_r, True, time_ms)

                verification_results["left"][param_name]["pos_errors"].append(pos_err_l * 1000.0)
                verification_results["left"][param_name]["rot_errors"].append(np.degrees(rot_err_l))
                verification_results["right"][param_name]["pos_errors"].append(pos_err_r * 1000.0)
                verification_results["right"][param_name]["rot_errors"].append(np.degrees(rot_err_r))
                if time_ms is not None:
                    verification_results["left"][param_name]["time_costs"].append(time_ms)
                    verification_results["right"][param_name]["time_costs"].append(time_ms)

                # marker
                if quest3_arm_info_transformer is not None:
                    if param_name == "param1":
                        rgba = MARKER_COLOR_RESULT_PARAM1
                        namespace = MARKER_NAMESPACE_RESULT_PARAM1
                        pub_l = marker_pub_result_param1_left
                        pub_r = marker_pub_result_param1_right
                        base_id = 10
                    elif param_name == "param2":
                        rgba = MARKER_COLOR_RESULT_PARAM2
                        namespace = MARKER_NAMESPACE_RESULT_PARAM2
                        pub_l = marker_pub_result_param2_left
                        pub_r = marker_pub_result_param2_right
                        base_id = 12
                    else:  # param3
                        rgba = MARKER_COLOR_RESULT_PARAM3
                        namespace = MARKER_NAMESPACE_RESULT_PARAM3
                        pub_l = marker_pub_result_param2_left
                        pub_r = marker_pub_result_param2_right
                        base_id = 14
                    try:
                        if pub_l is not None:
                            publish_marker(
                                quest3_arm_info_transformer,
                                pub_l,
                                pos2_l,
                                quat2_l,
                                "left",
                                rgba=rgba,
                                marker_id=base_id,
                                point_idx=idx,
                                namespace=namespace,
                            )
                        if pub_r is not None:
                            publish_marker(
                                quest3_arm_info_transformer,
                                pub_r,
                                pos2_r,
                                quat2_r,
                                "right",
                                rgba=rgba,
                                marker_id=base_id + 1,
                                point_idx=idx,
                                namespace=namespace,
                            )
                    except Exception as e:
                        rospy.logwarn(f"[offline][点 {idx}][both][{param_name}] 发布实际位姿marker失败: {e}")

            if OFFLINE_VERIFICATION_DELAY > 0.0:
                rospy.sleep(OFFLINE_VERIFICATION_DELAY)
            continue
        else:
            pos1, quat1 = pose_item
            # 生成另一只手的随机位置和姿态（用于双臂IK约束）
            other_arm_side = "left" if ARM_SIDE == "right" else "right"
            other_pos, other_quat = generate_random_pose_in_workspace(other_arm_side, USE_RANDOM_ROTATION)

            if other_pos is None or other_quat is None:
                # 如果生成失败，使用默认值
                if other_arm_side == "left":
                    other_pos = np.array([0.4, 0.3, 0.3])  # 默认位置
                else:
                    other_pos = np.array([0.4, -0.3, 0.3])  # 默认位置
                other_quat = euler_to_quaternion(roll_deg=0.0, pitch_deg=-90.0, yaw_deg=0.0)

            if pos1 is None or quat1 is None:
                rospy.logwarn(f"[offline][点 {idx}] 随机位置或姿态生成失败，跳过")
                continue

            pos1_mm = pos1 * 1000.0
            rospy.loginfo(
                f"[offline][点 {idx}] 期望位置(mm)=[{pos1_mm[0]:.1f}, {pos1_mm[1]:.1f}, {pos1_mm[2]:.1f}], "
                f"姿态模式={rotation_mode}，开始三套参数IK验证"
            )

            # 发布期望位姿的 marker（单臂离线验证）
            if quest3_arm_info_transformer is not None:
                marker_pub_target = marker_pub_target_left if ARM_SIDE == "left" else marker_pub_target_right
                if marker_pub_target is not None:
                    try:
                        publish_marker(
                            quest3_arm_info_transformer,
                            marker_pub_target,
                            pos1,
                            quat1,
                            ARM_SIDE,
                            rgba=MARKER_COLOR_TARGET,
                            marker_id=0 if ARM_SIDE == "left" else 1,
                            point_idx=idx,
                            namespace=MARKER_NAMESPACE_TARGET,
                        )
                    except Exception as e:
                        rospy.logwarn(f"[offline][点 {idx}] 发布期望位姿marker失败: {e}")

            q0_joints = init_q0_joints_right if ARM_SIDE == "right" else init_q0_joints_left
            arms_to_verify = [(ARM_SIDE, pos1, quat1, other_pos, other_quat, q0_joints)]

        # 对 arms_to_verify 中的每只手分别做 param1/param2/param3
        for arm_side, pos1, quat1, other_pos, other_quat, q0_joints in arms_to_verify:
            results = []

            # ---------- param1 / param2 ----------
            for param_idx, ik_param in enumerate(ik_params_list):
                param_name = IK_PARAM_NAMES[param_idx]
                service_name = "/ik/two_arm_hand_pose_cmd_srv_muli_refer" if param_name == "param2" else None

                ik_result = call_ik_service(
                    pos1,
                    quat1,
                    arm_side,
                    ik_param_dict=ik_param,
                    other_arm_pos=other_pos,
                    other_arm_quat=other_quat,
                    q0_joints=q0_joints,
                    service_name=service_name,
                )

                if ik_result is None or ik_result[0] is None:
                    error_reason = ik_result[2] if ik_result is not None and len(ik_result) > 2 else "Unknown error"
                    rospy.logwarn(f"[offline][点 {idx}][{arm_side}][{param_name}] IK失败: {error_reason}")
                    save_pos_compare(idx, arm_side, param_name, pos1, None,
                                     expected_joints=None, ik_joints=None)
                    results.append(
                        {
                            "pos1": pos1,
                            "quat1": quat1,
                            "pos2": None,
                            "quat2": None,
                            "success": False,
                            "param_name": param_name,
                            "time_cost_ms": None,
                        }
                    )
                    if arm_side in verification_results and param_name in verification_results[arm_side]:
                        verification_results[arm_side][param_name]["pos_errors"].append(np.nan)
                        verification_results[arm_side][param_name]["rot_errors"].append(np.nan)
                    publish_failed_marker(idx, param_name, arm_side)
                    continue

                ik_joints, time_cost_ms, error_reason = ik_result

                # FK
                ik_joints_14 = build_dual_arm(
                    ik_joints,
                    arm_side,
                    np.radians(INIT_ARM_POS_DEG[7:]) if arm_side == "left" else np.radians(INIT_ARM_POS_DEG[:7]),
                )
                pos2, quat2 = call_fk_service_with_pose(ik_joints_14, arm_side)
                if pos2 is None or quat2 is None:
                    rospy.logwarn(f"[offline][点 {idx}][{arm_side}][{param_name}] FK(姿态2) 失败")
                    save_pos_compare(idx, arm_side, param_name, pos1, None,
                                     expected_joints=None, ik_joints=ik_joints)
                    results.append(
                        {
                            "pos1": pos1,
                            "quat1": quat1,
                            "pos2": None,
                            "quat2": None,
                            "success": False,
                            "param_name": param_name,
                            "time_cost_ms": time_cost_ms,
                        }
                    )
                    if arm_side in verification_results and param_name in verification_results[arm_side]:
                        verification_results[arm_side][param_name]["pos_errors"].append(np.nan)
                        verification_results[arm_side][param_name]["rot_errors"].append(np.nan)
                        if time_cost_ms is not None:
                            verification_results[arm_side][param_name]["time_costs"].append(time_cost_ms)
                    publish_failed_marker(idx, param_name, arm_side)
                    continue

                pos_error = np.linalg.norm(pos1 - pos2)
                rot_error = quaternion_angle_error(quat1, quat2)
                results.append(
                    {
                        "pos1": pos1,
                        "quat1": quat1,
                        "pos2": pos2,
                        "quat2": quat2,
                        "success": True,
                        "param_name": param_name,
                        "pos_error": pos_error,
                        "rot_error": rot_error,
                        "time_cost_ms": time_cost_ms,
                        "ik_joints": ik_joints,
                    }
                )
                save_pos_compare(idx, arm_side, param_name, pos1, pos2,
                                 expected_joints=None, ik_joints=ik_joints)

                # publish marker
                if quest3_arm_info_transformer is not None:
                    if param_name == "param1":
                        result_pub = marker_pub_result_param1_left if arm_side == "left" else marker_pub_result_param1_right
                        rgba = MARKER_COLOR_RESULT_PARAM1
                        namespace = MARKER_NAMESPACE_RESULT_PARAM1
                        base_id = 10
                    else:  # param2
                        result_pub = marker_pub_result_param2_left if arm_side == "left" else marker_pub_result_param2_right
                        rgba = MARKER_COLOR_RESULT_PARAM2
                        namespace = MARKER_NAMESPACE_RESULT_PARAM2
                        base_id = 12
                    if result_pub is not None:
                        try:
                            marker_id = base_id if arm_side == "left" else base_id + 1
                            publish_marker(
                                quest3_arm_info_transformer,
                                result_pub,
                                pos2,
                                quat2,
                                arm_side,
                                rgba=rgba,
                                marker_id=marker_id,
                                point_idx=idx,
                                namespace=namespace,
                            )
                        except Exception as e:
                            rospy.logwarn(f"[offline][点 {idx}][{arm_side}][{param_name}] 发布实际位姿marker失败: {e}")

                # stats
                pos_error_mm = pos_error * 1000.0
                rot_error_deg = np.degrees(rot_error)
                if arm_side in verification_results and param_name in verification_results[arm_side]:
                    verification_results[arm_side][param_name]["pos_errors"].append(pos_error_mm)
                    verification_results[arm_side][param_name]["rot_errors"].append(rot_error_deg)
                    if time_cost_ms is not None:
                        verification_results[arm_side][param_name]["time_costs"].append(time_cost_ms)

            # ---------- param3 ----------
            param_name = "param3"
            ik_result = call_ik_service(
                pos1,
                quat1,
                arm_side,
                ik_param_dict=IK_SOLVE_PARAM_3,
                other_arm_pos=other_pos,
                other_arm_quat=other_quat,
                q0_joints=q0_joints,
                service_name="/ik/two_arm_hand_pose_cmd_srv_muli_refer",
            )

            if ik_result is None or ik_result[0] is None:
                error_reason = ik_result[2] if ik_result is not None and len(ik_result) > 2 else "Unknown error"
                rospy.logwarn(f"[offline][点 {idx}][{arm_side}][{param_name}] IK失败: {error_reason}")
                save_error_reason(idx, arm_side, param_name, error_reason, status="fail")
                save_pos_compare(idx, arm_side, param_name, pos1, None,
                                 expected_joints=None, ik_joints=None)
                results.append(
                    {
                        "pos1": pos1,
                        "quat1": quat1,
                        "pos2": None,
                        "quat2": None,
                        "success": False,
                        "param_name": param_name,
                        "time_cost_ms": None,
                    }
                )
                if arm_side in verification_results and param_name in verification_results[arm_side]:
                    verification_results[arm_side][param_name]["pos_errors"].append(np.nan)
                    verification_results[arm_side][param_name]["rot_errors"].append(np.nan)
                publish_failed_marker(idx, param_name, arm_side)
            else:
                ik_joints, time_cost_ms, error_reason = ik_result
                ik_joints_14 = build_dual_arm(
                    ik_joints,
                    arm_side,
                    np.radians(INIT_ARM_POS_DEG[7:]) if arm_side == "left" else np.radians(INIT_ARM_POS_DEG[:7]),
                )
                pos2, quat2 = call_fk_service_with_pose(ik_joints_14, arm_side)
                if pos2 is None or quat2 is None:
                    rospy.logwarn(f"[offline][点 {idx}][{arm_side}][{param_name}] FK(姿态2) 失败")
                    save_pos_compare(idx, arm_side, param_name, pos1, None,
                                     expected_joints=None, ik_joints=ik_joints)
                    results.append(
                        {
                            "pos1": pos1,
                            "quat1": quat1,
                            "pos2": None,
                            "quat2": None,
                            "success": False,
                            "param_name": param_name,
                            "time_cost_ms": time_cost_ms,
                        }
                    )
                    if arm_side in verification_results and param_name in verification_results[arm_side]:
                        verification_results[arm_side][param_name]["pos_errors"].append(np.nan)
                        verification_results[arm_side][param_name]["rot_errors"].append(np.nan)
                        if time_cost_ms is not None:
                            verification_results[arm_side][param_name]["time_costs"].append(time_cost_ms)
                    publish_failed_marker(idx, param_name, arm_side)
                    save_error_reason(idx, arm_side, param_name, "FK failed after IK success", status="fail")
                else:
                    pos_error = np.linalg.norm(pos1 - pos2)
                    rot_error = quaternion_angle_error(quat1, quat2)
                    results.append(
                        {
                            "pos1": pos1,
                            "quat1": quat1,
                            "pos2": pos2,
                            "quat2": quat2,
                            "success": True,
                            "param_name": param_name,
                            "pos_error": pos_error,
                            "rot_error": rot_error,
                            "time_cost_ms": time_cost_ms,
                        }
                    )
                    save_pos_compare(idx, arm_side, param_name, pos1, pos2,
                                     expected_joints=None, ik_joints=ik_joints)
                    save_error_reason(idx, arm_side, param_name, error_reason if error_reason else "success", status="success")

                    # stats
                    pos_error_mm = pos_error * 1000.0
                    rot_error_deg = np.degrees(rot_error)
                    if arm_side in verification_results and param_name in verification_results[arm_side]:
                        verification_results[arm_side][param_name]["pos_errors"].append(pos_error_mm)
                        verification_results[arm_side][param_name]["rot_errors"].append(rot_error_deg)
                        if time_cost_ms is not None:
                            verification_results[arm_side][param_name]["time_costs"].append(time_cost_ms)

                    # publish marker (param3 uses param2 publisher but different namespace/color)
                    if quest3_arm_info_transformer is not None:
                        result_pub = marker_pub_result_param2_left if arm_side == "left" else marker_pub_result_param2_right
                        if result_pub is not None:
                            try:
                                marker_id = 14 if arm_side == "left" else 15
                                publish_marker(
                                    quest3_arm_info_transformer,
                                    result_pub,
                                    pos2,
                                    quat2,
                                    arm_side,
                                    rgba=MARKER_COLOR_RESULT_PARAM3,
                                    marker_id=marker_id,
                                    point_idx=idx,
                                    namespace=MARKER_NAMESPACE_RESULT_PARAM3,
                                )
                            except Exception as e:
                                rospy.logwarn(f"[offline][点 {idx}][{arm_side}][param3] 发布实际位姿marker失败: {e}")

            # 保存详细误差（每只手臂各自保存三套）
            if len(results) >= 2:
                save_detailed_error(
                    idx, arm_side, "param1",
                    results[0]["pos1"], results[0]["quat1"], results[0]["pos2"], results[0]["quat2"],
                    results[0]["success"], results[0].get("time_cost_ms"),
                )
                save_detailed_error(
                    idx, arm_side, "param2",
                    results[1]["pos1"], results[1]["quat1"], results[1]["pos2"], results[1]["quat2"],
                    results[1]["success"], results[1].get("time_cost_ms"),
                )
                if len(results) >= 3:
                    save_detailed_error(
                        idx, arm_side, "param3",
                        results[2]["pos1"], results[2]["quat1"], results[2]["pos2"], results[2]["quat2"],
                        results[2]["success"], results[2].get("time_cost_ms"),
                    )

        # 每个点之间加入延时，便于在 RViz 中观察 marker 变化
        if OFFLINE_VERIFICATION_DELAY > 0.0:
            rospy.sleep(OFFLINE_VERIFICATION_DELAY)

    rospy.loginfo("[offline] 所有关节点离线FK-IK-FK验证完成")

    # 打印统计信息（仿照在线脚本的格式）
    def print_statistics_offline(arm_side, param_name):
        if arm_side not in verification_results:
            return
        if param_name not in verification_results[arm_side]:
            return

        pos_errors = verification_results[arm_side][param_name]["pos_errors"]
        rot_errors = verification_results[arm_side][param_name]["rot_errors"]
        time_costs = verification_results[arm_side][param_name]["time_costs"]

        if len(pos_errors) == 0:
            rospy.logwarn(f"[offline] {arm_side} 臂 [{param_name}] 没有有效的验证结果")
            return

        pos_errors = np.array(pos_errors)
        rot_errors = np.array(rot_errors)

        rospy.loginfo(
            f"========== {arm_side.upper()} 臂 [{param_name}] 验证统计 =========="
        )
        rospy.loginfo(f"总验证点数: {len(pos_errors)}")
        rospy.loginfo("位置误差 (mm):")
        # 过滤nan值后再计算统计量
        valid_pos_errors = pos_errors[~np.isnan(pos_errors)]
        valid_rot_errors = rot_errors[~np.isnan(rot_errors)]
        if len(valid_pos_errors) > 0:
            rospy.loginfo(f"  均值: {np.mean(valid_pos_errors):.4f}")
            rospy.loginfo(f"  最大值: {np.max(valid_pos_errors):.4f}")
            rospy.loginfo(f"  最小值: {np.min(valid_pos_errors):.4f}")
            rospy.loginfo(f"  标准差: {np.std(valid_pos_errors):.4f}")
        else:
            rospy.loginfo(f"  均值: nan (所有点都失败)")
            rospy.loginfo(f"  最大值: nan")
            rospy.loginfo(f"  最小值: nan")
            rospy.loginfo(f"  标准差: nan")
        
        # 统计位置误差模的个数比例（包括nan）
        total_count = len(pos_errors)
        count_nan = np.sum(np.isnan(pos_errors))
        # 对于非nan值进行统计（如果上面已经计算过valid_pos_errors，这里可以重用）
        count_lt_0_5 = np.sum(valid_pos_errors < 0.5)
        count_0_5_to_1 = np.sum((valid_pos_errors >= 0.5) & (valid_pos_errors < 1.0))
        count_1_to_10 = np.sum((valid_pos_errors >= 1.0) & (valid_pos_errors < 10.0))
        count_10_to_100 = np.sum((valid_pos_errors >= 10.0) & (valid_pos_errors < 100.0))
        count_ge_100 = np.sum(valid_pos_errors >= 100.0)
        
        rospy.loginfo("  位置误差分布:")
        rospy.loginfo(f"    nan: {count_nan} ({count_nan/total_count*100:.2f}%)")
        rospy.loginfo(f"    < 0.5 mm: {count_lt_0_5} ({count_lt_0_5/total_count*100:.2f}%)")
        rospy.loginfo(f"    0.5 ~ 1 mm: {count_0_5_to_1} ({count_0_5_to_1/total_count*100:.2f}%)")
        rospy.loginfo(f"    1 ~ 10 mm: {count_1_to_10} ({count_1_to_10/total_count*100:.2f}%)")
        rospy.loginfo(f"    10 ~ 100 mm: {count_10_to_100} ({count_10_to_100/total_count*100:.2f}%)")
        rospy.loginfo(f"    >= 100 mm: {count_ge_100} ({count_ge_100/total_count*100:.2f}%)")
        
        rospy.loginfo("旋转误差 (度):")
        if len(valid_rot_errors) > 0:
            rospy.loginfo(f"  均值: {np.mean(valid_rot_errors):.4f}")
            rospy.loginfo(f"  最大值: {np.max(valid_rot_errors):.4f}")
            rospy.loginfo(f"  最小值: {np.min(valid_rot_errors):.4f}")
            rospy.loginfo(f"  标准差: {np.std(valid_rot_errors):.4f}")
        else:
            rospy.loginfo(f"  均值: nan (所有点都失败)")
            rospy.loginfo(f"  最大值: nan")
            rospy.loginfo(f"  最小值: nan")
            rospy.loginfo(f"  标准差: nan")
        if len(time_costs) > 0:
            time_costs = np.array(time_costs)
            rospy.loginfo("求解时间 (ms):")
            rospy.loginfo(f"  均值: {np.mean(time_costs):.4f}")
            rospy.loginfo(f"  最大值: {np.max(time_costs):.4f}")
            rospy.loginfo(f"  最小值: {np.min(time_costs):.4f}")
            rospy.loginfo(f"  标准差: {np.std(time_costs):.4f}")
        rospy.loginfo("============================================")

    # 打印统计信息
    if ARM_SIDE == "both":
        for param_name in IK_PARAM_NAMES:
            print_statistics_offline("left", param_name)
            print_statistics_offline("right", param_name)
    elif ARM_SIDE in ("left", "right"):
        for param_name in IK_PARAM_NAMES:
            print_statistics_offline(ARM_SIDE, param_name)


if __name__ == "__main__":
    main()


