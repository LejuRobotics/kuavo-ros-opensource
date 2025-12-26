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
import rospy
import numpy as np
from kuavo_msgs.srv import fkSrv

from function.motion_utils import (
    generate_filtered_joint_targets,
    check_workspace,
    quaternion_angle_error,
    pose_difference,
)
from function.file_utils import get_config_dir
from function.data_utils import (
    call_fk_service_both_arms,
    build_dual_arm,
    call_ik_service,
)


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

NUM_DEBUG_PTS = 50000
TIME_PER_POINT = 3.0  # 仅用于与在线脚本保持一致，这里不实际用到轨迹
ARM_SIDE = "right"    # 仅验证右臂，如需左臂可改为 "left"
USE_PREVIOUS_TARGET_AS_REFERENCE = True  # True: 使用上一次的期望关节作为参考关节; False: 使用初始关节作为参考关节

INIT_ARM_POS_DEG = [20, 0, 0, -30, 0, 0, 0,
                    20, 0, 0, -30, 0, 0, 0]

RIGHT_ARM_WORKSPACE = {
    "x_min": 0.15, "x_max": 0.75,
    "y_min": -0.62, "y_max": -0.02,
    "z_min": 0.0,  "z_max": 0.7,
}

LEFT_ARM_WORKSPACE = {
    "x_min": 0.15, "x_max": 0.75,
    "y_min": 0.02, "y_max": 0.62,
    "z_min": 0.0,  "z_max": 0.7,
}

# 和在线脚本保持同一套参数
IK_SOLVE_PARAM_1 = {
    "major_optimality_tol": 1e-3,
    "major_feasibility_tol": 1e-3,
    "minor_feasibility_tol": 3e-3,
    "major_iterations_limit": 100,
    "oritation_constraint_tol": 10e-3,
    "pos_constraint_tol": 1e-3,
    "pos_cost_weight": 10,
}

IK_SOLVE_PARAM_2 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 3e-3,
    'major_iterations_limit': 100,
    'oritation_constraint_tol': 10e-3,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10.0,
}

IK_SOLVE_PARAM_3 = {
    'major_optimality_tol': 1e-3,
    'major_feasibility_tol': 1e-3,
    'minor_feasibility_tol': 3e-3,
    'major_iterations_limit': 100,
    'oritation_constraint_tol': 10e-3,
    'pos_constraint_tol': 1e-3,
    'pos_cost_weight': 10.0,
}


IK_PARAM_NAMES = ["param1", "param2", "param3"]


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


def save_error_reason(point_idx, arm_side, param_name, error_reason):
    """
    保存逆解失败原因
    格式：点索引, 错误原因
    """
    file_name = f"fk_ik_offline_error_reason_{arm_side}_arm_{param_name}.txt"
    path = os.path.join(get_config_dir(), file_name)
    
    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}"
        f.write(f"{idx_str}, {error_reason}\n")


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
            
            # 计算总角度误差
            roll_deg = np.degrees(diff['roll_diff'])
            pitch_deg = np.degrees(diff['pitch_diff'])
            yaw_deg = np.degrees(diff['yaw_diff'])
            total_angle_error = np.sqrt(roll_deg**2 + pitch_deg**2 + yaw_deg**2)
            
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
                f"[{format_with_sign(roll_deg)},  {format_with_sign(pitch_deg)},  {format_with_sign(yaw_deg)}],    "
                f"time={time_str}\n"
            )


# ==================== 主流程 ====================

def main():
    rospy.init_node("arm_fk_ik_offline_verifier")

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
                    "# 点索引,位置误差模(mm),[x差值(mm), y差值(mm), z差值(mm)],总角度误差(度) [roll差值(度), pitch差值(度), yaw差值(度)],time=求解时间(ms)\n"
                )
            
            # 为param3初始化错误原因文件
            if param_name == "param3":
                error_reason_file = f"fk_ik_offline_error_reason_{arm}_arm_{param_name}.txt"
                error_reason_path = os.path.join(get_config_dir(), error_reason_file)
                with open(error_reason_path, "w") as f:
                    f.write("# 点索引, 错误原因\n")

    # 生成随机关节角（与在线脚本一致）
    rospy.loginfo(
        f"[offline] 目标有效点数 = {NUM_DEBUG_PTS} ... (ARM_SIDE={ARM_SIDE})"
    )
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
            timeout_s=1000.0,
            fk_service_callback=call_fk_service_both_arms,
        )
    except TimeoutError as e:
        rospy.logerr(str(e))
        return

    rospy.loginfo(
        f"[offline] 工作空间过滤完成: {len(filtered_targets)}/{NUM_DEBUG_PTS} 个有效点"
    )

    # 将过滤后的期望关节角保存到文件，便于离线查看和复现
    try:
        targets_array = np.array(filtered_targets, dtype=float)
        joints_file = os.path.join(
            get_config_dir(), f"fk_ik_offline_expected_joints_{ARM_SIDE}_arm.txt"
        )
        np.savetxt(joints_file, targets_array, fmt="%.4f", delimiter=", ")
        rospy.loginfo(f"[offline] 期望关节角已保存到: {joints_file}")
    except Exception as e:
        rospy.logwarn(f"[offline] 保存期望关节角到文件失败: {e}")

    # 逐点做 FK -> IK(param1/param2) -> FK 一致性验证
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

    # 存储上一次的期望关节角，用于下一次的参考关节
    # 根据 USE_PREVIOUS_TARGET_AS_REFERENCE 参数决定参考关节的选择方式
    if ARM_SIDE == "right":
        # 右臂使用 INIT_ARM_POS_DEG 的后 7 个关节
        init_q0_joints = np.radians(INIT_ARM_POS_DEG[7:])
    elif ARM_SIDE == "left":
        # 左臂使用 INIT_ARM_POS_DEG 的前 7 个关节
        init_q0_joints = np.radians(INIT_ARM_POS_DEG[:7])
    else:
        init_q0_joints = None
    
    # 初始化 prev_q0_joints，第一次使用初始关节
    prev_q0_joints = init_q0_joints

    for idx, target_deg in enumerate(filtered_targets, start=1):
        target_rad = np.radians(target_deg)

        # 当前关节下，两只手的姿态
        if ARM_SIDE == "left":
            pos1, quat1 = call_fk_service_with_pose(target_rad, "left")
            other_pos, other_quat = call_fk_service_with_pose(target_rad, "right")
        elif ARM_SIDE == "right":
            pos1, quat1 = call_fk_service_with_pose(target_rad, "right")
            other_pos, other_quat = call_fk_service_with_pose(target_rad, "left")
        else:
            rospy.logwarn(f"[offline] ARM_SIDE 配置无效: {ARM_SIDE}")
            return

        if pos1 is None or quat1 is None:
            rospy.logwarn(f"[offline][点 {idx}] FK(姿态1) 失败，跳过")
            continue

        # 只打印当前验证手的7个关节，避免多余信息
        if ARM_SIDE == "left":
            target_view_deg = np.round(target_deg[:7], 2)
        elif ARM_SIDE == "right":
            target_view_deg = np.round(target_deg[7:], 2)
        else:
            target_view_deg = np.round(target_deg, 2)

        rospy.loginfo(
            f"[offline][点 {idx}] 目标关节(度)={target_view_deg}，开始三套参数IK验证"
        )

        # 根据 USE_PREVIOUS_TARGET_AS_REFERENCE 参数决定参考关节
        if USE_PREVIOUS_TARGET_AS_REFERENCE:
            # 使用上一次的期望关节作为参考关节
            q0_joints = prev_q0_joints
        else:
            # 使用初始关节作为参考关节
            q0_joints = init_q0_joints

        results = []
        # 先计算 param1 和 param2
        for param_idx, ik_param in enumerate(ik_params_list):
            param_name = IK_PARAM_NAMES[param_idx]

            # param2 使用多参考点服务
            service_name = None
            if param_name == "param2":
                service_name = "/ik/two_arm_hand_pose_cmd_srv_muli_refer"

            ik_result = call_ik_service(
                pos1,
                quat1,
                ARM_SIDE,
                ik_param_dict=ik_param,
                other_arm_pos=other_pos,
                other_arm_quat=other_quat,
                q0_joints=q0_joints,  # 传入期望关节角作为参考关节
                service_name=service_name,  # param2 使用多参考点服务
            )

            if ik_result is None or ik_result[0] is None:
                error_reason = ik_result[2] if ik_result is not None and len(ik_result) > 2 else "Unknown error"
                rospy.logwarn(f"[offline][点 {idx}][{param_name}] IK失败: {error_reason}")
                # 提取对应手臂的期望关节角
                if ARM_SIDE == "left":
                    expected_joints_7 = target_rad[:7]
                elif ARM_SIDE == "right":
                    expected_joints_7 = target_rad[7:]
                else:
                    expected_joints_7 = None
                
                # 保存位置对比（失败时也保存期望位姿）
                save_pos_compare(idx, ARM_SIDE, param_name, pos1, None,
                                expected_joints=expected_joints_7, ik_joints=None)
                
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
                # 失败时也要添加到统计中（使用nan）
                if (
                    ARM_SIDE in verification_results
                    and param_name in verification_results[ARM_SIDE]
                ):
                    verification_results[ARM_SIDE][param_name]["pos_errors"].append(np.nan)
                    verification_results[ARM_SIDE][param_name]["rot_errors"].append(np.nan)
                continue

            ik_joints, time_cost_ms, error_reason = ik_result

            # 将单臂7关节扩展为14关节，再做 FK
            if ARM_SIDE == "left":
                ik_joints_14 = build_dual_arm(
                    ik_joints, "left", np.radians(INIT_ARM_POS_DEG[7:])
                )
            else:
                ik_joints_14 = build_dual_arm(
                    ik_joints, "right", np.radians(INIT_ARM_POS_DEG[:7])
                )

            pos2, quat2 = call_fk_service_with_pose(ik_joints_14, ARM_SIDE)
            if pos2 is None or quat2 is None:
                rospy.logwarn(f"[offline][点 {idx}][{param_name}] FK(姿态2) 失败")
                # 提取对应手臂的期望关节角
                if ARM_SIDE == "left":
                    expected_joints_7 = target_rad[:7]
                elif ARM_SIDE == "right":
                    expected_joints_7 = target_rad[7:]
                else:
                    expected_joints_7 = None
                
                # 保存位置对比（失败时也保存期望位姿）
                save_pos_compare(idx, ARM_SIDE, param_name, pos1, None,
                                expected_joints=expected_joints_7, ik_joints=ik_joints)
                
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
                # 失败时也要添加到统计中（使用nan）
                if (
                    ARM_SIDE in verification_results
                    and param_name in verification_results[ARM_SIDE]
                ):
                    verification_results[ARM_SIDE][param_name]["pos_errors"].append(np.nan)
                    verification_results[ARM_SIDE][param_name]["rot_errors"].append(np.nan)
                    if time_cost_ms is not None:
                        verification_results[ARM_SIDE][param_name]["time_costs"].append(time_cost_ms)
                continue

            pos_error = np.linalg.norm(pos1 - pos2)
            rot_error = quaternion_angle_error(quat1, quat2)

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
                    "ik_joints": ik_joints,  # 保存关节角，用于 param3 的参考
                }
            )

            # 保存位置对比（包含关节角误差）
            # 提取对应手臂的期望关节角
            if ARM_SIDE == "left":
                expected_joints_7 = target_rad[:7]
            elif ARM_SIDE == "right":
                expected_joints_7 = target_rad[7:]
            else:
                expected_joints_7 = None
            
            save_pos_compare(idx, ARM_SIDE, param_name, pos1, pos2, 
                            expected_joints=expected_joints_7, ik_joints=ik_joints)

            # 存入统计
            pos_error_mm = pos_error * 1000.0
            rot_error_deg = np.degrees(rot_error)
            if (
                ARM_SIDE in verification_results
                and param_name in verification_results[ARM_SIDE]
            ):
                verification_results[ARM_SIDE][param_name]["pos_errors"].append(
                    pos_error_mm
                )
                verification_results[ARM_SIDE][param_name]["rot_errors"].append(
                    rot_error_deg
                )
                if time_cost_ms is not None:
                    verification_results[ARM_SIDE][param_name]["time_costs"].append(
                        time_cost_ms
                    )

        # 计算 param3：使用和 param1、param2 相同的参考关节，使用伪逆服务
        param_name = "param3"
        ik_result = call_ik_service(
            pos1,
            quat1,
            ARM_SIDE,
            ik_param_dict=IK_SOLVE_PARAM_3,
            other_arm_pos=other_pos,
            other_arm_quat=other_quat,
            q0_joints=q0_joints,  # 使用和 param1、param2 相同的参考关节
            service_name="/ik/two_arm_hand_pose_cmd_srv",  # 使用伪逆服务
        )

        if ik_result is None or ik_result[0] is None:
            error_reason = ik_result[2] if ik_result is not None and len(ik_result) > 2 else "Unknown error"
            rospy.logwarn(f"[offline][点 {idx}][{param_name}] IK失败: {error_reason}")
            
            # 保存错误原因
            save_error_reason(idx, ARM_SIDE, param_name, error_reason)
            
            # 提取对应手臂的期望关节角
            if ARM_SIDE == "left":
                expected_joints_7 = target_rad[:7]
            elif ARM_SIDE == "right":
                expected_joints_7 = target_rad[7:]
            else:
                expected_joints_7 = None
            
            # 保存位置对比（失败时也保存期望位姿）
            save_pos_compare(idx, ARM_SIDE, param_name, pos1, None,
                            expected_joints=expected_joints_7, ik_joints=None)
            
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
            # 失败时也要添加到统计中（使用nan）
            if (
                ARM_SIDE in verification_results
                and param_name in verification_results[ARM_SIDE]
            ):
                verification_results[ARM_SIDE][param_name]["pos_errors"].append(np.nan)
                verification_results[ARM_SIDE][param_name]["rot_errors"].append(np.nan)
        else:
            ik_joints, time_cost_ms, error_reason = ik_result

            # 将单臂7关节扩展为14关节，再做 FK
            if ARM_SIDE == "left":
                ik_joints_14 = build_dual_arm(
                    ik_joints, "left", np.radians(INIT_ARM_POS_DEG[7:])
                )
            else:
                ik_joints_14 = build_dual_arm(
                    ik_joints, "right", np.radians(INIT_ARM_POS_DEG[:7])
                )

            pos2, quat2 = call_fk_service_with_pose(ik_joints_14, ARM_SIDE)
            if pos2 is None or quat2 is None:
                rospy.logwarn(f"[offline][点 {idx}][{param_name}] FK(姿态2) 失败")
                # 提取对应手臂的期望关节角
                if ARM_SIDE == "left":
                    expected_joints_7 = target_rad[:7]
                elif ARM_SIDE == "right":
                    expected_joints_7 = target_rad[7:]
                else:
                    expected_joints_7 = None
                
                # 保存位置对比（失败时也保存期望位姿）
                save_pos_compare(idx, ARM_SIDE, param_name, pos1, None,
                                expected_joints=expected_joints_7, ik_joints=ik_joints)
                
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
                # 失败时也要添加到统计中（使用nan）
                if (
                    ARM_SIDE in verification_results
                    and param_name in verification_results[ARM_SIDE]
                ):
                    verification_results[ARM_SIDE][param_name]["pos_errors"].append(np.nan)
                    verification_results[ARM_SIDE][param_name]["rot_errors"].append(np.nan)
                    if time_cost_ms is not None:
                        verification_results[ARM_SIDE][param_name]["time_costs"].append(time_cost_ms)
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

                # 保存位置对比（包含关节角误差）
                # 提取对应手臂的期望关节角
                if ARM_SIDE == "left":
                    expected_joints_7 = target_rad[:7]
                elif ARM_SIDE == "right":
                    expected_joints_7 = target_rad[7:]
                else:
                    expected_joints_7 = None
                
                save_pos_compare(idx, ARM_SIDE, param_name, pos1, pos2,
                                expected_joints=expected_joints_7, ik_joints=ik_joints)

                # 存入统计
                pos_error_mm = pos_error * 1000.0
                rot_error_deg = np.degrees(rot_error)
                if (
                    ARM_SIDE in verification_results
                    and param_name in verification_results[ARM_SIDE]
                ):
                    verification_results[ARM_SIDE][param_name]["pos_errors"].append(
                        pos_error_mm
                    )
                    verification_results[ARM_SIDE][param_name]["rot_errors"].append(
                        rot_error_deg
                    )
                    if time_cost_ms is not None:
                        verification_results[ARM_SIDE][param_name]["time_costs"].append(
                            time_cost_ms
                        )

        # 保存详细误差（需要三套结果）
        if len(results) >= 2:
            save_detailed_error(
                idx,
                ARM_SIDE,
                "param1",
                results[0]["pos1"],
                results[0]["quat1"],
                results[0]["pos2"],
                results[0]["quat2"],
                results[0]["success"],
                results[0].get("time_cost_ms"),
            )
            save_detailed_error(
                idx,
                ARM_SIDE,
                "param2",
                results[1]["pos1"],
                results[1]["quat1"],
                results[1]["pos2"],
                results[1]["quat2"],
                results[1]["success"],
                results[1].get("time_cost_ms"),
            )
            # 保存 param3 的详细误差（如果存在）
            if len(results) >= 3:
                save_detailed_error(
                    idx,
                    ARM_SIDE,
                    "param3",
                    results[2]["pos1"],
                    results[2]["quat1"],
                    results[2]["pos2"],
                    results[2]["quat2"],
                    results[2]["success"],
                    results[2].get("time_cost_ms"),
                )

        # 如果使用上一次期望关节作为参考，更新 prev_q0_joints
        if USE_PREVIOUS_TARGET_AS_REFERENCE:
            if ARM_SIDE == "left":
                prev_q0_joints = target_rad[:7]  # 左臂前7个关节
            elif ARM_SIDE == "right":
                prev_q0_joints = target_rad[7:]  # 右臂后7个关节

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

    # 目前只支持单臂_ARM_SIDE，默认 right
    if ARM_SIDE in ("left", "right"):
        for param_name in IK_PARAM_NAMES:
            print_statistics_offline(ARM_SIDE, param_name)


if __name__ == "__main__":
    main()


