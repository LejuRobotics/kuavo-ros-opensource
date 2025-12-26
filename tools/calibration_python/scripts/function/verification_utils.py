#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证结果保存和统计工具函数：
- 保存位置对比结果
- 保存详细误差
- 保存错误原因
- 打印统计信息
"""

import os
import numpy as np
import rospy
from function.file_utils import get_config_dir, format_with_sign
from function.motion_utils import pose_difference, quaternion_to_euler


def save_pos_compare(point_idx, arm_side, param_name, pos1, pos2, 
                     quat1=None, quat2=None, expected_joints=None, ik_joints=None,
                     file_prefix="fk_ik", config_dir=None):
    """
    保存期望/解算位置对比（单位 mm），以及关节角误差（单位 度）
    
    Args:
        point_idx: 点索引
        arm_side: 手臂侧（"left" 或 "right"）
        param_name: 参数名称
        pos1: 期望位置（不能为None）
        pos2: 解算位置（可以为None，表示失败）
        quat1: 期望姿态四元数（可选，用于保存姿态信息）
        quat2: 解算姿态四元数（可选，用于保存姿态信息）
        expected_joints: 期望关节角（弧度，7个关节，可选）
        ik_joints: 逆解关节角（弧度，7个关节，可选）
        file_prefix: 文件前缀，默认"fk_ik"，离线脚本使用"fk_ik_offline"
        config_dir: 配置目录路径（可选），如果提供则使用，否则调用get_config_dir()获取
    """
    if pos1 is None:
        return

    pos1_mm = np.array(pos1) * 1000.0
    
    # 格式化解算位置
    if pos2 is None:
        pos2_str = "[   nan,     nan,     nan]"
    else:
        pos2_mm = np.array(pos2) * 1000.0
        pos2_x = format_with_sign(pos2_mm[0], precision=1)
        pos2_y = format_with_sign(pos2_mm[1], precision=1)
        pos2_z = format_with_sign(pos2_mm[2], precision=1)
        pos2_str = f"[{pos2_x:>7},  {pos2_y:>7},  {pos2_z:>7}]"

    file_name = {
        "left": f"{file_prefix}_pos_compare_left_arm_{param_name}.txt",
        "right": f"{file_prefix}_pos_compare_right_arm_{param_name}.txt",
        "both": f"{file_prefix}_pos_compare_both_arms_{param_name}.txt",
    }.get(arm_side, f"{file_prefix}_pos_compare_{param_name}.txt")

    if config_dir is None:
        config_dir = get_config_dir()
    path = os.path.join(config_dir, file_name)
    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}" if file_prefix == "fk_ik_offline" else str(point_idx)
        
        # 格式化期望位置，使用固定宽度对齐
        pos1_x = format_with_sign(pos1_mm[0], precision=1)
        pos1_y = format_with_sign(pos1_mm[1], precision=1)
        pos1_z = format_with_sign(pos1_mm[2], precision=1)
        pos1_str = f"[{pos1_x:>7},  {pos1_y:>7},  {pos1_z:>7}]"
        
        line = f"{idx_str},  {pos1_str},  {pos2_str}"
        
        # 如果提供了姿态信息，添加姿态对比
        if quat1 is not None and quat2 is not None:
            roll1, pitch1, yaw1 = quaternion_to_euler(quat1)
            roll2, pitch2, yaw2 = quaternion_to_euler(quat2)
            roll1_deg = np.degrees(roll1)
            pitch1_deg = np.degrees(pitch1)
            yaw1_deg = np.degrees(yaw1)
            roll2_deg = np.degrees(roll2)
            pitch2_deg = np.degrees(pitch2)
            yaw2_deg = np.degrees(yaw2)
            line += f",  [{roll1_deg:.2f},  {pitch1_deg:.2f},  {yaw1_deg:.2f}],  [{roll2_deg:.2f},  {pitch2_deg:.2f},  {yaw2_deg:.2f}]"
        elif quat1 is not None:
            # 只有期望姿态，没有解算姿态
            roll1, pitch1, yaw1 = quaternion_to_euler(quat1)
            roll1_deg = np.degrees(roll1)
            pitch1_deg = np.degrees(pitch1)
            yaw1_deg = np.degrees(yaw1)
            line += f",  [{roll1_deg:.2f},  {pitch1_deg:.2f},  {yaw1_deg:.2f}],  [nan,  nan,  nan]"
        
        # 如果有关节角信息，添加关节角误差
        if expected_joints is not None and ik_joints is not None:
            expected_joints = np.array(expected_joints)
            ik_joints = np.array(ik_joints)
            
            # 计算每个关节的误差（弧度转度）
            joint_errors_rad = ik_joints - expected_joints
            joint_errors_deg = np.degrees(joint_errors_rad)
            
            # 格式化每个关节的误差，使用固定宽度对齐
            joint_error_parts = [f"{format_with_sign(err, precision=2):>7}" for err in joint_errors_deg]
            joint_error_str = ",  ".join(joint_error_parts)
            
            line += f",  [{joint_error_str}]"
        elif expected_joints is not None:
            # 如果只有期望关节角，没有逆解关节角，关节误差也用nan
            line += ",  [   nan,     nan,     nan,     nan,     nan,     nan,     nan]"
        
        line += "\n"
        f.write(line)


def save_detailed_error(point_idx, arm_side, param_name, pos1, quat1, pos2, quat2, 
                       success, time_cost_ms=None, file_prefix="fk_ik", config_dir=None):
    """
    保存详细误差（位置模 / xyz / 欧拉角差 / 求解时间）
    
    Args:
        point_idx: 点索引
        arm_side: 手臂侧
        param_name: 参数名称
        pos1: 期望位置
        quat1: 期望姿态四元数
        pos2: 解算位置（可以为None）
        quat2: 解算姿态四元数（可以为None）
        success: 是否成功
        time_cost_ms: 求解时间（毫秒，可选）
        file_prefix: 文件前缀，默认"fk_ik"，离线脚本使用"fk_ik_offline"
        config_dir: 配置目录路径（可选），如果提供则使用，否则调用get_config_dir()获取
    """
    file_name = f"{file_prefix}_error_detailed_{arm_side}_arm_{param_name}.txt"
    if config_dir is None:
        config_dir = get_config_dir()
    path = os.path.join(config_dir, file_name)

    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}" if file_prefix == "fk_ik_offline" else str(point_idx)
        if (not success) or pos2 is None or quat2 is None:
            time_str = "nan" if time_cost_ms is None else f"{time_cost_ms:.1f}"
            if file_prefix == "fk_ik_offline":
                f.write(
                    f"{idx_str},    "
                    f"nan,    [nan,  nan,  nan],    nan,    [nan,  nan,  nan],    time={time_str}\n"
                )
            else:
                f.write(
                    f"{point_idx},    "
                    f"nan,    nan,    nan,    nan,    nan,    nan,    nan\n"
                )
        else:
            diff = pose_difference(pos1, quat1, pos2, quat2)
            
            if file_prefix == "fk_ik_offline":
                # 离线脚本格式：包含总角度误差和时间
                time_str = f"{time_cost_ms:.1f}" if time_cost_ms is not None else "nan"
                roll_deg = np.degrees(diff['roll_diff'])
                pitch_deg = np.degrees(diff['pitch_diff'])
                yaw_deg = np.degrees(diff['yaw_diff'])
                total_angle_error_deg = np.degrees(diff['total_angle_error'])
                
                pos_norm_mm = diff['pos_norm'] * 1000.0
                pos_diff_x_mm = diff['pos_diff_x'] * 1000.0
                pos_diff_y_mm = diff['pos_diff_y'] * 1000.0
                pos_diff_z_mm = diff['pos_diff_z'] * 1000.0
                
                f.write(
                    f"{idx_str},    "
                    f"{format_with_sign(pos_norm_mm)},    "
                    f"[{format_with_sign(pos_diff_x_mm)},  {format_with_sign(pos_diff_y_mm)},  {format_with_sign(pos_diff_z_mm)}],    "
                    f"{total_angle_error_deg:.2f},    "
                    f"[{format_with_sign(roll_deg, precision=2)},  {format_with_sign(pitch_deg, precision=2)},  {format_with_sign(yaw_deg, precision=2)}],    "
                    f"time={time_str}\n"
                )
            else:
                # 在线脚本格式：简化的格式
                pos_norm_mm = diff['pos_norm'] * 1000.0
                pos_diff_x_mm = diff['pos_diff_x'] * 1000.0
                pos_diff_y_mm = diff['pos_diff_y'] * 1000.0
                pos_diff_z_mm = diff['pos_diff_z'] * 1000.0
                roll_deg = np.degrees(diff['roll_diff'])
                pitch_deg = np.degrees(diff['pitch_diff'])
                yaw_deg = np.degrees(diff['yaw_diff'])
                
                f.write(
                    f"{point_idx},    "
                    f"{pos_norm_mm:.1f},    "
                    f"{pos_diff_x_mm:.1f},    "
                    f"{pos_diff_y_mm:.1f},    "
                    f"{pos_diff_z_mm:.1f},    "
                    f"{roll_deg:.1f},    "
                    f"{pitch_deg:.1f},    "
                    f"{yaw_deg:.1f}\n"
                )


def save_error_reason(point_idx, arm_side, param_name, error_reason, file_prefix="fk_ik_offline"):
    """
    保存逆解失败原因
    
    Args:
        point_idx: 点索引
        arm_side: 手臂侧
        param_name: 参数名称
        error_reason: 错误原因
        file_prefix: 文件前缀，默认"fk_ik_offline"
    """
    file_name = f"{file_prefix}_error_reason_{arm_side}_arm_{param_name}.txt"
    path = os.path.join(get_config_dir(), file_name)
    
    with open(path, "a") as f:
        idx_str = f"{point_idx:03d}" if file_prefix == "fk_ik_offline" else str(point_idx)
        f.write(f"{idx_str}, {error_reason}\n")


def print_statistics(arm_side, param_name, verification_results, color_purple="\033[95m", color_reset="\033[0m", 
                     include_time=False):
    """
    打印验证结果的统计信息
    
    Args:
        arm_side: 手臂侧
        param_name: 参数集名称
        verification_results: 验证结果字典，结构为 {arm_side: {param_name: {'pos_errors': [], 'rot_errors': [], ...}}}
        color_purple: 紫色ANSI颜色码
        color_reset: 重置ANSI颜色码
        include_time: 是否包含时间统计（离线脚本使用）
    """
    if arm_side not in verification_results:
        return
    
    if param_name not in verification_results[arm_side]:
        return

    pos_errors = verification_results[arm_side][param_name].get('pos_errors', [])
    rot_errors = verification_results[arm_side][param_name].get('rot_errors', [])
    time_costs = verification_results[arm_side][param_name].get('time_costs', []) if include_time else []

    if len(pos_errors) == 0:
        rospy.logwarn(f"{arm_side} 臂 [{param_name}] 没有有效的验证结果")
        return

    pos_errors = np.array(pos_errors)
    rot_errors = np.array(rot_errors)

    rospy.loginfo(f"\n{color_purple}========== {arm_side.upper()} 臂 [{param_name}] 验证统计 =========={color_reset}")
    rospy.loginfo(f"总验证点数: {len(pos_errors)}")
    
    # 位置误差统计
    rospy.loginfo("位置误差 (mm):")
    valid_pos_errors = pos_errors[~np.isnan(pos_errors)]
    valid_rot_errors = rot_errors[~np.isnan(rot_errors)]
    
    if len(valid_pos_errors) > 0:
        rospy.loginfo(f"  均值: {np.mean(valid_pos_errors):.4f}")
        rospy.loginfo(f"  最大值: {np.max(valid_pos_errors):.4f}")
        rospy.loginfo(f"  最小值: {np.min(valid_pos_errors):.4f}")
        rospy.loginfo(f"  标准差: {np.std(valid_pos_errors):.4f}")
        
        # 离线脚本包含位置误差分布统计
        if include_time:
            total_count = len(pos_errors)
            count_nan = np.sum(np.isnan(pos_errors))
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
    else:
        rospy.loginfo("  均值: nan (所有点都失败)")
        rospy.loginfo("  最大值: nan")
        rospy.loginfo("  最小值: nan")
        rospy.loginfo("  标准差: nan")
    
    # 旋转误差统计
    rospy.loginfo("旋转误差 (度):")
    if len(valid_rot_errors) > 0:
        rospy.loginfo(f"  均值: {np.mean(valid_rot_errors):.4f}")
        rospy.loginfo(f"  最大值: {np.max(valid_rot_errors):.4f}")
        rospy.loginfo(f"  最小值: {np.min(valid_rot_errors):.4f}")
        rospy.loginfo(f"  标准差: {np.std(valid_rot_errors):.4f}")
    else:
        rospy.loginfo("  均值: nan (所有点都失败)")
        rospy.loginfo("  最大值: nan")
        rospy.loginfo("  最小值: nan")
        rospy.loginfo("  标准差: nan")
    
    # 时间统计（仅离线脚本）
    if include_time and len(time_costs) > 0:
        time_costs = np.array(time_costs)
        rospy.loginfo("求解时间 (ms):")
        rospy.loginfo(f"  均值: {np.mean(time_costs):.4f}")
        rospy.loginfo(f"  最大值: {np.max(time_costs):.4f}")
        rospy.loginfo(f"  最小值: {np.min(time_costs):.4f}")
        rospy.loginfo(f"  标准差: {np.std(time_costs):.4f}")
    
    rospy.loginfo(f"{color_purple}==========================================={color_reset}\n")

