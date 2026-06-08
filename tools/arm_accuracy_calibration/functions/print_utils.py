# -*- coding: utf-8 -*-
"""
打印工具：表格形式输出位姿和误差
"""
import numpy as np
from typing import Optional, Dict, Any, List

from .transform_utils import (
    rotation_matrix_to_euler,
    quaternion_array_to_euler,
    pose_to_transform_matrix,
)


def print_calibration_matrices_table(
    T1: Optional[np.ndarray] = None,
    T2: Optional[np.ndarray] = None,
    T3: Optional[np.ndarray] = None,
    result_in_j1: Optional[Dict[str, np.ndarray]] = None,
    throttle_sec: float = 1.0,
) -> None:
    """
    以表格形式打印标定矩阵及所有关节在 joint_1 下的位姿

    参数:
        T1: joint_2 在 joint_1 下的变换矩阵
        T2: joint_3 在 joint_2 下的变换矩阵
        T3: joint_4 在 joint_3 下的变换矩阵
        result_in_j1: 各关节在 joint_1 下的位姿，如 {"joint_1": T, "joint_2": T, ...}
        throttle_sec: 打印节流时间(秒)，0 表示不节流
    """
    try:
        import rospy
        use_throttle = throttle_sec > 0
    except Exception:
        use_throttle = False

    def append_row(lines: List[str], label: str, T) -> None:
        if T is not None:
            pos = T[:3, 3]
            R = T[:3, :3]
            yaw, pitch, roll = rotation_matrix_to_euler(R)
            lines.append(
                f"{label:<22} | {pos[0]:>12.1f} | {pos[1]:>12.1f} | {pos[2]:>12.1f} | "
                f"{yaw:>12.1f} | {pitch:>12.1f} | {roll:>12.1f}"
            )
        else:
            lines.append(f"{label:<22} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")

    header = f"{'项目':<22} | {'X (mm)':>12} | {'Y (mm)':>12} | {'Z (mm)':>12} | {'Yaw (deg)':>12} | {'Pitch (deg)':>12} | {'Roll (deg)':>12}"
    sep = "=" * 110
    line_sep = "-" * 110

    lines = []
    lines.append("")
    lines.append("--- 三个矩阵 ---")
    lines.append(sep)
    lines.append(header)
    lines.append(line_sep)
    append_row(lines, "T_joint2_in_joint1", T1)
    append_row(lines, "T_joint3_in_joint2", T2)
    append_row(lines, "T_joint4_in_joint3", T3)

    if result_in_j1:
        lines.append("")
        lines.append("--- 所有关节在 joint_1 下 ---")
        lines.append(sep)
        lines.append(header)
        lines.append(line_sep)
        for jname, T in result_in_j1.items():
            append_row(lines, jname, T)

    msg = "\n".join(lines)
    if use_throttle:
        rospy.loginfo_throttle(throttle_sec, msg)
    else:
        try:
            rospy.loginfo(msg)
        except Exception:
            print(msg)


def _append_error_row(
    lines: List[str],
    label: str,
    pos_mocap,
    yaw_mocap,
    pitch_mocap,
    roll_mocap,
    pos_fk,
    yaw_fk,
    pitch_fk,
    roll_fk,
) -> None:
    """追加误差行"""
    if pos_mocap is not None and yaw_mocap is not None and pos_fk is not None and yaw_fk is not None:
        pos_error = pos_fk - pos_mocap
        pos_error_norm = np.linalg.norm(pos_error)
        lines.append(
            f"{label:<20} | {pos_error[0]:>12.1f} | {pos_error[1]:>12.1f} | {pos_error[2]:>12.1f} | "
            f"{yaw_fk - yaw_mocap:>12.1f} | {pitch_fk - pitch_mocap:>12.1f} | {roll_fk - roll_mocap:>12.1f}"
        )
        lines.append(f"{'位置误差模长':<20} | {pos_error_norm:>12.1f} mm")
    else:
        lines.append(f"{label:<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")


def print_accuracy_table(
    result_shoulder: Optional[dict] = None,
    result_elbow: Optional[dict] = None,
    fk_poses: Optional[Dict[str, Any]] = None,
):
    """
    打印精度对比表格（仅 shoulder 系与 elbow 系）

    参数:
        result_shoulder: 各 joint 在 shoulder 系下的位姿，如 {"elbow": T, "tool": T}
        result_elbow: 各 joint 在 elbow 系下的位姿，如 {"tool": T}
        fk_poses: get_arm_fk_with_refer_frame 的返回，含 elbow_shoulder, end_shoulder, end_elbow(若有)
    """
    try:
        import rospy
        log_func = rospy.loginfo
    except Exception:
        log_func = print

    def append_row(label: str, T):
        if T is not None:
            pos = T[:3, 3]
            R = T[:3, :3]
            yaw, pitch, roll = rotation_matrix_to_euler(R)
            lines.append(
                f"{label:<20} | {pos[0]:>12.1f} | {pos[1]:>12.1f} | {pos[2]:>12.1f} | "
                f"{yaw:>12.1f} | {pitch:>12.1f} | {roll:>12.1f}"
            )
        else:
            lines.append(f"{label:<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")

    lines = []

    # --- Shoulder 坐标系 ---
    if result_shoulder:
        lines.append("")
        lines.append("--- 关节在 shoulder 坐标系下 ---")
        lines.append("=" * 100)
        lines.append(f"{'项目':<20} | {'X (mm)':>12} | {'Y (mm)':>12} | {'Z (mm)':>12} | {'Yaw (deg)':>12} | {'Pitch (deg)':>12} | {'Roll (deg)':>12}")
        lines.append("-" * 100)
        for joint_name, T in result_shoulder.items():
            label = f"{joint_name} (shoulder)"
            append_row(label, T)

        if fk_poses:
            lines.append("-" * 100)
            # FK elbow (shoulder)
            elbow_sh = fk_poses.get("elbow_shoulder")
            T_elbow_fk_sh = pose_to_transform_matrix(*elbow_sh) if elbow_sh else None
            append_row("FK肘关节 (shoulder)", T_elbow_fk_sh)
            T_elbow_mocap_sh = result_shoulder.get("elbow")
            if T_elbow_mocap_sh is not None and elbow_sh is not None:
                pos_elbow_mocap_sh = T_elbow_mocap_sh[:3, 3]
                R_elbow_sh = T_elbow_mocap_sh[:3, :3]
                yaw_es, pitch_es, roll_es = rotation_matrix_to_euler(R_elbow_sh)
                pos_elbow_fk_sh = elbow_sh[0]
                yaw_efs, pitch_efs, roll_efs = quaternion_array_to_euler(elbow_sh[1])
                lines.append("-" * 100)
                _append_error_row(
                    lines, "误差 肘关节 (FK-mocap)",
                    pos_elbow_mocap_sh, yaw_es, pitch_es, roll_es,
                    pos_elbow_fk_sh, yaw_efs, pitch_efs, roll_efs,
                )
            else:
                lines.append("-" * 100)
                lines.append(f"{'误差 肘关节 (FK-mocap)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")

            lines.append("-" * 100)
            # FK end (shoulder)
            end_sh = fk_poses.get("end_shoulder")
            T_end_fk_sh = pose_to_transform_matrix(*end_sh) if end_sh else None
            append_row("FK末端 (shoulder)", T_end_fk_sh)
            T_tool_mocap_sh = result_shoulder.get("tool")
            if T_tool_mocap_sh is not None and end_sh is not None:
                pos_tool_mocap_sh = T_tool_mocap_sh[:3, 3]
                R_tool_sh = T_tool_mocap_sh[:3, :3]
                yaw_ts, pitch_ts, roll_ts = rotation_matrix_to_euler(R_tool_sh)
                pos_end_fk_sh = end_sh[0]
                yaw_tfs, pitch_tfs, roll_tfs = quaternion_array_to_euler(end_sh[1])
                lines.append("-" * 100)
                _append_error_row(
                    lines, "误差 末端 (FK-mocap)",
                    pos_tool_mocap_sh, yaw_ts, pitch_ts, roll_ts,
                    pos_end_fk_sh, yaw_tfs, pitch_tfs, roll_tfs,
                )
            else:
                lines.append("-" * 100)
                lines.append(f"{'误差 末端 (FK-mocap)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")
        else:
            lines.append(f"{'FK肘关节 (shoulder)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")
            lines.append(f"{'FK末端 (shoulder)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")
        lines.append("=" * 100)

    # --- Elbow 坐标系 ---
    if result_elbow:
        lines.append("")
        lines.append("--- 关节在 elbow 坐标系下 ---")
        lines.append("=" * 100)
        lines.append(f"{'项目':<20} | {'X (mm)':>12} | {'Y (mm)':>12} | {'Z (mm)':>12} | {'Yaw (deg)':>12} | {'Pitch (deg)':>12} | {'Roll (deg)':>12}")
        lines.append("-" * 100)
        for joint_name, T in result_elbow.items():
            append_row(f"{joint_name} (elbow)", T)
        if fk_poses:
            end_elbow = fk_poses.get("end_elbow")
            T_end_fk_elbow = pose_to_transform_matrix(*end_elbow) if end_elbow else None
            if T_end_fk_elbow is not None:
                append_row("FK末端 (elbow)", T_end_fk_elbow)
                T_tool_mocap_el = result_elbow.get("tool")
                if T_tool_mocap_el is not None:
                    pos_tool_el = T_tool_mocap_el[:3, 3]
                    R_tool_el = T_tool_mocap_el[:3, :3]
                    yaw_te, pitch_te, roll_te = rotation_matrix_to_euler(R_tool_el)
                    pos_end_fk_el = end_elbow[0]
                    yaw_tfe, pitch_tfe, roll_tfe = quaternion_array_to_euler(end_elbow[1])
                    lines.append("-" * 100)
                    _append_error_row(
                        lines, "误差 末端 (FK-mocap)",
                        pos_tool_el, yaw_te, pitch_te, roll_te,
                        pos_end_fk_el, yaw_tfe, pitch_tfe, roll_tfe,
                    )
            else:
                lines.append(f"{'FK末端 (elbow)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")
        else:
            lines.append(f"{'FK末端 (elbow)':<20} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12} | {'N/A':>12}")
        lines.append("=" * 100)

    log_func("\n".join(lines))
