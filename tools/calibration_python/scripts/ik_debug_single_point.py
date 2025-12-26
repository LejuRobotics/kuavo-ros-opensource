#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IK 调试脚本：
- 直接使用指定的右臂末端位姿（位置和欧拉角）
- 将欧拉角转换为四元数
- 调用 IK 求解右臂关节角
- 打印 IK 求解得到的关节角，并验证 FK/IK 一致性
"""

import os
import sys
import numpy as np
import rospy
import rospkg
from scipy.spatial.transform import Rotation

from kuavo_msgs.srv import fkSrv
from function.data_utils import call_ik_service
from function.file_utils import get_config_dir
from function.marker_utils import publish_marker, init_marker_publishers

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


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    将欧拉角（度）转换为四元数 (xyzw)
    使用 ZYX 顺序（roll, pitch, yaw）
    
    Args:
        roll_deg: 滚转角（度）
        pitch_deg: 俯仰角（度）
        yaw_deg: 偏航角（度）
    
    Returns:
        numpy.ndarray: 四元数 [x, y, z, w]
    """
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    
    # 计算半角
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    # ZYX 顺序（roll, pitch, yaw）
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([x, y, z, w])


def call_fk_service_both_arms(joints_deg_14, timeout=2.0):
    """调用 /ik/fk_srv，返回左右臂末端 pos, quat（单位：米、quat_xyzw）"""
    service_name = "/ik/fk_srv"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        fk_client = rospy.ServiceProxy(service_name, fkSrv)

        joints_rad = np.radians(joints_deg_14).tolist()
        res = fk_client(joints_rad)
        if not res.success:
            rospy.logwarn("FK service returned success = False")
            return None, None, None, None

        pos_right = np.array(res.hand_poses.right_pose.pos_xyz)
        quat_right = np.array(res.hand_poses.right_pose.quat_xyzw)
        pos_left = np.array(res.hand_poses.left_pose.pos_xyz)
        quat_left = np.array(res.hand_poses.left_pose.quat_xyzw)
        return pos_right, quat_right, pos_left, quat_left
    except Exception as e:
        rospy.logwarn(f"FK 服务调用失败: {e}")
        return None, None, None, None


def main():
    rospy.init_node("ik_debug_single_point")

    # 初始化可视化marker发布器
    COLOR_PURPLE = "\033[95m"
    COLOR_RESET = "\033[0m"
    # 初始化 Quest3 可视化用的 transformer 和 marker 发布器
    (quest3_arm_info_transformer,
     marker_pub_target_left, marker_pub_target_right,
     marker_pub_result_param1_left, marker_pub_result_param1_right,
     marker_pub_result_param2_left, marker_pub_result_param2_right) = init_marker_publishers(Quest3ArmInfoTransformer)

    # 本脚本目前只用到 “param1” 这一组结果 marker，把它们起一个简单的别名
    marker_pub_result_left = marker_pub_result_param1_left
    marker_pub_result_right = marker_pub_result_param1_right
    
    if quest3_arm_info_transformer is not None:
        rospy.loginfo(f"{COLOR_PURPLE}Marker发布器初始化完成，将在rviz中显示期望位姿（红色）和IK解算位姿（蓝色）{COLOR_RESET}")
        rospy.sleep(0.5)  # 等待发布器注册

    # 直接使用终端中指定的右臂姿态
    # 位置: [0.3590, -0.1400, 0.5500] (单位：米)
    # 注意：这个位置对应的是 zarm_r7_end_effector 的位置
    pos_right_end_effector = np.array([0.3590, -0.1400, 0.5500])
    
    # 姿态(欧拉角): roll=179.97°, pitch=-89.99°, yaw=-179.97°
    roll_deg = 179.97
    pitch_deg = -89.99
    yaw_deg = -179.97
    
    # 将欧拉角转换为四元数
    quat_right = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
    
    # ===== 关键转换：从 zarm_r7_end_effector 转换到 zarm_r7_link =====
    # 从URDF看，zarm_r7_end_effector 相对于 zarm_r7_link 的偏移是 [0, 0.03, -0.17] (米)
    # 这个偏移是在 zarm_r7_link 的局部坐标系下的
    # 需要将这个偏移从 zarm_r7_link 的局部坐标系转换到 base_link 坐标系
    offset_in_link_frame = np.array([0.0, 0.03, -0.17])  # URDF中的偏移
    
    # 将四元数转换为旋转矩阵
    rot = Rotation.from_quat([quat_right[0], quat_right[1], quat_right[2], quat_right[3]])
    rot_matrix = rot.as_matrix()
    
    # 将偏移从 zarm_r7_link 的局部坐标系转换到 base_link 坐标系
    offset_in_base_frame = rot_matrix @ offset_in_link_frame
    
    # 计算 zarm_r7_link 的位置（IK服务期望的目标frame）
    pos_right_link = pos_right_end_effector - offset_in_base_frame
    
    print(f"\n{COLOR_PURPLE}===== Frame转换验证 ====={COLOR_RESET}")
    print(f"期望位置 (zarm_r7_end_effector, mm): {np.array2string(pos_right_end_effector * 1000.0, precision=1, separator=', ')}")
    print(f"偏移量 (zarm_r7_link局部坐标系, mm): {np.array2string(offset_in_link_frame * 1000.0, precision=1, separator=', ')}")
    print(f"偏移量 (base_link坐标系, mm): {np.array2string(offset_in_base_frame * 1000.0, precision=1, separator=', ')}")
    print(f"转换后位置 (zarm_r7_link, mm): {np.array2string(pos_right_link * 1000.0, precision=1, separator=', ')}")
    print(f"{COLOR_PURPLE}注意：IK服务期望的目标frame是 zarm_r7_link，不是 zarm_r7_end_effector{COLOR_RESET}\n")
    
    # 发布期望位姿的marker（红色）- 使用 end_effector 的位置
    if quest3_arm_info_transformer is not None and marker_pub_target_right is not None:
        publish_marker(quest3_arm_info_transformer, marker_pub_target_right,
                     pos_right_end_effector, quat_right, "right", 0, is_target=True)
        rospy.loginfo(f"{COLOR_PURPLE}已发布期望位姿marker（红色，zarm_r7_end_effector）到rviz{COLOR_RESET}")

    print("==== 使用指定的右臂末端位姿进行 IK 求解 ====")
    print(f"右臂末端位置 (zarm_r7_end_effector, m): {np.array2string(pos_right_end_effector, precision=4, separator=', ')}")
    print(f"右臂末端位置 (zarm_r7_end_effector, mm): {np.array2string(pos_right_end_effector * 1000.0, precision=1, separator=', ')}")
    print(f"IK目标位置 (zarm_r7_link, m): {np.array2string(pos_right_link, precision=4, separator=', ')}")
    print(f"IK目标位置 (zarm_r7_link, mm): {np.array2string(pos_right_link * 1000.0, precision=1, separator=', ')}")
    print(f"右臂末端姿态(欧拉角): roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°")
    print("右臂末端姿态 quat_xyzw:", np.array2string(quat_right, precision=6, separator=", "))

    # 参考关节 q0：使用默认值（可以后续根据需要调整）
    # 这里使用零关节角作为初始猜测
    q0_right_deg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q0_right_rad = np.radians(q0_right_deg)

    # 使用与离线脚本 param2 一致的一套 IK 参数（高精度、pos_cost_weight=0）
    IK_SOLVE_PARAM_DEBUG = {
        "major_optimality_tol": 1e-3,
        "major_feasibility_tol": 1e-3,
        "minor_feasibility_tol": 3e-3,
        "major_iterations_limit": 100,
        "oritation_constraint_tol": 10e-3,
        "pos_constraint_tol": 1e-3,
        "pos_cost_weight": 10,
    }

    # 调用 IK：右臂，只求解右手，不传入左手臂位姿
    # 注意：IK服务期望的目标frame是 zarm_r7_link，所以使用转换后的位置
    print(f"\n{COLOR_PURPLE}===== IK求解输入 ====={COLOR_RESET}")
    print(f"IK目标位置 (zarm_r7_link, mm): {np.array2string(pos_right_link * 1000.0, precision=1, separator=', ')}")
    print(f"IK目标姿态 quat_xyzw: {np.array2string(quat_right, precision=6, separator=', ')}")
    
    ik_result = call_ik_service(
        pos_right_link,  # 使用转换后的 zarm_r7_link 位置
        quat_right,
        arm_side="right",
        ik_param_dict=IK_SOLVE_PARAM_DEBUG,
        other_arm_pos=None,
        other_arm_quat=None,
        q0_joints=q0_right_rad,
    )

    print("\n==== IK 结果 ====")
    if ik_result is None or ik_result[0] is None:
        print("IK 求解失败")
        if len(ik_result) > 2 and ik_result[2]:
            print(f"错误原因: {ik_result[2]}")
        return

    ik_joints_rad, time_cost_ms, error_reason = ik_result
    ik_joints_deg = np.degrees(ik_joints_rad)
    print("IK 解 (rad, 7):")
    print(np.array2string(ik_joints_rad, precision=6, separator=", "))
    print("IK 解 (deg, 7):")
    print(np.array2string(ik_joints_deg, precision=4, separator=", "))
    print(f"IK 求解时间: {time_cost_ms:.3f} ms")

    # 用 IK 解替换右臂 7 个关节，再做一次 FK，对比期望姿态和实际姿态
    # 构造一个14关节的数组（左臂7个关节使用默认值，右臂使用IK解）
    joints_deg_14_ik = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + ik_joints_deg.tolist())
    pos_fk_ik, quat_fk_ik, _, _ = call_fk_service_both_arms(joints_deg_14_ik)

    print(f"\n{COLOR_PURPLE}==== FK( IK 解 ) 与期望姿态对比 ===={COLOR_RESET}")
    print(f"期望位置 (zarm_r7_end_effector, mm): {np.array2string(pos_right_end_effector * 1000.0, precision=1, separator=', ')}")
    print(f"期望位置 (zarm_r7_link, mm): {np.array2string(pos_right_link * 1000.0, precision=1, separator=', ')}")
    if pos_fk_ik is not None:
        print(f"FK返回位置 (zarm_r7_link, mm): {np.array2string(pos_fk_ik * 1000.0, precision=1, separator=', ')}")
        print(f"{COLOR_PURPLE}注意：FK服务返回的是 zarm_r7_link 的位置，不是 zarm_r7_end_effector{COLOR_RESET}")

        # 位置误差（mm）- 对比 zarm_r7_link 的位置
        pos_err_mm_link = (pos_fk_ik - pos_right_link) * 1000.0
        pos_err_norm_mm_link = np.linalg.norm(pos_err_mm_link)
        print(f"\n位置误差 (zarm_r7_link vs zarm_r7_link, mm): {np.array2string(pos_err_mm_link, precision=1, separator=', ')}")
        print(f"位置误差模 (mm): {pos_err_norm_mm_link:.2f}")
        
        # 如果要将FK返回的 zarm_r7_link 位置转换回 zarm_r7_end_effector 位置进行对比
        # 需要根据当前姿态计算偏移
        rot_fk = Rotation.from_quat([quat_fk_ik[0], quat_fk_ik[1], quat_fk_ik[2], quat_fk_ik[3]])
        rot_matrix_fk = rot_fk.as_matrix()
        offset_in_base_frame_fk = rot_matrix_fk @ offset_in_link_frame
        pos_fk_end_effector = pos_fk_ik + offset_in_base_frame_fk
        
        print(f"\nFK返回位置 (转换到zarm_r7_end_effector, mm): {np.array2string(pos_fk_end_effector * 1000.0, precision=1, separator=', ')}")
        pos_err_mm_eef = (pos_fk_end_effector - pos_right_end_effector) * 1000.0
        pos_err_norm_mm_eef = np.linalg.norm(pos_err_mm_eef)
        print(f"位置误差 (zarm_r7_end_effector vs zarm_r7_end_effector, mm): {np.array2string(pos_err_mm_eef, precision=1, separator=', ')}")
        print(f"位置误差模 (mm): {pos_err_norm_mm_eef:.2f}")
        
        # 发布IK解算位姿的marker（蓝色）
        if quest3_arm_info_transformer is not None and marker_pub_result_right is not None and quat_fk_ik is not None:
            publish_marker(quest3_arm_info_transformer, marker_pub_result_right,
                         pos_fk_ik, quat_fk_ik, "right", 0, is_target=False)
            rospy.loginfo(f"{COLOR_PURPLE}已发布IK解算位姿marker（蓝色）到rviz{COLOR_RESET}")
    else:
        print("FK( IK 解 ) 右臂末端位置 (mm): FK 失败")
    print("期望右臂末端姿态 quat_xyzw:", np.array2string(quat_right, precision=6, separator=", "))
    if quat_fk_ik is not None:
        print("FK( IK 解 ) 右臂末端姿态 quat_xyzw:", np.array2string(quat_fk_ik, precision=6, separator=", "))

        # 姿态误差（角度）
        # 通过四元数夹角计算旋转误差：angle = 2 * acos(|dot(q1, q2)|)
        dot_q = float(np.clip(np.dot(quat_right, quat_fk_ik), -1.0, 1.0))
        angle_rad = 2.0 * np.arccos(abs(dot_q))
        angle_deg = np.degrees(angle_rad)
        print(f"姿态误差 (总角度, 度): {angle_deg:.3f}")
    else:
        print("FK( IK 解 ) 右臂末端姿态 quat_xyzw: FK 失败")
    
    # 保持节点运行，以便在rviz中查看marker
    if quest3_arm_info_transformer is not None:
        rospy.loginfo(f"{COLOR_PURPLE}节点保持运行，可在rviz中查看marker。按Ctrl+C退出。{COLOR_RESET}")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("节点退出")


if __name__ == "__main__":
    main()


