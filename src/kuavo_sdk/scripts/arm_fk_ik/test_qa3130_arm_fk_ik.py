#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QA#3130 kuavo5 手臂正逆解回归测试。

前置条件（需先手动或由 run_qa3130_arm_fk_ik_test.sh 启动）：
  export ROBOT_VERSION=52
  roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch

用法：
  rosrun kuavo_sdk test_qa3130_arm_fk_ik.py
  rosrun kuavo_sdk test_qa3130_arm_fk_ik.py --joint_angles_id 1
  rosrun kuavo_sdk test_qa3130_arm_fk_ik.py --skip-motion   # 仅验证 FK/IK 服务，不驱动手臂运动
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import rospkg
import rospy

from kuavo_msgs.msg import armTargetPoses, ikSolveParam, twoArmHandPoseCmd
from kuavo_msgs.srv import (
    changeArmCtrlMode,
    changeArmCtrlModeRequest,
    fkSrv,
    twoArmHandPoseCmdSrv,
)

try:
    kuavo_common_path = rospkg.RosPack().get_path("kuavo_common")
    kuavo_common_python_path = os.path.join(kuavo_common_path, "python")
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion
except (rospkg.ResourceNotFound, ImportError):
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    kuavo_common_python_path = os.path.abspath(
        os.path.join(current_file_dir, "../../../kuavo_common/python")
    )
    if kuavo_common_python_path not in sys.path:
        sys.path.insert(0, kuavo_common_python_path)
    from robot_version import RobotVersion


POS_ERR_THRESH_MM = 10.0
STRICT_ROUNDTRIP_CASES = {1}
SERVICE_WAIT_TIMEOUT = 120.0
MOTION_WAIT_SEC = 6.0

KUAVO5_JOINT_ANGLES = {
    1: [
        -1.0, 1.0, -0.3, -1.2, 0.0, -0.5, -0.2,
        -1.9, -0.5, -0.0, -1.0, -0.0, 0.5, 0.65,
    ],
    2: [
        -1.8, 1.0, -1.5, -1.8, 0.0, -0.0, -0.8,
        -1.8, -1.0, 1.5, -1.8, 0.0, -0.0, -0.8,
    ],
    3: [
        -0.12404927166595045, 0.5207316843701423, -0.7438398355748528,
        -1.6878711580314953, 0.5882163232478015, 0.6960587737022195,
        -0.1833288094691406, 0.06531010122606275, 0.2915284371448789,
        -0.27502683875196365, -1.6756463742815109, 0.27429680674163065,
        0.29221918357765564, -0.040906271746990595,
    ],
}


def make_ik_param(constraint_mode=None, pos_cost_weight=0.0):
    p = ikSolveParam()
    p.major_optimality_tol = 1e-3
    p.major_feasibility_tol = 1e-3
    p.minor_feasibility_tol = 1e-3
    p.major_iterations_limit = 100
    p.oritation_constraint_tol = 1e-3
    p.pos_constraint_tol = 1e-3
    p.pos_cost_weight = pos_cost_weight
    if constraint_mode is not None:
        p.constraint_mode = int(constraint_mode)
    return p


def fk_ik_roundtrip_error_mm(joint_angles, use_custom_ik_param, ik_param=None):
    fk_hand_poses = fk_srv_client(joint_angles)
    eef_pose_msg = twoArmHandPoseCmd()
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    if use_custom_ik_param:
        eef_pose_msg.ik_param = ik_param
    eef_pose_msg.joint_angles_as_q0 = True
    eef_pose_msg.hand_poses.left_pose.joint_angles = joint_angles[:7]
    eef_pose_msg.hand_poses.right_pose.joint_angles = joint_angles[7:]
    eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array(fk_hand_poses.left_pose.pos_xyz)
    eef_pose_msg.hand_poses.left_pose.quat_xyzw = fk_hand_poses.left_pose.quat_xyzw
    eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
    eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array(fk_hand_poses.right_pose.pos_xyz)
    eef_pose_msg.hand_poses.right_pose.quat_xyzw = fk_hand_poses.right_pose.quat_xyzw
    eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    res = call_ik_srv(eef_pose_msg)
    if not res.success:
        return False, None, None, res.time_cost
    fk_after_ik = fk_srv_client(list(res.q_arm))
    left_err = 1e3 * np.linalg.norm(
        np.array(fk_after_ik.left_pose.pos_xyz) - np.array(fk_hand_poses.left_pose.pos_xyz)
    )
    right_err = 1e3 * np.linalg.norm(
        np.array(fk_after_ik.right_pose.pos_xyz) - np.array(fk_hand_poses.right_pose.pos_xyz)
    )
    return True, left_err, right_err, res.time_cost


def wait_services():
    rospy.wait_for_service("/ik/fk_srv", timeout=SERVICE_WAIT_TIMEOUT)
    rospy.wait_for_service("/ik/two_arm_hand_pose_cmd_srv", timeout=SERVICE_WAIT_TIMEOUT)
    rospy.wait_for_service("/arm_traj_change_mode", timeout=SERVICE_WAIT_TIMEOUT)


def set_arm_control_mode(mode):
    client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)
    req = changeArmCtrlModeRequest()
    req.control_mode = mode
    resp = client(req)
    if not resp.result:
        raise RuntimeError(f"切换手臂控制模式失败 mode={mode}: {resp.message}")


def fk_srv_client(joint_angles):
    fk_srv = rospy.ServiceProxy("/ik/fk_srv", fkSrv)
    fk_result = fk_srv(joint_angles)
    if not fk_result.success:
        raise RuntimeError("FK 服务返回 success=False")
    return fk_result.hand_poses


def call_ik_srv(eef_pose_msg):
    ik_srv = rospy.ServiceProxy("/ik/two_arm_hand_pose_cmd_srv", twoArmHandPoseCmdSrv)
    return ik_srv(eef_pose_msg)


def publish_arm_target_poses(times, values):
    pub = rospy.Publisher("kuavo_arm_target_poses", armTargetPoses, queue_size=10)
    rospy.sleep(0.5)
    msg = armTargetPoses()
    msg.times = times
    msg.values = values
    rate = rospy.Rate(10)
    deadline = rospy.Time.now() + rospy.Duration(5.0)
    while pub.get_num_connections() == 0 and not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rate.sleep()
    pub.publish(msg)


def verify_robot_version(expected_major=5):
    robot_version_int = rospy.get_param("robot_version")
    robot_version = RobotVersion.create(robot_version_int)
    major = robot_version.major()
    if major != expected_major:
        raise RuntimeError(
            f"期望 {expected_major} 代机器人 (ROBOT_VERSION 首位为 {expected_major})，"
            f"当前 robot_version={robot_version_int} ({robot_version.version_name()})"
        )
    rospy.loginfo(
        "机器人版本: %s (major=%d, minor=%d)",
        robot_version.version_name(),
        major,
        robot_version.minor(),
    )
    return robot_version


def run_case(joint_angles_id, skip_motion, strict_roundtrip=False):
    joint_angles = KUAVO5_JOINT_ANGLES[joint_angles_id]
    rospy.loginfo("=== Case joint_angles_id=%d ===", joint_angles_id)

    # 1) 与 QA 脚本 robot_arm_fk_ik.py 相同参数：必须 IK success
    script_param = make_ik_param()
    ok, left_err, right_err, time_cost = fk_ik_roundtrip_error_mm(
        joint_angles, use_custom_ik_param=True, ik_param=script_param
    )
    if not ok:
        raise RuntimeError(
            f"IK 求解失败 (joint_angles_id={joint_angles_id})，"
            "与 QA 现象一致：正逆解不可用"
        )
    rospy.loginfo(
        "QA 脚本参数 IK success, time_cost=%.2f ms, roundtrip left=%.2f mm, right=%.2f mm",
        time_cost,
        left_err,
        right_err,
    )

    if strict_roundtrip and joint_angles_id in STRICT_ROUNDTRIP_CASES:
        if left_err > POS_ERR_THRESH_MM or right_err > POS_ERR_THRESH_MM:
            raise RuntimeError(
                f"QA 脚本参数 FK->IK->FK 闭环误差超限 (阈值 {POS_ERR_THRESH_MM} mm): "
                f"left={left_err:.2f} mm, right={right_err:.2f} mm"
            )

    # 2) 节点默认 IK 参数：验证 IK 可用
    ok2, left_err2, right_err2, time_cost2 = fk_ik_roundtrip_error_mm(
        joint_angles, use_custom_ik_param=False
    )
    if not ok2:
        raise RuntimeError(f"节点默认参数 IK 求解失败 (joint_angles_id={joint_angles_id})")
    rospy.loginfo(
        "节点默认参数 IK success, time_cost=%.2f ms, roundtrip left=%.2f mm, right=%.2f mm",
        time_cost2,
        left_err2,
        right_err2,
    )
    if strict_roundtrip and joint_angles_id in STRICT_ROUNDTRIP_CASES:
        if left_err2 > POS_ERR_THRESH_MM or right_err2 > POS_ERR_THRESH_MM:
            raise RuntimeError(
                f"节点默认参数 FK->IK->FK 闭环误差超限 (阈值 {POS_ERR_THRESH_MM} mm): "
                f"left={left_err2:.2f} mm, right={right_err2:.2f} mm"
            )

    if not skip_motion:
        fk_hand_poses = fk_srv_client(joint_angles)
        degrees_list = [math.degrees(rad) for rad in joint_angles]
        publish_arm_target_poses([3], degrees_list)
        rospy.loginfo("已下发 FK 关节角，等待到位...")
        time.sleep(MOTION_WAIT_SEC)

        publish_arm_target_poses([3], [0.0] * 14)
        rospy.loginfo("已回零，准备 IK 驱动...")
        time.sleep(MOTION_WAIT_SEC)

        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.ik_param = script_param
        eef_pose_msg.use_custom_ik_param = True
        eef_pose_msg.joint_angles_as_q0 = False
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array(fk_hand_poses.left_pose.pos_xyz)
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = fk_hand_poses.left_pose.quat_xyzw
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array(fk_hand_poses.right_pose.pos_xyz)
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = fk_hand_poses.right_pose.quat_xyzw
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
        res = call_ik_srv(eef_pose_msg)
        if not res.success:
            raise RuntimeError("运动验证阶段 IK 失败")
        publish_arm_target_poses([3], [math.degrees(v) for v in res.q_arm])
        rospy.loginfo("已下发 IK 关节角，等待到位...")
        time.sleep(MOTION_WAIT_SEC)

    return True


def main():
    parser = argparse.ArgumentParser(description="QA#3130 kuavo5 手臂正逆解回归测试")
    parser.add_argument(
        "--joint_angles_id",
        type=int,
        choices=[1, 2, 3],
        nargs="+",
        default=[1, 2, 3],
        help="测试用关节角编号，默认全部运行",
    )
    parser.add_argument(
        "--strict-roundtrip",
        action="store_true",
        help="对 case 1 额外检查 FK->IK->FK 闭环精度 (<10mm)",
    )
    parser.add_argument(
        "--skip-motion",
        action="store_true",
        help="跳过手臂运动，仅验证 FK/IK 服务",
    )
    args = parser.parse_args()
    strict_roundtrip = args.strict_roundtrip

    rospy.init_node("test_qa3130_arm_fk_ik", anonymous=True)
    wait_services()
    verify_robot_version(expected_major=5)

    if not args.skip_motion:
        set_arm_control_mode(2)
    else:
        rospy.loginfo("skip-motion 模式：跳过手臂控制模式切换，仅验证 FK/IK 服务")

    failed = []
    for case_id in args.joint_angles_id:
        try:
            run_case(case_id, args.skip_motion, args.strict_roundtrip)
            rospy.loginfo("[PASS] joint_angles_id=%d", case_id)
        except Exception as exc:
            rospy.logerr("[FAIL] joint_angles_id=%d: %s", case_id, exc)
            failed.append(case_id)

    if not args.skip_motion:
        try:
            set_arm_control_mode(1)
        except Exception as exc:
            rospy.logwarn("恢复手臂控制模式失败: %s", exc)

    if failed:
        rospy.logerr("测试失败，失败 case: %s", failed)
        sys.exit(1)

    rospy.loginfo("全部 case 通过: %s", args.joint_angles_id)
    sys.exit(0)


if __name__ == "__main__":
    main()
