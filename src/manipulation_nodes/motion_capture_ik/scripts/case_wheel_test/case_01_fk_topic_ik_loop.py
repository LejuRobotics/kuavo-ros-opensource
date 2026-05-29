#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Case01：FK 闭环 + Topic IK。FK/IK 目标位姿约定为 waist_yaw_link 系。"""

import time
import numpy as np

from arms_ik_api import ArmsIKApi


def main():
    api = ArmsIKApi("case_01_fk_topic_ik_loop")

    # 14维双臂关节角（rad）
    q_arm = np.array([-1.38, 0, -0.29, -0.43, 0.0, -0.17, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    # 计算正解得到的末端位姿（参考系: waist_yaw_link）
    fk_res = api.call_fk(q_arm.tolist())
    print("[Case1] FK success:", fk_res.success)
    if not fk_res.success:
        return

    # FK 正解位姿作为 IK 目标（参考系: waist_yaw_link）
    target_pose = fk_res.hand_poses
    print("[Case1] FK left_pos:", target_pose.left_pose.pos_xyz)
    print("[Case1] FK right_pos:", target_pose.right_pose.pos_xyz)

    # 执行关节跟踪
    api.publish_arm_traj(q_arm.tolist())
    time.sleep(2.5)

    # 回到零位
    q_zero = np.zeros(14, dtype=float)
    api.publish_arm_traj(q_zero.tolist())
    time.sleep(2.5)

    # 执行逆解
    # 可输入left_joint_angles、right_joint_angles作为 IK 初值（joint_angles_as_q0），不输入则默认获取当前关节角度作为初值
    cmd = api.build_two_arm_cmd(
        left_pos_xyz=target_pose.left_pose.pos_xyz,
        left_quat_xyzw=target_pose.left_pose.quat_xyzw,
        right_pos_xyz=target_pose.right_pose.pos_xyz,
        right_quat_xyzw=target_pose.right_pose.quat_xyzw,
        ik_param=api.default_ik_param(constraint_mode=3),
        joint_angles_as_q0=True,
        # left_joint_angles=q_arm[0:7],
        # right_joint_angles=q_arm[7:14],
    )
    api.reset_ik_result_topic_buffer()
    api.publish_ik_topic(cmd)

    ik_msg = api.wait_ik_result_topic(timeout_sec=1.0)
    if ik_msg is None:
        print("[Case1] 未在超时内收到 /ik/result")
        return

    err = api.calc_pose_error(target_pose, ik_msg)
    api.print_ik_summary("Case1_topic_ik", True, None, err)

    q_from_ik = list(ik_msg.left_pose.joint_angles) + list(ik_msg.right_pose.joint_angles)
    api.publish_arm_traj(q_from_ik)
    time.sleep(0.2)


if __name__ == "__main__":
    main()
