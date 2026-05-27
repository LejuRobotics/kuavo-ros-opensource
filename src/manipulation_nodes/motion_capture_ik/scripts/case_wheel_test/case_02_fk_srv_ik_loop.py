#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np

from arms_ik_api import ArmsIKApi


def main():
    api = ArmsIKApi("case_02_fk_srv_ik_loop")

    q_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     -1.38, 0, -0.29, -0.43, 0.0, -0.17, 0.0], dtype=float)
    fk_res = api.call_fk(q_arm.tolist())
    print("[Case2] FK success:", fk_res.success)
    if not fk_res.success:
        return

    # 计算正解得到的末端位姿
    target_pose = fk_res.hand_poses

    # 执行关节跟踪
    api.publish_arm_traj(q_arm.tolist())
    time.sleep(2.5)

    # 关节回零
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
    ik_res = api.call_ik_srv(cmd)
    err = api.calc_pose_error(target_pose, ik_res.hand_poses)
    api.print_ik_summary("Case2_srv_ik", ik_res.success, ik_res.time_cost, err)

    if ik_res.success and len(ik_res.q_arm) >= 14:
        api.publish_arm_traj(ik_res.q_arm[:14])
        time.sleep(0.2)


if __name__ == "__main__":
    main()
