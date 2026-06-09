#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自由空间逆运动学示例 (P1)

演示如何使用 arm_ik_free 进行自由空间逆运动学求解，
与 arm_ik 不同，arm_ik_free 可指定肘部参考位置和优化参数。
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from kuavo_humanoid_sdk import KuavoPose, KuavoIKParams


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    # 自由空间 IK 需要 WithIK 选项初始化
    if not KuavoSDK.Init(options=KuavoSDK.Options.WithIK,
                         log_level='INFO',
                         websocket_mode=True,
                         websocket_host=args.host,
                         websocket_port=args.port):
        print("Failed to initialize Kuavo SDK (WithIK required for arm_ik_free)!")
        exit(1)

    robot = KuavoRobot()
    robot.arm_reset()
    robot_state = KuavoRobotState()

    # 目标末端位姿：手在身体前方
    left_pos = [0.45, 0.28, 0.25]    # 左手目标位置
    right_pos = [0.45, -0.20, 0.25]  # 右手目标位置
    left_orientation = [0.38, -0.45, -0.56, 0.57]
    right_orientation = [-0.41158, -0.503073, 0.577546, 0.493919]

    # ======== 基本用法：不指定肘部和优化参数 ========
    print("=== 基本 arm_ik_free（默认参数）===")
    res = robot.arm_ik_free(
        left_pose=KuavoPose(position=left_pos, orientation=left_orientation),
        right_pose=KuavoPose(position=right_pos, orientation=right_orientation),
    )
    if res:
        print(f"求解成功，关节角度: {[f'{q:.3f}' for q in res[:6]]}... (共 {len(res)} 维)")
        # 控制手臂到求解结果
        arm_q0 = robot_state.arm_joint_state().position
        times = [1.0, 3.0]
        robot.control_arm_joint_trajectory(times, [arm_q0, res])
        time.sleep(4.0)

    # ======== 指定肘部参考位置 ========
    print("\n=== arm_ik_free（指定肘部位置）===")
    # 指定左右肘部期望位置，帮助 IK 收敛到期望姿态
    left_elbow = [0.25, 0.30, 0.15]
    right_elbow = [0.25, -0.30, 0.15]
    res2 = robot.arm_ik_free(
        left_pose=KuavoPose(position=left_pos, orientation=left_orientation),
        right_pose=KuavoPose(position=right_pos, orientation=right_orientation),
        left_elbow_pos_xyz=left_elbow,
        right_elbow_pos_xyz=right_elbow,
    )
    if res2:
        print(f"求解成功，关节角度: {[f'{q:.3f}' for q in res2[:6]]}...")
        arm_q0 = robot_state.arm_joint_state().position
        times = [1.0, 3.0]
        robot.control_arm_joint_trajectory(times, [arm_q0, res2])
        time.sleep(4.0)

    # ======== 使用自定义 IK 参数 ========
    print("\n=== arm_ik_free（自定义优化参数）===")
    ik_params = KuavoIKParams(
        major_optimality_tol=1e-4,   # 更严的最优性容差
        major_feasibility_tol=1e-4,  # 更严的可行性容差
        major_iterations_limit=200,  # 更多迭代次数
        pos_cost_weight=0.0,         # 位置代价权重=0 追求高精度
    )
    res3 = robot.arm_ik_free(
        left_pose=KuavoPose(position=left_pos, orientation=left_orientation),
        right_pose=KuavoPose(position=right_pos, orientation=right_orientation),
        params=ik_params,
    )
    if res3:
        print(f"求解成功，关节角度: {[f'{q:.3f}' for q in res3[:6]]}...")
        arm_q0 = robot_state.arm_joint_state().position
        times = [1.0, 3.0]
        robot.control_arm_joint_trajectory(times, [arm_q0, res3])
        time.sleep(4.0)

    # 复位
    robot.arm_reset()
    print("\n自由空间逆运动学演示完成。")


if __name__ == "__main__":
    main()
