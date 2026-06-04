#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂躯干位姿控制示例 (P2)

演示轮臂机器人躯干的六自由度位姿控制，
对齐 ROS SDK demo: src/demo/wheel_arm_demo/cmd_torso_pose_test.py

该 demo 以 initialTorsoPose_ = [0.196123, 0.0005, 0.789919] 为基准，
所有指令叠加在初始位姿之上（绝对坐标）。每次指令后等待
/lb_torso_pose_reach_time 反馈再发下一条。

控制序列（与 demo 一致）:
  1. 抬高: z=0.4
  2. 前移: x=0.2, z=0.4
  3. 左转: x=0.2, z=0.4, yaw=1.047 (≈60°)
  4. 右转: x=0.2, z=0.4, yaw=-1.047
  5. 前倾: x=0.2, z=0.4, pitch=-0.524 (≈-30°)
  6. 后仰: x=0.2, z=0.4, pitch=0.524
  7. 归位: 全部归零

参数说明: x, y, z (米), roll, pitch, yaw (弧度)
  angular.x→roll, angular.y→pitch, angular.z→yaw
  注意：轮臂躯干不支持 roll 旋转 (angular.x 保持 0)
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()

    # ======== 躯干位姿控制（6自由度） ========
    # WS SDK 通过 control_torso_pose() 封装了话题发布，
    # 但不暴露 reach_time 反馈，使用固定 sleep 作为近似等待
    print("=== 躯干位姿控制 ===")
    print("（参考 ROS demo cmd_torso_pose_test.py，绝对坐标模式）")

    # 步骤1: 抬高躯干（z 叠加 0.4m）
    print("步骤1: z=0.4, 其他归零 ...")
    robot.control_torso_pose(x=0.0, y=0.0, z=0.4, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(4.0)

    # 步骤2: 保持 z=0.4 的同时前移 x=0.2m
    print("步骤2: x=0.2, z=0.4, 其他归零 ...")
    robot.control_torso_pose(x=0.2, y=0.0, z=0.4, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(4.0)

    # 步骤3: 保持所有位置，左转 yaw=1.047 rad (约60°)
    print("步骤3: x=0.2, z=0.4, yaw=1.047 ...")
    robot.control_torso_pose(x=0.2, y=0.0, z=0.4, roll=0.0, pitch=0.0, yaw=1.047)
    time.sleep(4.0)

    # 步骤4: 保持位置，右转 yaw=-1.047 rad
    print("步骤4: x=0.2, z=0.4, yaw=-1.047 ...")
    robot.control_torso_pose(x=0.2, y=0.0, z=0.4, roll=0.0, pitch=0.0, yaw=-1.047)
    time.sleep(4.0)

    # 步骤5: 保持位置，前倾 pitch=-0.524 rad (约-30°)
    print("步骤5: x=0.2, z=0.4, pitch=-0.524 ...")
    robot.control_torso_pose(x=0.2, y=0.0, z=0.4, roll=0.0, pitch=-0.524, yaw=0.0)
    time.sleep(4.0)

    # 步骤6: 保持位置，后仰 pitch=0.524 rad
    print("步骤6: x=0.2, z=0.4, pitch=0.524 ...")
    robot.control_torso_pose(x=0.2, y=0.0, z=0.4, roll=0.0, pitch=0.524, yaw=0.0)
    time.sleep(4.0)

    # 步骤7: 恢复零位（等价于 demo 最后一步回到 initialTorsoPose_）
    print("步骤7: 恢复零位 (x=0, y=0, z=0, roll=0, pitch=0, yaw=0) ...")
    robot.control_torso_pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(4.0)

    print("\n躯干位姿控制演示完成。")


if __name__ == "__main__":
    main()
