#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末端力控制示例 (P1)

演示如何使用 control_hand_wrench 对机器人末端执行器进行力/力矩控制，
通过 6 维力控指令 [Fx, Fy, Fz, Tx, Ty, Tz] 控制左右手末端的力和力矩。
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

    # 力控指令格式: [Fx, Fy, Fz, Tx, Ty, Tz]
    #   Fx, Fy, Fz: 力分量，单位 N（牛顿）
    #   Tx, Ty, Tz: 力矩分量，单位 N·m（牛顿·米）

    # 零力指令（双手松劲）
    zero_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print("清除末端力控指令（零力）...")
    robot.control_hand_wrench(zero_wrench, zero_wrench)
    time.sleep(0.5)

    # 双手施加正向 Z 轴力（抬升力）
    print("双手施加 Z 轴正方向力（上抬）...")
    left_wrench = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]   # 左手: Fz=5N
    right_wrench = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]  # 右手: Fz=5N
    robot.control_hand_wrench(left_wrench, right_wrench)
    time.sleep(2.0)

    # 双手施加 X 轴正向力（前推）
    print("双手施加 X 轴正方向力（前推）...")
    left_wrench = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 左手: Fx=5N
    right_wrench = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 右手: Fx=5N
    robot.control_hand_wrench(left_wrench, right_wrench)
    time.sleep(2.0)

    # 双手 Z 轴力矩（拧紧/拧松）
    print("双手施加 Z 轴力矩...")
    left_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]   # 左手: Tz=1Nm
    right_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]  # 右手: Tz=-1Nm
    robot.control_hand_wrench(left_wrench, right_wrench)
    time.sleep(2.0)

    # 清零力控
    print("清除力控指令，恢复零力状态...")
    robot.control_hand_wrench(zero_wrench, zero_wrench)

    print("\n末端力控制演示完成。")


if __name__ == "__main__":
    main()
