#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
腰部列表式控制示例 (P0)
演示如何使用 control_waist_pos 通过列表接口控制机器人腰部旋转，
接口与 ROS SDK 的 list 格式兼容。
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

    # 单角度控制：列表第一个元素作为 yaw 角度（单位：度）
    print("控制腰部转到 0 度（正前）...")
    robot.control_waist_pos([0.0])
    time.sleep(1.0)

    print("控制腰部转到 30 度...")
    robot.control_waist_pos([30.0])
    time.sleep(1.5)

    print("控制腰部转到 -30 度...")
    robot.control_waist_pos([-30.0])
    time.sleep(1.5)

    # 循环摆动
    print("开始腰部循环摆动...")
    cycles = 2
    step = 5
    interval = 0.2

    for cycle in range(cycles):
        # 从左到右
        for yaw in range(-30, 31, step):
            robot.control_waist_pos([float(yaw)])
            time.sleep(interval)
        # 从右到左
        for yaw in range(30, -31, -step):
            robot.control_waist_pos([float(yaw)])
            time.sleep(interval)

    # 回到正前方
    robot.control_waist_pos([0.0])
    print("腰部控制演示完成。")


if __name__ == "__main__":
    main()
