#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂躯干位姿控制示例

演示轮臂机器人躯干的六自由度位姿控制。
roll, pitch, yaw 为绝对欧拉角（弧度）。

控制序列: 复位 → 抬高 → 前移 → 左转 → 右转 → 回正 → 前倾 → 归位

注意：轮臂躯干不支持 roll 旋转 (angular.x 保持 0)
"""

import math
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

_INIT = [0.196, 0.001, 0.790]  # [x, y, z] 初始躯干位姿


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9090)
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True,
                           websocket_host=args.host, websocket_port=args.port):
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()

    print("=== 躯干位姿控制 ===")
    print(f"初始位姿: {_INIT}")

    # 每个步骤的偏移量: (dx, dy, dz, yaw_deg, pitch_deg, 说明)
    # roll 固定为 0（轮臂不支持 roll 旋转）
    steps = [
        (0,    0, 0,      0,   0,   "复位"),
        (0,    0, 0.15,   0,   0,   "抬高"),
        (0.1,  0, 0.15,   0,   0,   "前移"),
        (0.1,  0, 0.15,  45,   0,   "左转"),
        (0.1,  0, 0.15, -45,   0,   "右转"),
        (0,    0, 0,       0,   0,   "回正"),
        (0,    0, 0.15,    0, -15,   "前倾"),
        (0,    0, 0,       0,   0,   "归位"),
    ]

    for i, (dx, dy, dz, yaw_deg, pitch_deg, desc) in enumerate(steps, 1):
        x = _INIT[0] + dx
        y = _INIT[1] + dy
        z = _INIT[2] + dz
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        print(f"步骤{i}: {desc}  "
              f"offset=({dx:.3f},{dy:.3f},{dz:.3f})  "
              f"yaw={yaw_deg}°  pitch={pitch_deg}°")
        robot.control_torso_pose(x=x, y=y, z=z, roll=0.0, pitch=pitch, yaw=yaw)
        time.sleep(4.0)

    print("\n躯干位姿控制演示完成。")


if __name__ == "__main__":
    main()
