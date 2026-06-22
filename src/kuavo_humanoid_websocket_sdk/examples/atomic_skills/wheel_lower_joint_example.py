#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂下部关节控制示例 (P2)

演示轮臂机器人 4 个下部关节的轨迹控制，
对齐 ROS SDK demo: src/demo/wheel_arm_demo/cmd_leg_joint_test.py

话题 /lb_leg_traj, 消息类型 sensor_msgs/JointState
关节名称: ['joint1', 'joint2', 'joint3', 'joint4']
参数单位：度
demo 中每条指令发布后等待 /lb_leg_joint_reach_time 反馈
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

    # ======== 下部关节控制 ========
    print("=== 下部关节控制 ===")
    print("（参考 ROS demo cmd_leg_joint_test.py）")

    # 测试数据1: 典型关节角度（与 demo 一致）
    joint_msg1 = [14.90, -32.01, 18.03, 0.0]
    print(f"发送下部关节目标1: {joint_msg1}")
    robot.control_wheel_lower_joint(joint_msg1)
    time.sleep(4.0)

    # 测试数据2: 第四关节转 90 度（与 demo 一致）
    joint_msg2 = [14.90, -32.01, 18.03, 90.0]
    print(f"发送下部关节目标2: {joint_msg2}")
    robot.control_wheel_lower_joint(joint_msg2)
    time.sleep(4.0)

    # 测试数据3: 恢复零位（与 demo 一致）
    joint_msg3 = [0.0, 0.0, 0.0, 0.0]
    print(f"发送下部关节目标3 (归零): {joint_msg3}")
    robot.control_wheel_lower_joint(joint_msg3)
    time.sleep(4.0)

    print("\n下部关节控制演示完成。")


if __name__ == "__main__":
    main()
