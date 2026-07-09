#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节链接位姿查询示例 (P0)

演示如何使用 get_link_position 和 get_link_pose 获取
指定机械臂关节链接的位置和完整位姿，支持不同参考坐标系。
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotTools


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
    robot_tools = KuavoRobotTools()

    # 进入站姿以确保 tf 树稳定
    robot.stance()
    time.sleep(1.0)

    # 定义要查询的关节链接名称
    link_names = [
        "zarm_l1_link",   # 左臂第1关节
        "zarm_l2_link",   # 左臂第2关节
        "zarm_r1_link",   # 右臂第1关节
        "zarm_r2_link",   # 右臂第2关节
    ]

    # 默认参考系 base_link
    print("=== 关节链接位置（参考系: base_link）===")
    for name in link_names:
        result = robot_tools.get_link_position(name, reference_frame="base_link")
        if result:
            pos, ori = result
            print(f"  {name}: 位置({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        else:
            print(f"  {name}: 获取失败")

    # 获取完整位姿（position + orientation）
    print("\n=== 关节链接完整位姿（参考系: base_link）===")
    for name in link_names[:2]:  # 只查前两个链接
        pose = robot_tools.get_link_pose(name, reference_frame="base_link")
        if pose:
            print(f"  {name}:")
            print(f"    position:    ({pose.position[0]:.3f}, {pose.position[1]:.3f}, {pose.position[2]:.3f})")
            print(f"    orientation: ({pose.orientation[0]:.3f}, {pose.orientation[1]:.3f}, {pose.orientation[2]:.3f}, {pose.orientation[3]:.3f})")
        else:
            print(f"  {name}: 获取失败")

    # 使用不同参考系 odom
    print("\n=== 关节链接位置（参考系: odom）===")
    result = robot_tools.get_link_position("zarm_l1_link", reference_frame="odom")
    if result:
        pos, ori = result
        print(f"  zarm_l1_link: 位置({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    else:
        print("  获取失败")

    print("\n关节链接位姿查询演示完成。")


if __name__ == "__main__":
    main()
