#!/usr/bin/env python3
# coding: utf-8
"""
机器人控制器示例

此示例演示如何使用 KuavoRobotController 类获取和切换机器人控制器，
并结合行走功能展示不同控制器的效果。
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk.kuavo.robot_controller import KuavoRobotController


def walk_for_a_while(robot, controller_name: str, duration=5.0):
    """让机器人行走一段时间

    Args:
        robot: KuavoRobot 实例
        controller_name (str): 当前控制器名称
        duration: 行走时长（秒）
    """
    print(f"开始行走，持续 {duration} 秒...")

    # 发送行走命令
    # walk 参数：linear_x, linear_y, angular_z (单位：m/s, rad/s)
    success = robot.walk(linear_x=0.3, linear_y=0.3, angular_z=0.3)

    if success:
        print("行走命令发送成功")
        time.sleep(duration)

        # 停止行走
        robot.walk(linear_x=0.0, linear_y=0.0, angular_z=0.0)
        print("行走停止")
    else:
        print("行走命令发送失败")

    # 进入站立模式
    print("进入站立模式...")
    robot.stance()
    print("已站立")

    # 操作完成后延时 5 秒
    time.sleep(5.0)


def main():
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot_controller = KuavoRobotController()
    robot = KuavoRobot()

    # 1. 获取控制器列表
    print("\n" + "=" * 50)
    print("1. 获取控制器列表")
    print("=" * 50)
    controller_info = robot_controller.get_controller_list()

    if controller_info is None:
        print("Get controller list failed")
        return

    print(f"控制器数量：{controller_info.count}")
    print(f"当前控制器名称：{controller_info.current_controller}")
    print(f"可用控制器列表:")
    for name in controller_info.controller_names:
        marker = " <-- 当前" if name == controller_info.current_controller else ""
        print(f"  - {name}{marker}")
    print(f"状态消息：{controller_info.message}")

    # 2. 使用当前控制器行走一段时间
    print("\n" + "=" * 50)
    print("2. 使用当前控制器行走")
    print("=" * 50)
    print(f"当前控制器：{controller_info.current_controller}")
    walk_for_a_while(robot, controller_name=controller_info.current_controller, duration=3.0)

    # 3. 切换到下一个控制器
    print("\n" + "=" * 50)
    print("3. 切换到下一个控制器")
    print("=" * 50)

    result = robot_controller.switch_to_next_controller()
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    print(f"从 {result.current_controller} 切换到 {result.target_controller}")

    if result.success:
        # 等待控制器切换完成
        time.sleep(1.0)

        # 操作完成后延时 5 秒
        time.sleep(5.0)

        # 4. 使用新控制器行走一段时间
        print("\n" + "=" * 50)
        print("4. 使用新控制器行走")
        print("=" * 50)
        walk_for_a_while(robot, controller_name=result.target_controller, duration=3.0)

    # 5. 再次切换到下一个控制器
    print("\n" + "=" * 50)
    print("5. 再次切换到下一个控制器")
    print("=" * 50)
    result = robot_controller.switch_to_next_controller()
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    print(f"从 {result.current_controller} 切换到 {result.target_controller}")

    # 操作完成后延时 5 秒
    time.sleep(5.0)

    print("\n" + "=" * 50)
    print("示例完成")
    print("=" * 50)


if __name__ == "__main__":
    main()
