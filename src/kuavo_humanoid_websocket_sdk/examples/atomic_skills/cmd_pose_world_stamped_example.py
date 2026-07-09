#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TwistStamped 位姿控制示例 (P2)

演示如何使用 control_command_pose_world_stamped 在 odom 世界坐标系下
以绝对坐标控制机器人位姿，支持 dict 和 ROS TwistStamped 两种输入格式。

参考 demo: cmd_pos_world_test.py (/cmd_pose_world)
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

    # 进入站姿（command_pose_world 是 stance 的子状态）
    robot.stance()
    time.sleep(1.0)

    # ======== 使用 dict 格式 ========
    # TwistStamped 消息结构（与 /cmd_pose_world 话题一致）:
    #   linear.x, linear.y : 世界坐标系绝对位置（米）
    #   angular.z          : 世界坐标系绝对偏航角（弧度）
    #   注：linear.z / angular.x / angular.y 通常为 0
    print("=== 使用 dict 格式控制位姿（世界坐标系绝对坐标）===")

    # 测试1: 移动到世界坐标 (x=1.0, y=2.0, yaw=1.57)
    print("测试1: 移动到世界坐标 (x=1.0, y=2.0, yaw=1.57)...")
    robot.control_command_pose_world_stamped({
        "twist": {
            "linear": {"x": 1.0, "y": 2.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 1.57},
        }
    })
    # 等待运动完成（实际应用中建议订阅 /lb_cmd_pose_reach_time 反馈）
    time.sleep(8.0)

    # 回到 stance 后再发下一条指令
    robot.stance()
    time.sleep(1.0)

    # 测试2: 移动到世界坐标 (x=-0.5, y=1.5, yaw=3.14)
    print("测试2: 移动到世界坐标 (x=-0.5, y=1.5, yaw=3.14)...")
    robot.control_command_pose_world_stamped({
        "twist": {
            "linear": {"x": -0.5, "y": 1.5, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 3.14},
        }
    })
    time.sleep(8.0)

    robot.stance()
    time.sleep(1.0)

    # 测试3: 回到原点 (x=0, y=0, yaw=0)
    print("测试3: 回到原点 (x=0, y=0, yaw=0)...")
    robot.control_command_pose_world_stamped({
        "twist": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    })
    time.sleep(8.0)

    robot.stance()
    time.sleep(1.0)

    # ======== 方式2：使用 ROS TwistStamped（需要 geometry_msgs） ========
    # 如果运行在 ROS 环境中，可以直接传入 ROS TwistStamped 对象：
    #
    #   from geometry_msgs.msg import TwistStamped, Twist, Vector3
    #   msg = TwistStamped()
    #   msg.twist = Twist()
    #   msg.twist.linear = Vector3(x=1.0, y=2.0, z=0.0)
    #   msg.twist.angular = Vector3(x=0.0, y=0.0, z=1.57)
    #   robot.control_command_pose_world_stamped(msg)

    robot.stance()
    print("\nTwistStamped 位姿控制演示完成。")


if __name__ == "__main__":
    main()
