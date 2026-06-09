#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮臂手臂控制模式切换示例 (P2)

演示轮臂机器人的手臂轨迹控制模式切换，
对齐 ROS SDK demo: src/demo/wheel_arm_demo/wheel_control_mode_swither.py

注意: ROS demo 的 wheel_control_mode_swither.py 控制的是 MPC 控制模式
      (/mobile_manipulator_mpc_control, 0-4: NoControl/ArmOnly/BaseOnly/BaseArm/ArmEeOnly)，
      与本示例的手臂轨迹控制模式 (ArmFixed/AutoSwing/ExternalControl) 是不同概念。
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk.interfaces.data_types import KuavoArmCtrlMode


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

    # ======== 手臂控制模式切换 ========
    # 对齐 ROS SDK: 底层调用 /change_arm_ctrl_mode 服务
    # 模式说明:
    #   ArmFixed (0):        保持当前位置（回零时使用）
    #   AutoSwing (1):       回零，手臂回到初始目标位置
    #   ExternalControl (2): 外部轨迹控制模式
    print("=== 手臂控制模式切换 ===")

    print("1. 切换到固定位置模式 (ArmFixed)...")
    robot.wheel_control.set_arm_ctrl_mode(KuavoArmCtrlMode.ArmFixed)
    time.sleep(1.0)

    print("2. 切换到自动回零模式 (AutoSwing)...")
    robot.wheel_control.set_arm_ctrl_mode(KuavoArmCtrlMode.AutoSwing)
    time.sleep(1.0)

    print("3. 切换到外部控制模式 (ExternalControl)...")
    robot.wheel_control.set_arm_ctrl_mode(KuavoArmCtrlMode.ExternalControl)
    time.sleep(1.0)

    # 快捷方法：回零后自动切回外部控制模式
    print("\n4. 回零并切回外部模式 (reset_and_set_external)...")
    robot.wheel_control.reset_and_set_external()
    time.sleep(2.0)

    print("\n手臂控制模式切换演示完成。")


if __name__ == "__main__":
    main()
