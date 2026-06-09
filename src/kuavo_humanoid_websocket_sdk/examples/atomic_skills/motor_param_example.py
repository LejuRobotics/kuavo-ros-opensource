#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电机参数和 Base Pitch 限位示例 (P2)

演示如何使用 change_motor_param / get_motor_param 读写电机 PID 参数，
以及 enable_base_pitch_limit 启用/禁用 base pitch 限位保护。
"""

import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot


# 电机参数数据结构
class MotorParam:
    """电机 PID 参数"""
    def __init__(self, motor_id: int, Kp: float = 0.0, Kd: float = 0.0):
        self.id = motor_id
        self.Kp = Kp
        self.Kd = Kd

    def __repr__(self):
        return f"MotorParam(id={self.id}, Kp={self.Kp:.4f}, Kd={self.Kd:.4f})"


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

    # ======== 获取电机参数 ========
    print("=== 获取所有电机 PID 参数 ===")
    success, params = robot.get_motor_param()
    if success:
        print(f"共 {len(params)} 个电机:")
        for p in params:
            print(f"  {p}")
    else:
        print("获取电机参数失败")
        return

    # ======== 修改电机参数 ========
    print("\n=== 修改指定电机的 PID 参数 ===")
    # 修改电机 0 的 Kp 和 Kd（示例使用当前 Kp+10）
    if len(params) > 0:
        first_param = params[0]
        new_kp = first_param.Kp + 10.0
        new_kd = first_param.Kd + 0.5
        print(f"修改电机 {int(first_param.id)}: Kp -> {new_kp}, Kd -> {new_kd}")
        success, msg = robot.change_motor_param([
            MotorParam(motor_id=int(first_param.id), Kp=new_kp, Kd=new_kd)
        ])
        print(f"  修改{'成功' if success else '失败'}: {msg}")
        time.sleep(0.5)

        # 恢复原值
        print(f"恢复电机 {int(first_param.id)}: Kp -> {first_param.Kp}, Kd -> {first_param.Kd}")
        success, msg = robot.change_motor_param([
            MotorParam(motor_id=int(first_param.id), Kp=first_param.Kp, Kd=first_param.Kd)
        ])
        print(f"  恢复{'成功' if success else '失败'}: {msg}")

    # ======== Base Pitch 限位 ========
    print("\n=== Base Pitch 限位控制 ===")

    # 开启 base pitch 限位
    print("开启 base pitch 限位保护...")
    success, msg = robot.enable_base_pitch_limit(True)
    print(f"  开启{'成功' if success else '失败'}: {msg}")
    time.sleep(0.5)

    # 关闭 base pitch 限位
    print("关闭 base pitch 限位保护...")
    success, msg = robot.enable_base_pitch_limit(False)
    print(f"  关闭{'成功' if success else '失败'}: {msg}")

    print("\n电机参数与 Base Pitch 限位演示完成。")


if __name__ == "__main__":
    main()
