#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮臂控制示例

展示如何通过KuavoWheelArm控制轮臂。
基于实际的lbLegControlSrv服务，轮臂控制只有一种方法：通过target_joints设置4个关节的目标角度。
"""

import time
import math

from kuavo_humanoid_sdk.kuavo.wheel_arm import KuavoWheelArm

def wait_action_down(wheel_arm, target_positions):
    while True:
        new_positions = wheel_arm.get_wheel_arm_joint_positions()
        print(f"当前关节位置: {[f'{pos*180/math.pi:.1f}' for pos in new_positions]} (度)")
        
        # 计算位置误差
        errors = [abs(target - actual) for target, actual in zip(target_positions, new_positions)]
        print(f"位置误差: {[f'{err*180/math.pi:.3f}' for err in errors]} (度)")
        if sum(errors) < 0.5 / (180/math.pi):
            time.sleep(1.0)
            break
        time.sleep(1.0)
    pass

def main():
    """主函数"""
    print("=== 轮臂控制示例 ===")
    
    try:
        # 创建KuavoWheelArm实例
        print("正在初始化KuavoWheelArm...")
        wheel_arm = KuavoWheelArm()
        
        print("轮臂控制模块已就绪")
        
        # 获取当前关节位置
        print("\n--- 获取当前关节位置 (来自传感器数据) ---")
        init_positions = wheel_arm.get_wheel_arm_joint_positions()
        print(f"当前关节位置: {[f'{pos:.3f}' for pos in init_positions]} (弧度)")
        print(f"当前关节位置: {[f'{pos*180/math.pi:.1f}' for pos in init_positions]} (度)")
        
        # 基本关节位置控制示例
        print("\n--- 基本关节位置控制 ---")

        actions = [[0.0, -0.0, 0.00, 0.0],
                   [0.1, -0.2, 0.05, 0.4],
                   [0.0, -0.0, 0.00, 0.0],
                   [0.1, -0.2, 0.05, 0.4],
                   [0.0, -0.0, 0.00, 0.0],
                   [0.1, -0.2, 0.05, 0.4],
                   [0.0, -0.0, 0.00, 0.0]]
        for target_positions in actions:
            print(f"目标关节位置: {[f'{pos:.3f}' for pos in target_positions]} (弧度)")
            print(f"目标关节位置: {[f'{pos*180/math.pi:.1f}' for pos in target_positions]} (度)")
            
            if wheel_arm.control_wheel_arm_joint_positions(target_positions):
                print("关节位置控制命令发送成功")
                
                # 等待一段时间让机器人执行
                print("等待机器人执行...")
                
                
                # 再次获取关节位置，检查是否到达目标
                print("\n--- 检查执行结果 ---")
                wait_action_down(wheel_arm, target_positions)
                
            else:
                print("关节位置控制失败")
            
            
            print("\n=== 示例执行完成 ===")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 