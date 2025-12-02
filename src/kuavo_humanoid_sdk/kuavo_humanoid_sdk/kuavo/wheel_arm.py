#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮臂控制模块

提供轮臂控制的主要接口，基于实际的lbLegControlSrv服务。
轮臂控制只有一种方法：通过target_joints设置4个关节的目标角度。
"""

import math
from typing import List

from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore


class KuavoWheelArm:
    """轮臂控制类。
    """

    def __init__(self):
        """初始化轮臂控制"""
        try:
            # 获取KuavoRobotCore实例
            self._core = KuavoRobotCore()
            self._core.initialize()
            
            # 检查轮臂控制模块是否可用
            if self._core.is_wheel_arm_initialized():
                SDKLogger.info("[KuavoWheelArm] 轮臂控制模块初始化完成")
            else:
                SDKLogger.warning("[KuavoWheelArm] 轮臂控制模块未就绪")
        except Exception as e:
            SDKLogger.error(f"[KuavoWheelArm] 初始化失败: {e}")
            self._core = None

    def is_available(self) -> bool:
        """检查轮臂控制是否可用

        Returns:
            bool: 是否可用
        """
        return (self._core is not None and self._core.is_wheel_arm_initialized())

    def control_wheel_arm_joint_positions(self, positions: List[float]) -> bool:
        """控制轮臂关节位置

        Args:
            positions: 关节位置列表，4个关节的角度值（弧度）

        Returns:
            bool: 是否成功控制
        """
        if not self.is_available():
            SDKLogger.error("[KuavoWheelArm] 轮臂控制模块不可用")
            return False
        
        return self._core.control_wheel_arm_joint_positions(positions)

    def get_wheel_arm_joint_positions(self) -> List[float]:
        """获取轮臂当前关节位置

        Returns:
            List[float]: 4个关节的当前位置（弧度）
        """
        if not self.is_available():
            SDKLogger.error("[KuavoWheelArm] 轮臂控制模块不可用")
            return [0.0] * 4
        
        return self._core.get_wheel_arm_joint_positions()


if __name__ == "__main__":
    # 测试代码
    print("轮臂控制模块测试")
    
    # 创建轮臂控制实例
    wheel_arm = KuavoWheelArm()
    
    if wheel_arm.is_available():
        print("轮臂控制模块可用")
        
        # 测试获取当前关节位置
        print("\n--- 测试获取关节位置 (来自传感器数据) ---")
        current_positions = wheel_arm.get_wheel_arm_joint_positions()
        print(f"当前关节位置: {[f'{pos:.3f}' for pos in current_positions]} (弧度)")
        print(f"关节名称对应: [knee_joint, leg_joint, waist_pitch_joint, waist_yaw_joint]")
        print(f"角度(度): {[f'{pos*180/3.14159:.1f}' for pos in current_positions]}")
        
        # 测试关节位置控制
        print("\n--- 测试关节位置控制 ---")
        target_positions = [0.1, -0.2, 0.05, 0.1]
        print(f"目标关节位置: {[f'{pos:.3f}' for pos in target_positions]} (弧度)")
        
        success = wheel_arm.control_wheel_arm_joint_positions(target_positions)
        print(f"关节位置控制: {'成功' if success else '失败'}")
        
        if success:
            import time
            print("等待1秒后再次读取关节位置...")
            time.sleep(1.0)
            
            # 再次获取关节位置
            new_positions = wheel_arm.get_wheel_arm_joint_positions()
            print(f"执行后位置: {[f'{pos:.3f}' for pos in new_positions]} (弧度)")
            
            # 计算误差
            errors = [abs(target - actual) for target, actual in zip(target_positions, new_positions)]
            print(f"位置误差: {[f'{err:.3f}' for err in errors]} (弧度)")
        
    else:
        print("轮臂控制模块不可用") 