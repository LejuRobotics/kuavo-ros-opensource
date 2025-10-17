#!/usr/bin/env python3

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox

def main():
    if not KuavoSDK.Init(options=KuavoSDK.Options.Normal, log_level="DEBUG"):
        print("❌ 初始化失败")
        return

    print("初始化机器人...")
   
    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()
    
    # 初始化箱子抓取策略
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)
    
    success = grasp_strategy.walk_to_pose(
        # 目标位姿 /odom 坐标系下
        KuavoPose(
            position=(-1.5, 1.0, 0.0),  # 右后方
            orientation=(0.0, 0.0, 0.0, 1.0)
        ),
        target_distance=0.15,
        approach_speed=0.2,
        timeout=15.0
    )

    if not success:
        print("❌ 行走到目标位姿失败")
        return
    
    print("✅ 行走到目标位姿成功")

if __name__ == "__main__":
    main() 