#!/usr/bin/env python3
"""寻找 tag2 二维码目标示例

这个示例展示了如何使用 KuavoGraspBox 策略类来寻找场景中的 tag2 二维码目标。
主要功能包括:

- 初始化机器人及相关组件
- 配置目标 AprilTag 信息
- 使用头部和身体旋转搜索目标

用法:
    python3 lookup_tag2_example.py

注意:
    运行前需要确保:
    1. 机器人已正确连接
    2. AprilTag 标签尺寸配置正确
"""

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion
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
    
    # 创建AprilTag数据对象
    target_april_tag = AprilTagData(
        id=[2],         # 所要查找的 AprilTag ID
        size=[0.088],   # 所要查找的 AprilTag 标签尺寸
        pose=[PoseQuaternion(
            position=(0.0, -1.0, 0.8),        # 所要查找的 AprilTag 位置所在 /odom 坐标系下的位置(大致)
            orientation=(0.0, 0.0, 0.0, 1.0)  # 所要查找的 AprilTag 位置所在 /odom 坐标系下的朝向(四元数)
        )]
    )

    # 最大搜寻时间: 单位秒
    max_search_time = 15.0
    
    try:
        print("========== 开始执行寻找 tag2 二维码目标 ==========")
        
        grasp_strategy.robot.disable_head_tracking()
        print("✅ 已关闭头部追踪")
        
        find_success = grasp_strategy.head_find_target(
            target_april_tag, 
            max_search_time=max_search_time,
            search_pattern="rotate_body" #  表示使用身体旋转来对齐目标的朝向
        )
        if not find_success:
            print("❌ 寻找目标失败，无法继续执行")
            return
        
        print("✅ 已找到目标 tag2 二维码")        
    except Exception as e:
        print(f"执行过程中发生错误: {e}")

if __name__ == "__main__":
    main() 