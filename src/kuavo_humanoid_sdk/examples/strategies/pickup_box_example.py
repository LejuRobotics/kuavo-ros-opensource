#!/usr/bin/env python3

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

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
    
    robot_position = robot_state.robot_position()
    robot_orientation = robot_state.robot_orientation()

    box_size = (0.4, 0.3, 0.22) # 箱子尺寸(长、宽、高)，单位：米
    box_mass = 2.0              # 箱子重量，单位：千克
    box_info = BoxInfo(
        pose= KuavoPose(
            # 这只是一个测试位置
            # 0.39 0.09 -0.25 是测试位置，相对于 base_link 躯干
            position=(robot_position[0] + 0.39, robot_position[1]+0.09, robot_position[2]-0.25),
            orientation= robot_orientation
        ),
        size=box_size,                  # 箱子尺寸(长、宽、高)，单位：米
        mass=box_mass                   # 箱子重量，单位：千克
    )
    
    # 手臂归位
    robot.arm_reset()

    if not grasp_strategy.arm_transport_target_up(
            box_info,
            arm_mode="manipulation_mpc"
        ):
            print("❌ 提起箱子失败")
            return
 
    print("✅ 提起箱子成功")

if __name__ == "__main__":
    main() 