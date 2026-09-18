#!/usr/bin/env python3
import time
import numpy as np
from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

def main():
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
        id=[2],  # AprilTag ID
        size=[0.088],  # AprilTag 标签尺寸
        pose=[PoseQuaternion(
            position=(0.0, -1.0, 0.8),  # 位置
            orientation=(0.0, 0.0, 0.0, 1.0)  # 四元数方向
        )]
    )
    
    # 创建箱子信息对象
    box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.5, 0.0, 0.4),  # 箱子位置
            orientation=(0.0, 0.0, 0.0, 1.0)  # 箱子朝向
        ),
        size=(0.3, 0.2, 0.15),  # 箱子尺寸(长、宽、高)，单位：米
        mass=1.0  # 箱子重量，单位：千克
    )
    
    # 创建放置位置的箱子信息
    placement_box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.8, 0.3, 0.5),  # 目标放置位置坐标
            orientation=(0.0, 0.0, 0.0, 1.0)  # 目标放置方向（四元数）
        ),
        size=(0.3, 0.2, 0.15),  # 使用相同尺寸
        mass=1.0  # 使用相同质量
    )
    
    time.sleep(1)
    # 执行完整抓取策略
    try:
        print("========== 开始执行箱子抓取策略 ==========")
        
        # 步骤1：使用头部寻找目标
        print("\n1. 寻找目标箱子...")
        grasp_strategy.robot.disable_head_tracking()
        print("✅ 已关闭头部追踪")
        
        find_success = grasp_strategy.head_find_target(
            target_april_tag, 
            max_search_time=15.0,
            search_pattern="rotate_body" #  rotate_head
        )
        
        if not find_success:
            print("❌ 寻找目标失败，无法继续执行")
            return
        
        print("✅ 已找到目标箱子")
        time.sleep(1)  # 短暂暂停
        
        # 步骤2：走路接近目标
        # for i in range(1):
        print("\n2. 走路接近目标...")
        approach_success = grasp_strategy.walk_approach_target(
            target_april_tag,
            target_distance=0.6,  # 与目标箱子保持0.6米的距离
            approach_speed=0.2    # 接近速度0.2米/秒
        )
        
        if not approach_success:
            print("❌ 接近目标失败，无法继续执行")
            return
            
        print("✅ 已成功接近目标")
        time.sleep(1)  # 短暂暂停
        
        # # 步骤3：手臂移动到抓取位置
        # print("\n3. 手臂移动到抓取位置...")
        # # 使用KuavoPose对象
        # grasp_pose = KuavoPose(
        #     position=(0.5, 0.0, 0.4),  # 示例位置
        #     orientation=(0.0, 0.0, 0.0, 1.0)  # 示例朝向（四元数）
        # )
        
        # move_success = grasp_strategy.arm_move_to_target(
        #     grasp_pose,
        #     approach_speed=0.15
        # )
        
        # if not move_success:
        #     print("❌ 手臂移动失败，无法继续执行")
        #     return
            
        # print("✅ 手臂已到达抓取位置")
        # time.sleep(1)  # 短暂暂停
        
        # # 步骤4：提起箱子
        # print("\n4. 提起箱子...")
        # transport_up_success = grasp_strategy.arm_transport_target_up(
        #     box_info,
        #     arm_mode="manipulation_mpc"
        # )
        
        # if not transport_up_success:
        #     print("❌ 提起箱子失败")
        #     return
            
        # print("✅ 成功提起箱子")
        # time.sleep(2)  # 展示一下成功提起的状态
        
        # # 步骤5：放下箱子
        # print("\n5. 放下箱子...")
        # transport_down_success = grasp_strategy.arm_transport_target_down(
        #     placement_box_info,
        #     arm_mode="manipulation_mpc"
        # )
        
        # if not transport_down_success:
        #     print("❌ 放下箱子失败")
        #     return
            
        # print("✅ 成功放下箱子")
        
        print("\n========== 箱子抓取任务完成 ==========")
        
    except Exception as e:
        print(f"执行过程中发生错误: {e}")
    finally:
        # 确保机器人回到安全状态
        print("将机器人恢复到安全姿态...")
        # 这里可以添加使机器人回到默认姿态的代码

if __name__ == "__main__":
    main() 