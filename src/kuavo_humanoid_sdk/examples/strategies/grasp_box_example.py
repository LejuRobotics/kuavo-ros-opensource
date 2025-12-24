#!/usr/bin/env python3
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo
from kuavo_humanoid_sdk.common.logger import SDKLogger
"""
    安全须知:
    请注意
    实物机器人当中因为搬箱子需要用到末端力控制, 请把 arm_transport_target_up 当中的sim_mode设置为False
    仿真机器人当中因为物理性质差异，所以不需要用到末端力控制 请把 arm_transport_target_up 当中的sim_mode设置为True

    长春一汽绿色箱子质量为1.5kg, 请根据实际情况修改箱子质量
"""
def main(real: bool = True):
    start_time = time.time()  # 记录开始时间
    # Initialize SDK 
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)

    SDKLogger.info("初始化机器人...")

    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()
    
    # 初始化箱子抓取策略
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)
    
    # 粘贴在箱子上的 AprilTag 信息: 需要 ID，尺寸和基于odom坐标系下的大致位姿
    box_tag = AprilTagData(
        id=[1],                                 # AprilTag ID
        size=[0.1],                             # AprilTag 标签尺寸
        pose=[PoseQuaternion(
            # TODO: 需要根据实际情况调整
            position=(0.0, -1.5, 0.8),          # 基于odom坐标系下的大致位置, 查找时会对齐到这个方向
            orientation=(0.0, 0.0, 0.0, 1.0)    # 四元数方向
        )]
    )
    
    # 粘贴在放置位置的 AprilTag 信息: 需要 ID，尺寸和基于odom坐标系下的大致位姿
    placement_tag = AprilTagData(
        id=[0],                                 # AprilTag ID
        size=[0.1],                             # AprilTag 标签尺寸
        pose=[PoseQuaternion(
            # TODO: 需要根据实际情况调整
            position=(0.0, 1.5, 1.5),           # 基于odom坐标系下的大致位置, 查找时会对齐到这个方向
            orientation=(0.0, 0.0, 1.0, 0.0)    # 四元数方向 - 旋转180度
        )]
    )    

    # 创建箱子信息对象
    box_size = (0.3, 0.4, 0.22) # xyz( 宽, 长, 高)
    box_mass = 1.5
    
    time.sleep(1)
    
    # 执行完整抓取策略
    try:
        SDKLogger.info("========== 开始执行箱子抓取策略 ==========")
        
        # !!! Important: 关闭 basePitch 限制, 否则无法进行大幅度的前后倾动作 会导致摔倒
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        grasp_strategy.robot.enable_base_pitch_limit(False)  # 关闭 basePitch 限制
        # Retry up to 3 times to check pitch limit status
        for i in range(3):
            if not robot_state.pitch_limit_enabled():
                SDKLogger.info("✅ 已关闭 basePitch 限制")
                break
            else:
                if i < 2:  # Don't log on last attempt
                    SDKLogger.info(f"⚠️ 第{i+1}次检查 basePitch 限制状态...")
                time.sleep(0.5)  # Brief pause between retries
        else:  # Loop completed without break
            SDKLogger.info("❌ 关闭 basePitch 限制失败")
            return
        # ----------------------------------------------------------- 
            
        # 步骤1：使用头部寻找目标
        SDKLogger.info("1. 寻找目标箱子...")
        grasp_strategy.robot.disable_head_tracking()
        SDKLogger.info("✅ 已关闭头部追踪")
        
        find_success = grasp_strategy.head_find_target(
            box_tag, 
            max_search_time=15.0,
            search_pattern="rotate_body" #  rotate_head
        )
        
        if not find_success:
            SDKLogger.info("❌ 寻找目标失败，无法继续执行")
            return
        
        SDKLogger.info("✅ 已找到目标箱子")
        time.sleep(1)  # 短暂暂停
        box_tag_data = robot_vision.get_data_by_id_from_odom(box_tag.id[0]) # 拿到箱子tag数据 odom系
        if not box_tag_data:
            SDKLogger.info(f"❌ 未识别到 AprilTag ID 为{box_tag.id[0]} 的目标箱子")
            return
        
        SDKLogger.info(f"box_tag_data: {box_tag_data}")

        box_info = BoxInfo.from_apriltag(
            box_tag_data,
            xyz_offset=(box_size[0]/2, 0.0, -0.00),    # tag 粘贴在箱子正面，为了得到箱子中心点，因此偏移量为箱子宽度的一半
            size=box_size,                  # 箱子尺寸，单位：米
            mass=box_mass                   # 箱子重量，单位：千克
        )

        SDKLogger.info(f"box_info: {box_info}")

        # 步骤2：走路接近目标
        # for i in range(1):
        SDKLogger.info("2. 走路接近目标...")
        approach_success = grasp_strategy.walk_approach_target(
            box_tag.id[0],
            target_distance=0.4,  # 与目标箱子保持0.7米的距离
            approach_speed=0.2    # 接近速度0.2米/秒
        )
        
        if not approach_success:
            SDKLogger.info("❌ 接近目标失败，无法继续执行")
            return
            
        SDKLogger.info("✅ 已成功接近目标")
        time.sleep(1)  # 短暂暂停
        
        # 步骤3：手臂移动到抓取位置
        SDKLogger.info("3. 手臂移动到抓取位置...")
        move_success = grasp_strategy.arm_move_to_target(
            box_info,
            arm_mode="manipulation_mpc"
        )
        
        if not move_success:
            SDKLogger.info("❌ 手臂移动失败，无法继续执行")
            return
            
        SDKLogger.info("✅ 手臂已到达抓取位置")
        # grasp_strategy.robot.arm_reset()
        time.sleep(1.0)  # 短暂暂停
        
        
        # 步骤4：提起箱子
        SDKLogger.info("4. 提起箱子...")
        transport_up_success = grasp_strategy.arm_transport_target_up(
            box_info,
            arm_mode="manipulation_mpc",
            sim_mode=not real
        )
        
        if not transport_up_success:
            SDKLogger.info("❌ 提起箱子失败")
            return
        time.sleep(1.0)  # 短暂暂停
            
        SDKLogger.info("✅ 成功提起箱子")
        # grasp_strategy.robot.arm_reset()
        time.sleep(1.0)  # 展示一下成功提起的状态

        # return # FIXME:测试提起箱子成功   
        
        # 步骤5：关闭头部追踪
        SDKLogger.info("5. 关闭头部追踪...")
        grasp_strategy.robot.disable_head_tracking()
        SDKLogger.info("✅ 已关闭头部追踪")
        time.sleep(1.0)  # 短暂暂停
        if not grasp_strategy.head_find_target(
            placement_tag, 
            max_search_time=15.0,
            search_pattern="rotate_body" #  rotate_head
        ):
            SDKLogger.info("❌ 寻找目标失败，无法继续执行")
            return
        placement_tag_data = robot_vision.get_data_by_id_from_odom(placement_tag.id[0])
        if placement_tag_data is None:
            SDKLogger.info(f"❌ 未识别到 AprilTag ID 为{placement_tag.id[0]} 的目标箱子")
            return
        
        # 步骤6：移动到放置位置
        SDKLogger.info("6. 移动到放置位置...")
        move_success = grasp_strategy.walk_approach_target(
            placement_tag.id[0],
            target_distance=0.4,
            approach_speed=0.2,
        )
        if not move_success:
            SDKLogger.info("❌ 移动到放置位置失败")
            return
        time.sleep(1.0)  # 短暂暂停

        # 步骤7：放下箱子
        placement_box_info = BoxInfo.from_apriltag(
            placement_tag_data,
            xyz_offset=(box_size[0]/2, 0.0, -0.5), # tag 粘贴在货架上，需要把箱子放下距离货架的高度 -0.5m
            size=box_size,  # 箱子尺寸(长、宽、高)，单位：米
            mass=box_mass  # 箱子重量，单位：千克
        )
        SDKLogger.info("7. 放下箱子...")
        transport_down_success = grasp_strategy.arm_transport_target_down(
            placement_box_info,
            arm_mode="manipulation_mpc"
        )
        
        if not transport_down_success:
            SDKLogger.info("❌ 放下箱子失败")
            return
            
        SDKLogger.info("✅ 成功放下箱子")
        time.sleep(1.0)  # 短暂暂停        

        # 步骤8: 回到初始位置
        SDKLogger.info("8. 回到初始位置...")
        grasp_strategy.robot.control_command_pose_world(0, 0, 0, 0)
        time.sleep(10) # 等待10s转身完成
        total_time = time.time() - start_time  # 计算总时间
        SDKLogger.info(f"========== 搬箱子任务完成，总耗时: {total_time:.2f}秒 ==========")
    except Exception as e:
        SDKLogger.info(f"执行过程中发生错误: {e}")
    finally:
        # 确保机器人回到安全状态
        SDKLogger.info("将机器人恢复到安全姿态...")
        # 这里可以添加使机器人回到默认姿态的代码

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', default=False)
    args, unknown = parser.parse_known_args()
    main(not args.sim)