#!/usr/bin/env python3
"""
WSSDK 搬箱策略示例

通过 rosbridge WebSocket 连接 ROS，演示完整的箱子抓取流程：
1. 头部旋转搜索 AprilTag 目标
2. 走路接近目标
3. 手臂抓取/搬运/放置（可选启用）

前置条件：
- 下位机 rosbridge_websocket 已启动 (roslaunch rosbridge_server rosbridge_websocket.launch)
- 下位机 websocket_sdk_start_node 已启动
- 下位机 Tag Tracker 已启动 (roslaunch ar_control robot_strategies.launch real:=true)
- 机器人已站立

用法：
    export KUAVO_REAL=true
    python3 grasp_box_example.py
"""

import os
import sys
import time
import numpy as np

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo


def main():
    # ================================================================
    # WSSDK 初始化（通过 rosbridge WebSocket 连接 ROS）
    # ================================================================
    is_real = os.environ.get('KUAVO_REAL', 'false').lower() == 'true'

    websocket_host = os.environ.get('WEBSOCKET_HOST', '127.0.0.1')
    websocket_port = int(os.environ.get('WEBSOCKET_PORT', '9090'))

    print(f"初始化 WSSDK (websocket_mode=True, host={websocket_host}:{websocket_port})...")

    if not KuavoSDK.Init(
        log_level='INFO',
        websocket_mode=True,
        websocket_host=websocket_host,
        websocket_port=websocket_port,
    ):
        print("❌ Init KuavoSDK failed!")
        print("   请检查：")
        print("   1. 下位机 rosbridge WebSocket 是否已启动")
        print("   2. websocket_host/websocket_port 配置是否正确")
        sys.exit(1)

    print("✅ WSSDK 初始化成功")

    # ================================================================
    # 初始化机器人组件
    # ================================================================
    print("\n初始化机器人组件...")
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()

    # 初始化搬箱策略
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)

    # ================================================================
    # 配置 AprilTag 目标
    # ================================================================
    target_april_tag = AprilTagData(
        id=[2],                          # AprilTag ID（箱子上的码）
        size=[0.088],                    # 标签物理尺寸（米）
        pose=[PoseQuaternion(
            position=(0.0, -1.0, 0.8),   # 目标初始猜测位置
            orientation=(0.0, 0.0, 0.0, 1.0)
        )]
    )

    # 箱子信息（用于搬运/放置步骤）
    box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.5, 0.0, 0.4),
            orientation=(0.0, 0.0, 0.0, 1.0)
        ),
        size=(0.3, 0.2, 0.15),          # 箱子尺寸（长、宽、高）米
        mass=1.0                          # 箱子质量 kg
    )

    placement_box_info = BoxInfo(
        pose=KuavoPose(
            position=(0.8, 0.3, 0.5),
            orientation=(0.0, 0.0, 0.0, 1.0)
        ),
        size=(0.3, 0.2, 0.15),
        mass=1.0
    )

    time.sleep(1)

    # ================================================================
    # 执行搬箱策略
    # ================================================================
    try:
        print("\n========== 开始执行箱子抓取策略 ==========")

        # ---- 步骤1：关闭头部追踪，搜索目标 ----
        print("\n1. 寻找目标箱子...")
        grasp_strategy.robot.disable_head_tracking()
        print("   ✅ 已关闭头部追踪")

        find_success = grasp_strategy.head_find_target(
            target_april_tag,
            max_search_time=15.0,
            search_pattern="rotate_body"  # 转身体搜索；也可用 "rotate_head"
        )

        if not find_success:
            print("   ❌ 寻找目标失败，退出")
            return
        print("   ✅ 已找到目标箱子")
        time.sleep(1)

        # ---- 步骤2：走路接近目标 ----
        print("\n2. 走路接近目标...")
        approach_success = grasp_strategy.walk_approach_target(
            target_april_tag,
            target_distance=0.6,   # 保持 0.6 米距离
            approach_speed=0.2      # 接近速度 0.2 m/s
        )

        if not approach_success:
            print("   ❌ 接近目标失败，退出")
            return
        print("   ✅ 已成功接近目标")
        time.sleep(1)

        # ---- 步骤3（可选）：手臂抓取 → 搬运 → 放置 ----
        # 取消下方注释以启用完整抓取-搬运-放置流程：
        #
        # print("\n3. 手臂移动到抓取位置...")
        # grasp_pose = KuavoPose(
        #     position=(0.5, 0.0, 0.4),
        #     orientation=(0.0, 0.0, 0.0, 1.0)
        # )
        # move_success = grasp_strategy.arm_move_to_target(grasp_pose, approach_speed=0.15)
        # if not move_success:
        #     print("   ❌ 手臂移动失败，退出")
        #     return
        # print("   ✅ 手臂已到达抓取位置")
        # time.sleep(1)
        #
        # print("\n4. 提起箱子...")
        # transport_up_success = grasp_strategy.arm_transport_target_up(
        #     box_info, arm_mode="manipulation_mpc"
        # )
        # if not transport_up_success:
        #     print("   ❌ 提起箱子失败，退出")
        #     return
        # print("   ✅ 成功提起箱子")
        # time.sleep(2)
        #
        # print("\n5. 放下箱子...")
        # transport_down_success = grasp_strategy.arm_transport_target_down(
        #     placement_box_info, arm_mode="manipulation_mpc"
        # )
        # if not transport_down_success:
        #     print("   ❌ 放下箱子失败，退出")
        #     return
        # print("   ✅ 成功放下箱子")

        print("\n========== 箱子抓取任务完成 ==========")

    except Exception as e:
        print(f"\n❌ 执行过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n将机器人恢复到安全姿态...")


if __name__ == "__main__":
    main()
