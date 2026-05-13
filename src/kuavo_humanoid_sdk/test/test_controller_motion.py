#!/usr/bin/env python3
# coding: utf-8
"""
测试不同 Controller 下的运动接口 - 交互式命令行版本

使用方法:
    python test_controller_motion.py

启动后输入命令来执行测试，输入 help 查看可用命令。
"""

import time
import math
from typing import Tuple
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk.kuavo.robot_controller import KuavoRobotController
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoIKParams, KuavoManipulationMpcCtrlMode, KuavoManipulationMpcControlFlow, KuavoManipulationMpcFrame


def get_controller_list(controller_manager: KuavoRobotController):
    """获取当前 controller 列表"""
    controller_info = controller_manager.get_controller_list()
    if controller_info and controller_info.success:
        return controller_info.controller_names, controller_info.current_controller
    print("获取 controller 列表失败")
    return ['mpc', 'amp_controller'], 'unknown'


def switch_controller(controller_manager: KuavoRobotController, controller_name: str) -> bool:
    """切换到指定的 controller"""
    result = controller_manager.switch_controller(controller_name)
    if result.success:
        print(f"✓ 成功切换到 controller: {controller_name}")
        time.sleep(2.0)
        return True
    else:
        msg = getattr(result, 'message', 'unknown') if hasattr(result, 'message') else ''
        print(f"✗ 切换 controller 失败：{controller_name}, 消息：{msg}")
        return False


def print_help():
    """打印帮助信息"""
    print("""
可用命令:
  控制器相关:
    list                  - 列出所有可用 controllers
    switch <name>         - 切换到指定 controller
    current               - 显示当前 controller
    current_name          - 使用 get_current_controller_name 获取当前控制器名称
    next                  - 切换到下一个控制器
    prev                  - 切换到上一个控制器
    rl_mode [true|false]  - 设置 RL 切换模式 (默认 true=直接切换)
    fall_down [true|false]- 设置倒地状态 (默认 true=倒地)
    vmp                   - 切换到 VMP 控制器
    dance                 - 切换到舞蹈控制器

  运动接口测试:
    stance                - 站立模式
    trot                  - 小跑
    walk [vx] [vy] [wz]   - 行走 (默认 vx=0.1, 持续 2 秒)
    squat [h] [p]         - 下蹲 (默认 h=-0.1, p=0.0)
    torso [x] [y] [z] [r] [p] [y]  - 控制躯干位姿
    step [x] [y] [z] [w] [dt]  - 单步移动
    cmd_pose [x] [y] [z] [w]  - 控制指令位姿
    cmd_pose_w [x] [y] [z] [w]  - 控制指令世界位姿
    head [yaw] [pitch]    - 控制头部
    track <id>            - 启用头部跟踪
    untrack               - 禁用头部跟踪
    waist [x] [y]         - 控制腰部位置
    wrench <l_fx> ... <r_tz>  - 控制手部力 (12 个参数)
    arm_reset             - 手臂复位
    mpc_reset             - Manipulation MPC 复位
    arm_joint [j1] ...    - 控制手臂关节位置
    fixed_arm             - 设置固定手臂模式
    swing_arm             - 设置自动摆臂模式

  通用接口测试:
    ext_arm               - 设置外部控制手臂模式
    mpc_mode <mode>       - 设置 MPC 模式 (ArmOnly/BaseOnly/BaseArm)
    mpc_flow <flow>       - 设置 MPC 控制流 (DirectToWbc/WithPlanning)
    mpc_frame <frame>     - 设置 MPC 坐标系 (WorldFrame/BaseFrame)
    end_pose              - 控制末端执行器位姿 (测试位姿)
    motor_param           - 获取电机参数
    base_pitch <0|1>      - 启用/禁用底座俯仰限制
    collision             - 检查手臂碰撞

  其他:
    help                  - 显示此帮助信息
    quit / exit           - 退出程序

示例:
    switch mpc
    walk 0.2 0.0 0.0
    squat -0.15 0.0
    head 0.5 -0.3
    mpc_mode ArmOnly
    next
    rl_mode true
    vmp
    dance
""")


def execute_command(cmd_str: str, robot: KuavoRobot, controller_manager: KuavoRobotController, 
                    current_controller: str) -> Tuple[bool, str]:
    """执行命令，返回 (是否继续循环，新的 current_controller)"""
    parts = cmd_str.strip().split()
    if not parts:
        return True, current_controller
    
    cmd = parts[0].lower()
    args = parts[1:]
    
    try:
        # 控制器相关
        if cmd == 'list':
            names, curr = get_controller_list(controller_manager)
            print(f"可用 controllers: {names}")
            print(f"当前 controller: {curr}")
            return True, curr
        
        elif cmd == 'switch':
            if not args:
                print("用法：switch <controller_name>")
                return True, current_controller
            target = args[0]
            if switch_controller(controller_manager, target):
                _, new_curr = get_controller_list(controller_manager)
                return True, new_curr
            return True, current_controller
        
        elif cmd == 'current':
            _, curr = get_controller_list(controller_manager)
            print(f"当前 controller: {curr}")
            return True, curr
        
        # 运动接口测试
        elif cmd == 'stance':
            result = robot.stance()
            print(f"stance: {'✓ 成功' if result else '✗ 失败'}")
        
        elif cmd == 'trot':
            result = robot.trot()
            print(f"trot: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'walk':
            vx = float(args[0]) if len(args) > 0 else 0.1
            vy = float(args[1]) if len(args) > 1 else 0.0
            wz = float(args[2]) if len(args) > 2 else 0.0
            result = robot.walk(linear_x=vx, linear_y=vy, angular_z=wz)
            print(f"walk(vx={vx}, vy={vy}, wz={wz}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(2.0)
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=0.0)
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'squat':
            h = float(args[0]) if len(args) > 0 else -0.1
            p = float(args[1]) if len(args) > 1 else 0.0
            result = robot.squat(height=h, pitch=p)
            print(f"squat(h={h}, p={p}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.5)
            robot.squat(height=0.0, pitch=0.0)
            robot.stance()
        
        elif cmd == 'torso':
            x = float(args[0]) if len(args) > 0 else 0.0
            y = float(args[1]) if len(args) > 1 else 0.0
            z = float(args[2]) if len(args) > 2 else -0.1
            r = float(args[3]) if len(args) > 3 else 0.0
            p = float(args[4]) if len(args) > 4 else 0.0
            yaw = float(args[5]) if len(args) > 5 else 0.0
            result = robot.control_torso_pose(x, y, z, r, p, yaw)
            print(f"torso: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'step':
            x = float(args[0]) if len(args) > 0 else 0.1
            y = float(args[1]) if len(args) > 1 else 0.0
            z = float(args[2]) if len(args) > 2 else 0.0
            w = float(args[3]) if len(args) > 3 else 0.0
            dt = float(args[4]) if len(args) > 4 else 0.4
            result = robot.step_by_step(target_pose=[x, y, z, w], dt=dt)
            print(f"step_by_step: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(2.0)
            robot.stance()
        
        elif cmd == 'cmd_pose':
            x = float(args[0]) if len(args) > 0 else 0.0
            y = float(args[1]) if len(args) > 1 else 0.0
            z = float(args[2]) if len(args) > 2 else 0.0
            w = float(args[3]) if len(args) > 3 else 0.0
            result = robot.control_command_pose(x, y, z, w)
            print(f"control_command_pose: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'cmd_pose_w':
            x = float(args[0]) if len(args) > 0 else 0.0
            y = float(args[1]) if len(args) > 1 else 0.0
            z = float(args[2]) if len(args) > 2 else 0.0
            w = float(args[3]) if len(args) > 3 else 0.0
            result = robot.control_command_pose_world(x, y, z, w)
            print(f"control_command_pose_world: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'head':
            yaw = float(args[0]) if len(args) > 0 else 0.0
            pitch = float(args[1]) if len(args) > 1 else 0.0
            result = robot.control_head(yaw=yaw, pitch=pitch)
            print(f"head(yaw={yaw}, pitch={pitch}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
            robot.control_head(yaw=0.0, pitch=0.0)
        
        elif cmd == 'track':
            if not args:
                print("用法：track <target_id>")
                return True, current_controller
            target_id = int(args[0])
            result = robot.enable_head_tracking(target_id=target_id)
            print(f"enable_head_tracking(id={target_id}): {'✓ 成功' if result else '✗ 失败'}")
        
        elif cmd == 'untrack':
            result = robot.disable_head_tracking()
            print(f"disable_head_tracking: {'✓ 成功' if result else '✗ 失败'}")
        
        elif cmd == 'waist':
            x = float(args[0]) if len(args) > 0 else 0.0
            y = float(args[1]) if len(args) > 1 else 0.0
            result = robot.control_waist_pos([x, y])
            print(f"control_waist_pos: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
            robot.stance()
        
        elif cmd == 'wrench':
            if len(args) < 12:
                print("用法：wrench <l_fx> <l_fy> <l_fz> <l_tx> <l_ty> <l_tz> <r_fx> <r_fy> <r_fz> <r_tx> <r_ty> <r_tz>")
                return True, current_controller
            wrench_vals = [float(x) for x in args[:12]]
            left_wrench = wrench_vals[0:6]
            right_wrench = wrench_vals[6:12]
            result = robot.control_hand_wrench(left_wrench=left_wrench, right_wrench=right_wrench)
            print(f"control_hand_wrench: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
            robot.stance()
        
        elif cmd == 'arm_reset':
            result = robot.arm_reset()
            print(f"arm_reset: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(2.0)
            robot.stance()
        
        elif cmd == 'mpc_reset':
            result = robot.manipulation_mpc_reset()
            print(f"manipulation_mpc_reset: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'arm_joint':
            arm_dof = 6
            try:
                arm_dof = robot._robot_info.arm_joint_dof
            except:
                pass
            if len(args) < arm_dof:
                print(f"用法：arm_joint <j1> <j2> ... (需要 {arm_dof} 个参数)")
                return True, current_controller
            positions = [float(x) for x in args[:arm_dof]]
            result = robot.control_arm_joint_positions(positions)
            print(f"control_arm_joint_positions: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'fixed_arm':
            result = robot.set_fixed_arm_mode()
            print(f"set_fixed_arm_mode: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
            robot.stance()
        
        elif cmd == 'swing_arm':
            result = robot.set_auto_swing_arm_mode()
            print(f"set_auto_swing_arm_mode: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
            robot.stance()
        
        # 通用接口测试
        elif cmd == 'ext_arm':
            result = robot.set_external_control_arm_mode()
            print(f"set_external_control_arm_mode: {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)

        # ========== 控制器切换相关测试 ==========
        elif cmd == 'current_name':
            # 1.1 使用 get_current_controller_name 获取当前控制器名称
            current_name = controller_manager.get_current_controller_name()
            if current_name is not None:
                print(f"当前控制器名称 (get_current_controller_name): {current_name}")
            else:
                print("当前控制器名称尚未查询到 (返回 None)")

        elif cmd == 'next':
            # 2. 切换到下一个控制器
            print("\n" + "=" * 50)
            print("2. 切换到下一个控制器")
            print("=" * 50)
            result = controller_manager.switch_to_next_controller()
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            print(f"Current: {result.current_controller} (idx: {result.current_index})")
            print(f"Next: {result.target_controller} (idx: {result.target_index})")
            time.sleep(1.0)
            _, new_curr = get_controller_list(controller_manager)
            return True, new_curr

        elif cmd == 'prev':
            # 3. 切换到上一个控制器
            print("\n" + "=" * 50)
            print("3. 切换到上一个控制器")
            print("=" * 50)
            result = controller_manager.switch_to_previous_controller()
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            print(f"Current: {result.current_controller} (idx: {result.current_index})")
            print(f"Previous: {result.target_controller} (idx: {result.target_index})")
            time.sleep(1.0)
            _, new_curr = get_controller_list(controller_manager)
            return True, new_curr

        elif cmd == 'rl_mode':
            # 4. 设置 RL 切换模式
            enable = True
            if args:
                enable = args[0].lower() in ('true', '1', 'yes', 'on')
            print("\n" + "=" * 50)
            print(f"4. 设置 RL 切换模式 ({'直接切换' if enable else '条件切换'})")
            print("=" * 50)
            result = controller_manager.set_rl_switch_mode(enable=enable)
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            time.sleep(0.5)

        elif cmd == 'fall_down':
            # 5. 设置倒地状态
            enable = True
            if args:
                enable = args[0].lower() in ('true', '1', 'yes', 'on')
            print("\n" + "=" * 50)
            print(f"5. 设置倒地状态 ({'倒地' if enable else '站立'})")
            print("=" * 50)
            result = controller_manager.set_fall_down_state(enable=enable)
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            time.sleep(0.5)

        elif cmd == 'vmp':
            # 6. 切换到 VMP 控制器
            print("\n" + "=" * 50)
            print("6. 切换到 VMP 控制器")
            print("=" * 50)
            result = controller_manager.switch_to_vmp_controller()
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            time.sleep(1.0)
            _, new_curr = get_controller_list(controller_manager)
            return True, new_curr

        elif cmd == 'dance':
            # 7. 切换到舞蹈控制器
            print("\n" + "=" * 50)
            print("7. 切换到舞蹈控制器")
            print("=" * 50)
            result = controller_manager.switch_to_dance_controller()
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            time.sleep(1.0)
            _, new_curr = get_controller_list(controller_manager)
            return True, new_curr

        # ========== 通用接口测试 ==========
        elif cmd == 'mpc_mode':
            if not args:
                print("用法：mpc_mode <ArmOnly|BaseOnly|BaseArm>")
                return True, current_controller
            mode_str = args[0]
            if mode_str.lower() == 'armonly':
                mode = KuavoManipulationMpcCtrlMode.ArmOnly
            elif mode_str.lower() == 'baseonly':
                mode = KuavoManipulationMpcCtrlMode.BaseOnly
            elif mode_str.lower() == 'basearm':
                mode = KuavoManipulationMpcCtrlMode.BaseArm
            else:
                print("无效的模式，使用 ArmOnly")
                mode = KuavoManipulationMpcCtrlMode.ArmOnly
            result = robot.set_manipulation_mpc_mode(mode)
            print(f"set_manipulation_mpc_mode({mode_str}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
        
        elif cmd == 'mpc_flow':
            if not args:
                print("用法：mpc_flow <DirectToWbc|WithPlanning>")
                return True, current_controller
            flow_str = args[0]
            if flow_str.lower() == 'directtowbc':
                flow = KuavoManipulationMpcControlFlow.DirectToWbc
            elif flow_str.lower() == 'withplanning':
                flow = KuavoManipulationMpcControlFlow.WithPlanning
            else:
                print("无效的控制流，使用 DirectToWbc")
                flow = KuavoManipulationMpcControlFlow.DirectToWbc
            result = robot.set_manipulation_mpc_control_flow(flow)
            print(f"set_manipulation_mpc_control_flow({flow_str}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
        
        elif cmd == 'mpc_frame':
            if not args:
                print("用法：mpc_frame <WorldFrame|BaseFrame>")
                return True, current_controller
            frame_str = args[0]
            if frame_str.lower() == 'worldframe':
                frame = KuavoManipulationMpcFrame.WorldFrame
            elif frame_str.lower() == 'baseframe':
                frame = KuavoManipulationMpcFrame.BaseFrame
            else:
                print("无效的坐标系，使用 WorldFrame")
                frame = KuavoManipulationMpcFrame.WorldFrame
            result = robot.set_manipulation_mpc_frame(frame)
            print(f"set_manipulation_mpc_frame({frame_str}): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(0.5)
        
        elif cmd == 'end_pose':
            # end_pose [lx] [ly] [lz] [lw] [rx] [ry] [rz] [rw]
            # 默认值：左手 [0.3, 0.4, 0.9, 1.0], 右手 [0.3, -0.5, 1.0, 1.0]
            #end_pose 0.3 0.4 -0.9 1.0 0.3 -0.5 -1.0 1.0
            lx = float(args[0]) if len(args) > 0 else 0.3
            ly = float(args[1]) if len(args) > 1 else 0.4
            lz = float(args[2]) if len(args) > 2 else 0.9
            lw = float(args[3]) if len(args) > 3 else 1.0
            rx = float(args[4]) if len(args) > 4 else 0.3
            ry = float(args[5]) if len(args) > 5 else -0.5
            rz = float(args[6]) if len(args) > 6 else 1.0
            rw = float(args[7]) if len(args) > 7 else 1.0
            left_pose = KuavoPose(position=[lx, ly, lz], orientation=[0.0, 0.0, 0.0, lw])
            right_pose = KuavoPose(position=[rx, ry, rz], orientation=[0.0, 0.0, 0.0, rw])
            result = robot.control_robot_end_effector_pose(
                left_pose=left_pose,
                right_pose=right_pose,
                frame=KuavoManipulationMpcFrame.WorldFrame
            )
            print(f"control_robot_end_eff_pose(L:[{lx},{ly},{lz}], R:[{rx},{ry},{rz}]): {'✓ 成功' if result else '✗ 失败'}")
            time.sleep(1.0)
            robot.stance()
        
        elif cmd == 'base_arm_reach':
            # BaseArm 模式专用：触摸低位物体（ArmOnly 无法达到）
            # 演示躯干 + 手臂协调的必要性
            print("\n=== BaseArm 模式：触摸低位物体 ===")
            
            # 1. 设置 BaseArm 模式
            robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.BaseArm)
            robot.set_manipulation_mpc_frame(KuavoManipulationMpcFrame.WorldFrame)
            time.sleep(0.5)
            
            # 2. 躯干下降 15cm + 前移 10cm
            print("1. 躯干下降 + 前移...")
            robot.control_torso_pose(0.1, 0.0, -0.15, 0.0, 0.0, 0.0)
            time.sleep(1.5)
            
            # 3. 伸手臂到低位 (z=0.1m，接近地面)
            print("2. 伸手臂到低位 (z=0.1m)...")
            left_pose = KuavoPose(position=[0.5, 0.2, 0.15], orientation=[0.0, 0.0, 0.0, 1.0])
            right_pose = KuavoPose(position=[0.5, -0.2, 0.15], orientation=[0.0, 0.0, 0.0, 1.0])
            result = robot.control_robot_end_effector_pose(
                left_pose=left_pose,
                right_pose=right_pose,
                frame=KuavoManipulationMpcFrame.WorldFrame
            )
            print(f"   结果：{'✓ 成功 (BaseArm 才能达到此位置)' if result else '✗ 失败'}")
            time.sleep(2.0)
            
            # 4. 恢复
            print("3. 恢复站立...")
            robot.stance()
            robot.set_manipulation_mpc_mode(KuavoManipulationMpcCtrlMode.ArmOnly)
            print("=== 测试完成 ===\n")
        
        elif cmd == 'motor_param':
            success, motor_params = robot.get_motor_param()
            if success and motor_params:
                print(f"✓ 获取到 {len(motor_params)} 个电机参数")
            else:
                print("✗ 获取电机参数失败")
        
        elif cmd == 'base_pitch':
            if not args:
                print("用法：base_pitch <0|1>")
                return True, current_controller
            enable = int(args[0]) != 0
            success, msg = robot.enable_base_pitch_limit(enable=enable)
            print(f"enable_base_pitch_limit({enable}): {'✓ 成功' if success else '✗ 失败'}")
            time.sleep(0.5)
        
        elif cmd == 'collision':
            is_collision = robot.is_arm_collision()
            if is_collision:
                print("✗ 检测到手臂碰撞")
            else:
                print("✓ 正常 (无碰撞)")
        
        elif cmd in ('quit', 'exit'):
            print("退出程序...")
            return False, current_controller
        
        elif cmd == 'help':
            print_help()
        
        else:
            print(f"未知命令：{cmd}，输入 help 查看可用命令")
        
        return True, current_controller
    
    except Exception as e:
        print(f"执行命令时出错：{e}")
        return True, current_controller


def main():
    # 初始化 SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    controller_manager = KuavoRobotController()

    # 获取 controller 列表
    controller_names, current_controller = get_controller_list(controller_manager)
    print(f"\n可用 controllers: {controller_names}")
    print(f"当前 controller: {current_controller}")
    print("\n输入 help 查看可用命令，输入 quit 退出\n")

    initial_controller = current_controller

    try:
        while True:
            try:
                cmd_input = input(f"[{current_controller}]> ").strip()
            except EOFError:
                print("\n收到 EOF，退出程序")
                break
            
            if not cmd_input:
                continue
            
            should_continue, new_controller = execute_command(
                cmd_input, robot, controller_manager, current_controller
            )
            current_controller = new_controller
            
            if not should_continue:
                break
    
    except KeyboardInterrupt:
        print("\n测试被中断")
    
    finally:
        # 恢复到初始 controller
        if initial_controller and initial_controller != current_controller:
            print(f"\n恢复到初始 controller: {initial_controller}")
            switch_controller(controller_manager, initial_controller)

        # 回到安全状态
        robot.stance()
        print("\n已退出")


if __name__ == "__main__":
    main()
