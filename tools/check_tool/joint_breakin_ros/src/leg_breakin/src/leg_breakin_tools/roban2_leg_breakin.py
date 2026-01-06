#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import signal
from EcMasterConfig import EcMasterConfig

# 设置Python为无缓冲模式，确保实时输出
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.flush()
sys.stderr.flush()

# 全局停止标志
stop_program = False

# 信号处理函数
def signal_handler(signum, frame):
    """处理中断信号"""
    global stop_program
    print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 收到停止信号，正在安全停止Roban2腿部磨线程序...")
    stop_program = True
    # 创建停止信号文件，通知C++层停止
    try:
        with open("/tmp/leg_stop_signal", "w") as f:
            f.write(f"stop_signal_{time.time()}")
    except Exception as e:
        print(f"创建停止信号文件失败: {e}")
    
    # 立即退出程序
    print(f"{time.strftime('%H:%M:%S', time.localtime())} Roban2腿部磨线程序正在退出...")
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 检查是否有root权限
if os.geteuid() != 0:
    print("\033[31merror: 请使用root权限运行\033[0m")
    sys.exit(1)

# 获取当前脚本所在目录（而非工作目录）
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "build_lib/roban2"
target_path = os.path.join(script_dir, relative_path)
sys.path.append(target_path)

import ec_master_wrap
g_EcMasterConfig = EcMasterConfig()

# 获取机器人型号并设置到C++层
robot_type, _ = g_EcMasterConfig.get_robot_type_and_slave_num(g_EcMasterConfig.robot_version)
ec_master_wrap.set_robot_model(robot_type)

# 设置驱动器类型到C++层
ec_master_wrap.set_driver_type(g_EcMasterConfig.driver_type)
# 设置C++层的全局驱动器类型
ec_master_wrap.set_global_driver_type(g_EcMasterConfig.driver_type)
print(f"\033[1;33m机器人型号: {robot_type}, 驱动器类型: {g_EcMasterConfig.driver_type}\033[0m")

def main():
    # 注意：运行时长不再从stdin读取，而是由主程序通过ROS话题 /breakin/can_start_new_round 控制
    
    # 初始化编码器范围和命令参数
    # 注意：get_encoder_range 现在使用关节ID，而不是从站ID
    for joint_id in range(1, g_EcMasterConfig.slave_num+1):
        encoder_range = g_EcMasterConfig.get_encoder_range(joint_id)
        if encoder_range is not None:
            # 将关节ID转换为从站ID
            slave_id = g_EcMasterConfig.getSlaveIdByJointId(joint_id)
            if slave_id is not None:
                ec_master_wrap.setEncoderRange(slave_id, encoder_range)
    ec_master_wrap.set_command_args(g_EcMasterConfig.command_args)
    
    # 设置从站ID到关节ID的映射关系（如果需要）
    for slave_id, joint_id in g_EcMasterConfig.slave2joint.items():
        # 注意：如果C++层需要这个映射，需要实现setSlave2Joint函数
        # 目前C++层直接使用从站ID，所以这里暂时不需要
        pass

    # 判断是否需要检查手臂心跳
    # 根据运行模式决定：
    # 1. 单独腿部磨线：不需要检查手臂心跳
    # 2. 手腿一起磨线：需要检查手臂心跳
    # 
    # 检测方式：通过环境变量 CHECK_ARM_HEARTBEAT 明确指定
    # - 如果设置为 "false" 或 "0"，则不检查（单独腿部磨线模式）
    # - 如果设置为 "true" 或 "1"，则检查（手腿同步模式）
    # - 如果未设置，默认检查（安全模式，适用于手腿同步）
    check_arm_heartbeat_env = os.environ.get("CHECK_ARM_HEARTBEAT", "").lower()
    if check_arm_heartbeat_env in ["0", "false", "no"]:
        check_arm_heartbeat = False
        print("\033[1;33m模式：单独腿部磨线（不检查手臂心跳）\033[0m")
    else:
        # 默认检查手臂心跳（手腿同步模式或未明确指定时）
        check_arm_heartbeat = True
        if check_arm_heartbeat_env in ["1", "true", "yes"]:
            print("\033[1;33m模式：手腿同步磨线（检查手臂心跳）\033[0m")
        else:
            print("\033[1;33m模式：默认手腿同步模式（检查手臂心跳）\033[0m")
            print("\033[1;33m提示：如果是单独腿部磨线，请设置环境变量 CHECK_ARM_HEARTBEAT=false\033[0m")

    try:
        # 直接调用C++层实现的Roban2腿部磨线函数，所有逻辑都在C++层完成
        # C++层会阻塞直到运动完成并自动退出
        # C++层会处理心跳、同步和停止信号
        # 运行时长由主程序通过ROS话题 /breakin/can_start_new_round 控制
        success = ec_master_wrap.Roban2LegBreakin(check_arm_heartbeat)
        if not success:
            print("\033[1;31m✘ Roban2磨线运动失败或被中断\033[0m")
            return
        
        print("\033[1;32m✓ Roban2磨线运动完成\033[0m")
    except KeyboardInterrupt:
        print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 用户中断，正在安全停止...")
        # 创建停止信号文件
        try:
            with open("/tmp/leg_stop_signal", "w") as f:
                f.write(f"stop_signal_{time.time()}")
        except Exception as e:
            print(f"创建停止信号文件失败: {e}")
    finally:
        # 清理停止信号文件
        try:
            if os.path.exists("/tmp/leg_stop_signal"):
                os.remove("/tmp/leg_stop_signal")
        except Exception as e:
            print(f"清理停止信号文件失败: {e}")

if __name__ == "__main__":
    main()
