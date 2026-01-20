#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import signal
import threading
from EcMasterConfig import EcMasterConfig

# 设置Python为无缓冲模式，确保实时输出
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.flush()
sys.stderr.flush()

# 心跳文件路径
HEARTBEAT_FILE = "/tmp/leg_heartbeat"

# 全局停止标志
stop_program = threading.Event()

# 心跳写入函数
def write_heartbeat():
    """写入心跳文件"""
    # 如果收到停止信号，不再写入心跳
    if stop_program.is_set():
        return
    try:
        with open(HEARTBEAT_FILE, 'w') as f:
            f.write(f"{time.time()}\n")
            f.write(f"{time.strftime('%H:%M:%S', time.localtime())}\n")
            f.write(f"leg_moving\n")
    except Exception as e:
        print(f"写入心跳文件失败: {e}")

# 在程序启动时立即创建心跳文件
try:
    write_heartbeat()
    print(f"{time.strftime('%H:%M:%S', time.localtime())} 心跳文件已创建，程序已启动")
except Exception as e:
    print(f"{time.strftime('%H:%M:%S', time.localtime())} 创建心跳文件失败: {e}")

# 信号处理函数
def signal_handler(signum, frame):
    """处理中断信号"""
    print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 收到停止信号，正在安全停止Kuavo5腿部磨线程序...")
    stop_program.set()
    # 创建停止信号文件，通知C++层停止
    try:
        with open("/tmp/leg_stop_signal", "w") as f:
            f.write(f"stop_signal_{time.time()}")
    except Exception as e:
        print(f"创建停止信号文件失败: {e}")
    
    # 立即退出程序
    print(f"{time.strftime('%H:%M:%S', time.localtime())} Kuavo5腿部磨线程序正在退出...")
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
relative_path = "build/src"
target_path = os.path.join(script_dir, relative_path)
sys.path.append(target_path)

import ec_master_wrap
g_EcMasterConfig = EcMasterConfig()

# 获取机器人型号（仅用于显示）
robot_type, _ = g_EcMasterConfig.get_robot_type_and_slave_num(g_EcMasterConfig.robot_version)
print(f"\033[1;33m机器人型号: {robot_type}, 驱动器类型: {g_EcMasterConfig.driver_type}\033[0m")

def main():
    # 清理旧的腿部准备完成信号文件（如果存在）
    try:
        if os.path.exists("/tmp/leg_ready_signal"):
            os.remove("/tmp/leg_ready_signal")
            print(f"{time.strftime('%H:%M:%S', time.localtime())} 已清理旧的腿部准备完成信号文件")
    except Exception as e:
        print(f"{time.strftime('%H:%M:%S', time.localtime())} 清理旧的信号文件失败: {e}")
    
    # 从stdin读取运行时长
    try:
        duration_input = input().strip()
        total_duration = float(duration_input)
        if total_duration <= 0:
            print("\033[1;31m错误：运行时长必须大于0\033[0m")
            return
    except (ValueError, EOFError) as e:
        print(f"\033[1;31m错误：读取运行时长失败: {e}\033[0m")
        return
    
    # 初始化编码器范围和命令参数
    for i in range(1, g_EcMasterConfig.slave_num+1):
        encoder_range = g_EcMasterConfig.get_encoder_range(i)
        if encoder_range is not None:
            ec_master_wrap.setEncoderRange(i, encoder_range)
    ec_master_wrap.set_command_args(g_EcMasterConfig.command_args)

    for slave_id, joint_id in g_EcMasterConfig.slave2joint.items():
        ec_master_wrap.setSlave2Joint(slave_id, joint_id)

    # 每个动作持续时间（秒），参考 leg_breakin_tools 中的 1.0 秒
    motion_duration = 1.0
    
    print("正在执行15个电机动作组运动...")
    print("注意：动作序列定义在C++文件中（ec_master_wrap.cpp）")
    print(f"每个动作持续时间: {motion_duration}秒")
    print(f"总运行时长: {total_duration}秒（循环执行）")

    # 创建心跳监控线程
    heartbeat_stop = threading.Event()
    
    def heartbeat_thread():
        """心跳线程，每0.5秒写入心跳文件"""
        while not heartbeat_stop.is_set() and not stop_program.is_set():
            write_heartbeat()
            time.sleep(0.5)
    
    # 启动心跳线程
    heartbeat_thread_obj = threading.Thread(target=heartbeat_thread, daemon=True)
    heartbeat_thread_obj.start()
    
    # 注意：腿部准备完成信号文件由C++代码在开始读取电机位置时创建
    # 这里不需要手动创建，等待C++代码创建即可

    try:
        # 调用函数：15个电机动作组运动（使用动作序列，插值运动）
        # 动作序列定义在C++文件中（ec_master_wrap.cpp），模仿 leg_breakin_tools 的实现方式
        # 如需修改动作序列，请编辑 ec_master/src/Sharelib/User/ec_master_wrap.cpp 文件中的 motor_actions 数组
        success = ec_master_wrap.Motor15ActionGroup(motion_duration, total_duration)
        if not success:
            print("\033[1;31m✘ Kuavo5磨线运动失败或被中断\033[0m")
            return
        
        print("\033[1;32m✓ Kuavo5磨线运动完成\033[0m")
    except KeyboardInterrupt:
        print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 用户中断，正在安全停止...")
        stop_program.set()
        # 创建停止信号文件
        try:
            with open("/tmp/leg_stop_signal", "w") as f:
                f.write(f"stop_signal_{time.time()}")
        except Exception as e:
            print(f"创建停止信号文件失败: {e}")
    finally:
        # 停止心跳线程
        heartbeat_stop.set()
        heartbeat_thread_obj.join(timeout=2.0)
        
        # 清理停止信号文件
        try:
            if os.path.exists("/tmp/leg_stop_signal"):
                os.remove("/tmp/leg_stop_signal")
        except Exception as e:
            print(f"清理停止信号文件失败: {e}")

if __name__ == "__main__":
    main()
