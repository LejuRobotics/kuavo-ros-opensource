#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time
import sys

COMMAND_FILE = "/tmp/leju_claw_command.json"
STATUS_FILE = "/tmp/leju_claw_status.json"

def send_command(positions, velocities=None, torques=None):
    """发送控制指令到夹爪服务器"""
    if velocities is None:
        velocities = [0.0] * len(positions)
    if torques is None:
        torques = [0.0] * len(positions)
    
    command = {
        "positions": positions,
        "velocities": velocities,
        "torques": torques
    }
    
    try:
        with open(COMMAND_FILE, 'w') as f:
            json.dump(command, f, indent=2)
        print(f"[✓] 已发送控制指令: positions={positions}")
        return True
    except Exception as e:
        print(f"[✗] 发送指令失败: {e}")
        return False

def read_status():
    """读取夹爪服务器状态"""
    if not os.path.exists(STATUS_FILE):
        return None
    
    try:
        with open(STATUS_FILE, 'r') as f:
            status = json.load(f)
        return status
    except Exception as e:
        print(f"[✗] 读取状态失败: {e}")
        return None

def wait_for_ready(timeout=30):
    """等待夹爪服务器就绪"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        status = read_status()
        if status and status.get("status") == "ready":
            print(f"[✓] 夹爪服务器已就绪")
            return True
        time.sleep(0.1)
    
    print(f"[✗] 等待服务器就绪超时")
    return False

def move_to_position(positions, wait=True, timeout=60):
    """控制夹爪运动到指定位置"""
    if not send_command(positions):
        return False
    
    if not wait:
        return True
    
    # 等待命令文件被删除
    start_time = time.time()
    command_processed = False
    while time.time() - start_time < 2.0:
        if not os.path.exists(COMMAND_FILE):
            command_processed = True
            break
        time.sleep(0.05)
    
    # 等待状态变为running
    start_time = time.time()
    running_detected = False
    while time.time() - start_time < 3.0:
        status = read_status()
        if status:
            status_str = status.get("status", "")
            if status_str == "running":
                running_detected = True
                break
            elif status_str == "error":
                print(f"[✗] 运动失败: {status.get('message', '未知错误')}")
                return False
        time.sleep(0.1)
    
    # 等待运动完成
    start_time = time.time()
    last_status = "running" if running_detected else None
    
    while time.time() - start_time < timeout:
        status = read_status()
        if status:
            status_str = status.get("status", "")
            current_positions = status.get('positions', [])
            
            if status_str == "ready":
                if last_status == "running":
                    print(f"[✓] 夹爪已到达目标位置: {current_positions}")
                    return True
                
                # 检查位置是否接近目标位置
                if current_positions and len(current_positions) == len(positions):
                    all_reached = True
                    for i in range(len(positions)):
                        if abs(current_positions[i] - positions[i]) > 2.0:
                            all_reached = False
                            break
                    
                    if all_reached:
                        print(f"[✓] 夹爪已到达目标位置: {current_positions}")
                        return True
                    else:
                        # 位置还没到达，可能是旧状态，继续等待
                        # 但如果命令文件已被删除且等待时间较长，可能是真的完成了
                        elapsed = time.time() - start_time
                        if elapsed > 1.0 and command_processed:
                            # 等待超过1秒且命令已处理，可能是旧状态，继续等待
                            pass
            elif status_str == "error":
                print(f"[✗] 运动失败: {status.get('message', '未知错误')}")
                return False
            elif status_str == "running":
                last_status = "running"
        
        time.sleep(0.1)
    
    print(f"[✗] 等待运动完成超时")
    return False

def main():
    """示例用法"""
    print("=" * 60)
    print("LEJU 夹爪控制客户端")
    print("=" * 60)
    
    if not os.path.exists(STATUS_FILE):
        print("[!] 警告: 未检测到夹爪服务器状态文件")
        print("[!] 请确保 lejuclaw_server 程序正在运行")
        response = input("是否继续? (y/n): ")
        if response.lower() != 'y':
            return
    
    print("\n[→] 等待夹爪服务器就绪...")
    if not wait_for_ready():
        print("[✗] 服务器未就绪，退出")
        return
    
    print("\n" + "=" * 60)
    print("开始控制夹爪运动")
    print("=" * 60)
    
    print("\n[→] 运动到0%位置...")
    move_to_position([0.0, 0.0])
    time.sleep(1)
    
    print("\n[→] 运动到100%位置...")
    move_to_position([100.0, 100.0])
    time.sleep(1)
    
    print("\n[→] 运动到50%位置...")
    move_to_position([50.0, 50.0])
    time.sleep(1)
    
    print("\n[✓] 控制示例完成")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        try:
            positions = [float(x) for x in sys.argv[1:]]
            if len(positions) == 2:
                # 验证位置值范围
                if positions[0] < 0 or positions[0] > 100 or positions[1] < 0 or positions[1] > 100:
                    print("[✗] 错误: 位置值必须在0-100之间")
                    print("0为开爪，100为关爪")
                    sys.exit(1)
                
                print(f"[→] 控制夹爪运动: 左夹爪={positions[0]}%, 右夹爪={positions[1]}%")
                move_to_position(positions)
                
                # 进入持续输入循环
                print("\n" + "=" * 60)
                print("进入持续控制模式（按 Ctrl+C 退出）")
                print("=" * 60)
                
                while True:
                    try:
                        print("\n请输入左右夹爪的目标位置（0-100，0为开爪，100为关爪）")
                        left_input = input("左夹爪位置（0-100）：").strip()
                        right_input = input("右夹爪位置（0-100）：").strip()
                        
                        left_pos = float(left_input)
                        right_pos = float(right_input)
                        
                        if left_pos < 0 or left_pos > 100 or right_pos < 0 or right_pos > 100:
                            print("[✗] 错误: 位置值必须在0-100之间")
                            print("0为开爪，100为关爪")
                            continue
                        
                        print(f"[→] 控制夹爪运动: 左夹爪={left_pos}%, 右夹爪={right_pos}%")
                        move_to_position([left_pos, right_pos])
                        
                    except ValueError:
                        print("[✗] 错误: 位置值必须是数字，请重新输入")
                        continue
                    except KeyboardInterrupt:
                        print("\n[!] 收到退出信号，正在退出...")
                        break
            else:
                print("[✗] 错误: 需要提供2个位置值（左夹爪和右夹爪）")
                print("用法: python3 lejuclaw_controller.py <左夹爪位置> <右夹爪位置>")
                print("位置范围: 0-100 (0为开爪，100为关爪)")
                print("示例: python3 lejuclaw_controller.py 50.0 50.0")
                sys.exit(1)
        except ValueError:
            print("[✗] 错误: 位置值必须是数字")
            print("用法: python3 lejuclaw_controller.py <左夹爪位置> <右夹爪位置>")
            sys.exit(1)
        except KeyboardInterrupt:
            print("\n[!] 收到退出信号，正在退出...")
    else:
        main()
