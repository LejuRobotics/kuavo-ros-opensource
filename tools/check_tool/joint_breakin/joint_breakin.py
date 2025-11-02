#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo机器人手臂和腿部统一磨线启动脚本
统一启动Kuavo机器人的手臂磨线（RUIWO电机）和腿部磨线（EC_Master电机）
"""

import os
import sys
import subprocess
import threading
import time
import signal
import shutil
from pathlib import Path

class Colors:
    """终端颜色定义"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'

class KuavoUnifiedBreakin:
    def __init__(self):
        self.current_dir = Path(__file__).parent.absolute()
        # 项目根目录是当前目录的父目录的父目录的父目录
        self.project_root = self.current_dir.parent.parent.parent
        
        # 定义脚本路径
        # 手臂磨线脚本路径
        self.arm_breakin_script = self.current_dir / "arm_breakin" / "arm_breakin_roban2.sh"
        # 手臂电机校准脚本（motorevo_controller）
        self.motorevo_tool_sh = self.project_root / "tools" / "check_tool" / "motorevo_tool.sh"
        
        # 零点设置脚本路径
        # 添加arm_setzero.sh脚本路径（对应Hardware_tool.py中的arm_setzero()函数）
        self.arm_setzero_sh_script = self.current_dir.parent / "arm_setzero.sh"
        self.arm_setzero_sh_script_alt = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "arm_setzero.sh"
        
        self.arm_setzero_script = self.current_dir.parent / "ruiwo_zero_set.sh"
        # RUIWO零点设置脚本的两个可能路径
        self.ruiwo_zero_script = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        self.ruiwo_zero_script_alt = self.project_root / "installed" / "share" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        
        # 腿部磨线脚本路径（根据版本动态选择）
        self.leg_breakin_script_kuavo4 = self.current_dir / "leg_breakin_tools" / "kuavo4_leg_breakin.py"
        self.leg_breakin_script_kuavo5 = self.current_dir / "leg_breakin_tools" / "kuavo5_leg_breakin.py"
        self.leg_breakin_script_roban2 = self.current_dir / "leg_breakin_tools" / "roban2_leg_breakin.py"
        self.leg_breakin_script = None  # 将在检测版本后设置
        
        self.arm_process = None
        self.leg_process = None
        self.log_dir = Path("/tmp/kuavo_breakin_logs")
        self.robot_version = None
        self.robot_type = None  # 添加机器人类型属性
        
        # 心跳监控相关
        self.arm_heartbeat_file = "/tmp/arm_heartbeat"
        self.leg_heartbeat_file = "/tmp/leg_heartbeat"
        self.emergency_stop_file = "/tmp/emergency_stop"
        self.safety_monitor_thread = None
        self.stop_safety_monitor = threading.Event()
        
        # 全局停止标志
        self.stop_all_processes = threading.Event()
        
        # 注册全局信号处理器
        self._setup_global_signal_handlers()
    
    def _setup_global_signal_handlers(self):
        """设置全局信号处理器"""
        def global_signal_handler(signum, frame):
            self.print_colored(f"\n收到停止信号 (SIG{signum})，正在停止所有磨线程序...", Colors.YELLOW)
            self.stop_all_processes.set()
            
            # 停止安全监控
            self.stop_safety_monitor.set()
            
            # 停止手臂磨线进程
            if self.arm_process and self.arm_process.poll() is None:
                self.print_colored("正在停止手臂磨线进程...", Colors.YELLOW)
                try:
                    # 向整个进程组发送SIGTERM信号
                    os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
                    # 等待进程退出，最多等待5秒
                    try:
                        self.arm_process.wait(timeout=5)
                        self.print_colored("✓ 手臂磨线进程已停止", Colors.GREEN)
                    except subprocess.TimeoutExpired:
                        self.print_colored("手臂磨线进程未响应，强制终止...", Colors.RED)
                        os.killpg(os.getpgid(self.arm_process.pid), signal.SIGKILL)
                        self.print_colored("✓ 手臂磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"停止手臂磨线进程失败: {e}", Colors.RED)
            
            # 停止腿部磨线进程
            if self.leg_process and self.leg_process.poll() is None:
                self.print_colored("正在停止腿部磨线进程...", Colors.YELLOW)
                try:
                    # 直接强制终止，不等待响应
                    os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                    self.print_colored("✓ 腿部磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"强制终止腿部磨线进程失败: {e}", Colors.RED)
                    # 备用方案：使用subprocess的kill方法
                    try:
                        self.leg_process.kill()
                        self.print_colored("✓ 腿部磨线进程已强制终止（备用方案）", Colors.GREEN)
                    except Exception as e2:
                        self.print_colored(f"备用终止方案也失败: {e2}", Colors.RED)
            
            # 清理信号文件
            try:
                if os.path.exists("/tmp/leg_ready_signal"):
                    os.remove("/tmp/leg_ready_signal")
                if os.path.exists("/tmp/arm_disable_signal"):
                    os.remove("/tmp/arm_disable_signal")
                if os.path.exists("/tmp/arm_stop_signal"):
                    os.remove("/tmp/arm_stop_signal")
                if os.path.exists("/tmp/leg_stop_signal"):
                    os.remove("/tmp/leg_stop_signal")
                self.print_colored("✓ 已清理所有信号文件", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"清理信号文件失败: {e}", Colors.YELLOW)
            
            self.print_colored("所有磨线程序已停止，程序退出", Colors.GREEN)
            sys.exit(0)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, global_signal_handler)
        signal.signal(signal.SIGTERM, global_signal_handler)
        
    def get_robot_type_from_version(self, version):
        """根据版本号获取机器人类型"""
        if version is None:
            return None
            
        try:
            version_num = int(version)
            if 13 <= version_num <= 14:
                return "roban2"
            elif 40 <= version_num <= 49:
                return "kuavo4"
            elif 50 <= version_num <= 52:
                return "kuavo5"
            else:
                return None
        except (ValueError, TypeError):
            return None
        
    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
        
    def cleanup_logs(self):
        """清理日志目录"""
        try:
            if self.log_dir.exists():
                shutil.rmtree(self.log_dir)
                self.print_colored(f"✓ 已清理日志目录: {self.log_dir}", Colors.GREEN)
            else:
                self.print_colored("日志目录不存在，无需清理", Colors.YELLOW)
        except Exception as e:
            self.print_colored(f"清理日志目录失败: {e}", Colors.RED)
    
    def cleanup_heartbeat_files(self):
        """清理心跳文件"""
        heartbeat_files = [
            self.arm_heartbeat_file,
            self.leg_heartbeat_file,
            self.emergency_stop_file,
            "/tmp/leg_ready_signal",
            "/tmp/arm_disable_signal"
        ]
        
        for file_path in heartbeat_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
                    self.print_colored(f"✓ 已清理心跳文件: {file_path}", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"清理心跳文件失败 {file_path}: {e}", Colors.YELLOW)
    
    def start_safety_monitor(self):
        """启动安全监控线程"""
        if self.safety_monitor_thread and self.safety_monitor_thread.is_alive():
            return
            
        self.stop_safety_monitor.clear()
        self.safety_monitor_thread = threading.Thread(target=self._safety_monitor_loop, daemon=True)
        self.safety_monitor_thread.start()
        self.print_colored("✓ 安全监控已启动", Colors.GREEN)
    
    def stop_safety_monitor(self):
        """停止安全监控线程"""
        if self.safety_monitor_thread and self.safety_monitor_thread.is_alive():
            self.stop_safety_monitor.set()
            self.safety_monitor_thread.join(timeout=2.0)
            self.print_colored("✓ 安全监控已停止", Colors.GREEN)
    
    def _safety_monitor_loop(self):
        """安全监控主循环"""
        self.print_colored("心跳监控：等待第一轮动作开始...", Colors.YELLOW)
        
        # 等待第一轮动作开始（大约15秒后开始监控）
        time.sleep(15.0)
        
        self.print_colored("第一轮动作开始，启动心跳监控...", Colors.GREEN)
        
        # 心跳超时阈值（秒）
        HEARTBEAT_TIMEOUT = 4.0
        # 检查间隔（秒）
        CHECK_INTERVAL = 1.0
        
        # 记录上次心跳时间，用于连续超时判断
        arm_heartbeat_warning_count = 0  # 手臂连续警告次数
        leg_heartbeat_warning_count = 0  # 腿部连续警告次数
        
        # 持续监控直到程序结束
        while not self.stop_safety_monitor.is_set():
            time.sleep(CHECK_INTERVAL)
            
            # 检查紧急停止信号
            if os.path.exists(self.emergency_stop_file):
                self.print_colored("❌ 检测到紧急停止信号！", Colors.RED)
                try:
                    with open(self.emergency_stop_file, 'r') as f:
                        content = f.read()
                        self.print_colored(f"紧急停止文件内容: {content}", Colors.YELLOW)
                except Exception as e:
                    self.print_colored(f"读取紧急停止文件失败: {e}", Colors.YELLOW)
                self._emergency_stop()
                break
            
            # 检查是否有arm_disable_signal文件
            if os.path.exists("/tmp/arm_disable_signal"):
                self.print_colored("⚠️ 检测到手臂失能信号文件存在，这会导致腿部停止运动", Colors.YELLOW)
                try:
                    with open("/tmp/arm_disable_signal", 'r') as f:
                        content = f.read()
                        self.print_colored(f"手臂失能信号内容: {content}", Colors.YELLOW)
                except Exception as e:
                    self.print_colored(f"读取手臂失能信号失败: {e}", Colors.YELLOW)
            
            # 检查进程状态
            arm_running = self.arm_process and self.arm_process.poll() is None
            leg_running = self.leg_process and self.leg_process.poll() is None
            
            # 如果两个进程都已结束，停止监控
            if not arm_running and not leg_running:
                self.print_colored("✓ 两个进程都已结束，停止心跳监控", Colors.GREEN)
                break
            
            # 如果任一进程结束，但另一个还在运行，立即触发停止
            if not arm_running and leg_running:
                self.print_colored("❌ 检测到手臂进程已结束，但腿部进程仍在运行！", Colors.RED)
                self.print_colored("正在发送停止信号给腿部磨线程序...", Colors.YELLOW)
                self._send_stop_signals()
                self._emergency_stop()
                break
            elif arm_running and not leg_running:
                self.print_colored("❌ 检测到腿部进程已结束，但手臂进程仍在运行！", Colors.RED)
                self.print_colored("正在发送停止信号给手臂磨线程序...", Colors.YELLOW)
                self._send_stop_signals()
                self._emergency_stop()
                break
            
            # 检查心跳超时（只有在进程还在运行时才检查）
            heartbeat_issue = False
            issue_message = ""
            
            # 检查手臂心跳
            if arm_running:
                if os.path.exists(self.arm_heartbeat_file):
                    try:
                        stat = os.stat(self.arm_heartbeat_file)
                        time_diff = time.time() - stat.st_mtime
                        if time_diff > HEARTBEAT_TIMEOUT:
                            # 连续多次超时认为是问题
                            arm_heartbeat_warning_count += 1
                            if arm_heartbeat_warning_count >= 2:  # 连续2次检查都超时停止
                                heartbeat_issue = True
                                issue_message = f"手臂心跳超时（{time_diff:.1f}秒未更新，连续{arm_heartbeat_warning_count}次检查）"
                        else:
                            arm_heartbeat_warning_count = 0  # 心跳正常，重置计数
                    except Exception as e:
                        self.print_colored(f"检查手臂心跳文件失败: {e}", Colors.YELLOW)
                else:
                    # 如果进程运行但心跳文件不存在，可能刚开始或心跳写入有问题
                    # 给容错时间
                    arm_heartbeat_warning_count += 1
                    if arm_heartbeat_warning_count >= 2:  # 连续2次检查都没有心跳文件认为有问题
                        heartbeat_issue = True
                        issue_message = "手臂心跳文件不存在，连续多次检查未创建"
            
            # 检查腿部心跳
            if leg_running:
                if os.path.exists(self.leg_heartbeat_file):
                    try:
                        stat = os.stat(self.leg_heartbeat_file)
                        time_diff = time.time() - stat.st_mtime
                        if time_diff > HEARTBEAT_TIMEOUT:
                            leg_heartbeat_warning_count += 1
                            if leg_heartbeat_warning_count >= 2:
                                heartbeat_issue = True
                                if issue_message:
                                    issue_message += f"；腿部心跳超时（{time_diff:.1f}秒未更新，连续{leg_heartbeat_warning_count}次检查）"
                                else:
                                    issue_message = f"腿部心跳超时（{time_diff:.1f}秒未更新，连续{leg_heartbeat_warning_count}次检查）"
                        else:
                            leg_heartbeat_warning_count = 0  # 心跳正常，重置计数
                    except Exception as e:
                        self.print_colored(f"检查腿部心跳文件失败: {e}", Colors.YELLOW)
                else:
                    leg_heartbeat_warning_count += 1
                    if leg_heartbeat_warning_count >= 2:
                        heartbeat_issue = True
                        if issue_message:
                            issue_message += "；腿部心跳文件不存在，连续多次检查未创建"
                        else:
                            issue_message = "腿部心跳文件不存在，连续多次检查未创建"
            
            # 如果检测到心跳问题，立即停止（需要连续多次超时）
            if heartbeat_issue:
                self.print_colored(f"❌ 检测到心跳异常：{issue_message}", Colors.RED)
                self.print_colored("正在发送停止信号给手臂和腿部磨线程序...", Colors.YELLOW)
                self._send_stop_signals()
                self._emergency_stop()
                break
            elif arm_heartbeat_warning_count > 0 or leg_heartbeat_warning_count > 0:
                # 有警告但还没到停止阈值，只记录日志（每2次警告打印一次）
                total_warnings = arm_heartbeat_warning_count + leg_heartbeat_warning_count
                if total_warnings == 1 or total_warnings % 2 == 0:
                    self.print_colored(f"⚠️ 心跳检查警告：手臂{arm_heartbeat_warning_count}次，腿部{leg_heartbeat_warning_count}次，继续监控...", Colors.YELLOW)
    
    def _send_stop_signals(self):
        """发送停止信号给手臂和腿部磨线程序"""
        try:
            # 创建手臂停止信号文件
            if self.arm_process and self.arm_process.poll() is None:
                try:
                    with open("/tmp/arm_stop_signal", "w") as f:
                        f.write(f"stop_signal_{time.time()}\n")
                    self.print_colored("✓ 已发送停止信号给手臂磨线程序", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"发送手臂停止信号失败: {e}", Colors.RED)
            
            # 创建腿部停止信号文件
            if self.leg_process and self.leg_process.poll() is None:
                try:
                    with open("/tmp/leg_stop_signal", "w") as f:
                        f.write(f"stop_signal_{time.time()}\n")
                    self.print_colored("✓ 已发送停止信号给腿部磨线程序", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"发送腿部停止信号失败: {e}", Colors.RED)
        except Exception as e:
            self.print_colored(f"发送停止信号时出错: {e}", Colors.RED)
    
    def _emergency_stop(self):
        """紧急停止所有电机"""
        self.print_colored("🚨 执行紧急停止程序...", Colors.RED)
        
        try:
            # 创建紧急停止信号文件
            with open(self.emergency_stop_file, 'w') as f:
                f.write(f"emergency_stop_{time.time()}")
            
            # 检查进程是否还在运行
            arm_running = self.arm_process and self.arm_process.poll() is None
            leg_running = self.leg_process and self.leg_process.poll() is None
            
            # 停止所有磨线进程
            if arm_running:
                self.print_colored("停止手臂磨线进程...", Colors.YELLOW)
                self.arm_process.terminate()
                try:
                    self.arm_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.arm_process.kill()
            else:
                self.print_colored("手臂磨线进程已结束", Colors.YELLOW)
            
            if leg_running:
                self.print_colored("停止腿部磨线进程...", Colors.YELLOW)
                # 直接强制终止，不等待响应
                try:
                    # 向整个进程组发送SIGKILL信号，立即强制终止
                    os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                    self.print_colored("✓ 腿部磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"强制终止腿部磨线进程失败: {e}", Colors.RED)
                    # 备用方案：使用subprocess的kill方法
                    try:
                        self.leg_process.kill()
                        self.print_colored("✓ 腿部磨线进程已强制终止（备用方案）", Colors.GREEN)
                    except Exception as e2:
                        self.print_colored(f"备用终止方案也失败: {e2}", Colors.RED)
            else:
                self.print_colored("腿部磨线进程已结束", Colors.YELLOW)
            
            # 清理信号文件
            self._cleanup_signal_files()
            
            if arm_running or leg_running:
                self.print_colored("❌ 紧急停止完成，所有电机已停止", Colors.RED)
            else:
                self.print_colored("✓ 程序正常结束", Colors.GREEN)
            
        except Exception as e:
            self.print_colored(f"紧急停止过程中出错: {e}", Colors.RED)
    
    def _cleanup_signal_files(self):
        """清理所有信号文件"""
        signal_files = [
            self.arm_heartbeat_file,
            self.leg_heartbeat_file,
            self.emergency_stop_file,
            "/tmp/leg_ready_signal",
            "/tmp/arm_disable_signal"
        ]
        
        for file_path in signal_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                self.print_colored(f"清理信号文件 {file_path} 失败: {e}", Colors.YELLOW)
        
    def get_robot_version(self):
        """读取机器人的版本号"""
        home_dir = os.path.expanduser('/home/lab/')
        bashrc_path = os.path.join(home_dir, '.bashrc')

        if os.path.exists(bashrc_path):
            with open(bashrc_path, 'r') as file:
                lines = file.readlines()
            for line in reversed(lines):
                line = line.strip()
                if line.startswith("export ROBOT_VERSION=") and "#" not in line:
                    version = line.split("=")[1].strip()
                    self.print_colored(f"---------- 检测到 ROBOT_VERSION = {version} ----------", Colors.CYAN)
                    return version
        self.print_colored("警告：ROBOT_VERSION 未找到或无效，需要手动选择版本", Colors.YELLOW)
        return None
    
    def detect_leg_script_version(self):
        """根据机器人版本检测并选择腿部磨线脚本"""
        self.robot_version = self.get_robot_version()
        
        if self.robot_version is None:
            # 手动选择版本
            self.print_colored("无法自动检测机器人版本，请手动选择：", Colors.YELLOW)
            print("1. Kuavo4版本 (40-49)")
            print("2. Kuavo5版本 (50-52)")
            print("3. Roban2版本 (13-14)")
            while True:
                choice = input("请输入选项（1/2/3）：").strip()
                if choice == '1':
                    self.leg_breakin_script = self.leg_breakin_script_kuavo4
                    self.robot_type = "kuavo4"
                    self.print_colored("已选择 Kuavo4 腿部磨线脚本", Colors.GREEN)
                    return True
                elif choice == '2':
                    self.leg_breakin_script = self.leg_breakin_script_kuavo5
                    self.robot_type = "kuavo5"
                    self.print_colored("已选择 Kuavo5 腿部磨线脚本", Colors.GREEN)
                    return True
                elif choice == '3':
                    self.leg_breakin_script = self.leg_breakin_script_roban2
                    self.robot_type = "roban2"
                    self.print_colored("已选择 Roban2 腿部磨线脚本", Colors.GREEN)
                    return True
                else:
                    self.print_colored("输入无效，请重新选择！", Colors.RED)
        else:
            # 根据版本号自动选择
            self.robot_type = self.get_robot_type_from_version(self.robot_version)
            
            if self.robot_type == "roban2":
                self.leg_breakin_script = self.leg_breakin_script_roban2
                self.print_colored(f"根据版本 {self.robot_version} 自动选择 Roban2 腿部磨线脚本", Colors.GREEN)
            elif self.robot_type == "kuavo4":
                self.leg_breakin_script = self.leg_breakin_script_kuavo4
                self.print_colored(f"根据版本 {self.robot_version} 自动选择 Kuavo4 腿部磨线脚本", Colors.GREEN)
            elif self.robot_type == "kuavo5":
                self.leg_breakin_script = self.leg_breakin_script_kuavo5
                self.print_colored(f"根据版本 {self.robot_version} 自动选择 Kuavo5 腿部磨线脚本", Colors.GREEN)
            else:
                self.print_colored(f"版本 {self.robot_version} 不在支持范围内，请手动选择", Colors.YELLOW)
                return self.detect_leg_script_version()  # 递归调用手动选择
            
        
        return True
        
    def check_root_permission(self):
        """检查root权限"""
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 kuavo_joint_breakin.py", Colors.YELLOW)
            sys.exit(1)
            
    def check_scripts_exist(self):
        """检查脚本文件是否存在"""
        self.print_colored("检查脚本文件...", Colors.YELLOW)
        
        if not self.arm_breakin_script.exists():
            self.print_colored(f"错误：手臂磨线脚本不存在: {self.arm_breakin_script}", Colors.RED)
            sys.exit(1)
            
        if self.arm_setzero_sh_script.exists():
            self.print_colored(f"✓ 找到arm_setzero.sh脚本: {self.arm_setzero_sh_script}", Colors.GREEN)
        elif self.arm_setzero_sh_script_alt.exists():
            self.print_colored(f"✓ 找到arm_setzero.sh脚本: {self.arm_setzero_sh_script_alt}", Colors.GREEN)
        else:
            self.print_colored(f"警告：arm_setzero.sh脚本不存在，将跳过此步骤", Colors.YELLOW)
            self.print_colored(f"  查找路径1: {self.arm_setzero_sh_script}", Colors.YELLOW)
            self.print_colored(f"  查找路径2: {self.arm_setzero_sh_script_alt}", Colors.YELLOW)
            
        if not self.arm_setzero_script.exists():
            self.print_colored(f"错误：手臂零点设置脚本不存在: {self.arm_setzero_script}", Colors.RED)
            sys.exit(1)
            
        # 检查RUIWO零点设置脚本（检查两个可能的路径）
        if self.ruiwo_zero_script.exists():
            self.print_colored(f"✓ 找到RUIWO零点设置脚本: {self.ruiwo_zero_script}", Colors.GREEN)
        elif self.ruiwo_zero_script_alt.exists():
            self.print_colored(f"✓ 找到RUIWO零点设置脚本: {self.ruiwo_zero_script_alt}", Colors.GREEN)
            # 更新为实际存在的路径
            self.ruiwo_zero_script = self.ruiwo_zero_script_alt
        else:
            self.print_colored(f"错误：RUIWO零点设置脚本不存在", Colors.RED)
            self.print_colored(f"  查找路径1: {self.ruiwo_zero_script}", Colors.RED)
            self.print_colored(f"  查找路径2: {self.ruiwo_zero_script_alt}", Colors.RED)
            sys.exit(1)
            
        # 检查腿部磨线脚本（在版本检测后）
        if self.leg_breakin_script is None:
            self.print_colored("错误：腿部磨线脚本未选择，请先执行版本检测", Colors.RED)
            sys.exit(1)
            
        if not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            sys.exit(1)
            
        self.print_colored(f"✓ 手臂磨线脚本: {self.arm_breakin_script}", Colors.GREEN)
        self.print_colored(f"✓ 手臂零点设置脚本: {self.arm_setzero_script}", Colors.GREEN)
        self.print_colored(f"✓ RUIWO零点设置脚本: {self.ruiwo_zero_script}", Colors.GREEN)
        self.print_colored(f"✓ 腿部磨线脚本: {self.leg_breakin_script}", Colors.GREEN)
        print()
        
    def run_arm_zero_setup(self):
        """运行手臂零点设置"""
        self.print_colored("开始执行手臂零点设置...", Colors.GREEN)
        self.print_colored("注意：请确保手臂已摆正到零点位置", Colors.YELLOW)
        
        try:
            # 步骤1：执行arm_setzero.sh脚本（对应Hardware_tool.py中的arm_setzero()函数）
            arm_setzero_sh_script = None
            if self.arm_setzero_sh_script.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script
            elif self.arm_setzero_sh_script_alt.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script_alt
                
            if arm_setzero_sh_script:
                self.print_colored("步骤1：执行arm_setzero.sh脚本...", Colors.BLUE)
                result0 = subprocess.run(
                    ["bash", str(arm_setzero_sh_script)],
                    text=True
                )
                
                if result0.returncode != 0:
                    self.print_colored("arm_setzero.sh脚本执行失败", Colors.RED)
                    return False
                else:
                    self.print_colored("✓ arm_setzero.sh脚本执行完成", Colors.GREEN)
            else:
                self.print_colored("跳过arm_setzero.sh脚本（脚本不存在）", Colors.YELLOW)
            
            self.print_colored("步骤2：执行手臂零点设置...", Colors.BLUE)
            result1 = subprocess.run(
                ["bash", str(self.arm_setzero_script)],
                cwd=str(self.arm_setzero_script.parent),
                text=True
            )
            
            if result1.returncode != 0:
                self.print_colored("手臂零点设置失败", Colors.RED)
                return False
                
            self.print_colored("步骤3：执行RUIWO零点设置...", Colors.BLUE)
            result2 = subprocess.run(
                ["bash", str(self.ruiwo_zero_script)],
                cwd=str(self.ruiwo_zero_script.parent),
                text=True
            )
            
            if result2.returncode != 0:
                self.print_colored("RUIWO零点设置失败", Colors.RED)
                return False
                
            self.print_colored("✓ 手臂零点设置完成", Colors.GREEN)
            return True
            
        except Exception as e:
            self.print_colored(f"手臂零点设置出错: {e}", Colors.RED)
            return False
        
    def run_arm_breakin(self):
        """运行手臂磨线"""
        self.print_colored("启动手臂磨线...", Colors.GREEN)
        self.print_colored("注意：手臂磨线使用RUIWO电机控制", Colors.YELLOW)
        
        # 版本14：跳过零点设置，改为先执行电机校准 --cali
        if str(self.robot_version) == "14":
            self.print_colored("版本14：跳过手臂零点设置步骤（步骤1/2/3）", Colors.CYAN)
            # 执行 motorevo_tool.sh --cali
            try:
                if not self.motorevo_tool_sh.exists():
                    self.print_colored(f"错误：未找到校准脚本 {self.motorevo_tool_sh}", Colors.RED)
                    return 1
                self.print_colored("开始执行电机校准 (--cali)...", Colors.BLUE)
                result_cali = subprocess.run(
                    ["bash", str(self.motorevo_tool_sh), "--cali"],
                    cwd=str(self.motorevo_tool_sh.parent),
                    text=True
                )
                if result_cali.returncode != 0:
                    self.print_colored("电机校准失败 (--cali)", Colors.RED)
                    return 1
                self.print_colored("✓ 电机校准完成", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"执行电机校准出错: {e}", Colors.RED)
                return 1
        else:
            self.print_colored("在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
            if not self.run_arm_zero_setup():
                self.print_colored("零点设置失败，无法继续执行手臂磨线", Colors.RED)
                return 1
        
        print()
        self.print_colored("手臂磨线测试时长：", Colors.YELLOW)
        arm_min_duration = 15.0
        while True:
            try:
                arm_duration_input = input(f"请输入手臂磨线测试时长（秒），最少{arm_min_duration}秒: ").strip()
                if arm_duration_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                arm_duration = int(arm_duration_input)
                if arm_duration <= 0:
                    self.print_colored("测试时长必须大于0，请重新输入", Colors.RED)
                    continue
                if arm_duration < arm_min_duration:
                    self.print_colored(f"错误：测试时长必须至少{arm_min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                arm_rounds = int(arm_duration // arm_min_duration)
                self.print_colored(f"手臂磨线将完整运行 {arm_rounds} 轮（每轮{arm_min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        # 单独运行手臂磨线时，立即开始，不等待腿部信号
        self.print_colored("开始执行手臂磨线！", Colors.GREEN)
        self.print_colored("提示：在执行过程中，输入 'q' 并回车，可以在当前完整动作周期完成后提前结束程序。", Colors.CYAN)
        
        try:
            # 使用Popen来支持真正的交互式输入
            self.print_colored("正在启动手臂磨线程序...", Colors.BLUE)
            self.print_colored("注意：程序启动后，您可以直接与子进程交互", Colors.YELLOW)
            self.print_colored("如果遇到电机失能等问题，请直接输入 'c' 并按回车来失能电机", Colors.CYAN)
            
            # 统一使用arm_breakin_roban2.sh脚本
            cmd = ["bash", str(self.arm_breakin_script)]
            cwd = str(self.arm_breakin_script.parent)

            # 使用Popen启动子进程，保持交互式环境
            process = subprocess.Popen(
                cmd,
                cwd=cwd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=0,  # 无缓冲
                env=dict(os.environ, PYTHONUNBUFFERED='1')  # 确保Python子进程也无缓冲
            )
            
            # 发送初始输入（测试时长）
            process.stdin.write(str(arm_duration) + "\n")
            process.stdin.flush()
            
            # 实时读取输出并转发到终端
            import threading
            import queue
            import signal
            
            output_queue = queue.Queue()
            stop_flag = threading.Event()
            
            def read_output():
                while not stop_flag.is_set():
                    try:
                        line = process.stdout.readline()
                        if not line and process.poll() is not None:
                            break
                        if line:
                            output_queue.put(line)
                    except Exception:
                        break
            
            # 启动输出读取线程
            output_thread = threading.Thread(target=read_output, daemon=False)
            output_thread.start()
            
            def signal_handler(signum, frame):
                self.print_colored("收到中断信号，正在安全停止...", Colors.YELLOW)
                stop_flag.set()
                if process.poll() is None:
                    try:
                        process.terminate()
                        # 等待进程退出，最多等待3秒
                        try:
                            process.wait(timeout=3)
                            self.print_colored("✓ 手臂磨线进程已停止", Colors.GREEN)
                        except subprocess.TimeoutExpired:
                            self.print_colored("手臂磨线进程未响应，强制终止...", Colors.RED)
                            process.kill()
                            self.print_colored("✓ 手臂磨线进程已强制终止", Colors.GREEN)
                    except Exception as e:
                        self.print_colored(f"停止手臂磨线进程失败: {e}", Colors.RED)
            
            # 注册信号处理器
            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)
            
            try:
                # 主循环：处理输出和用户输入
                while not stop_flag.is_set():
                    # 检查进程是否结束
                    if process.poll() is not None:
                        break
                    
                    # 处理输出
                    try:
                        while True:
                            line = output_queue.get_nowait()
                            print(line.rstrip())
                    except queue.Empty:
                        pass
                    
                    # 检查用户输入（非阻塞）
                    import select
                    import sys
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        try:
                            user_input = input()
                            if user_input.strip().lower() == 'q':
                                self.print_colored("用户请求退出，完成本轮动作后停止...", Colors.YELLOW)
                                stop_flag.set()
                                process.stdin.write("q\n")
                                process.stdin.flush()
                                break
                            else:
                                # 将用户输入转发给子进程
                                process.stdin.write(user_input + "\n")
                                process.stdin.flush()
                        except EOFError:
                            break
                
                # 等待进程结束
                return_code = process.wait()
                return return_code
                
            finally:
                # 清理资源
                stop_flag.set()
                if output_thread.is_alive():
                    output_thread.join(timeout=2.0)
                # 恢复默认信号处理器
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                signal.signal(signal.SIGTERM, signal.SIG_DFL)
            
        except Exception as e:
            self.print_colored(f"手臂磨线运行出错: {e}", Colors.RED)
            return 1
            
    def check_and_compile_leg_breakin(self):
        """检查腿部磨线脚本是否存在"""
        if self.leg_breakin_script is None:
            self.print_colored("错误：腿部磨线脚本未选择，请先执行版本检测", Colors.RED)
            return False
            
        if not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            return False
        
        # 检查leg_breakin_tools的依赖，根据机器人类型选择编译目录
        ec_master_dir = self.leg_breakin_script.parent
        if self.robot_type:
            build_dir = ec_master_dir / "build_lib" / self.robot_type
        else:
            pass
        ec_master_so = build_dir / "ec_master_wrap.so"
        
        if not build_dir.exists() or not ec_master_so.exists():
            self.print_colored("检测到EC_Master工具未编译，开始自动编译...", Colors.YELLOW)
            self.print_colored("这可能需要数十秒时间，请耐心等待...", Colors.CYAN)
            
            try:
                build_dir.mkdir(exist_ok=True)
                
                self.print_colored("步骤1：执行cmake配置...", Colors.BLUE)
                cmake_result = subprocess.run(
                    ["cmake", ".."],
                    cwd=str(build_dir),
                    capture_output=True,
                    text=True
                )
                
                if cmake_result.returncode != 0:
                    self.print_colored(f"cmake配置失败: {cmake_result.stderr}", Colors.RED)
                    return False
                
                self.print_colored("步骤2：执行make编译...", Colors.BLUE)
                make_result = subprocess.run(
                    ["make"],
                    cwd=str(build_dir),
                    capture_output=True,
                    text=True
                )
                
                if make_result.returncode != 0:
                    self.print_colored(f"make编译失败: {make_result.stderr}", Colors.RED)
                    return False
                
                if not ec_master_so.exists():
                    self.print_colored("编译完成但未找到ec_master_wrap.so文件", Colors.RED)
                    return False
                
                self.print_colored("✓ EC_Master工具编译成功", Colors.GREEN)
                return True
                
            except Exception as e:
                self.print_colored(f"编译过程中出错: {e}", Colors.RED)
                return False
        else:
            self.print_colored("✓ EC_Master工具已编译，无需重新编译", Colors.GREEN)
            return True

    def run_leg_breakin(self):
        """运行腿部磨线"""
        self.print_colored("启动腿部磨线...", Colors.GREEN)
        self.print_colored("注意：腿部磨线使用EC_Master电机控制", Colors.YELLOW)
        
        if not self.check_and_compile_leg_breakin():
            self.print_colored("腿部磨线脚本编译失败，无法继续执行", Colors.RED)
            return 1
        
        print()
        self.print_colored("腿部磨线参数：", Colors.YELLOW)
        min_duration = 15.0
        while True:
            try:
                duration_input = input(f"请输入腿部磨线运行时长（秒），最少{min_duration}秒: ").strip()
                if duration_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                duration = float(duration_input)
                if duration <= 0:
                    self.print_colored("运行时长必须大于0，请重新输入", Colors.RED)
                    continue
                if duration < min_duration:
                    self.print_colored(f"错误：运行时长必须至少{min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                leg_rounds = int(duration // min_duration)
                self.print_colored(f"腿部磨线将完整运行 {leg_rounds} 轮（每轮{min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的数字", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        try:
            leg_script_dir = self.leg_breakin_script.parent
            # leg_breakin_tools现在直接执行磨线功能，只需要输入时间
            result = subprocess.run(
                ["python3", str(self.leg_breakin_script)],
                cwd=str(leg_script_dir),
                input=f"{duration}\n",  # 只输入时间
                text=True
            )
            return result.returncode
        except Exception as e:
            self.print_colored(f"腿部磨线运行出错: {e}", Colors.RED)
            return 1
            
    def get_user_inputs(self):
        """获取用户输入参数"""
        print()
        self.print_colored("=" * 20, Colors.CYAN)
        self.print_colored("      参数配置", Colors.CYAN)
        self.print_colored("=" * 20, Colors.CYAN)
        
        print()
        self.print_colored("磨线测试时长：", Colors.YELLOW)
        min_duration = 15.0
        while True:
            try:
                duration_input = input(f"请输入磨线测试时长（秒），最少{min_duration}秒: ").strip()
                if duration_input.lower() == 'q':
                    return None, None
                duration = int(duration_input)
                if duration <= 0:
                    self.print_colored("测试时长必须大于0，请重新输入", Colors.RED)
                    continue
                if duration < min_duration:
                    self.print_colored(f"错误：测试时长必须至少{min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                rounds = int(duration // min_duration)
                self.print_colored(f"磨线将完整运行 {rounds} 轮（每轮{min_duration}秒）", Colors.CYAN)
                self.print_colored(f"手臂和腿部将使用相同的时长：{duration} 秒", Colors.GREEN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return None, None
        
        print()
        self.print_colored("=" * 40, Colors.CYAN)
        self.print_colored("          轮数计算总结", Colors.CYAN)
        self.print_colored("=" * 40, Colors.CYAN)
        self.print_colored(f"磨线时长：{duration:.1f} 秒 ÷ 15.0 秒/轮 = {rounds} 轮", Colors.GREEN)
        self.print_colored(f"手臂和腿部将同步运行 {rounds} 轮", Colors.GREEN)
        self.print_colored("=" * 40, Colors.CYAN)
        
        return duration, duration
        
    def run_arm_breakin_background(self, log_file, arm_duration, wait_for_leg_signal=True):
        """在后台运行手臂磨线"""
        def delayed_arm_breakin():
            log_file_handle = None
            try:
                sync_signal_file = "/tmp/leg_ready_signal"
                
                # 打开日志文件，保持打开状态直到进程结束
                log_file_handle = open(log_file, 'a', buffering=1)
                
                # 先写入日志信息
                if wait_for_leg_signal:
                    log_file_handle.write("等待腿部准备完成信号（手臂磨线进程已启动，正在初始化...）...\n")
                    log_file_handle.flush()
                else:
                    log_file_handle.write("手臂磨线立即开始执行...\n")
                    log_file_handle.flush()
                
                # 执行手臂磨线 - 先启动进程，让它开始初始化（CAN连接、使能电机等）
                input_pipe = subprocess.PIPE
                
                # 统一使用arm_breakin_roban2.sh脚本
                cmd = ["bash", str(self.arm_breakin_script)]
                if wait_for_leg_signal:
                    # 同时运行时，添加 --wait-for-leg 参数，强制等待腿部信号
                    cmd.append("--wait-for-leg")
                cwd = str(self.arm_breakin_script.parent)

                # 启动进程，将stdout和stderr重定向到日志文件
                # 注意：log_file_handle需要保持打开，直到进程结束
                self.arm_process = subprocess.Popen(
                    cmd,
                    cwd=cwd,
                    stdout=log_file_handle,
                    stderr=subprocess.STDOUT,
                    stdin=input_pipe,
                    text=True,
                    preexec_fn=os.setsid  # 创建新的进程组，便于信号传递
                )
                
                if wait_for_leg_signal:
                    # 等待腿部准备完成的信号，在等待期间进程会进行初始化
                    # 进程会等待stdin输入时长参数，所以不会立即开始执行
                    while not os.path.exists(sync_signal_file):
                        time.sleep(0.1)
                    
                    log_file_handle.write("收到腿部准备完成信号，开始执行手臂磨线...\n")
                    log_file_handle.flush()
                    
                    # 收到信号后，发送时长参数，开始执行
                    if arm_duration is not None:
                        self.arm_process.stdin.write(str(arm_duration) + "\n")
                        self.arm_process.stdin.flush()
                        self.arm_process.stdin.close()
                else:
                    # 立即执行（用于单独运行手臂磨线）
                    if arm_duration is not None:
                        self.arm_process.stdin.write(str(arm_duration) + "\n")
                        self.arm_process.stdin.flush()
                        self.arm_process.stdin.close()
                
                # 保持日志文件句柄打开，直到进程结束
                # 主程序会通过poll()检查进程状态，这里只需要保持文件句柄打开
                self.arm_process.wait()
                # 进程结束后关闭文件句柄在finally块中处理
                        
            except Exception as e:
                if log_file_handle:
                    log_file_handle.write(f"手臂磨线执行出错: {e}\n")
                    log_file_handle.flush()
            finally:
                # 关闭日志文件句柄
                if log_file_handle:
                    log_file_handle.close()
        
        try:
            arm_thread = threading.Thread(target=delayed_arm_breakin, daemon=True)
            arm_thread.start()
            return 99999
        except Exception as e:
            self.print_colored(f"启动手臂磨线失败: {e}", Colors.RED)
            return None
            
    def run_leg_breakin_background(self, log_file, duration):
        """在后台运行腿部磨线"""
        try:
            with open(log_file, 'w') as f:
                input_pipe = subprocess.PIPE
                leg_script_dir = self.leg_breakin_script.parent
                self.leg_process = subprocess.Popen(
                    ["python3", str(self.leg_breakin_script)],
                    cwd=str(leg_script_dir),
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    stdin=input_pipe,
                    text=True,
                    env=dict(os.environ, PYTHONUNBUFFERED='1'),  # 确保Python子进程也无缓冲
                    preexec_fn=os.setsid  # 创建新的进程组，便于信号传递
                )
                
                # leg_breakin_tools现在直接执行磨线功能，只需要输入时间
                self.leg_process.stdin.write(f"{duration}\n")
                self.leg_process.stdin.flush()
                self.leg_process.stdin.close()
                    
            return self.leg_process.pid
        except Exception as e:
            self.print_colored(f"启动腿部磨线失败: {e}", Colors.RED)
            return None
            
    def run_both_breakin(self):
        """同时运行手臂和腿部磨线"""
        self.print_colored("同时启动手臂和腿部磨线...", Colors.GREEN)
        self.print_colored("注意：", Colors.YELLOW)
        self.print_colored("  - 手臂磨线使用RUIWO电机控制", Colors.YELLOW)
        self.print_colored("  - 腿部磨线使用EC_Master电机控制", Colors.YELLOW)
        self.print_colored("  - 两个程序将并行运行", Colors.YELLOW)
        self.print_colored("  - 启用安全监控：任一系统故障将立即停止所有运动", Colors.RED)
        
        # 先检查腿部磨线编译状态
        self.print_colored("检查腿部磨线编译状态...", Colors.BLUE)
        if not self.check_and_compile_leg_breakin():
            self.print_colored("腿部磨线脚本编译失败，无法继续执行", Colors.RED)
            return 1
        
        # 版本14：同时运行时跳过手臂零点设置三步，改为先执行电机校准 --cali
        if str(self.robot_version) == "14":
            self.print_colored("版本14：同时运行时跳过手臂零点设置步骤（步骤1/2/3）", Colors.CYAN)
            # 执行 motorevo_tool.sh --cali
            try:
                if not self.motorevo_tool_sh.exists():
                    self.print_colored(f"错误：未找到校准脚本 {self.motorevo_tool_sh}", Colors.RED)
                    return 1
                self.print_colored("开始执行电机校准 (--cali)...", Colors.BLUE)
                result_cali = subprocess.run(
                    ["bash", str(self.motorevo_tool_sh), "--cali"],
                    cwd=str(self.motorevo_tool_sh.parent),
                    text=True
                )
                if result_cali.returncode != 0:
                    self.print_colored("电机校准失败 (--cali)", Colors.RED)
                    return 1
                self.print_colored("✓ 电机校准完成", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"执行电机校准出错: {e}", Colors.RED)
                return 1
        else:
            self.print_colored("在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
            if not self.run_arm_zero_setup():
                self.print_colored("零点设置失败，无法继续执行手臂磨线", Colors.RED)
                return 1
        
        arm_duration, leg_duration = self.get_user_inputs()
        if arm_duration is None or leg_duration is None:
            self.print_colored("用户取消操作", Colors.YELLOW)
            return 0
        
        print()
        self.print_colored("参数配置完成，开始启动程序...", Colors.GREEN)
        
        self.log_dir.mkdir(exist_ok=True)
        
        # 清理之前的同步信号文件和失能信号文件
        sync_signal_file = "/tmp/leg_ready_signal"
        arm_disable_signal_file = "/tmp/arm_disable_signal"
        
        if os.path.exists(sync_signal_file):
            try:
                os.remove(sync_signal_file)
                self.print_colored("已清理之前的同步信号文件", Colors.BLUE)
            except Exception as e:
                self.print_colored(f"清理同步信号文件失败: {e}", Colors.YELLOW)
                
        if os.path.exists(arm_disable_signal_file):
            try:
                os.remove(arm_disable_signal_file)
                self.print_colored("已清理之前的手臂失能信号文件", Colors.BLUE)
            except Exception as e:
                self.print_colored(f"清理手臂失能信号文件失败: {e}", Colors.YELLOW)
        
        self.print_colored("启动手臂磨线进程...", Colors.BLUE)
        arm_log = self.log_dir / "arm_breakin.log"
        arm_pid = self.run_arm_breakin_background(arm_log, arm_duration, wait_for_leg_signal=True)
        
        if arm_pid is None:
            self.print_colored("手臂磨线启动失败", Colors.RED)
            return 1
            
        if arm_pid == 99999:
            self.print_colored("手臂磨线进程已启动（等待腿部信号）", Colors.GREEN)
        else:
            self.print_colored(f"手臂磨线进程ID: {arm_pid}", Colors.GREEN)
        self.print_colored("手臂磨线将等待腿部准备完成后开始运行...", Colors.YELLOW)
        
        time.sleep(1)
        
        self.print_colored("启动腿部磨线进程...", Colors.BLUE)
        leg_log = self.log_dir / "leg_breakin.log"
        leg_pid = self.run_leg_breakin_background(leg_log, leg_duration)
        
        if leg_pid is None:
            self.print_colored("腿部磨线启动失败", Colors.RED)
            if self.arm_process:
                self.arm_process.terminate()
            return 1
                    
        print()
        self.print_colored("两个磨线程序已启动！", Colors.GREEN)
        
        # 启动心跳监控
        self.start_safety_monitor()
        
        self.print_colored("监控命令：", Colors.YELLOW)
        print(f"  查看手臂磨线日志: less +F {arm_log}")
        print(f"  查看腿部磨线日志: less +F {leg_log}")
        print()
        self.print_colored("按 Ctrl+C 可以停止所有磨线程序", Colors.BLUE)
        self.print_colored("安全监控：如果检测到任一种电机停止，将自动停止所有运动", Colors.RED)
        
        try:
            while True:
                # 检查进程状态
                arm_finished = self.arm_process and self.arm_process.poll() is not None
                leg_finished = self.leg_process and self.leg_process.poll() is not None
                
                # 如果任一进程结束，立即检查心跳并停止另一个
                if arm_finished and not leg_finished:
                    self.print_colored("手臂磨线进程已结束，检测到心跳停止，立即停止腿部磨线...", Colors.RED)
                    self._send_stop_signals()
                    # 给腿部进程一些时间响应停止信号
                    time.sleep(2)
                    if self.leg_process and self.leg_process.poll() is None:
                        self.print_colored("腿部进程未响应停止信号，强制终止...", Colors.YELLOW)
                        try:
                            os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                            self.print_colored("✓ 腿部磨线进程已强制终止", Colors.GREEN)
                        except Exception as e:
                            self.print_colored(f"强制终止失败: {e}", Colors.RED)
                    break
                elif leg_finished and not arm_finished:
                    self.print_colored("腿部磨线进程已结束，检测到心跳停止，立即停止手臂磨线...", Colors.RED)
                    self._send_stop_signals()
                    # 给手臂进程一些时间响应停止信号
                    time.sleep(2)
                    if self.arm_process and self.arm_process.poll() is None:
                        self.print_colored("手臂进程未响应停止信号，强制终止...", Colors.YELLOW)
                        try:
                            os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
                            try:
                                self.arm_process.wait(timeout=3)
                            except subprocess.TimeoutExpired:
                                os.killpg(os.getpgid(self.arm_process.pid), signal.SIGKILL)
                            self.print_colored("✓ 手臂磨线进程已强制终止", Colors.GREEN)
                        except Exception as e:
                            self.print_colored(f"强制终止失败: {e}", Colors.RED)
                    break
                elif arm_finished and leg_finished:
                    self.print_colored("两个磨线进程都已结束", Colors.GREEN)
                    break
                
                time.sleep(2)
        except KeyboardInterrupt:
            self.print_colored("\n用户中断，正在停止所有磨线程序...", Colors.YELLOW)
            self._send_stop_signals()
            self.stop_all_processes.set()
            
        # 程序结束时清理信号文件
        try:
            if os.path.exists("/tmp/leg_ready_signal"):
                os.remove("/tmp/leg_ready_signal")
            if os.path.exists("/tmp/arm_disable_signal"):
                os.remove("/tmp/arm_disable_signal")
            self.print_colored("已清理所有信号文件", Colors.BLUE)
        except Exception as e:
            self.print_colored(f"清理信号文件失败: {e}", Colors.YELLOW)
        
        self.print_colored("磨线程序执行完成", Colors.GREEN)
        return 0
        
    def show_menu(self):
        """显示菜单"""
        self.print_colored("请选择启动模式：", Colors.YELLOW)
        print("1. 仅启动手臂磨线")
        print("2. 仅启动腿部磨线")
        print("3. 同时启动手臂和腿部磨线")
        print("q. 退出")
        print()
        
    def run(self):
        """主运行函数"""
        self.print_colored("=" * 40, Colors.BLUE)
        self.print_colored("  Kuavo机器人统一磨线启动脚本", Colors.BLUE)
        self.print_colored("=" * 40, Colors.BLUE)
        print()
        
        # 在程序开始时清理之前的日志和心跳文件
        self.print_colored("清理之前的日志和心跳文件...", Colors.YELLOW)
        self.cleanup_logs()
        self.cleanup_heartbeat_files()
        
        self.check_root_permission()
        
        # 进行版本检测和脚本选择
        if not self.detect_leg_script_version():
            self.print_colored("版本检测失败，程序退出", Colors.RED)
            return 1
        
        self.check_scripts_exist()
        
        # 显示版本信息
        if self.robot_version:
            self.print_colored(f"当前机器人版本：{self.robot_version}", Colors.CYAN)
        if self.robot_type:
            if self.robot_type == "roban2":
                self.print_colored("使用 Roban2 腿部磨线脚本（适用于版本13-14）", Colors.BLUE)
            elif self.robot_type == "kuavo4":
                self.print_colored("使用 Kuavo4 腿部磨线脚本（适用于版本40-49）", Colors.BLUE)
            elif self.robot_type == "kuavo5":
                self.print_colored("使用 Kuavo5 腿部磨线脚本（适用于版本50-52）", Colors.BLUE)
        print()
        
        # 提示用户
        self.print_colored("请吊高机器人，将【手臂和腿部】全部摆到【零点位置】。", Colors.YELLOW)
        self.print_colored("最好将吊架的【万向环取掉】，避免机器旋转，腿部踢到支架。", Colors.YELLOW)
        self.print_colored("确保无干涉、周围无障碍物。准备就绪后再选择启动模式。", Colors.YELLOW)
        print()
        
        while True:
            self.show_menu()
            choice = input("请输入选项 (1-3 或 q): ").strip()
            
            if choice == "1":
                return self.run_arm_breakin()
            elif choice == "2":
                return self.run_leg_breakin()
            elif choice == "3":
                return self.run_both_breakin()
            elif choice == "q":
                self.print_colored("退出程序", Colors.YELLOW)
                return 0
            else:
                self.print_colored("无效选项，请输入 1-3 或 q", Colors.RED)

def main():
    app = None
    try:
        app = KuavoUnifiedBreakin()
        exit_code = app.run()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}程序被用户中断{Colors.NC}")
        sys.exit(0)
    except Exception as e:
        print(f"{Colors.RED}程序运行出错: {e}{Colors.NC}")
        sys.exit(1)

if __name__ == "__main__":
    main()
