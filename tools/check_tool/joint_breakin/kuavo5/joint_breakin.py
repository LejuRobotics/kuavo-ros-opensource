#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo5机器人磨线启动脚本
统一启动Kuavo5机器人的手臂磨线和腿部磨线
仅支持Kuavo5机器人（版本50+）
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

class Kuavo5Breakin:
    def __init__(self):
        self.current_dir = Path(__file__).parent.absolute()
        # 项目根目录是当前目录的父目录的父目录的父目录的父目录
        self.project_root = self.current_dir.parent.parent.parent.parent
        
        # 定义脚本路径
        # check_tool 目录路径（当前目录的父目录的父目录）
        self.check_tool_dir = self.current_dir.parent.parent
        
        # 手臂磨线脚本路径（单CAN配置，使用arm_breakin.sh）
        self.arm_breakin_script = self.current_dir / "arm" / "arm_breakin.sh"
        
        # 零点设置脚本路径
        # 添加arm_setzero.sh脚本路径（对应Hardware_tool.py中的arm_setzero()函数）
        # 优先使用 kuavo5/arm 目录下的脚本
        self.arm_setzero_sh_script = self.current_dir / "arm" / "arm_setzero.sh"
        # 备用路径1：check_tool 目录下
        self.arm_setzero_sh_script_alt1 = self.check_tool_dir / "arm_setzero.sh"
        # 备用路径2：项目根目录下的安装路径
        self.arm_setzero_sh_script_alt2 = self.project_root / "installed" / "share" / "hardware_plant" / "lib" / "ruiwo_controller" / "arm_setzero.sh"
        # 备用路径3：项目源码路径
        self.arm_setzero_sh_script_alt3 = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "arm_setzero.sh"
        
        # ruiwo_zero_set.sh 脚本路径（check_tool 目录下）
        self.arm_setzero_script = self.check_tool_dir / "ruiwo_zero_set.sh"
        # 备用路径
        self.arm_setzero_script_alt = self.project_root / "installed" / "share" / "hardware_plant" / "lib" / "ruiwo_controller" / "setZero.sh"
        
        # RUIWO零点设置脚本的多个可能路径
        self.ruiwo_zero_script = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        self.ruiwo_zero_script_alt1 = self.project_root / "installed" / "share" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        self.ruiwo_zero_script_alt2 = self.check_tool_dir / "ruiwo_zero_set.sh"  # 有些情况下可能使用这个
        
        # 腿部磨线脚本路径
        self.leg_breakin_script = self.current_dir / "leg" / "kuavo5_leg_breakin.py"
        
        self.arm_process = None
        self.leg_process = None
        self.log_dir = Path("/tmp/kuavo5_breakin_logs")
        self.robot_version = None
        
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
                    os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
                    try:
                        self.arm_process.wait(timeout=5)
                        self.print_colored("✓ 手臂磨线进程已停止", Colors.GREEN)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(self.arm_process.pid), signal.SIGKILL)
                        self.print_colored("✓ 手臂磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"停止手臂磨线进程失败: {e}", Colors.RED)
            
            # 停止腿部磨线进程
            if self.leg_process and self.leg_process.poll() is None:
                self.print_colored("正在停止腿部磨线进程...", Colors.YELLOW)
                try:
                    os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                    self.print_colored("✓ 腿部磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    self.print_colored(f"强制终止腿部磨线进程失败: {e}", Colors.RED)
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
        
    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
        
    def cleanup_logs(self):
        """清理日志目录"""
        try:
            if self.log_dir.exists():
                shutil.rmtree(self.log_dir)
        except Exception as e:
            self.print_colored(f"清理日志目录失败: {e}", Colors.RED)
    
    def cleanup_heartbeat_files(self):
        """清理心跳文件和信号文件"""
        heartbeat_files = [
            self.arm_heartbeat_file,
            self.leg_heartbeat_file,
            self.emergency_stop_file,
            "/tmp/leg_ready_signal",
            "/tmp/arm_disable_signal",
            "/tmp/leg_stop_signal",
            "/tmp/arm_stop_signal"
        ]
        
        for file_path in heartbeat_files:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                self.print_colored(f"清理心跳/信号文件失败 {file_path}: {e}", Colors.YELLOW)
    
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
    
    def check_root_permission(self):
        """检查root权限"""
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 joint_breakin.py", Colors.YELLOW)
            sys.exit(1)
            
    def check_scripts_exist(self, check_arm=True, check_leg=True):
        """检查脚本文件是否存在"""
        if check_arm:
            if not self.arm_breakin_script.exists():
                self.print_colored(f"错误：手臂磨线脚本不存在: {self.arm_breakin_script}", Colors.RED)
                sys.exit(1)
            
            # 检查零点设置相关脚本 - arm_setzero.sh（可选，有多个备用路径）
            arm_setzero_found = False
            if self.arm_setzero_sh_script.exists():
                arm_setzero_found = True
            elif self.arm_setzero_sh_script_alt1.exists():
                self.arm_setzero_sh_script = self.arm_setzero_sh_script_alt1
                arm_setzero_found = True
            elif self.arm_setzero_sh_script_alt2.exists():
                self.arm_setzero_sh_script = self.arm_setzero_sh_script_alt2
                arm_setzero_found = True
            elif self.arm_setzero_sh_script_alt3.exists():
                self.arm_setzero_sh_script = self.arm_setzero_sh_script_alt3
                arm_setzero_found = True
            
            if not arm_setzero_found:
                self.print_colored(f"警告：arm_setzero.sh脚本不存在，将跳过此步骤", Colors.YELLOW)
                self.print_colored(f"  已查找路径:", Colors.YELLOW)
                self.print_colored(f"    1. {self.current_dir / 'arm' / 'arm_setzero.sh'}", Colors.YELLOW)
                self.print_colored(f"    2. {self.check_tool_dir / 'arm_setzero.sh'}", Colors.YELLOW)
                self.print_colored(f"    3. {self.arm_setzero_sh_script_alt2}", Colors.YELLOW)
                self.print_colored(f"    4. {self.arm_setzero_sh_script_alt3}", Colors.YELLOW)
            
            # 检查 ruiwo_zero_set.sh 脚本
            if not self.arm_setzero_script.exists():
                if self.arm_setzero_script_alt.exists():
                    self.arm_setzero_script = self.arm_setzero_script_alt
                else:
                    self.print_colored(f"错误：手臂零点设置脚本不存在", Colors.RED)
                    self.print_colored(f"  查找路径1: {self.check_tool_dir / 'ruiwo_zero_set.sh'}", Colors.RED)
                    self.print_colored(f"  查找路径2: {self.arm_setzero_script_alt}", Colors.RED)
                    sys.exit(1)
            
            # 检查RUIWO零点设置脚本（检查多个可能的路径）
            ruiwo_zero_found = False
            if self.ruiwo_zero_script.exists():
                ruiwo_zero_found = True
            elif self.ruiwo_zero_script_alt1.exists():
                self.ruiwo_zero_script = self.ruiwo_zero_script_alt1
                ruiwo_zero_found = True
            elif self.ruiwo_zero_script_alt2.exists():
                self.ruiwo_zero_script = self.ruiwo_zero_script_alt2
                ruiwo_zero_found = True
            
            if not ruiwo_zero_found:
                self.print_colored(f"错误：RUIWO零点设置脚本不存在", Colors.RED)
                self.print_colored(f"  已查找路径:", Colors.RED)
                self.print_colored(f"    1. {self.project_root / 'src' / 'kuavo-ros-control-lejulib' / 'hardware_plant' / 'lib' / 'ruiwo_controller' / 'setMotorZero.sh'}", Colors.RED)
                self.print_colored(f"    2. {self.project_root / 'installed' / 'share' / 'hardware_plant' / 'lib' / 'ruiwo_controller' / 'setMotorZero.sh'}", Colors.RED)
                self.print_colored(f"    3. {self.check_tool_dir / 'ruiwo_zero_set.sh'}", Colors.RED)
                sys.exit(1)
        
        if check_leg and not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            sys.exit(1)
    
    def check_and_compile_leg_breakin(self):
        """检查腿部磨线编译产物是否存在"""
        if not self.leg_breakin_script.exists():
            return False
        
        leg_dir = self.leg_breakin_script.parent
        build_dir = leg_dir / "build_lib"
        ec_master_so = build_dir / "ec_master_wrap.so"
        
        if not build_dir.exists():
            self.print_colored(f"错误：编译目录不存在: {build_dir}", Colors.RED)
            self.print_colored("请先编译EC_Master工具", Colors.YELLOW)
            return False
        
        if not ec_master_so.exists():
            self.print_colored(f"错误：EC_Master编译产物不存在: {ec_master_so}", Colors.RED)
            self.print_colored("请先编译EC_Master工具", Colors.YELLOW)
            return False
        
        return True

    def run_arm_zero_setup(self):
        """运行手臂零点设置"""
        self.print_colored("开始执行手臂零点设置...", Colors.GREEN)
        self.print_colored("注意：请确保手臂已摆正到零点位置", Colors.YELLOW)
        
        try:
            # 步骤1：执行arm_setzero.sh脚本（对应Hardware_tool.py中的arm_setzero()函数）
            arm_setzero_sh_script = None
            if self.arm_setzero_sh_script.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script
            elif self.arm_setzero_sh_script_alt1.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script_alt1
            elif self.arm_setzero_sh_script_alt2.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script_alt2
            elif self.arm_setzero_sh_script_alt3.exists():
                arm_setzero_sh_script = self.arm_setzero_sh_script_alt3
                
            if arm_setzero_sh_script:
                self.print_colored(f"步骤1：执行arm_setzero.sh脚本... ({arm_setzero_sh_script})", Colors.BLUE)
                result0 = subprocess.run(
                    ["bash", str(arm_setzero_sh_script)],
                    text=True,
                    cwd=str(arm_setzero_sh_script.parent)
                )
                
                if result0.returncode != 0:
                    self.print_colored("arm_setzero.sh脚本执行失败", Colors.RED)
                    return False
                else:
                    self.print_colored("✓ arm_setzero.sh脚本执行完成", Colors.GREEN)
            else:
                self.print_colored("跳过arm_setzero.sh脚本（脚本不存在）", Colors.YELLOW)
            
            self.print_colored(f"步骤2：执行手臂零点设置... ({self.arm_setzero_script})", Colors.BLUE)
            result1 = subprocess.run(
                ["bash", str(self.arm_setzero_script)],
                cwd=str(self.arm_setzero_script.parent),
                text=True
            )
            
            if result1.returncode != 0:
                self.print_colored("手臂零点设置失败", Colors.RED)
                return False
                
            self.print_colored(f"步骤3：执行RUIWO零点设置... ({self.ruiwo_zero_script})", Colors.BLUE)
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
        
        # 单CAN配置：在执行手臂磨线之前，需要先完成零点设置
        self.print_colored("单CAN配置：在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
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
            
            # 使用arm_breakin.sh脚本
            # 单独运行手臂磨线时，添加 --no-wait-leg 参数，不等待腿部信号
            cmd = ["bash", str(self.arm_breakin_script), "--no-wait-leg"]
            cwd = str(self.arm_breakin_script.parent)

            # 使用Popen启动子进程，保持交互式环境
            self.print_colored(f"执行命令: {' '.join(cmd)}", Colors.CYAN)
            self.print_colored(f"工作目录: {cwd}", Colors.CYAN)
            
            process = subprocess.Popen(
                cmd,
                cwd=cwd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,  # 行缓冲
                env=dict(os.environ, PYTHONUNBUFFERED='1')  # 确保Python子进程也无缓冲
            )
            
            # 等待进程启动并读取一些初始输出，确认程序正常运行
            import time
            time.sleep(0.5)  # 等待进程启动
            
            # 检查进程是否已经退出
            if process.poll() is not None:
                # 进程已经退出，读取所有输出
                remaining_output = process.stdout.read()
                if remaining_output:
                    self.print_colored(f"进程输出: {remaining_output}", Colors.RED)
                self.print_colored(f"错误：手臂磨线进程立即退出，退出码: {process.returncode}", Colors.RED)
                return 1
            
            # 发送初始输入（测试时长）- 延迟发送，确保程序已经准备好接收输入
            time.sleep(0.5)  # 再等待一下，确保程序初始化完成
            try:
                process.stdin.write(str(arm_duration) + "\n")
                process.stdin.flush()
            except BrokenPipeError:
                self.print_colored("错误：无法向进程发送输入，进程可能已退出", Colors.RED)
                return 1
            
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

    def run_leg_breakin(self):
        """运行腿部磨线"""
        if not self.check_and_compile_leg_breakin():
            self.print_colored("腿部磨线编译产物检查失败，无法继续执行", Colors.RED)
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
            # 单独腿部磨线时，不需要检查手臂心跳
            env = dict(os.environ, PYTHONUNBUFFERED='1', CHECK_ARM_HEARTBEAT='false')
            if self.robot_version:
                env['ROBOT_VERSION'] = str(self.robot_version)
            result = subprocess.run(
                ["python3", str(self.leg_breakin_script)],
                cwd=str(leg_script_dir),
                input=f"{duration}\n",
                text=True,
                env=env
            )
            return result.returncode
        except Exception as e:
            self.print_colored(f"腿部磨线运行出错: {e}", Colors.RED)
            return 1
    
    def start_safety_monitor(self):
        """启动安全监控线程"""
        if self.safety_monitor_thread and self.safety_monitor_thread.is_alive():
            return
            
        self.stop_safety_monitor.clear()
        self.safety_monitor_thread = threading.Thread(target=self._safety_monitor_loop, daemon=True)
        self.safety_monitor_thread.start()
        self.print_colored("✓ 安全监控已启动", Colors.GREEN)
    
    def _safety_monitor_loop(self):
        """安全监控主循环"""
        self.print_colored("心跳监控：等待第一轮动作开始...", Colors.YELLOW)
        
        # 等待第一轮动作开始（大约15秒后开始监控）
        time.sleep(15.0)
        
        self.print_colored("第一轮动作开始，启动心跳监控...", Colors.GREEN)
        
        # 心跳超时阈值（秒）
        HEARTBEAT_TIMEOUT = 2.0
        # 检查间隔（秒）- 每0.5秒检查一次
        CHECK_INTERVAL = 0.5
        
        # 记录上次心跳时间，用于连续超时判断
        # 连续警告次数：2秒超时 / 0.5秒检查间隔 = 4次
        arm_heartbeat_warning_count = 0
        leg_heartbeat_warning_count = 0
        
        # 持续监控直到程序结束
        while not self.stop_safety_monitor.is_set():
            time.sleep(CHECK_INTERVAL)
            
            # 检查紧急停止信号
            if os.path.exists(self.emergency_stop_file):
                self.print_colored("❌ 检测到紧急停止信号！", Colors.RED)
                self._emergency_stop()
                break
            
            # 检查是否有arm_disable_signal文件
            if os.path.exists("/tmp/arm_disable_signal"):
                self.print_colored("⚠️ 检测到手臂失能信号文件存在，这会导致腿部停止运动", Colors.YELLOW)
            
            # 检查进程状态
            arm_running = self.arm_process and self.arm_process.poll() is None
            leg_running = self.leg_process and self.leg_process.poll() is None
            
            # 如果两个进程都已结束，停止监控
            if not arm_running and not leg_running:
                self.print_colored("✓ 两个进程都已结束，停止心跳监控", Colors.GREEN)
                break
            
            # 如果任一进程结束，但另一个还在运行，立即触发停止
            if not arm_running and leg_running:
                self.print_colored("磨线结束", Colors.GREEN)
                self._send_stop_signals(silent=True)
                self._emergency_stop(silent=True)
                break
            elif arm_running and not leg_running:
                self.print_colored("磨线结束", Colors.GREEN)
                self._send_stop_signals(silent=True)
                self._emergency_stop(silent=True)
                break
            
            # 检查心跳超时
            heartbeat_issue = False
            issue_message = ""
            
            # 检查手臂心跳
            if arm_running:
                if os.path.exists(self.arm_heartbeat_file):
                    try:
                        stat = os.stat(self.arm_heartbeat_file)
                        time_diff = time.time() - stat.st_mtime
                        if time_diff > HEARTBEAT_TIMEOUT:
                            arm_heartbeat_warning_count += 1
                            # 连续4次检查（2秒）都超时才算异常
                            if arm_heartbeat_warning_count >= 4:
                                heartbeat_issue = True
                                issue_message = f"手臂心跳超时（{time_diff:.1f}秒未更新，连续{arm_heartbeat_warning_count}次检查）"
                        else:
                            arm_heartbeat_warning_count = 0
                    except Exception as e:
                        self.print_colored(f"检查手臂心跳文件失败: {e}", Colors.YELLOW)
                else:
                    arm_heartbeat_warning_count += 1
                    # 连续4次检查（2秒）都未创建才算异常
                    if arm_heartbeat_warning_count >= 4:
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
                            # 连续4次检查（2秒）都超时才算异常
                            if leg_heartbeat_warning_count >= 4:
                                heartbeat_issue = True
                                if issue_message:
                                    issue_message += f"；腿部心跳超时（{time_diff:.1f}秒未更新，连续{leg_heartbeat_warning_count}次检查）"
                                else:
                                    issue_message = f"腿部心跳超时（{time_diff:.1f}秒未更新，连续{leg_heartbeat_warning_count}次检查）"
                        else:
                            leg_heartbeat_warning_count = 0
                    except Exception as e:
                        self.print_colored(f"检查腿部心跳文件失败: {e}", Colors.YELLOW)
                else:
                    leg_heartbeat_warning_count += 1
                    # 连续4次检查（2秒）都未创建才算异常
                    if leg_heartbeat_warning_count >= 4:
                        heartbeat_issue = True
                        if issue_message:
                            issue_message += "；腿部心跳文件不存在，连续多次检查未创建"
                        else:
                            issue_message = "腿部心跳文件不存在，连续多次检查未创建"
            
            # 如果检测到心跳问题，立即停止
            if heartbeat_issue:
                self.print_colored(f"❌ 检测到心跳异常：{issue_message}", Colors.RED)
                self.print_colored("正在发送停止信号给手臂和腿部磨线程序...", Colors.YELLOW)
                self._send_stop_signals()
                self._emergency_stop()
                break
    
    def _send_stop_signals(self, silent=False):
        """发送停止信号给手臂和腿部磨线程序"""
        try:
            if self.arm_process and self.arm_process.poll() is None:
                try:
                    with open("/tmp/arm_stop_signal", "w") as f:
                        f.write(f"stop_signal_{time.time()}\n")
                    if not silent:
                        self.print_colored("✓ 已发送停止信号给手臂磨线程序", Colors.GREEN)
                except Exception as e:
                    if not silent:
                        self.print_colored(f"发送手臂停止信号失败: {e}", Colors.RED)
            
            if self.leg_process and self.leg_process.poll() is None:
                try:
                    with open("/tmp/leg_stop_signal", "w") as f:
                        f.write(f"stop_signal_{time.time()}\n")
                    if not silent:
                        self.print_colored("✓ 已发送停止信号给腿部磨线程序", Colors.GREEN)
                except Exception as e:
                    if not silent:
                        self.print_colored(f"发送腿部停止信号失败: {e}", Colors.RED)
        except Exception as e:
            if not silent:
                self.print_colored(f"发送停止信号时出错: {e}", Colors.RED)
    
    def _emergency_stop(self, silent=False):
        """紧急停止所有电机"""
        if not silent:
            self.print_colored("🚨 执行紧急停止程序...", Colors.RED)
        
        try:
            with open(self.emergency_stop_file, 'w') as f:
                f.write(f"emergency_stop_{time.time()}")
            
            arm_running = self.arm_process and self.arm_process.poll() is None
            leg_running = self.leg_process and self.leg_process.poll() is None
            
            if arm_running:
                if not silent:
                    self.print_colored("停止手臂磨线进程...", Colors.YELLOW)
                self.arm_process.terminate()
                try:
                    self.arm_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.arm_process.kill()
            
            if leg_running:
                if not silent:
                    self.print_colored("停止腿部磨线进程...", Colors.YELLOW)
                try:
                    os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                    if not silent:
                        self.print_colored("✓ 腿部磨线进程已强制终止", Colors.GREEN)
                except Exception as e:
                    if not silent:
                        self.print_colored(f"强制终止腿部磨线进程失败: {e}", Colors.RED)
                    try:
                        self.leg_process.kill()
                        if not silent:
                            self.print_colored("✓ 腿部磨线进程已强制终止（备用方案）", Colors.GREEN)
                    except Exception as e2:
                        if not silent:
                            self.print_colored(f"备用终止方案也失败: {e2}", Colors.RED)
            
            if not silent:
                if arm_running or leg_running:
                    self.print_colored("❌ 紧急停止完成，所有电机已停止", Colors.RED)
                else:
                    self.print_colored("✓ 程序正常结束", Colors.GREEN)
        except Exception as e:
            if not silent:
                self.print_colored(f"紧急停止过程中出错: {e}", Colors.RED)
    
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
        """在后台运行手臂磨线，输出实时显示到主窗口"""
        def delayed_arm_breakin():
            try:
                sync_signal_file = "/tmp/leg_ready_signal"
                
                if wait_for_leg_signal:
                    self.print_colored("[手臂] 等待腿部准备完成信号（手臂磨线进程已启动，正在初始化...）...", Colors.BLUE)
                else:
                    self.print_colored("[手臂] 手臂磨线立即开始执行...", Colors.BLUE)
                
                input_pipe = subprocess.PIPE
                
                cmd = ["bash", str(self.arm_breakin_script)]
                # arm_breakin.py 默认等待腿部信号，只有单独运行时才需要 --no-wait-leg 参数
                if not wait_for_leg_signal:
                    cmd.append("--no-wait-leg")
                cwd = str(self.arm_breakin_script.parent)

                self.print_colored(f"[手臂] 执行命令: {' '.join(cmd)}", Colors.CYAN)
                self.print_colored(f"[手臂] 工作目录: {cwd}", Colors.CYAN)
                
                self.arm_process = subprocess.Popen(
                    cmd,
                    cwd=cwd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=input_pipe,
                    text=True,
                    bufsize=1,
                    env=dict(os.environ, PYTHONUNBUFFERED='1'),
                    preexec_fn=os.setsid
                )
                
                # 等待进程启动
                time.sleep(0.5)
                
                # 检查进程是否已经退出
                if self.arm_process.poll() is not None:
                    remaining_output = self.arm_process.stdout.read()
                    if remaining_output:
                        self.print_colored(f"[手臂] 进程输出: {remaining_output}", Colors.RED)
                    self.print_colored(f"[手臂] 错误：手臂磨线进程立即退出，退出码: {self.arm_process.returncode}", Colors.RED)
                    return None
                
                def read_arm_output():
                    try:
                        for line in iter(self.arm_process.stdout.readline, ''):
                            if not line:
                                break
                            print(f"{Colors.BLUE}[手臂]{Colors.NC} {line.rstrip()}")
                            sys.stdout.flush()
                    except Exception as e:
                        self.print_colored(f"[手臂] 读取输出出错: {e}", Colors.RED)
                
                output_thread = threading.Thread(target=read_arm_output, daemon=True)
                output_thread.start()
                
                if wait_for_leg_signal:
                    while not os.path.exists(sync_signal_file):
                        time.sleep(0.1)
                    
                    self.print_colored("[手臂] 收到腿部准备完成信号，开始执行手臂磨线...", Colors.GREEN)
                    
                    if arm_duration is not None:
                        try:
                            time.sleep(0.5)  # 等待程序准备好接收输入
                            self.arm_process.stdin.write(str(arm_duration) + "\n")
                            self.arm_process.stdin.flush()
                            self.arm_process.stdin.close()
                        except BrokenPipeError:
                            self.print_colored("[手臂] 错误：无法向进程发送输入，进程可能已退出", Colors.RED)
                            return None
                else:
                    if arm_duration is not None:
                        try:
                            time.sleep(0.5)  # 等待程序准备好接收输入
                            self.arm_process.stdin.write(str(arm_duration) + "\n")
                            self.arm_process.stdin.flush()
                            self.arm_process.stdin.close()
                        except BrokenPipeError:
                            self.print_colored("[手臂] 错误：无法向进程发送输入，进程可能已退出", Colors.RED)
                            return None
                
                self.arm_process.wait()
                output_thread.join(timeout=1.0)
                        
            except Exception as e:
                self.print_colored(f"[手臂] 手臂磨线执行出错: {e}", Colors.RED)
        
        try:
            arm_thread = threading.Thread(target=delayed_arm_breakin, daemon=True)
            arm_thread.start()
            return 99999
        except Exception as e:
            self.print_colored(f"启动手臂磨线失败: {e}", Colors.RED)
            return None
    
    def run_leg_breakin_background(self, log_file, duration, check_arm_heartbeat=False):
        """在后台运行腿部磨线，输出实时显示到主窗口"""
        try:
            input_pipe = subprocess.PIPE
            leg_script_dir = self.leg_breakin_script.parent
            env = dict(os.environ, PYTHONUNBUFFERED='1')
            env['CHECK_ARM_HEARTBEAT'] = 'true' if check_arm_heartbeat else 'false'
            if self.robot_version:
                env['ROBOT_VERSION'] = str(self.robot_version)
            self.leg_process = subprocess.Popen(
                ["python3", str(self.leg_breakin_script)],
                cwd=str(leg_script_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=input_pipe,
                text=True,
                bufsize=1,
                env=env,
                preexec_fn=os.setsid
            )
            
            self.leg_process.stdin.write(f"{duration}\n")
            self.leg_process.stdin.flush()
            self.leg_process.stdin.close()
            
            def read_leg_output():
                try:
                    for line in iter(self.leg_process.stdout.readline, ''):
                        if not line:
                            break
                        print(f"{Colors.CYAN}[腿部]{Colors.NC} {line.rstrip()}")
                        sys.stdout.flush()
                except Exception as e:
                    self.print_colored(f"[腿部] 读取输出出错: {e}", Colors.RED)
            
            output_thread = threading.Thread(target=read_leg_output, daemon=True)
            output_thread.start()
                    
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
        
        # 先检查腿部磨线编译产物
        self.print_colored("检查腿部磨线编译产物...", Colors.BLUE)
        if not self.check_and_compile_leg_breakin():
            self.print_colored("腿部磨线编译产物检查失败，无法继续执行", Colors.RED)
            return 1
        
        # 单CAN配置：使用 run_arm_zero_setup
        self.print_colored("单CAN配置：在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
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
        
        # 清理之前的同步信号文件
        signal_files_to_clean = [
            ("/tmp/leg_ready_signal", "同步信号文件"),
            ("/tmp/arm_disable_signal", "手臂失能信号文件"),
            ("/tmp/leg_stop_signal", "腿部停止信号文件"),
            ("/tmp/arm_stop_signal", "手臂停止信号文件")
        ]
        
        for file_path, description in signal_files_to_clean:
            if os.path.exists(file_path):
                try:
                    os.remove(file_path)
                    self.print_colored(f"已清理之前的{description}", Colors.BLUE)
                except Exception as e:
                    self.print_colored(f"清理{description}失败: {e}", Colors.YELLOW)
        
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
        leg_pid = self.run_leg_breakin_background(leg_log, leg_duration, check_arm_heartbeat=True)
        
        if leg_pid is None:
            self.print_colored("腿部磨线启动失败", Colors.RED)
            if self.arm_process:
                self.arm_process.terminate()
            return 1
                    
        print()
        self.print_colored("两个磨线程序已启动！", Colors.GREEN)
        self.print_colored("手部和脚部的输出将实时显示在此窗口中", Colors.CYAN)
        self.print_colored("  [手臂] - 手臂磨线输出（蓝色）", Colors.BLUE)
        self.print_colored("  [腿部] - 腿部磨线输出（青色）", Colors.CYAN)
        print()
        
        # 启动心跳监控
        self.start_safety_monitor()
        
        self.print_colored("按 Ctrl+C 可以停止所有磨线程序", Colors.BLUE)
        self.print_colored("安全监控：如果检测到任一种电机停止，将自动停止所有运动", Colors.RED)
        print()
        
        try:
            while True:
                arm_finished = self.arm_process and self.arm_process.poll() is not None
                leg_finished = self.leg_process and self.leg_process.poll() is not None
                
                if arm_finished and not leg_finished:
                    self.print_colored("磨线结束", Colors.GREEN)
                    self._send_stop_signals(silent=True)
                    time.sleep(2)
                    if self.leg_process and self.leg_process.poll() is None:
                        try:
                            os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                        except Exception:
                            pass
                    break
                elif leg_finished and not arm_finished:
                    self.print_colored("磨线结束", Colors.GREEN)
                    self._send_stop_signals(silent=True)
                    time.sleep(2)
                    if self.arm_process and self.arm_process.poll() is None:
                        try:
                            os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
                            try:
                                self.arm_process.wait(timeout=3)
                            except subprocess.TimeoutExpired:
                                os.killpg(os.getpgid(self.arm_process.pid), signal.SIGKILL)
                        except Exception:
                            pass
                    break
                elif arm_finished and leg_finished:
                    self.print_colored("磨线结束", Colors.GREEN)
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
        except Exception:
            pass
        
        return 0
        
    def show_menu(self):
        """显示菜单"""
        self.print_colored("请选择启动模式：", Colors.YELLOW)
        arm_available = self.arm_breakin_script.exists()
        leg_available = self.leg_breakin_script.exists()
        
        if arm_available:
            print("1. 仅启动手臂磨线")
        if leg_available:
            print("2. 仅启动腿部磨线")
        if arm_available and leg_available:
            print("3. 同时启动手臂和腿部磨线")
        print("q. 退出")
        print()
        
    def run(self):
        """主运行函数"""
        print()
        
        # 在程序开始时清理之前的日志和心跳文件
        self.cleanup_logs()
        self.cleanup_heartbeat_files()
        
        self.check_root_permission()
        
        # 获取机器人版本
        self.robot_version = self.get_robot_version()
        
        # 检查版本是否支持（Kuavo5版本50+）
        if self.robot_version:
            try:
                version_num = int(self.robot_version)
                if version_num < 50:
                    self.print_colored(f"警告：版本 {self.robot_version} 可能不是Kuavo5（Kuavo5版本通常为50+）", Colors.YELLOW)
            except (ValueError, TypeError):
                pass
        
        # 检查脚本是否存在
        self.check_scripts_exist()
        
        arm_available = self.arm_breakin_script.exists()
        leg_available = self.leg_breakin_script.exists()
        
        if not arm_available and not leg_available:
            self.print_colored("错误：未找到任何磨线脚本", Colors.RED)
            sys.exit(1)
        
        print()
        
        # 提示用户
        self.print_colored("请吊高机器人，将【手臂和腿部】全部摆到【零点位置】。", Colors.YELLOW)
        self.print_colored("最好将吊架的【万向环锁住】，避免机器旋转、腿部踢到移位机。", Colors.YELLOW)
        self.print_colored("确保无干涉、周围无障碍物。准备就绪后再选择启动模式。", Colors.YELLOW)
        print()
        
        while True:
            self.show_menu()
            choice = input("请输入选项 (1-3 或 q): ").strip()
            
            if choice == "1" and arm_available:
                return self.run_arm_breakin()
            elif choice == "2" and leg_available:
                return self.run_leg_breakin()
            elif choice == "3" and arm_available and leg_available:
                return self.run_both_breakin()
            elif choice == "q":
                self.print_colored("退出程序", Colors.YELLOW)
                return 0
            else:
                self.print_colored("无效选项，请重新选择", Colors.RED)
                print()

def main():
    app = None
    try:
        app = Kuavo5Breakin()
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
