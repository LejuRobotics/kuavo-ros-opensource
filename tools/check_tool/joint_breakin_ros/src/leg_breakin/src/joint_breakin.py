#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Roban2机器人腿部磨线启动脚本
仅启动腿部磨线（EC_Master电机，自研CAN）
支持Roban2机器人（版本13-14）
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
        
        # 腿部磨线脚本路径（根据版本动态选择）
        self.leg_breakin_script_roban2 = self.current_dir / "leg_breakin_roban2_v14" / "roban2_leg_breakin.py"
        self.leg_breakin_script = None  # 将在检测版本后设置
        
        self.leg_process = None
        self.robot_version = None
        self.robot_type = None
        
        # 全局停止标志
        self.stop_all_processes = threading.Event()
        
        # 注册全局信号处理器
        self._setup_global_signal_handlers()
    
    def _setup_global_signal_handlers(self):
        """设置全局信号处理器"""
        def global_signal_handler(signum, frame):
            self.stop_all_processes.set()
            
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
            
            self.print_colored("腿部磨线程序已停止，程序退出", Colors.GREEN)
            sys.exit(0)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, global_signal_handler)
        signal.signal(signal.SIGTERM, global_signal_handler)
        
    def get_robot_type_from_version(self, version):
        """根据版本号获取机器人类型（支持roban2）"""
        if version is None:
            return None
            
        try:
            version_num = int(version)
            if 13 <= version_num <= 14:
                return "roban2"
            else:
                return None
        except (ValueError, TypeError):
            return None
        
    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
        
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
        """检测机器人版本并设置腿部磨线脚本（支持roban2）"""
        self.robot_version = self.get_robot_version()
        
        # 根据版本号自动选择
        self.robot_type = self.get_robot_type_from_version(self.robot_version)
        
        if self.robot_type == "roban2":
            self.leg_breakin_script = self.leg_breakin_script_roban2
            self.print_colored("腿部磨线：使用 Roban2 腿部磨线脚本（适用于版本13-14）", Colors.BLUE)
            return True
        else:
            if self.robot_version:
                self.print_colored(f"错误：版本 {self.robot_version} 不在支持范围内（支持版本13-14的Roban2）", Colors.RED)
            else:
                self.print_colored("错误：无法检测到机器人版本，且仅支持Roban2（版本13-14）", Colors.RED)
            return False
        
    def check_root_permission(self):
        """检查root权限"""
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 kuavo_joint_breakin.py", Colors.YELLOW)
            sys.exit(1)
            
    def check_scripts_exist(self):
        """检查脚本文件是否存在"""
        # 检查腿部磨线脚本
        if not self.leg_breakin_script or not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            sys.exit(1)
        
    def check_and_compile_leg_breakin(self):
        """检查腿部磨线编译产物是否存在"""
        if not self.leg_breakin_script or not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            return False
        
        # 各版本腿部磨线脚本约定：在对应目录下的 build_lib 中放置 ec_master_wrap.so
        ec_master_dir = self.leg_breakin_script.parent
        build_dir = ec_master_dir / "build_lib"
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

    def run_leg_breakin(self):
        """运行腿部磨线"""
        if not self.check_and_compile_leg_breakin():
            self.print_colored("腿部磨线编译产物检查失败，无法继续执行", Colors.RED)
            return 1
        
        print()
        self.print_colored("腿部磨线参数：", Colors.YELLOW)
        min_duration = 14.0
        # 如果由ROS主控制器管理时长，则不在这里再次询问时长
        # 额外保护：如果 stdin 不是 TTY（被父进程用 Popen 关闭了 stdin），也跳过交互，避免卡住等待输入
        use_ros_time_control = (
            os.environ.get("ROS_BREAKIN_CONTROL_TIME", "").lower() == "true"
            or not sys.stdin.isatty()
        )
        if use_ros_time_control:
            # 这里只是给一个占位的duration值，真正的时长由 /breakin/can_start_new_round 控制
            duration = min_duration
            self.print_colored("当前由ROS主控制器通过 /breakin/can_start_new_round 管理运行时长，本脚本不再询问时长。", Colors.CYAN)
        else:
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
            env = dict(os.environ, PYTHONUNBUFFERED='1')
            # 将ROBOT_VERSION传递给子进程，避免EcMasterConfig警告
            if self.robot_version:
                env['ROBOT_VERSION'] = str(self.robot_version)
            result = subprocess.run(
                ["python3", str(self.leg_breakin_script)],
                cwd=str(leg_script_dir),
                input=f"{duration}\n",  # 只输入时间
                text=True,
                env=env  # 传递环境变量
            )
            return result.returncode
        except Exception as e:
            self.print_colored(f"腿部磨线运行出错: {e}", Colors.RED)
            return 1
            
    def run(self):
        """主运行函数"""
        print()
        
        
        self.check_root_permission()
        
        # 进行版本检测和脚本选择
        if not self.detect_leg_script_version():
            self.print_colored("版本检测失败，程序退出", Colors.RED)
            return 1
        
        self.check_scripts_exist()
        
        print()
        
        # 提示用户
        self.print_colored("请吊高机器人，将【腿部】摆到【零点位置】。", Colors.YELLOW)
        self.print_colored("最好将吊架的【万向环锁住】，避免机器旋转、腿部踢到移位机。", Colors.YELLOW)
        self.print_colored("确保无干涉、周围无障碍物。准备就绪后开始执行腿部磨线。", Colors.YELLOW)
        print()
        
        # 直接运行腿部磨线
        return self.run_leg_breakin()

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
