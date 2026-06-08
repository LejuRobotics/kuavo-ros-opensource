#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手臂磨线独立启动脚本（ROS版本）
功能类似 joint_breakin.py 中的功能1：单独手臂磨线
包含零点校准、启动ROS节点等功能
"""

import os
import sys
import subprocess
import threading
import time
import signal
from pathlib import Path

class Colors:
    """终端颜色定义"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'

class ArmBreakinStandalone:
    def __init__(self):
        # 获取当前脚本所在目录
        self.current_dir = Path(__file__).parent.absolute()
        
        # 查找项目根目录（通过查找 tools/check_tool 目录）
        # 脚本位置：joint_breakin_ros/src/breakin_control/scripts/
        # 需要向上找到包含 tools/check_tool 的目录
        self.project_root = self._find_project_root()
        
        # 定义脚本路径
        # 手臂电机校准脚本（motorevo_controller）
        self.motorevo_tool_sh = self.project_root / "tools" / "check_tool" / "motorevo_tool.sh"
        
        # 零点设置脚本路径
        # 查找 tools/check_tool 目录（与 joint_breakin.py 在同一层级）
        check_tool_dir = self.project_root / "tools" / "check_tool"
        self.arm_setzero_sh_script = check_tool_dir.parent / "arm_setzero.sh"
        self.arm_setzero_sh_script_alt = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "arm_setzero.sh"
        
        self.arm_setzero_script = check_tool_dir.parent / "ruiwo_zero_set.sh"
        # RUIWO零点设置脚本的两个可能路径
        self.ruiwo_zero_script = self.project_root / "src" / "kuavo-ros-control-lejulib" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        self.ruiwo_zero_script_alt = self.project_root / "installed" / "share" / "hardware_plant" / "lib" / "ruiwo_controller" / "setMotorZero.sh"
        
        # ROS节点名称（默认，将根据机器人版本和CAN总线类型动态选择）
        self.arm_breakin_node_name = "arm_breakin_node"  # roban_v14_dual
        self.arm_breakin_roban_v17_dual_node_name = "arm_breakin_roban_v17_dual_node"  # roban_v17_dual
        self.arm_breakin_kuavo_v52_dual_node_name = "arm_breakin_kuavo_v52_dual_node"  # kuavo_v52_dual
        self.arm_breakin_kuavo_v53_dual_node_name = "arm_breakin_kuavo_v53_dual_node"  # kuavo_v53_dual
        
        # 进程管理
        self.arm_process = None
        self.roscore_process = None
        self.stop_flag = threading.Event()
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def _find_project_root(self):
        """查找项目根目录（包含 tools/check_tool 的目录）"""
        current = self.current_dir
        
        # 向上查找，直到找到包含 tools/check_tool/motorevo_tool.sh 的目录
        for level in range(10):  # 最多向上查找10级
            # 检查是否存在 tools/check_tool/motorevo_tool.sh
            test_path = current / "tools" / "check_tool" / "motorevo_tool.sh"
            if test_path.exists():
                print(f"{Colors.GREEN}找到项目根目录: {current}{Colors.NC}")
                print(f"{Colors.GREEN}找到 motorevo_tool.sh: {test_path}{Colors.NC}")
                return current
            # 检查是否存在 tools/check_tool 目录
            test_dir = current / "tools" / "check_tool"
            if test_dir.exists() and test_dir.is_dir():
                # 检查 motorevo_tool.sh 是否存在
                motorevo_path = test_dir / "motorevo_tool.sh"
                if motorevo_path.exists():
                    print(f"{Colors.GREEN}找到项目根目录: {current}{Colors.NC}")
                    print(f"{Colors.GREEN}找到 motorevo_tool.sh: {motorevo_path}{Colors.NC}")
                    return current
            # 向上查找
            parent = current.parent
            if parent == current:  # 到达根目录
                break
            current = parent
        
        # 如果找不到，使用默认方法（向上6级）
        # scripts/ -> breakin_control/ -> src/ -> joint_breakin_ros/ -> check_tool/ -> tools/ -> 项目根目录
        default_root = self.current_dir.parent.parent.parent.parent.parent.parent
        print(f"{Colors.YELLOW}警告：使用默认项目根目录: {default_root}{Colors.NC}")
        return default_root
    
    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
    
    def _check_ros_master(self):
        """检查ROS master是否运行"""
        try:
            # 尝试获取ROS master URI
            result = subprocess.run(
                ["rostopic", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=2
            )
            return result.returncode == 0
        except Exception:
            # 如果rostopic命令不存在或失败，尝试其他方法
            try:
                import socket
                from urllib.parse import urlparse
                # 检查ROS_MASTER_URI环境变量
                master_uri = os.environ.get('ROS_MASTER_URI', '')
                if not master_uri:
                    return False
                # 尝试连接ROS master（默认端口11311）
                parsed = urlparse(master_uri)
                host = parsed.hostname or 'localhost'
                port = parsed.port or 11311
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex((host, port))
                sock.close()
                return result == 0
            except Exception:
                return False
    
    def _stop_roscore(self):
        """停止roscore（无论是否由本脚本启动）"""
        
        # 方法1：如果roscore是由本程序启动的，先尝试关闭它
        if self.roscore_process and self.roscore_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
                try:
                    self.roscore_process.wait(timeout=2)
                    self.print_colored("✓ roscore已停止", Colors.GREEN)
                    return
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGKILL)
                    self.print_colored("✓ roscore已强制停止", Colors.GREEN)
                    return
            except Exception as e:
                self.print_colored(f"停止roscore进程失败: {e}", Colors.YELLOW)
        
        # 方法2：使用killall命令关闭所有roscore进程（更可靠）
        try:
            result = subprocess.run(
                ['killall', 'roscore'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=3
            )
            if result.returncode == 0:
                self.print_colored("✓ 已通过killall停止roscore", Colors.GREEN)
        except FileNotFoundError:
            # killall命令不存在，尝试使用pkill
            try:
                subprocess.run(['pkill', '-f', 'roscore'], timeout=3)
                self.print_colored("✓ 已通过pkill停止roscore", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"停止roscore失败: {e}", Colors.YELLOW)
        except Exception as e:
            self.print_colored(f"停止roscore失败: {e}", Colors.YELLOW)
    
    def signal_handler(self, signum, frame):
        """信号处理函数"""
        self.stop_flag.set()
        
        # 发布停止信号到ROS话题
        try:
            import rospy
            from std_msgs.msg import Bool
            pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
            rospy.sleep(0.1)
            stop_msg = Bool()
            stop_msg.data = False
            pub_allow_run.publish(stop_msg)
            rospy.sleep(0.5)
        except:
            pass
        
        if self.arm_process and self.arm_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.arm_process.pid), signal.SIGTERM)
                try:
                    self.arm_process.wait(timeout=5)
                    self.print_colored("✓ 手臂磨线进程已停止", Colors.GREEN)
                except subprocess.TimeoutExpired:
                    self.print_colored("手臂磨线进程未响应，强制终止...", Colors.RED)
                    os.killpg(os.getpgid(self.arm_process.pid), signal.SIGKILL)
                    self.print_colored("✓ 手臂磨线进程已强制终止", Colors.GREEN)
            except Exception as e:
                self.print_colored(f"停止手臂磨线进程失败: {e}", Colors.RED)
        
        # 停止roscore（无论是否由本脚本启动）
        self._stop_roscore()
        
        sys.exit(0)
    
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
                    self.print_colored(f"检测到 ROBOT_VERSION = {version}", Colors.CYAN)
                    return version
        self.print_colored("警告：ROBOT_VERSION 未找到或无效", Colors.YELLOW)
        return None
    
    def check_can_bus_config(self):
        """检查CAN总线配置类型（dual_bus 或 single_bus）"""
        config_dir = os.path.expanduser('~/.config/lejuconfig')
        wiring_type_file = os.path.join(config_dir, 'CanbusWiringType.ini')
        
        if os.path.exists(wiring_type_file):
            try:
                with open(wiring_type_file, 'r') as f:
                    wiring_type = f.read().strip()
                if wiring_type == "dual_bus":
                    self.print_colored(f"检测到CAN总线配置：双总线 (dual_bus)", Colors.CYAN)
                    return "dual_bus"
                elif wiring_type == "single_bus":
                    self.print_colored(f"检测到CAN总线配置：单总线 (single_bus)", Colors.CYAN)
                    return "single_bus"
                else:
                    self.print_colored(f"警告：未知的CAN总线配置类型: {wiring_type}，默认使用单总线", Colors.YELLOW)
                    return "single_bus"
            except Exception as e:
                self.print_colored(f"读取CAN总线配置文件失败: {e}，默认使用单总线", Colors.YELLOW)
                return "single_bus"
        else:
            self.print_colored("未找到CAN总线配置文件，默认使用单总线配置", Colors.YELLOW)
            return "single_bus"
    
    def run_arm_zero_setup(self):
        """运行手臂零点设置"""
        self.print_colored("开始执行手臂零点设置...", Colors.GREEN)
        self.print_colored("注意：请确保手臂已摆正到零点位置", Colors.YELLOW)
        
        try:
            # 步骤1：执行arm_setzero.sh脚本
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
            
            # 步骤2：执行手臂零点设置
            self.print_colored("步骤2：执行手臂零点设置...", Colors.BLUE)
            result1 = subprocess.run(
                ["bash", str(self.arm_setzero_script)],
                cwd=str(self.arm_setzero_script.parent),
                text=True
            )
            
            if result1.returncode != 0:
                self.print_colored("手臂零点设置失败", Colors.RED)
                return False
                
            # 步骤3：执行RUIWO零点设置
            self.print_colored("步骤3：执行RUIWO零点设置...", Colors.BLUE)
            ruiwo_script = self.ruiwo_zero_script if self.ruiwo_zero_script.exists() else self.ruiwo_zero_script_alt
            if not ruiwo_script.exists():
                self.print_colored("错误：RUIWO零点设置脚本不存在", Colors.RED)
                return False
                
            result2 = subprocess.run(
                ["bash", str(ruiwo_script)],
                cwd=str(ruiwo_script.parent),
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
    
    def run_arm_breakin(self, skip_calibration=False):
        """运行手臂磨线
        
        Args:
            skip_calibration: 是否跳过零点校准（默认False，进行校准）
        """
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      手臂磨线独立启动（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()
        
        # 检查root权限
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 arm_breakin_standalone.py", Colors.YELLOW)
            return 1
        
        # 检测CAN总线配置
        can_bus_type = self.check_can_bus_config()
        use_motorevo_cali = (can_bus_type == "dual_bus")
        
        # 根据机器人版本和CAN总线类型选择节点
        robot_version = self.get_robot_version()
        selected_node_name = None
        
        # 调试信息：打印实际检测到的值
        self.print_colored(f"[调试] robot_version = {repr(robot_version)}, can_bus_type = {repr(can_bus_type)}", Colors.BLUE)
        
        if robot_version:
            # 改进的版本号解析：支持多种格式
            version_num = None
            robot_version_str = str(robot_version).strip()
            
            # 去除可能的引号
            if robot_version_str.startswith('"') and robot_version_str.endswith('"'):
                robot_version_str = robot_version_str[1:-1]
            elif robot_version_str.startswith("'") and robot_version_str.endswith("'"):
                robot_version_str = robot_version_str[1:-1]
            
            # 尝试直接转换为整数
            try:
                version_num = int(robot_version_str)
            except (ValueError, TypeError):
                # 如果直接转换失败，尝试从字符串中提取版本号（如 "kuavo5_v53" -> 53）
                import re
                # 匹配 v52, v53, v14 等格式
                match = re.search(r'v(\d+)', robot_version_str.lower())
                if match:
                    try:
                        version_num = int(match.group(1))
                    except (ValueError, TypeError):
                        pass
                # 如果还是没找到，尝试匹配纯数字（如 "kuavo5_v53" 中的 53）
                if version_num is None:
                    match = re.search(r'(\d+)', robot_version_str)
                    if match:
                        try:
                            version_num = int(match.group(1))
                        except (ValueError, TypeError):
                            pass
            
            if version_num is not None:
                # 确保 can_bus_type 去除空格
                can_bus_type_clean = str(can_bus_type).strip()
                self.print_colored(f"[调试] version_num = {version_num}, can_bus_type_clean = {repr(can_bus_type_clean)}", Colors.BLUE)
                
                if version_num == 14 and can_bus_type_clean == "dual_bus":
                    # roban_v14_dual 机型
                    selected_node_name = self.arm_breakin_node_name
                    self.print_colored(f"检测到机型：roban_v14_dual (版本 {version_num}, {can_bus_type})", Colors.GREEN)
                elif version_num == 17 and can_bus_type == "dual_bus":
                    # roban_v17_dual 机型
                    selected_node_name = self.arm_breakin_roban_v17_dual_node_name
                    self.print_colored(f"检测到机型：roban_v17_dual (版本 {version_num}, {can_bus_type})", Colors.GREEN)
                elif version_num == 52 and can_bus_type == "dual_bus":
                    # kuavo_v52_dual 机型
                    selected_node_name = self.arm_breakin_kuavo_v52_dual_node_name
                    self.print_colored(f"检测到机型：kuavo_v52_dual (版本 {version_num}, {can_bus_type})", Colors.GREEN)
                elif version_num == 53 and can_bus_type == "dual_bus":
                    # kuavo_v53_dual 机型
                    selected_node_name = self.arm_breakin_kuavo_v53_dual_node_name
                    self.print_colored(f"检测到机型：kuavo_v53_dual (版本 {version_num}, {can_bus_type})", Colors.GREEN)
                else:
                    # 未匹配到任何配置，直接报错退出
                    self.print_colored(f"错误：版本 {version_num} 和 CAN 总线类型 {can_bus_type_clean} 的组合未明确配置", Colors.RED)
                    self.print_colored("支持的配置组合：", Colors.RED)
                    self.print_colored("  - 版本 14 + dual_bus -> roban_v14_dual", Colors.RED)
                    self.print_colored("  - 版本 52 + dual_bus -> kuavo_v52_dual", Colors.RED)
                    self.print_colored("  - 版本 53 + dual_bus -> kuavo_v53_dual", Colors.RED)
                    return 1
            else:
                # 无法解析版本号，直接报错退出
                self.print_colored(f"错误：无法解析版本号 {robot_version}", Colors.RED)
                self.print_colored("请检查 ROBOT_VERSION 环境变量或 .bashrc 中的配置", Colors.RED)
                return 1
        else:
            # 未找到版本号，直接报错退出
            self.print_colored("错误：未找到 ROBOT_VERSION", Colors.RED)
            self.print_colored("请设置 ROBOT_VERSION 环境变量或在 /home/lab/.bashrc 中配置", Colors.RED)
            return 1
        
        if not selected_node_name:
            # 兜底检查：如果还是没有选择节点，报错退出
            self.print_colored("错误：未能选择手臂磨线节点", Colors.RED)
            return 1
        
        # 根据用户选择决定是否进行零点校准
        if skip_calibration:
            self.print_colored("跳过零点校准", Colors.CYAN)
        elif use_motorevo_cali:
            # 双CAN配置：使用 motorevo_tool.sh --cali
            self.print_colored("双CAN配置：使用电机校准 (motorevo_tool.sh --cali)", Colors.CYAN)
            self.print_colored(f"查找 motorevo_tool.sh: {self.motorevo_tool_sh}", Colors.BLUE)
            try:
                if not self.motorevo_tool_sh.exists():
                    self.print_colored(f"错误：未找到校准脚本 {self.motorevo_tool_sh}", Colors.RED)
                    self.print_colored(f"项目根目录: {self.project_root}", Colors.YELLOW)
                    # 尝试查找其他可能的位置
                    alt_paths = [
                        self.project_root.parent / "tools" / "check_tool" / "motorevo_tool.sh",
                        Path("/home/lab/wang/kuavo-ros-control/tools/check_tool/motorevo_tool.sh"),
                    ]
                    for alt_path in alt_paths:
                        if alt_path.exists():
                            self.print_colored(f"找到备用路径: {alt_path}", Colors.GREEN)
                            self.motorevo_tool_sh = alt_path
                            break
                    else:
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
            # 单CAN配置：使用 run_arm_zero_setup
            if not skip_calibration:
                self.print_colored("单CAN配置：在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
                if not self.run_arm_zero_setup():
                    self.print_colored("零点设置失败，无法继续执行手臂磨线", Colors.RED)
                    return 1
            else:
                self.print_colored("单CAN配置：跳过零点设置", Colors.CYAN)
        
        print()
        self.print_colored("开始执行手臂磨线！", Colors.GREEN)
        self.print_colored("提示：在执行过程中，按 Ctrl+C 可以安全停止程序", Colors.CYAN)
        print()
        
        # 检查ROS master是否运行
        if not self._check_ros_master():
            self.print_colored("警告：ROS master未运行", Colors.YELLOW)
            self.print_colored("正在自动启动roscore...", Colors.BLUE)
            
            # 启动roscore（后台运行）
            self.roscore_process = subprocess.Popen(
                ["roscore"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # 等待roscore启动
            self.print_colored("等待roscore启动（最多15秒）...", Colors.BLUE)
            for i in range(15):
                time.sleep(1)
                if self._check_ros_master():
                    self.print_colored("\n✓ ROS master已启动", Colors.GREEN)
                    # 额外等待1秒确保roscore完全就绪
                    time.sleep(1)
                    break
                if i < 14:
                    print(".", end="", flush=True)
            else:
                self.print_colored("\n错误：无法启动ROS master，请手动启动roscore", Colors.RED)
                self.print_colored("请使用以下命令启动roscore：", Colors.YELLOW)
                self.print_colored("  roscore &", Colors.CYAN)
                if self.roscore_process:
                    try:
                        self.roscore_process.terminate()
                    except:
                        pass
                return 1
        else:
            self.print_colored("✓ ROS master已在运行", Colors.GREEN)
        
        # 确保ROS环境变量正确设置
        ros_master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        if not ros_master_uri or ros_master_uri == '':
            ros_master_uri = 'http://localhost:11311'
            os.environ['ROS_MASTER_URI'] = ros_master_uri
        self.print_colored(f"ROS_MASTER_URI: {ros_master_uri}", Colors.BLUE)
        
        # 查找ROS工作空间
        ros_workspace = os.environ.get('ROS_WORKSPACE', None)
        if not ros_workspace:
            # 尝试从当前目录向上查找catkin工作空间
            current = Path.cwd()
            while current != current.parent:
                if (current / "devel" / "setup.bash").exists():
                    ros_workspace = str(current)
                    break
                current = current.parent
        
        if not ros_workspace:
            self.print_colored("错误：未找到ROS工作空间，请先source setup.bash", Colors.RED)
            return 1
        
        # 启动ROS节点
        roscore_started_by_this_script = False  # 在try块外定义，以便finally块访问
        try:
            import rospy
            from std_msgs.msg import Bool
            
            # 检查roscore是否已运行（主控制器可能已经启动）
            if not self._check_ros_master():
                # roscore未运行，需要启动
                if not self._start_roscore():
                    return 1
                roscore_started_by_this_script = True
                self.print_colored("✓ 已启动roscore", Colors.GREEN)
            else:
                self.print_colored("✓ ROS master已运行（由主控制器启动）", Colors.GREEN)
            
            # 初始化ROS节点（用于发布启动信号）
            rospy.init_node('arm_breakin_standalone_launcher', anonymous=True)
            
            # 检查standalone_mode是否已发布（主控制器可能已经发布）
            standalone_mode_already_published = False
            try:
                # 尝试订阅standalone_mode话题，检查是否已有消息
                standalone_mode_received = [False]
                def standalone_mode_callback(msg):
                    standalone_mode_received[0] = True
                
                temp_sub = rospy.Subscriber('/breakin/standalone_mode', Bool, standalone_mode_callback)
                rospy.sleep(0.5)  # 等待消息
                temp_sub.unregister()
                
                if standalone_mode_received[0]:
                    standalone_mode_already_published = True
                    self.print_colored("✓ standalone_mode已由主控制器发布", Colors.GREEN)
            except:
                pass
            
            # 创建发布者（1Hz发布allow_run，can_start_new_round由主程序发布）
            pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
            
            # 如果standalone_mode未发布，则发布它
            if not standalone_mode_already_published:
                pub_standalone_mode = rospy.Publisher('/breakin/standalone_mode', Bool, queue_size=10, latch=True)
                rospy.sleep(0.5)
                standalone_msg = Bool()
                standalone_msg.data = True
                pub_standalone_mode.publish(standalone_msg)
                self.print_colored("✓ 已发布 standalone_mode = True（单独运行模式）", Colors.GREEN)
            
            # 创建订阅者（监听手臂是否已开始）
            arm_started = False
            def arm_started_callback(msg):
                nonlocal arm_started
                arm_started = msg.data
            
            sub_arm_started = rospy.Subscriber('/breakin/arm_started', Bool, arm_started_callback)
            
            # 等待发布者注册
            rospy.sleep(0.5)
            
            # 先启动C++节点，然后等待它开始运行
            # 查找并组合全局工作空间和本地 joint_breakin_ros 工作空间
            script_dir = Path(__file__).parent.absolute()
            # joint_breakin_ros 根目录：joint_breakin_ros/src/breakin_control/scripts/ -> joint_breakin_ros
            local_ws = script_dir.parent.parent.parent
            global_ws = None

            # 通过 _find_project_root() 找到的项目根
            # 这里优先使用该根目录作为全局工作空间
            if self.project_root and (self.project_root / "devel" / "setup.bash").exists():
                global_ws = self.project_root

            local_setup = local_ws / "devel" / "setup.bash"
            global_setup = global_ws / "devel" / "setup.bash" if global_ws else None

            # 优先检查 build_lib 目录中是否有预编译的二进制文件
            build_lib_dir = local_ws / "build_lib"
            arm_breakin_node_path = None
            if build_lib_dir.is_dir():
                # 检查可能的路径（使用选定的节点名称）
                possible_paths = [
                    build_lib_dir / "devel" / "lib" / "arm_breakin" / selected_node_name,
                    build_lib_dir / "lib" / "arm_breakin" / selected_node_name,
                    build_lib_dir / "arm_breakin" / selected_node_name,
                    build_lib_dir / selected_node_name,
                ]
                for path in possible_paths:
                    if path.exists() and path.is_file():
                        arm_breakin_node_path = path
                        self.print_colored(f"✓ 在 build_lib 中找到二进制文件: {arm_breakin_node_path}", Colors.GREEN)
                        break

            setup_cmd_parts = []
            if global_setup and global_setup.exists():
                setup_cmd_parts.append(f"source {global_setup}")
            if local_setup.exists():
                setup_cmd_parts.append(f"source {local_setup}")

            # 如果 build_lib 中有 setup.bash，也 source 它
            build_lib_setup_paths = [
                build_lib_dir / "devel" / "setup.bash",
                build_lib_dir / "setup.bash",
            ]
            for build_lib_setup in build_lib_setup_paths:
                if build_lib_setup.exists():
                    setup_cmd_parts.append(f"source {build_lib_setup}")
                    self.print_colored(f"✓ 使用 build_lib 中的 setup.bash: {build_lib_setup}", Colors.GREEN)
                    break

            if not setup_cmd_parts:
                self.print_colored("错误：未找到可用的ROS工作空间，请先编译并source相应的devel/setup.bash", Colors.RED)
                return 1

            # 如果找到 build_lib 中的二进制文件，直接运行它；否则使用 rosrun
            if arm_breakin_node_path:
                # 直接运行二进制文件
                node_cmd = str(arm_breakin_node_path)
                setup_chain = " && ".join(setup_cmd_parts + [node_cmd])
                cmd = ["bash", "-c", setup_chain]
                self.print_colored(f"使用 build_lib 中的二进制文件: {arm_breakin_node_path}", Colors.BLUE)
            else:
                # 使用 rosrun（需要完整的 ROS 包结构）
                setup_chain = " && ".join(setup_cmd_parts + [f"rosrun arm_breakin {selected_node_name}"])
                cmd = ["bash", "-c", setup_chain]
                self.print_colored(f"使用 rosrun 启动节点 {selected_node_name}（需要完整的 ROS 包结构）", Colors.BLUE)
            
            # 设置ROS环境变量
            env = dict(os.environ)
            # 确保ROS环境变量正确传递
            if 'ROS_MASTER_URI' not in env or not env['ROS_MASTER_URI']:
                env['ROS_MASTER_URI'] = 'http://localhost:11311'
            
            # 如果使用 build_lib 中的二进制文件，需要设置库路径
            if arm_breakin_node_path:
                # 查找可能的库目录
                lib_dirs = []
                possible_lib_paths = [
                    build_lib_dir / "devel" / "lib",
                    build_lib_dir / "lib",
                ]
                for lib_path in possible_lib_paths:
                    if lib_path.exists() and lib_path.is_dir():
                        lib_dirs.append(str(lib_path))
                
                # 如果找到库目录，添加到 LD_LIBRARY_PATH
                if lib_dirs:
                    existing_ld_path = env.get('LD_LIBRARY_PATH', '')
                    new_ld_path = ':'.join(lib_dirs)
                    if existing_ld_path:
                        env['LD_LIBRARY_PATH'] = f"{new_ld_path}:{existing_ld_path}"
                    else:
                        env['LD_LIBRARY_PATH'] = new_ld_path
                    self.print_colored(f"设置 LD_LIBRARY_PATH: {env['LD_LIBRARY_PATH']}", Colors.BLUE)
            if 'ROS_IP' not in env:
                # 尝试获取本机IP
                try:
                    import socket
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    env['ROS_IP'] = s.getsockname()[0]
                    s.close()
                except:
                    pass
            
            # 打印实际使用的工作空间信息
            if global_setup and global_setup.exists():
                self.print_colored(f"使用全局ROS工作空间: {global_ws}", Colors.BLUE)
            if local_setup.exists():
                self.print_colored(f"使用本地ROS工作空间: {local_ws}", Colors.BLUE)
            
            self.print_colored("正在启动ROS手臂磨线节点...", Colors.BLUE)
            self.print_colored(f"启动命令: {' '.join(cmd)}", Colors.BLUE)
            self.print_colored(f"ROS_MASTER_URI: {env.get('ROS_MASTER_URI', '未设置')}", Colors.BLUE)
            
            # 再次确认roscore可用
            if not self._check_ros_master():
                self.print_colored("错误：roscore不可用，无法启动节点", Colors.RED)
                return 1
            
            # 启动进程
            self.print_colored(f"执行命令: {' '.join(cmd)}", Colors.BLUE)
            self.arm_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,  # 行缓冲
                env=env,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            self.print_colored(f"节点进程已启动，PID: {self.arm_process.pid}", Colors.GREEN)
            
            # 立即启动输出读取线程，以便看到节点输出和错误
            def read_output():
                try:
                    for line in iter(self.arm_process.stdout.readline, ''):
                        if not line:
                            break
                        print(line.rstrip())
                        sys.stdout.flush()
                except Exception as e:
                    self.print_colored(f"读取输出出错: {e}", Colors.RED)
            
            output_thread = threading.Thread(target=read_output, daemon=True)
            output_thread.start()
            
            # 等待节点启动（给ros::init()时间）
            time.sleep(2.0)
            
            # 检查进程是否还在运行
            if self.arm_process.poll() is not None:
                self.print_colored(f"错误：手臂节点启动失败，已退出，返回码: {self.arm_process.returncode}", Colors.RED)
                self.print_colored("请检查上面的节点输出，查看错误信息", Colors.YELLOW)
                return 1
            
            # 启动1Hz发布allow_run线程（can_start_new_round由主程序发布）
            stop_publish_flag = threading.Event()
            
            def publish_allow_run_loop():
                """1Hz发布allow_run话题的循环"""
                rate = rospy.Rate(1)  # 1Hz
                while not stop_publish_flag.is_set() and not rospy.is_shutdown():
                    # 持续发布allow_run = True
                    allow_msg = Bool()
                    allow_msg.data = True
                    pub_allow_run.publish(allow_msg)
                    rate.sleep()
            
            publish_thread = threading.Thread(target=publish_allow_run_loop, daemon=True)
            publish_thread.start()
            
            self.print_colored("✓ 已启动1Hz allow_run话题发布线程", Colors.GREEN)
            
            # 等待手臂节点开始运行（发布arm_started = True）
            self.print_colored("等待手臂节点开始运行...", Colors.BLUE)
            start_wait_timeout = 30.0  # 最多等待30秒
            start_wait_start = time.time()
            while not arm_started and (time.time() - start_wait_start) < start_wait_timeout:
                # 检查进程是否还在运行
                if self.arm_process.poll() is not None:
                    self.print_colored(f"错误：手臂节点已退出，返回码: {self.arm_process.returncode}", Colors.RED)
                    self.print_colored("请检查上面的节点输出，查看错误信息", Colors.YELLOW)
                    return 1
                # rospy的回调在后台线程自动处理，只需要sleep即可
                rospy.sleep(0.1)
            
            if not arm_started:
                self.print_colored("错误：等待手臂节点开始运行超时", Colors.RED)
                # 检查进程是否还在运行
                if self.arm_process.poll() is not None:
                    self.print_colored(f"手臂节点已退出，返回码: {self.arm_process.returncode}", Colors.RED)
                else:
                    self.print_colored("提示：节点仍在运行但未发布arm_started，请检查节点输出", Colors.YELLOW)
                return 1
            
            # 收到arm_started后，确认节点已开始运行
            self.print_colored("✓ 手臂节点已开始运行", Colors.GREEN)
            
            # 输出读取线程已在启动节点后立即启动，无需重复启动
            
            # 监控进程循环（只检查进程是否退出，不检查时间）
            while True:
                # 检查进程是否还在运行
                if self.arm_process.poll() is not None:
                    # 进程已退出
                    return_code = self.arm_process.returncode
                    break
                
                # 每0.1秒检查一次
                time.sleep(0.1)
            
            # 停止发布线程
            stop_publish_flag.set()
            publish_thread.join(timeout=1.0)
            
            # 发布停止信号
            stop_msg = Bool()
            stop_msg.data = False
            pub_allow_run.publish(stop_msg)
            rospy.sleep(0.5)  # 确保消息发送
            
            if return_code == 0:
                self.print_colored("✓ 手臂磨线完成", Colors.GREEN)
            else:
                self.print_colored(f"手臂磨线进程退出，返回码: {return_code}", Colors.YELLOW)
            
            # 停止roscore（无论是否由本脚本启动）
            self._stop_roscore()
            
            return return_code
            
        except ImportError:
            self.print_colored("错误：需要ROS环境，请先source setup.bash", Colors.RED)
            return 1
        except Exception as e:
            self.print_colored(f"手臂磨线运行出错: {e}", Colors.RED)
            import traceback
            traceback.print_exc()
            return 1

def main():
    try:
        # 检查环境变量或命令行参数，决定是否跳过校准
        skip_calibration = False
        
        # 方式1：环境变量（主控制器传递）
        if os.environ.get('SKIP_ARM_CALIBRATION', '').lower() in ('true', '1', 'yes'):
            skip_calibration = True
        
        # 方式2：命令行参数（直接运行时）
        if len(sys.argv) > 1:
            if sys.argv[1] in ('--skip-calibration', '--skip-cali', '-s'):
                skip_calibration = True
        
        app = ArmBreakinStandalone()
        exit_code = app.run_arm_breakin(skip_calibration=skip_calibration)
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}程序被用户中断{Colors.NC}")
        sys.exit(0)
    except Exception as e:
        print(f"{Colors.RED}程序运行出错: {e}{Colors.NC}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()

