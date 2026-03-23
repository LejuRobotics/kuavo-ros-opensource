#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磨线主控制器（ROS版本）
提供菜单选择，支持单独手臂磨线、单独腿部磨线、同时磨线
"""

import os
import sys
import subprocess
import signal
import time
import threading
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

class BreakinMainController:
    def __init__(self):
        # 获取当前脚本所在目录
        self.current_dir = Path(__file__).parent.absolute()
        # 自动向上查找 ROS 工作空间根目录（包含 src 子目录的那个目录）
        self.workspace_root = self._detect_workspace_root()
        # leg_breakin 目录和 EC_log 路径
        self.leg_breakin_dir = None
        self.leg_ec_log_dir = None
        
        # 脚本路径
        self.arm_breakin_standalone = self.current_dir / "arm_breakin_standalone.py"
        self.leg_breakin_standalone = self.current_dir / "leg_breakin_standalone.py"
        self.breakin_controller_simple = self.current_dir / "breakin_controller_simple.py"
        
        # 进程管理
        self.processes = []
        self.roscore_process = None
        self.ros_node_initialized = False
        self.ros_publishers = {}
        self.ros_node = None
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def _get_robot_version(self):
        """获取 ROBOT_VERSION：优先环境变量，其次读取 /home/lab/.bashrc"""
        # 优先从环境变量读取
        rv = os.environ.get("ROBOT_VERSION")
        if rv:
            rv_clean = str(rv).strip().strip('"').strip("'")
            if rv_clean:
                return rv_clean
        
        # 从 .bashrc 文件读取
        try:
            home_dir = os.path.expanduser('/home/lab/')
            bashrc_path = os.path.join(home_dir, '.bashrc')
            if os.path.exists(bashrc_path):
                with open(bashrc_path, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                # 从后往前查找，找到最后一个未注释的 export ROBOT_VERSION=
                for line in reversed(lines):
                    # 去除首尾空白字符
                    s = line.strip()
                    # 跳过空行和注释行（以 # 开头的行）
                    if not s or s.startswith('#'):
                        continue
                    # 检查是否包含 export ROBOT_VERSION=
                    if 'export ROBOT_VERSION=' in s or s.startswith('export ROBOT_VERSION='):
                        # 提取变量值，支持多种格式：export ROBOT_VERSION=17 或 export ROBOT_VERSION="17"
                        parts = s.split('=', 1)
                        if len(parts) == 2:
                            value = parts[1].strip().strip('"').strip("'")
                            if value:
                                return value
        except Exception:
            pass
        
        return ""
    
    def _is_kuavo5(self):
        """根据 ROBOT_VERSION 判断是否为 Kuavo5（版本50+）腿部磨线方案"""
        robot_version = self._get_robot_version()
        if not robot_version:
            return False
        
        rv_raw = str(robot_version).strip()
        rv = rv_raw.lower()
        
        # 字符串匹配：kuavo5_v52, kuavo5_v53, kuavo5, v5, kuavo5_v5 等
        if ("kuavo5_v52" in rv) or ("kuavo5_v53" in rv) or ("kuavo5" in rv) or rv.startswith("v5") or rv.startswith("kuavo5_v5"):
            return True
        
        # 数字版本判断：50+ 为 Kuavo5
        try:
            v = int(rv_raw)
            return v >= 50
        except (ValueError, TypeError):
            return False
    
    def _is_robot_version_52(self):
        """判断 ROBOT_VERSION 是否为 52"""
        robot_version = self._get_robot_version()
        if not robot_version:
            return False
        
        rv_raw = str(robot_version).strip()
        rv = rv_raw.lower()
        
        # 字符串匹配：包含 v52 或 kuavo5_v52
        if "v52" in rv or "kuavo5_v52" in rv:
            return True
        
        # 数字版本判断：52
        try:
            v = int(rv_raw)
            return v == 52
        except (ValueError, TypeError):
            return False
    
    def _select_leg_breakin_dir(self):
        """根据 ROBOT_VERSION 选择腿部磨线目录
        如果是版本52，询问用户选择机型
        如果是版本50-51，使用普通v52版本
        如果是版本17，使用 roban2_v17
        否则使用 roban2_v14
        返回选择的目录名称
        """
        # 如果不是 Kuavo5，根据版本选择
        if not self._is_kuavo5():
            robot_version = self._get_robot_version()
            if robot_version:
                try:
                    # 去除可能的引号和空白字符
                    version_str = str(robot_version).strip().strip('"').strip("'")
                    version_num = int(version_str)
                    if version_num == 17:
                        return "leg_breakin_roban2_v17"
                    elif 13 <= version_num <= 14:
                        return "leg_breakin_roban2_v14"
                except (ValueError, TypeError) as e:
                    self.print_colored(f"警告：无法解析版本号 '{robot_version}'，使用默认 roban2_v14: {e}", Colors.YELLOW)
            # 默认使用 roban2_v14
            return "leg_breakin_roban2_v14"
        
        # 如果是版本52，询问用户选择机型
        if self._is_robot_version_52():
            self.print_colored("=" * 50, Colors.CYAN)
            self.print_colored("      检测到 ROBOT_VERSION = 52", Colors.CYAN)
            self.print_colored("=" * 50, Colors.CYAN)
            print()
            self.print_colored("请选择机型：", Colors.YELLOW)
            print("1. 龙华25台版本（腿部80A）")
            print("2. 普通v52版本")
            print()
            
            while True:
                try:
                    choice = input("请输入选项 (1 或 2): ").strip()
                    if choice == "1":
                        self.print_colored("已选择：龙华25台版本（腿部80A）", Colors.GREEN)
                        return "leg_breakin_kuavo5_v52_80A"
                    elif choice == "2":
                        self.print_colored("已选择：普通v52版本", Colors.GREEN)
                        return "leg_breakin_kuavo5_v52"
                    elif choice.lower() == 'q':
                        self.print_colored("已取消操作", Colors.YELLOW)
                        sys.exit(0)
                    else:
                        self.print_colored("无效选项，请输入 1 或 2", Colors.RED)
                except KeyboardInterrupt:
                    self.print_colored("\n已取消操作", Colors.YELLOW)
                    sys.exit(0)
        
        # 如果是版本50-51，使用普通v52版本
        return "leg_breakin_kuavo5_v52"
    
    def _ensure_leg_breakin_dir_selected(self):
        """确保腿部磨线目录已选择（如果尚未选择，则进行选择）"""
        if self.leg_breakin_dir is None:
            self.leg_breakin_dir = self._select_leg_breakin_dir()
            self.leg_ec_log_dir = (
                self.workspace_root
                / "src"
                / "leg_breakin"
                / "src"
                / self.leg_breakin_dir
                / "EC_log"
            )

    def _detect_workspace_root(self):
        """从当前脚本目录向上查找，找到包含 src 目录的 ROS 工作空间根目录"""
        path = self.current_dir
        for _ in range(6):  # 最多向上找6层，防止死循环
            candidate_src = path / "src"
            if candidate_src.is_dir():
                return path
            if path.parent == path:
                break
            path = path.parent

        # 兜底：找不到就使用当前推断的 joint_breakin_ros 目录（兼容老路径）
        return self.current_dir.parent.parent.parent

    def _ensure_workspace_built(self):
        """确保当前ROS工作空间已经catkin_make过
        - 判断依据：优先检查 build_lib/lib 中是否存在关键编译产物
        - 其次检查：workspace_root 下是否存在 build/ 和 devel/setup.bash
        - 如果没有，则调用同目录下的 build_catkin_workspace.py 进行编译
        """
        build_lib_dir = self.workspace_root / "build_lib" / "lib"
        build_dir = self.workspace_root / "build"
        devel_setup = self.workspace_root / "devel" / "setup.bash"

        # 优先检查 build_lib 中是否存在预编译的库文件
        if build_lib_dir.exists():
            # 检查关键库文件是否存在
            key_files = [
                build_lib_dir / "libcanbus_sdk.so",
                build_lib_dir / "libmotorevo_controller.so",
                build_lib_dir / "arm_breakin" / "arm_breakin_node",
            ]
            found_files = [f for f in key_files if f.exists()]
            if len(found_files) > 0:
                self.print_colored(f"✓ 检测到 build_lib/lib 目录及其中的编译产物（找到 {len(found_files)} 个关键文件）", Colors.GREEN)
                return True

        # 如果 build_lib 不存在，检查传统的编译标志（向后兼容）
        if build_dir.is_dir() and devel_setup.exists():
            return True

        self.print_colored("检测到ROS工作空间尚未编译，正在编译，请稍等...", Colors.BLUE)
        self.print_colored(f"工作空间路径: {self.workspace_root}", Colors.BLUE)

        build_script = self.current_dir / "build_catkin_workspace.py"
        if not build_script.exists():
            self.print_colored(f"未找到编译脚本: {build_script}", Colors.RED)
            return False

        try:
            # 调用独立编译脚本，cwd 设为脚本所在目录（脚本内部会自动定位工作空间根）
            result = subprocess.run(
                [sys.executable, str(build_script)],
                cwd=str(self.current_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            # 打印部分编译输出，方便排查问题（不全部刷屏）
            if result.stdout:
                lines = result.stdout.splitlines()
                preview_lines = lines[-40:] if len(lines) > 40 else lines
                for line in preview_lines:
                    print(line)

            if result.returncode == 0:
                self.print_colored("✓ 工作空间编译成功", Colors.GREEN)
                return True
            else:
                self.print_colored("✗ 工作空间编译失败，请检查上面的输出日志", Colors.RED)
                return False
        except FileNotFoundError:
            # 找不到python命令或脚本
            self.print_colored("错误：无法调用 build_catkin_workspace.py，请确认 Python 环境和脚本存在", Colors.RED)
            return False
        except Exception as e:
            self.print_colored(f"执行编译脚本失败: {e}", Colors.RED)
            return False

    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
    
    def _kill_existing_processes(self):
        """杀掉之前残留的磨线相关进程"""
        
        # 要清理的进程名称列表
        process_names = [
            "arm_breakin_node",           # C++手臂磨线节点
            "arm_breakin_standalone.py",  # Python手臂磨线脚本
            "leg_breakin_standalone.py",  # Python腿部磨线脚本
            "breakin_controller_simple.py",  # 主控制器脚本
            "roban2_leg_breakin.py",      # 腿部磨线脚本
            "joint_breakin.py",           # 腿部磨线脚本（备用名称）
        ]
        
        killed_count = 0
        for proc_name in process_names:
            try:
                # 使用pkill命令杀掉匹配的进程
                result = subprocess.run(
                    ['pkill', '-f', proc_name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=2
                )
                if result.returncode == 0:
                    self.print_colored(f"  ✓ 已杀掉进程: {proc_name}", Colors.GREEN)
                    killed_count += 1
            except FileNotFoundError:
                # pkill命令不存在，尝试使用killall
                try:
                    result = subprocess.run(
                        ['killall', proc_name],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        timeout=2
                    )
                    if result.returncode == 0:
                        self.print_colored(f"  ✓ 已杀掉进程: {proc_name}", Colors.GREEN)
                        killed_count += 1
                except FileNotFoundError:
                    # killall也不存在，跳过
                    pass
            except Exception as e:
                # 忽略其他错误（进程可能不存在）
                pass
        
        # 等待进程完全退出
        if killed_count > 0:
            self.print_colored(f"等待 {killed_count} 个进程退出...", Colors.BLUE)
            time.sleep(1.0)
        
        if killed_count > 0:
            self.print_colored(f"✓ 已清理 {killed_count} 个残留进程", Colors.GREEN)

    def _ensure_leg_ec_log_dir(self):
        """确保腿部磨线的 EC_log 目录存在"""
        # 确保已选择腿部磨线目录
        self._ensure_leg_breakin_dir_selected()
        
        try:
            if not self.leg_ec_log_dir.exists():
                self.leg_ec_log_dir.mkdir(parents=True, exist_ok=True)
                self.print_colored(
                    f"✓ 已创建腿部磨线日志目录: {self.leg_ec_log_dir}", Colors.GREEN
                )
            else:
                self.print_colored(
                    f"✓ 检测到腿部磨线日志目录: {self.leg_ec_log_dir}", Colors.GREEN
                )
            return True
        except Exception as e:
            self.print_colored(
                f"✗ 创建腿部磨线日志目录失败: {self.leg_ec_log_dir}，错误: {e}",
                Colors.RED,
            )
            return False
    
    def signal_handler(self, signum, frame):
        """信号处理函数"""
        self.print_colored(f"\n收到停止信号，正在停止所有进程...", Colors.YELLOW)
        self.cleanup_processes()
        sys.exit(0)
    
    def cleanup_processes(self):
        """清理所有进程"""
        # 停止ROS发布
        if self.ros_node_initialized:
            try:
                import rospy
                if not rospy.is_shutdown():
                    # 发布停止信号
                    if 'pub_allow_run' in self.ros_publishers:
                        msg = self.ros_publishers['msg_type']()
                        msg.data = False
                        self.ros_publishers['pub_allow_run'].publish(msg)
                    rospy.signal_shutdown("主控制器退出")
            except:
                pass
        
        # 停止子进程
        for process in self.processes:
            if process and process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except:
                        pass
        
        # 停止roscore
        if self.roscore_process and self.roscore_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGTERM)
                self.roscore_process.wait(timeout=3)
            except:
                try:
                    os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGKILL)
                except:
                    pass
    
    def show_menu(self):
        """显示菜单"""
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      磨线主控制器（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()
        self.print_colored("请选择启动模式：", Colors.YELLOW)
        print("1. 仅启动手臂磨线")
        print("2. 仅启动腿部磨线")
        print("3. 同时启动手臂和腿部磨线")
        print("q. 退出")
        print()
    
    def _check_ros_master(self):
        """检查ROS master是否运行"""
        try:
            import rosgraph
            return rosgraph.is_master_online()
        except:
            return False
    
    def _start_roscore(self):
        """启动roscore（如果未运行）"""
        if self._check_ros_master():
            self.print_colored("✓ ROS master已运行", Colors.GREEN)
            return True
        
        self.print_colored("正在启动roscore...", Colors.BLUE)
        try:
            env = dict(os.environ)
            if 'ROS_MASTER_URI' not in env or not env['ROS_MASTER_URI']:
                env['ROS_MASTER_URI'] = 'http://localhost:11311'
            env.setdefault("PWD", str(self.workspace_root))

            # 构造 roscore 命令，优先直接找 roscore，其次尝试 source 常见的 setup.bash
            roscore_cmd = None
            roscore_path = shutil.which("roscore", path=env.get("PATH", ""))
            if roscore_path:
                roscore_cmd = [roscore_path]
            else:
                ros_distro = env.get("ROS_DISTRO")
                setup_candidates = []
                if ros_distro:
                    setup_candidates.append(f"/opt/ros/{ros_distro}/setup.bash")
                setup_candidates.extend([
                    "/opt/ros/noetic/setup.bash",
                    "/opt/ros/melodic/setup.bash",
                    "/opt/ros/kinetic/setup.bash",
                ])
                for setup in setup_candidates:
                    if Path(setup).exists():
                        roscore_cmd = ["bash", "-c", f"source {setup} && roscore"]
                        break

            if roscore_cmd is None:
                self.print_colored("错误：未找到 roscore。请先 source /opt/ros/<distro>/setup.bash 或安装 ROS。", Colors.RED)
                return False

            self.roscore_process = subprocess.Popen(
                roscore_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=env
            )
            
            # 等待roscore启动
            self.print_colored("等待roscore启动（最多10秒）...", Colors.BLUE)
            for _ in range(100):  # 最多等待10秒
                time.sleep(0.1)
                if self._check_ros_master():
                    self.print_colored("✓ ROS master已启动", Colors.GREEN)
                    return True
            
            self.print_colored("错误：roscore启动超时", Colors.RED)
            return False
            
        except Exception as e:
            self.print_colored(f"启动roscore失败: {e}", Colors.RED)
            return False
    
    def _init_ros_node(self, mode):
        """初始化ROS节点并发布模式话题"""
        try:
            import rospy
            from std_msgs.msg import Bool
            
            # 初始化ROS节点
            rospy.init_node('breakin_main_controller', anonymous=True)
            self.ros_node_initialized = True
            
            # 根据模式发布话题
            if mode == "arm_only":
                # 发布standalone_mode = True
                pub_standalone = rospy.Publisher('/breakin/standalone_mode', Bool, queue_size=10, latch=True)
                self.ros_publishers['pub_standalone_mode'] = pub_standalone
                
                # 创建allow_run和can_start_new_round发布者（稍后在循环中使用）
                pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
                pub_can_start_new_round = rospy.Publisher('/breakin/can_start_new_round', Bool, queue_size=10)
                self.ros_publishers['pub_allow_run'] = pub_allow_run
                self.ros_publishers['pub_can_start_new_round'] = pub_can_start_new_round
                self.ros_publishers['msg_type'] = Bool
                
                # 等待发布者注册
                rospy.sleep(0.5)
                
                # 发布standalone_mode = True
                standalone_msg = Bool()
                standalone_msg.data = True
                pub_standalone.publish(standalone_msg)
                self.print_colored("✓ 已发布 standalone_mode = True（单独运行模式）", Colors.GREEN)
                
            elif mode == "leg_only":
                # 发布standalone_mode = True
                pub_standalone = rospy.Publisher('/breakin/standalone_mode', Bool, queue_size=10, latch=True)
                self.ros_publishers['pub_standalone_mode'] = pub_standalone
                
                # 创建allow_run和can_start_new_round发布者（稍后在循环中使用）
                pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
                pub_can_start_new_round = rospy.Publisher('/breakin/can_start_new_round', Bool, queue_size=10)
                self.ros_publishers['pub_allow_run'] = pub_allow_run
                self.ros_publishers['pub_can_start_new_round'] = pub_can_start_new_round
                self.ros_publishers['msg_type'] = Bool
                
                # 等待发布者注册
                rospy.sleep(0.5)
                
                # 发布standalone_mode = True
                standalone_msg = Bool()
                standalone_msg.data = True
                pub_standalone.publish(standalone_msg)
                self.print_colored("✓ 已发布 standalone_mode = True（单独运行模式）", Colors.GREEN)
            elif mode == "both":
                # 发布standalone_mode = False（同时运行模式）
                pub_standalone = rospy.Publisher('/breakin/standalone_mode', Bool, queue_size=10, latch=True)
                self.ros_publishers['pub_standalone_mode'] = pub_standalone
                
                # 等待发布者注册
                rospy.sleep(0.5)
                
                # 发布standalone_mode = False
                standalone_msg = Bool()
                standalone_msg.data = False
                pub_standalone.publish(standalone_msg)
                self.print_colored("✓ 已发布 standalone_mode = False（同时运行模式）", Colors.GREEN)
            
            return True
            
        except Exception as e:
            self.print_colored(f"初始化ROS节点失败: {e}", Colors.RED)
            return False
    
    def run_arm_only(self):
        """仅启动手臂磨线"""
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      启动手臂磨线（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()
        
        # 0. 清理残留进程
        self._kill_existing_processes()

        # 1. 获取用户输入的磨线轮数
        while True:
            try:
                rounds_input = input(f"请输入要磨线的轮数: ").strip()
                if rounds_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                target_rounds = int(rounds_input)
                if target_rounds <= 0:
                    self.print_colored("轮数必须大于0，请重新输入", Colors.RED)
                    continue
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        print()
        self.print_colored(f"将进行 {target_rounds} 轮磨线（基于轮数控制，不限制时间）", Colors.GREEN)
        print()
        
        # 2. 询问是否进行零点校准
        while True:
            try:
                calib_input = input("是否进行手臂零点校准？(y/n，默认y): ").strip().lower()
                if calib_input == '':
                    calib_input = 'y'
                if calib_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                if calib_input in ('y', 'yes', '1'):
                    skip_calibration = False
                    self.print_colored("将进行零点校准", Colors.GREEN)
                    break
                elif calib_input in ('n', 'no', '0'):
                    skip_calibration = True
                    self.print_colored("将跳过零点校准", Colors.CYAN)
                    break
                else:
                    self.print_colored("输入无效，请输入 y 或 n", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        print()
        
        # 3. 启动roscore
        if not self._start_roscore():
            return 1
        
        # 4. 初始化ROS节点并发布模式话题
        if not self._init_ros_node("arm_only"):
            return 1
        
        # 5. 启动手臂磨线脚本
        if not self.arm_breakin_standalone.exists():
            self.print_colored(f"错误：未找到手臂磨线脚本 {self.arm_breakin_standalone}", Colors.RED)
            return 1
        
        try:
            import rospy
            from std_msgs.msg import Bool
            
            # 启动手臂磨线脚本（通过环境变量传递是否跳过校准）
            env = dict(os.environ)
            if skip_calibration:
                env['SKIP_ARM_CALIBRATION'] = 'true'
            else:
                env.pop('SKIP_ARM_CALIBRATION', None)  # 确保清除之前的值
            
            cmd = ["python3", str(self.arm_breakin_standalone)]
            process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                env=env
            )
            self.processes.append(process)
            
            # 订阅 arm_started / arm_running，用于起始时间和每轮结束检测
            start_time = None   # 第一次真正开始执行动作的时间
            stop_publish_flag = threading.Event()
            arm_started = False
            arm_running = False
            arm_has_run = False  # 标记是否至少运行过一轮（arm_running曾经为True）
            # 实际运行轮数：以本节点发布 start_new_round_arm=True 的次数为准
            published_rounds = 0
            completed_rounds = 0  # 已完成的轮数（包括第一轮）
            
            def arm_started_callback(msg):
                nonlocal arm_started, start_time
                if msg.data and not arm_started:
                    arm_started = True
                    # 以 arm_started 为基准开始记录时间
                    start_time = time.time()
                    self.print_colored("✓ 收到 arm_started = True，开始记录轮数", Colors.GREEN)
            
            def arm_running_callback(msg):
                nonlocal arm_running, arm_has_run
                arm_running = msg.data
                if arm_running:
                    arm_has_run = True  # 标记至少运行过一轮
            
            sub_arm_started = None
            sub_arm_running = None
            try:
                sub_arm_started = rospy.Subscriber('/breakin/arm_started', Bool, arm_started_callback)
                sub_arm_running = rospy.Subscriber('/breakin/arm_running', Bool, arm_running_callback)
            except Exception as e:
                self.print_colored(f"订阅手臂状态话题失败: {e}", Colors.YELLOW)
            
            # 100Hz 发布线程：发布 allow_run / can_start_new_round / start_new_round_arm
            def publish_topics_loop():
                nonlocal start_time, published_rounds
                rate = rospy.Rate(100)
                last_arm_running_state = None
                last_start_new_round_arm_state = False
                while not stop_publish_flag.is_set() and not rospy.is_shutdown():
                    # 一直允许当前这一轮运行
                    allow_msg = Bool()
                    allow_msg.data = True
                    self.ros_publishers['pub_allow_run'].publish(allow_msg)
                    
                    # 计算是否还能开始新一轮（基于轮数）
                    # 允许当前正在运行的轮完成，即使已经达到目标轮数
                    can_start_new_round_value = True
                    if start_time is not None:
                        # 计算已完成轮数：第一轮 + published_rounds（已触发的后续轮数）
                        completed_rounds_calc = 1 + published_rounds
                        # 如果已达到目标轮数，但当前还在运行中，仍然允许当前轮完成
                        if completed_rounds_calc >= target_rounds:
                            # 只有在不运行时，才停止（允许当前轮完成）
                            can_start_new_round_value = arm_running
                        else:
                            # 未达到目标轮数，允许开始新一轮
                            can_start_new_round_value = True
                    # 在未开始前，允许第一轮
                    can_start_msg = Bool()
                    can_start_msg.data = can_start_new_round_value
                    self.ros_publishers['pub_can_start_new_round'].publish(can_start_msg)
                    
                    # 检测一轮是否刚刚完成（arm_running 从 True 变为 False）
                    should_start_new_round = False
                    if start_time is not None:
                        both_ready = (not arm_running)  # 仅手臂
                        was_running = (last_arm_running_state is True)
                        just_finished = was_running and both_ready

                        if just_finished and can_start_new_round_value and not last_start_new_round_arm_state:
                            # 满足条件时，准备发布下一轮的触发信号
                            should_start_new_round = True
                    
                    if should_start_new_round:
                        # 发送一次脉冲，允许开始新一轮
                        start_new_round_arm_msg = Bool()
                        start_new_round_arm_msg.data = True
                        # 这里直接使用一个新的publisher，避免和run_both混淆
                        pub_start_new_round_arm = rospy.Publisher('/breakin/start_new_round_arm', Bool, queue_size=10)
                        pub_start_new_round_arm.publish(start_new_round_arm_msg)
                        last_start_new_round_arm_state = True
                        published_rounds += 1
                        completed_rounds = 1 + published_rounds  # 包括第一轮和刚触发的这一轮
                        
                        self.print_colored(
                            f"✓ [手臂单独] 发布 start_new_round_arm = True（开始第 {completed_rounds} 轮，共 {target_rounds} 轮）",
                            Colors.GREEN
                        )
                    else:
                        # 确保信号为False，不残留
                        start_new_round_arm_msg = Bool()
                        start_new_round_arm_msg.data = False
                        pub_start_new_round_arm = rospy.Publisher('/breakin/start_new_round_arm', Bool, queue_size=10)
                        pub_start_new_round_arm.publish(start_new_round_arm_msg)
                        if arm_running and last_start_new_round_arm_state:
                            last_start_new_round_arm_state = False
                    
                    last_arm_running_state = arm_running
                    rate.sleep()
            
            # 启动发布线程
            publish_thread = threading.Thread(target=publish_topics_loop, daemon=True)
            publish_thread.start()
            self.print_colored("✓ 已启动手臂单独模式的100Hz话题发布线程", Colors.GREEN)
            
            # 监控进程和轮数
            return_code = 0
            last_arm_running_state = None
            try:
                while True:
                    if process.poll() is not None:
                        return_code = process.returncode
                        break
                    
                    # 如果已经开始，检查是否达到目标轮数（只基于轮数，不基于时间）
                    if start_time is not None and arm_has_run:
                        completed_rounds_calc = 1 + published_rounds
                        # 只有当：1) 已完成轮数 >= 目标轮数
                        #         2) 当前不在运行中（arm_running = False）
                        #         3) 之前确实运行过（arm_has_run = True）
                        #         4) 从运行状态变为完成状态（检测状态变化）
                        was_running = (last_arm_running_state is True)
                        just_finished = was_running and (not arm_running)
                        
                        if completed_rounds_calc >= target_rounds and just_finished:
                            # 已完成所有轮数且当前轮刚刚完成，停止
                            self.print_colored(f"手臂磨线已完成 {target_rounds} 轮", Colors.GREEN)
                            stop_msg = Bool()
                            stop_msg.data = False
                            self.ros_publishers['pub_allow_run'].publish(stop_msg)
                            self.ros_publishers['pub_can_start_new_round'].publish(stop_msg)
                            # 给手臂节点一点时间收尾
                            time.sleep(2.0)
                            break
                    
                    # 更新上一次的运行状态
                    last_arm_running_state = arm_running
                    time.sleep(0.1)
            finally:
                # 在退出前输出实际运行轮数：第一轮本身也算一轮
                # 统计方式：如果真正开始过动作，则总轮数 = 1（第一轮） + 后续通过 start_new_round_arm 触发的轮数
                if start_time is not None:
                    total_rounds = 1 + published_rounds
                    self.print_colored(
                        f"实际完成磨线轮数： {total_rounds}/{target_rounds} 轮",
                        Colors.GREEN
                    )
                else:
                    self.print_colored("未开始执行动作，实际完成磨线轮数: 0 轮", Colors.YELLOW)

                # 停止发布线程
                stop_publish_flag.set()
                publish_thread.join(timeout=1.0)
                
                try:
                    stop_msg = Bool()
                    stop_msg.data = False
                    self.ros_publishers['pub_allow_run'].publish(stop_msg)
                    self.ros_publishers['pub_can_start_new_round'].publish(stop_msg)
                except Exception:
                    pass
                
                # 清理订阅者
                try:
                    if sub_arm_started is not None:
                        sub_arm_started.unregister()
                    if sub_arm_running is not None:
                        sub_arm_running.unregister()
                except Exception:
                    pass
                
                # 停止子进程（如果还在）
                if process.poll() is None:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        process.wait(timeout=3)
                    except Exception:
                        try:
                            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                            process.wait(timeout=1)
                        except Exception:
                            pass
            
            return return_code
            
        except Exception as e:
            self.print_colored(f"启动手臂磨线失败: {e}", Colors.RED)
            import traceback
            traceback.print_exc()
            return 1
    
    def run_leg_only(self):
        """仅启动腿部磨线"""
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      启动腿部磨线（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()

        # 0. 清理残留进程
        self._kill_existing_processes()

        # 0.1 确保腿部磨线日志目录存在
        if not self._ensure_leg_ec_log_dir():
            return 1
        
        # 1. 获取用户输入的磨线轮数
        while True:
            try:
                rounds_input = input(f"请输入要磨线的轮数: ").strip()
                if rounds_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                target_rounds = int(rounds_input)
                if target_rounds <= 0:
                    self.print_colored("轮数必须大于0，请重新输入", Colors.RED)
                    continue
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        print()
        self.print_colored(f"将进行 {target_rounds} 轮磨线（基于轮数控制，不限制时间）", Colors.GREEN)
        print()
        
        # 2. 启动roscore
        if not self._start_roscore():
            return 1
        
        # 3. 初始化ROS节点并发布模式话题
        if not self._init_ros_node("leg_only"):
            return 1
        
        # 4. 启动腿部磨线脚本
        if not self.leg_breakin_standalone.exists():
            self.print_colored(f"错误：未找到腿部磨线脚本 {self.leg_breakin_standalone}", Colors.RED)
            return 1
        
        try:
            import rospy
            from std_msgs.msg import Bool
            
            is_kuavo5 = self._is_kuavo5()

            # 启动100Hz发布线程（发布allow_run、can_start_new_round和start_new_round_leg）
            start_time = None  # 将在收到leg_started时设置
            stop_publish_flag = threading.Event()
            leg_started = False
            leg_running = False
            leg_has_run = False  # 标记是否至少运行过一轮（leg_running曾经为True）
            published_rounds = 0  # 实际运行轮数：以本节点发布 start_new_round_leg=True 的次数为准
            completed_rounds = 0  # 已完成的轮数（包括第一轮）
            
            def leg_started_callback(msg):
                nonlocal leg_started, start_time
                if msg.data and not leg_started:
                    leg_started = True
                    start_time = time.time()
                    self.print_colored("✓ 收到 leg_started = True，开始记录轮数", Colors.GREEN)
            
            def leg_running_callback(msg):
                nonlocal leg_running, leg_has_run
                leg_running = msg.data
                if leg_running:
                    leg_has_run = True  # 标记至少运行过一轮
            
            # 订阅leg_started和leg_running话题（由C++层发布）
            sub_leg_started = rospy.Subscriber('/breakin/leg_started', Bool, leg_started_callback)
            sub_leg_running = rospy.Subscriber('/breakin/leg_running', Bool, leg_running_callback)
            
            # 创建start_new_round_leg发布者
            pub_start_new_round_leg = rospy.Publisher('/breakin/start_new_round_leg', Bool, queue_size=10)
            
            def publish_topics_loop():
                """100Hz发布话题的循环"""
                nonlocal published_rounds
                rate = rospy.Rate(100)  # 100Hz
                last_leg_running_state = None
                last_start_new_round_leg_state = False
                
                while not stop_publish_flag.is_set() and not rospy.is_shutdown():
                    # 发布allow_run = True（持续运行）
                    allow_msg = Bool()
                    allow_msg.data = True
                    self.ros_publishers['pub_allow_run'].publish(allow_msg)
                    
                    # 计算can_start_new_round（基于轮数）
                    # 允许当前正在运行的轮完成，即使已经达到目标轮数
                    can_start_new_round_value = True
                    if start_time is not None:
                        # 计算已完成轮数：第一轮 + published_rounds（已触发的后续轮数）
                        completed_rounds_calc = 1 + published_rounds
                        # 如果已达到目标轮数，但当前还在运行中，仍然允许当前轮完成
                        if completed_rounds_calc >= target_rounds:
                            # 只有在不运行时，才停止（允许当前轮完成）
                            can_start_new_round_value = leg_running
                        else:
                            # 未达到目标轮数，允许开始新一轮
                            can_start_new_round_value = True
                    else:
                        # 在等待leg_started期间，允许开始第一轮
                        can_start_new_round_value = True
                    
                    # 发布can_start_new_round
                    can_start_msg = Bool()
                    can_start_msg.data = can_start_new_round_value
                    self.ros_publishers['pub_can_start_new_round'].publish(can_start_msg)
                    
                    # 检测一轮是否刚刚完成（leg_running 从 True 变为 False）
                    # 持续发布 start_new_round_leg = True，直到 leg_running 变为 True（开始新一轮）
                    should_start_new_round = False
                    if start_time is not None:
                        leg_ready = (not leg_running)  # 腿部完成本轮
                        was_running = (last_leg_running_state is True)
                        just_finished = was_running and leg_ready
                        
                        if just_finished and can_start_new_round_value and not last_start_new_round_leg_state:
                            # 满足条件时，准备发布下一轮的触发信号
                            should_start_new_round = True
                    
                    if should_start_new_round:
                        # 持续发布 True，直到 leg_running 变为 True
                        start_new_round_leg_msg = Bool()
                        start_new_round_leg_msg.data = True
                        pub_start_new_round_leg.publish(start_new_round_leg_msg)
                        last_start_new_round_leg_state = True
                        published_rounds += 1
                        completed_rounds = 1 + published_rounds  # 包括第一轮和刚触发的这一轮
                        
                        self.print_colored(
                            f"✓ [腿部单独] 发布 start_new_round_leg = True（开始第 {completed_rounds} 轮，共 {target_rounds} 轮）",
                            Colors.GREEN
                        )
                    elif last_start_new_round_leg_state:
                        # 如果之前发布了 True，检查 leg_running 是否变为 True（已开始新一轮）
                        if leg_running:
                            # 已经开始新一轮，停止发布 True，改为发布 False
                            start_new_round_leg_msg = Bool()
                            start_new_round_leg_msg.data = False
                            pub_start_new_round_leg.publish(start_new_round_leg_msg)
                            last_start_new_round_leg_state = False
                        else:
                            # 还在等待中，持续发布 True
                            start_new_round_leg_msg = Bool()
                            start_new_round_leg_msg.data = True
                            pub_start_new_round_leg.publish(start_new_round_leg_msg)
                    else:
                        # 确保信号为False，不残留
                        start_new_round_leg_msg = Bool()
                        start_new_round_leg_msg.data = False
                        pub_start_new_round_leg.publish(start_new_round_leg_msg)
                    
                    last_leg_running_state = leg_running
                    rate.sleep()
            
            publish_thread = threading.Thread(target=publish_topics_loop, daemon=True)
            publish_thread.start()
            self.print_colored("✓ 已启动100Hz话题发布线程", Colors.GREEN)
            
            # 等待订阅者注册
            rospy.sleep(1.0)
            
            # 启动腿部磨线脚本（ROS控时长）
            # 通过环境变量传递选择的腿部磨线目录
            env = dict(os.environ)
            env['LEG_BREAKIN_DIR'] = self.leg_breakin_dir
            cmd = ["python3", str(self.leg_breakin_standalone)]
            process = subprocess.Popen(cmd, preexec_fn=os.setsid, env=env)
            self.processes.append(process)

            # 等待腿部磨线开始运行（通过订阅leg_started话题）
            self.print_colored("等待腿部磨线开始运行...", Colors.BLUE)
            start_wait_timeout = 30.0
            start_wait_start = time.time()
            while not leg_started and (time.time() - start_wait_start) < start_wait_timeout:
                if process.poll() is not None:
                    return_code = process.returncode
                    self.print_colored(f"错误：腿部磨线脚本启动失败，已退出，返回码: {return_code}", Colors.RED)
                    return 1
                rospy.sleep(0.1)
            if not leg_started:
                self.print_colored("错误：等待腿部磨线开始运行超时", Colors.RED)
                if process.poll() is None:
                    self.print_colored("提示：脚本仍在运行但未发布leg_started，请检查输出", Colors.YELLOW)
                return 1
            
            # 监控进程和轮数
            last_leg_running_state = None
            while True:
                # 检查进程是否还在运行
                if process.poll() is not None:
                    return_code = process.returncode
                    break
                
                # 检查是否达到目标轮数（只基于轮数，不基于时间）
                if start_time is not None and leg_has_run:
                    completed_rounds_calc = 1 + published_rounds
                    # 只有当：1) 已完成轮数 >= 目标轮数
                    #         2) 当前不在运行中（leg_running = False）
                    #         3) 之前确实运行过（leg_has_run = True）
                    #         4) 从运行状态变为完成状态（检测状态变化）
                    was_running = (last_leg_running_state is True)
                    just_finished = was_running and (not leg_running)
                    
                    if completed_rounds_calc >= target_rounds and just_finished:
                        # 已完成所有轮数且当前轮刚刚完成，停止
                        self.print_colored(f"腿部磨线已完成 {target_rounds} 轮", Colors.GREEN)
                        # 发布停止信号
                        stop_msg = Bool()
                        stop_msg.data = False
                        self.ros_publishers['pub_allow_run'].publish(stop_msg)
                        self.ros_publishers['pub_can_start_new_round'].publish(stop_msg)
                        pub_start_new_round_leg.publish(stop_msg)
                        # 等待进程响应
                        time.sleep(2.0)
                        if process.poll() is None:
                            try:
                                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                                process.wait(timeout=3)
                            except:
                                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        return_code = process.returncode if process.poll() is not None else 0
                        break
                
                # 更新上一次的运行状态
                last_leg_running_state = leg_running
                
                # 处理ROS回调
                try:
                    rospy.sleep(0.1)
                except:
                    time.sleep(0.1)
            
            # 根据真实运行时间估算完成轮数：第一轮本身也算一轮
            # 统计方式：如果真正开始过动作，则总轮数 = 1（第一轮） + 后续通过 start_new_round_leg 触发的轮数
            if start_time is not None:
                total_rounds = 1 + published_rounds
                self.print_colored(f"实际完成磨线轮数: {total_rounds}/{target_rounds} 轮", Colors.GREEN)
            else:
                self.print_colored("未开始执行动作，完成磨线轮数: 0 轮", Colors.YELLOW)

            # 停止发布线程
            stop_publish_flag.set()
            publish_thread.join(timeout=1.0)
            
            # 发布停止信号
            stop_msg = Bool()
            stop_msg.data = False
            self.ros_publishers['pub_allow_run'].publish(stop_msg)
            self.ros_publishers['pub_can_start_new_round'].publish(stop_msg)
            pub_start_new_round_leg.publish(stop_msg)
            rospy.sleep(0.5)
            
            # 清理订阅者
            try:
                if sub_leg_started is not None:
                    sub_leg_started.unregister()
                if sub_leg_running is not None:
                    sub_leg_running.unregister()
            except Exception:
                pass
            
            if return_code == 0:
                self.print_colored("✓ 腿部磨线完成", Colors.GREEN)
            else:
                self.print_colored(f"腿部磨线进程退出，返回码: {return_code}", Colors.YELLOW)
            
            return return_code
            
        except ImportError:
            self.print_colored("错误：需要ROS环境，请先source setup.bash", Colors.RED)
            return 1
        except Exception as e:
            self.print_colored(f"启动腿部磨线失败: {e}", Colors.RED)
            import traceback
            traceback.print_exc()
            return 1
    
    def run_both(self):
        """同时启动手臂和腿部磨线"""
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      同时启动手臂和腿部磨线（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()

        # 0. 清理残留进程
        self._kill_existing_processes()

        # 0.1 确保腿部磨线日志目录存在
        if not self._ensure_leg_ec_log_dir():
            return 1
        
        # 0.2 询问是否进行手臂零点校准（在启动roscore之前询问）
        while True:
            try:
                calib_input = input("是否进行手臂零点校准？(y/n，默认y): ").strip().lower()
                if calib_input == '':
                    calib_input = 'y'
                if calib_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                if calib_input in ('y', 'yes', '1'):
                    skip_calibration = False
                    self.print_colored("将进行零点校准", Colors.GREEN)
                    break
                elif calib_input in ('n', 'no', '0'):
                    skip_calibration = True
                    self.print_colored("将跳过零点校准", Colors.CYAN)
                    break
                else:
                    self.print_colored("输入无效，请输入 y 或 n", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        print()
        
        # 1. 启动roscore
        if not self._start_roscore():
            return 1
        
        # 2. 初始化ROS节点并发布模式话题（standalone_mode = False）
        if not self._init_ros_node("both"):
            return 1
        
        # 3. 检查脚本是否存在
        if not self.arm_breakin_standalone.exists():
            self.print_colored(f"错误：未找到手臂磨线脚本 {self.arm_breakin_standalone}", Colors.RED)
            return 1
        if not self.leg_breakin_standalone.exists():
            self.print_colored(f"错误：未找到腿部磨线脚本 {self.leg_breakin_standalone}", Colors.RED)
            return 1
        if not self.breakin_controller_simple.exists():
            self.print_colored(f"错误：未找到主控制器脚本 {self.breakin_controller_simple}", Colors.RED)
            return 1
        
        try:
            # 4. 获取用户输入的磨线轮数（手臂和腿部使用相同的轮数）
            while True:
                try:
                    rounds_input = input(f"请输入要磨线的轮数: ").strip()
                    if rounds_input.lower() == 'q':
                        self.print_colored("已取消操作", Colors.YELLOW)
                        return 0
                    target_rounds = int(rounds_input)
                    if target_rounds <= 0:
                        self.print_colored("轮数必须大于0，请重新输入", Colors.RED)
                        continue
                    break
                except ValueError:
                    self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
                except KeyboardInterrupt:
                    self.print_colored("\n已取消操作", Colors.YELLOW)
                    return 0
            
            print()
            self.print_colored(f"手臂和腿部各将进行 {target_rounds} 轮磨线（基于轮数控制，不限制时间）", Colors.GREEN)
            print()
            
            # 6. 启动主控制器节点（用于发布start_together）
            self.print_colored("正在启动主控制器节点（发布start_together）...", Colors.BLUE)
            controller_cmd = ["python3", str(self.breakin_controller_simple)]
            controller_process = subprocess.Popen(
                controller_cmd,
                preexec_fn=os.setsid
            )
            self.processes.append(controller_process)
            self.print_colored(f"主控制器节点已启动，PID: {controller_process.pid}", Colors.GREEN)
            
            # 等待主控制器节点启动
            time.sleep(2.0)
            
            # 6. 同时启动手臂和腿部磨线脚本（ROS控时长）
            self.print_colored("正在启动手臂和腿部磨线脚本...", Colors.BLUE)
            arm_cmd = ["python3", str(self.arm_breakin_standalone)]
            leg_cmd = ["python3", str(self.leg_breakin_standalone)]
            
            # 启动手臂磨线进程（ROS控时长，通过环境变量传递是否跳过校准）
            env_arm = dict(os.environ)
            if skip_calibration:
                env_arm['SKIP_ARM_CALIBRATION'] = 'true'
            else:
                env_arm.pop('SKIP_ARM_CALIBRATION', None)  # 确保清除之前的值
            
            arm_process = subprocess.Popen(
                arm_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=env_arm
            )
            
            # 启动腿部磨线进程（ROS控时长，通过环境变量传递选择的目录）
            env_leg = dict(os.environ)
            env_leg['LEG_BREAKIN_DIR'] = self.leg_breakin_dir
            leg_process = subprocess.Popen(
                leg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=env_leg
            )
            
            self.processes.append(arm_process)
            self.processes.append(leg_process)
            
            self.print_colored(f"手臂磨线进程已启动，PID: {arm_process.pid}", Colors.GREEN)
            self.print_colored(f"腿部磨线进程已启动，PID: {leg_process.pid}", Colors.GREEN)
            self.print_colored("", Colors.NC)
            self.print_colored("提示：两个子程序会等待start_together信号才开始进入第二轮动作", Colors.CYAN)
            self.print_colored("如果start_together信号还没给出，它们会一直执行第一帧动作", Colors.CYAN)
            self.print_colored("", Colors.NC)
            
            # 启动输出读取线程（避免输出缓冲区阻塞）
            def read_arm_output():
                """读取手臂磨线进程的输出"""
                try:
                    for line in iter(arm_process.stdout.readline, ''):
                        if not line:
                            break
                        print(f"[手臂] {line.rstrip()}")
                        sys.stdout.flush()
                except Exception as e:
                    self.print_colored(f"读取手臂输出出错: {e}", Colors.RED)
            
            def read_leg_output():
                """读取腿部磨线进程的输出"""
                try:
                    for line in iter(leg_process.stdout.readline, ''):
                        if not line:
                            break
                        print(f"[腿部] {line.rstrip()}")
                        sys.stdout.flush()
                except Exception as e:
                    self.print_colored(f"读取腿部输出出错: {e}", Colors.RED)
            
            # 启动输出读取线程
            arm_output_thread = threading.Thread(target=read_arm_output, daemon=True)
            leg_output_thread = threading.Thread(target=read_leg_output, daemon=True)
            arm_output_thread.start()
            leg_output_thread.start()
            
            self.print_colored("✓ 已启动输出读取线程", Colors.GREEN)
            
            # 6. 初始化ROS节点并订阅话题，启动100Hz发布线程
            try:
                import rospy
                from std_msgs.msg import Bool
                
                # 初始化ROS节点（如果尚未初始化）
                if not self.ros_node_initialized:
                    rospy.init_node('breakin_main_controller_monitor', anonymous=True)
                    self.ros_node_initialized = True
                
                # 创建发布者（发布allow_run和start_new_round信号）
                pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
                pub_start_new_round_arm = rospy.Publisher('/breakin/start_new_round_arm', Bool, queue_size=10)
                pub_start_new_round_leg = rospy.Publisher('/breakin/start_new_round_leg', Bool, queue_size=10)
                # 注意：腿部磨线还在使用can_start_new_round topic（单独运行模式和第一轮检查），所以也需要发布
                pub_can_start_new_round = rospy.Publisher('/breakin/can_start_new_round', Bool, queue_size=10)
                
                # 运行状态标志
                arm_running = False
                leg_running = False
                arm_has_run = False  # 标记手臂是否至少运行过一轮
                leg_has_run = False  # 标记腿部是否至少运行过一轮
                should_stop = False
                start_time = None  # 将在收到start_together时设置
                start_together_received = False
                stop_publish_flag = threading.Event()
                last_arm_running_state = None
                last_leg_running_state = None
                last_start_new_round_arm_state = False
                last_start_new_round_leg_state = False
                # 实际运行轮数：以本节点发布 start_new_round_xxx=True 的次数为准
                published_rounds = 0
                completed_rounds = 0  # 已完成的轮数（包括第一轮）
                # 记录已触发的轮是否至少已经开始过（用于判断是否真正完成）
                # 当 published_rounds = N 时，表示已触发第 (N+1) 轮
                # rounds_started_mask: 第i轮（从0开始计数）是否至少已经开始过
                rounds_started_mask = set()  # 使用集合记录已开始过的轮数索引
                
                def start_together_callback(msg):
                    nonlocal start_time, start_together_received, rounds_started_mask
                    if msg.data and not start_together_received:
                        start_together_received = True
                        start_time = time.time()
                        # 标记第1轮（索引0）已开始
                        rounds_started_mask.add(0)
                        self.print_colored("✓ 收到 start_together = True，开始记录轮数", Colors.GREEN)
                
                def arm_running_cb(msg):
                    nonlocal arm_running, arm_has_run
                    arm_running = msg.data
                    if arm_running:
                        arm_has_run = True  # 标记至少运行过一轮
                
                def leg_running_cb(msg):
                    nonlocal leg_running, leg_has_run
                    leg_running = msg.data
                    if leg_running:
                        leg_has_run = True  # 标记至少运行过一轮
                
                # 创建订阅者
                sub_start_together = rospy.Subscriber('/breakin/start_together', Bool, start_together_callback)
                sub_arm_running = rospy.Subscriber('/breakin/arm_running', Bool, arm_running_cb)
                sub_leg_running = rospy.Subscriber('/breakin/leg_running', Bool, leg_running_cb)
                
                # 启动100Hz发布线程（发布allow_run和start_new_round信号）
                def publish_topics_loop():
                    """100Hz发布话题的循环"""
                    nonlocal last_start_new_round_arm_state, last_start_new_round_leg_state, last_arm_running_state, last_leg_running_state, published_rounds, rounds_started_mask
                    rate = rospy.Rate(100)  # 100Hz
                    while not stop_publish_flag.is_set() and not rospy.is_shutdown():
                        # 发布allow_run = True（持续运行）
                        allow_msg = Bool()
                        allow_msg.data = True
                        pub_allow_run.publish(allow_msg)
                        
                        # 计算是否可以开始新一轮（基于轮数）
                        # 允许当前正在运行的轮完成，即使已经达到目标轮数
                        can_start_new_round_value = True  # 默认值
                        if start_together_received and start_time is not None:
                            # 计算已完成轮数：第一轮 + published_rounds（已触发的后续轮数）
                            completed_rounds_calc = 1 + published_rounds
                            # 如果已达到目标轮数，但当前还在运行中，仍然允许当前轮完成
                            if completed_rounds_calc >= target_rounds:
                                # 只有在都不运行时，才停止（允许当前轮完成）
                                can_start_new_round_value = (arm_running or leg_running)
                            else:
                                # 未达到目标轮数，允许开始新一轮
                                can_start_new_round_value = True
                        else:
                            # 在等待start_together期间，允许开始第一轮
                            can_start_new_round_value = True
                        
                        # 发布can_start_new_round（腿部磨线还在使用这个topic）
                        can_start_msg = Bool()
                        can_start_msg.data = can_start_new_round_value
                        pub_can_start_new_round.publish(can_start_msg)
                        
                        # 检查是否可以开始新一轮（用于start_new_round_arm和start_new_round_leg）
                        # 条件：1. 手臂和腿部都完成本轮（arm_running=False, leg_running=False）
                        #       2. 已经收到start_together（第一轮已开始）
                        #       3. 之前都在运行（从运行状态变为完成状态）
                        #       4. 可以开始新一轮（can_start_new_round_value = True）
                        
                        # 检查是否需要发布start_new_round信号
                        should_start_new_round = False
                        if start_together_received and start_time is not None:
                            # 检查手臂和腿部是否都完成本轮（都为False表示都完成）
                            both_ready = (not arm_running and not leg_running)
                            
                            # 检查是否从"正在运行"变为"完成"（检测到状态变化）
                            # 只有当之前arm_running或leg_running为True，现在都为False时，才发布
                            if last_arm_running_state is not None and last_leg_running_state is not None:
                                was_running = (last_arm_running_state or last_leg_running_state)
                                # 检测到从运行状态变为完成状态
                                just_finished = was_running and both_ready
                            else:
                                # 初始化时，如果都为False，说明还在等待第一轮，不发布
                                just_finished = False
                            
                            if just_finished and can_start_new_round_value and not last_start_new_round_arm_state and not last_start_new_round_leg_state:
                                # 两个都准备好（从运行状态变为完成状态），且剩余时间足够，发布开始新一轮信号
                                should_start_new_round = True
                        
                        # 发布start_new_round_arm和start_new_round_leg（同时发布）
                        if should_start_new_round:
                            start_new_round_arm_msg = Bool()
                            start_new_round_arm_msg.data = True
                            pub_start_new_round_arm.publish(start_new_round_arm_msg)
                            last_start_new_round_arm_state = True
                            
                            start_new_round_leg_msg = Bool()
                            start_new_round_leg_msg.data = True
                            pub_start_new_round_leg.publish(start_new_round_leg_msg)
                            last_start_new_round_leg_state = True
                            published_rounds += 1
                            completed_rounds = 1 + published_rounds  # 包括第一轮和刚触发的这一轮
                            # 注意：此时新触发的轮（索引=published_rounds）还未开始，所以不加入 rounds_started_mask
                            
                            self.print_colored(
                                f"✓ 发布 start_new_round_arm = True 和 start_new_round_leg = True（开始第 {completed_rounds} 轮，共 {target_rounds} 轮）",
                                Colors.GREEN
                            )
                        elif last_start_new_round_arm_state or last_start_new_round_leg_state:
                            # 如果之前发布了 True，持续发布直到 arm_running 和 leg_running 都变为 True（已开始新一轮）
                            if arm_running and last_start_new_round_arm_state:
                                # 手臂已经开始新一轮，停止发布 True，改为发布 False
                                start_new_round_arm_msg = Bool()
                                start_new_round_arm_msg.data = False
                                pub_start_new_round_arm.publish(start_new_round_arm_msg)
                                last_start_new_round_arm_state = False
                                # 标记当前轮（索引=published_rounds）已开始
                                if published_rounds > 0:
                                    rounds_started_mask.add(published_rounds)
                            elif last_start_new_round_arm_state:
                                # 手臂还在等待中，持续发布 True
                                start_new_round_arm_msg = Bool()
                                start_new_round_arm_msg.data = True
                                pub_start_new_round_arm.publish(start_new_round_arm_msg)
                            
                            if leg_running and last_start_new_round_leg_state:
                                # 腿部已经开始新一轮，停止发布 True，改为发布 False
                                start_new_round_leg_msg = Bool()
                                start_new_round_leg_msg.data = False
                                pub_start_new_round_leg.publish(start_new_round_leg_msg)
                                last_start_new_round_leg_state = False
                                # 标记当前轮（索引=published_rounds）已开始
                                if published_rounds > 0:
                                    rounds_started_mask.add(published_rounds)
                            elif last_start_new_round_leg_state:
                                # 腿部还在等待中，持续发布 True
                                start_new_round_leg_msg = Bool()
                                start_new_round_leg_msg.data = True
                                pub_start_new_round_leg.publish(start_new_round_leg_msg)
                        else:
                            # 持续发布False，确保信号不会残留
                            start_new_round_arm_msg = Bool()
                            start_new_round_arm_msg.data = False
                            pub_start_new_round_arm.publish(start_new_round_arm_msg)
                            
                            start_new_round_leg_msg = Bool()
                            start_new_round_leg_msg.data = False
                            pub_start_new_round_leg.publish(start_new_round_leg_msg)
                        
                        # 更新上一次的状态（用于检测状态变化）
                        last_arm_running_state = arm_running
                        last_leg_running_state = leg_running
                        
                        rate.sleep()
                
                publish_thread = threading.Thread(target=publish_topics_loop, daemon=True)
                publish_thread.start()
                self.print_colored("✓ 已启动100Hz话题发布线程", Colors.GREEN)
                
                # 等待订阅者注册
                rospy.sleep(1.0)
                
                self.print_colored("✓ 已订阅 /breakin/start_together, /breakin/arm_running 和 /breakin/leg_running", Colors.GREEN)
                
            except Exception as e:
                self.print_colored(f"警告：无法初始化ROS监控节点: {e}", Colors.YELLOW)
                self.print_colored("将继续运行，但无法监控运行状态", Colors.YELLOW)
                should_stop = False
                start_time = None
                stop_publish_flag = threading.Event()
            
            # 7. 等待所有进程结束或检测到停止信号
            self.print_colored("等待所有进程完成或检测到停止信号...", Colors.BLUE)
            
            # 等待任一进程结束或检测到停止信号
            while True:
                arm_exited = arm_process.poll() is not None
                leg_exited = leg_process.poll() is not None
                controller_exited = controller_process.poll() is not None
                
                # 检查是否应该停止（arm_running或leg_running为False）
                if should_stop:
                    self.print_colored("检测到运行状态异常，正在停止所有子程序...", Colors.RED)
                    break
                
                # 检查是否达到目标轮数（仅在收到start_together后检查，只基于轮数，不基于时间）
                if start_time is not None and arm_has_run and leg_has_run:
                    completed_rounds_calc = 1 + published_rounds  # 已触发的轮数（包括第1轮）
                    
                    # 需要确保：1) 已触发足够的轮数  2) 所有已触发的轮都已经至少开始过  3) 当前不在运行中
                    #         4) 从运行状态变为完成状态（检测状态变化）
                    # 第1轮（索引0）总是已经完成，所以从第2轮（索引1）开始检查
                    all_triggered_rounds_started = True
                    if published_rounds > 0:
                        # 检查所有已触发的轮（索引1到published_rounds）是否至少已经开始过
                        for round_idx in range(1, published_rounds + 1):
                            if round_idx not in rounds_started_mask:
                                all_triggered_rounds_started = False
                                break
                    
                    # 检测是否从运行状态变为完成状态
                    was_running = (last_arm_running_state is True) or (last_leg_running_state is True)
                    just_finished = was_running and (not arm_running) and (not leg_running)
                    
                    if completed_rounds_calc >= target_rounds and all_triggered_rounds_started and just_finished:
                        # 已完成所有轮数，且所有已触发的轮都已经开始并完成，且当前轮刚刚完成，停止
                        self.print_colored(f"手臂和腿部磨线已完成 {target_rounds} 轮", Colors.GREEN)
                        # 发布停止信号
                        stop_msg = Bool()
                        stop_msg.data = False
                        pub_allow_run.publish(stop_msg)
                        pub_start_new_round_arm.publish(stop_msg)
                        pub_start_new_round_leg.publish(stop_msg)
                        pub_can_start_new_round.publish(stop_msg)
                        # 等待进程响应
                        time.sleep(2.0)
                        break
                
                if arm_exited or leg_exited or controller_exited:
                    break
                
                # 处理ROS回调
                try:
                    rospy.sleep(0.1)
                except:
                    time.sleep(0.1)
            
            # 根据话题触发次数统计实际运行轮数
            # 第一轮本身也算一轮，因此总轮数 = 1（第一轮） + 后续通过 start_new_round_* 触发的轮数
            if start_time is not None and start_together_received:
                total_rounds = 1 + published_rounds
                self.print_colored(
                    f"实际完成磨线轮数: {total_rounds}/{target_rounds} 轮",
                    Colors.GREEN
                )
            else:
                self.print_colored("未开始同步动作，实际完成磨线轮数: 0 轮", Colors.YELLOW)

            # 停止发布线程
            stop_publish_flag.set()
            if 'publish_thread' in locals():
                publish_thread.join(timeout=1.0)
            
            # 发布停止信号
            try:
                stop_msg = Bool()
                stop_msg.data = False
                pub_allow_run.publish(stop_msg)
                pub_start_new_round_arm.publish(stop_msg)
                pub_start_new_round_leg.publish(stop_msg)
                pub_can_start_new_round.publish(stop_msg)
                rospy.sleep(0.5)
            except:
                pass
            
            # 停止所有进程（如果还在运行）
            if not arm_exited:
                try:
                    os.killpg(os.getpgid(arm_process.pid), signal.SIGTERM)
                    arm_process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(arm_process.pid), signal.SIGKILL)
                        arm_process.wait(timeout=1)
                    except:
                        pass
            
            if not leg_exited:
                try:
                    os.killpg(os.getpgid(leg_process.pid), signal.SIGTERM)
                    leg_process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(leg_process.pid), signal.SIGKILL)
                        leg_process.wait(timeout=1)
                    except:
                        pass
            
            if not controller_exited:
                try:
                    os.killpg(os.getpgid(controller_process.pid), signal.SIGTERM)
                    controller_process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(controller_process.pid), signal.SIGKILL)
                        controller_process.wait(timeout=1)
                    except:
                        pass
            
            # 等待一下，确保进程完全退出
            time.sleep(0.5)
            
            # 重新检查进程状态并获取返回码
            arm_exited = arm_process.poll() is not None
            leg_exited = leg_process.poll() is not None
            controller_exited = controller_process.poll() is not None
            
            # 获取返回码（如果进程已退出）
            arm_return_code = arm_process.returncode if arm_exited else None
            leg_return_code = leg_process.returncode if leg_exited else None
            controller_return_code = controller_process.returncode if controller_exited else None
            
            # 报告结果
            # 如果返回码为None，说明进程被强制终止或仍在运行，视为失败
            # 如果返回码为0，说明正常退出
            # 如果返回码不为0，说明异常退出
            if arm_return_code == 0 and leg_return_code == 0:
                self.print_colored("✓ 手臂和腿部磨线都成功完成", Colors.GREEN)
                return 0
            else:
                # 区分正常退出和异常情况
                if arm_return_code == 0:
                    self.print_colored("✓ 手臂磨线正常完成", Colors.GREEN)
                elif arm_return_code is None:
                    self.print_colored("⚠️ 手臂磨线进程被强制终止或仍在运行", Colors.YELLOW)
                else:
                    self.print_colored(f"✗ 手臂磨线异常退出，返回码: {arm_return_code}", Colors.RED)
                
                if leg_return_code == 0:
                    self.print_colored("✓ 腿部磨线正常完成", Colors.GREEN)
                elif leg_return_code is None:
                    self.print_colored("⚠️ 腿部磨线进程被强制终止或仍在运行", Colors.YELLOW)
                else:
                    self.print_colored(f"✗ 腿部磨线异常退出，返回码: {leg_return_code}", Colors.RED)
                
                # 如果至少有一个失败，返回1
                if arm_return_code != 0 or leg_return_code != 0:
                    return 1
                # 如果都是None（被强制终止），也返回1
                return 1
            
        except Exception as e:
            self.print_colored(f"启动同时磨线失败: {e}", Colors.RED)
            import traceback
            traceback.print_exc()
            return 1
    
    def run(self):
        """主运行函数"""
        print()
        
        # 检查root权限
        if os.geteuid() != 0:
            self.print_colored("错误：需要root权限运行。", Colors.RED)
            self.print_colored("请先执行 sudo su，切换到root后再运行本脚本。", Colors.YELLOW)
            return 1

        # 如果是通过 "sudo python ..." 直接调用，要求先 sudo su 再运行
        sudo_cmd = os.environ.get("SUDO_COMMAND", "")
        if sudo_cmd and "python" in sudo_cmd:
            self.print_colored("检测到通过 sudo 直接调用 python 运行。", Colors.YELLOW)
            self.print_colored("请先执行 sudo su，再在root环境下运行本脚本。", Colors.RED)
            return 1

        # 在进入菜单逻辑之前，先确保工作空间已经编译（catkin_make）
        # 如果编译失败，则直接退出
        if not self._ensure_workspace_built():
            return 1

        while True:
            self.show_menu()
            choice = input("请输入选项 (1-3 或 q): ").strip()
            
            if choice == "1":
                return self.run_arm_only()
            elif choice == "2":
                return self.run_leg_only()
            elif choice == "3":
                return self.run_both()
            elif choice == "q":
                self.print_colored("退出程序", Colors.YELLOW)
                return 0
            else:
                self.print_colored("无效选项，请输入 1-3 或 q", Colors.RED)
                print()

def main():
    try:
        app = BreakinMainController()
        exit_code = app.run()
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
