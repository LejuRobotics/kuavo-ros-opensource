#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
腿部磨线独立启动脚本（ROS版本）
功能类似 joint_breakin.py 中的单独腿部磨线
包含ROS话题通信、启动腿部磨线脚本等功能
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

class LegBreakinStandalone:
    def __init__(self):
        # 获取当前脚本所在目录
        self.current_dir = Path(__file__).parent.absolute()
        
        # 查找项目根目录（通过查找 tools/check_tool 目录）
        # 脚本位置：joint_breakin_ros/src/breakin_control/scripts/
        # 需要向上找到包含 tools/check_tool 的目录
        self.project_root = self._find_project_root()
        
        # 腿部磨线脚本路径
        # scripts/ -> src/leg_breakin/src/
        leg_breakin_src = self.current_dir.parent.parent / "leg_breakin" / "src"
        self.leg_breakin_src = leg_breakin_src
        self.joint_breakin_script = leg_breakin_src / "joint_breakin.py"  # roban2 统一入口（ROS控时长，版本13-14）
        self.roban2_v17_leg_breakin_script = leg_breakin_src / "leg_breakin_roban2_v17" / "roban2_leg_breakin.py"  # roban2_v17（ROS控时长）
        self.kuavo5_leg_breakin_script_v52_80A = leg_breakin_src / "leg_breakin_kuavo5_v52_80A" / "kuavo5_leg_breakin.py"  # kuavo5_v52_80A（ROS控时长，龙华25台版本）
        self.kuavo5_leg_breakin_script_v52 = leg_breakin_src / "leg_breakin_kuavo5_v52" / "kuavo5_leg_breakin.py"  # kuavo5_v52（ROS控时长，普通v52版本）
        
        # 进程管理
        self.leg_process = None
        self.roscore_process = None
        self.stop_flag = threading.Event()
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def _get_robot_version(self):
        """获取 ROBOT_VERSION：优先环境变量，其次读取 /home/lab/.bashrc"""
        rv = os.environ.get("ROBOT_VERSION")
        if rv:
            return str(rv).strip()

        try:
            bashrc_path = os.path.join(os.path.expanduser('/home/lab/'), '.bashrc')
            if os.path.exists(bashrc_path):
                with open(bashrc_path, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                for line in reversed(lines):
                    s = line.strip()
                    if s.startswith("export ROBOT_VERSION=") and "#" not in s:
                        return s.split("=", 1)[1].strip()
        except Exception:
            pass

        return ""

    def _is_kuavo5(self, robot_version: str):
        rv_raw = str(robot_version or "").strip()
        rv = rv_raw.lower()

        # 明确字符串标识
        if ("kuavo5_v52" in rv) or ("kuavo5_v53" in rv) or ("kuavo5" in rv) or rv.startswith("v5") or rv.startswith("kuavo5_v5"):
            return True

        # 兼容 ROBOT_VERSION=52/53 这种纯数字写法：Kuavo5 版本 50+
        try:
            v = int(rv_raw)
            return v >= 50
        except Exception:
            return False
    
    def _is_robot_version_53(self, robot_version: str):
        """判断 ROBOT_VERSION 是否为 53"""
        rv_raw = str(robot_version or "").strip()
        rv = rv_raw.lower()
        
        # 字符串匹配：包含 v53 或 kuavo5_v53
        if "v53" in rv or "kuavo5_v53" in rv:
            return True
        
        # 数字版本判断：53
        try:
            v = int(rv_raw)
            return v == 53
        except (ValueError, TypeError):
            return False

    def _find_project_root(self):
        """查找项目根目录（包含 tools/check_tool 的目录）"""
        current = self.current_dir
        
        # 向上查找，直到找到包含 tools/check_tool 的目录
        for level in range(10):  # 最多向上查找10级
            # 检查是否存在 tools/check_tool 目录
            test_dir = current / "tools" / "check_tool"
            if test_dir.exists() and test_dir.is_dir():
                print(f"{Colors.GREEN}找到项目根目录: {current}{Colors.NC}")
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
            import rosgraph
            return rosgraph.is_master_online()
        except:
            return False
    
    def _start_roscore(self):
        """启动roscore（如果未运行）"""
        if self._check_ros_master():
            return True
        
        self.print_colored("正在启动roscore...", Colors.BLUE)
        try:
            env = dict(os.environ)
            if 'ROS_MASTER_URI' not in env or not env['ROS_MASTER_URI']:
                env['ROS_MASTER_URI'] = 'http://localhost:11311'
            
            self.roscore_process = subprocess.Popen(
                ['roscore'],
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
    
    def _stop_roscore(self):
        """停止roscore（无论是否由本脚本启动）"""
        try:
            import subprocess as sp
            # 尝试使用killall或pkill停止roscore
            try:
                sp.run(['killall', 'roscore'], check=False, timeout=2)
            except:
                try:
                    sp.run(['pkill', '-f', 'roscore'], check=False, timeout=2)
                except:
                    pass
        except Exception as e:
            self.print_colored(f"停止roscore失败: {e}", Colors.YELLOW)
    
    def signal_handler(self, signum, frame):
        """信号处理函数"""
        self.print_colored(f"\n收到停止信号，正在停止腿部磨线程序...", Colors.YELLOW)
        self.stop_flag.set()
        
        # 停止腿部磨线进程
        if self.leg_process and self.leg_process.poll() is None:
            self.print_colored("正在停止腿部磨线进程...", Colors.YELLOW)
            try:
                os.killpg(os.getpgid(self.leg_process.pid), signal.SIGTERM)
                self.leg_process.wait(timeout=3)
            except:
                try:
                    os.killpg(os.getpgid(self.leg_process.pid), signal.SIGKILL)
                except:
                    pass
        
        # 停止roscore
        self._stop_roscore()
        
        sys.exit(0)
    
    def run_leg_breakin(self):
        """运行腿部磨线"""
        # 检查root权限
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 leg_breakin_standalone.py", Colors.YELLOW)
            return 1
        
        robot_version = self._get_robot_version()
        if robot_version:
            os.environ["ROBOT_VERSION"] = robot_version

        is_kuavo5 = self._is_kuavo5(robot_version)

        # 选择腿部磨线脚本
        # 优先读取环境变量 LEG_BREAKIN_DIR（由主控制器传递）
        leg_breakin_dir = os.environ.get("LEG_BREAKIN_DIR")
        if leg_breakin_dir:
            # 根据环境变量选择对应的脚本
            if leg_breakin_dir == "leg_breakin_kuavo5_v52_80A":
                target_script = self.kuavo5_leg_breakin_script_v52_80A
            elif leg_breakin_dir == "leg_breakin_kuavo5_v52":
                target_script = self.kuavo5_leg_breakin_script_v52
            elif leg_breakin_dir == "leg_breakin_roban2_v17":
                target_script = self.roban2_v17_leg_breakin_script
            elif leg_breakin_dir == "leg_breakin_roban2_v14":
                target_script = self.joint_breakin_script
            else:
                if is_kuavo5:
                    target_script = self.kuavo5_leg_breakin_script_v52_80A
                else:
                    # 根据版本选择roban2脚本
                    try:
                        version_num = int(robot_version) if robot_version else 0
                        if version_num == 17:
                            target_script = self.roban2_v17_leg_breakin_script
                        else:
                            target_script = self.joint_breakin_script
                    except (ValueError, TypeError):
                        target_script = self.joint_breakin_script
        else:
            if is_kuavo5:
                target_script = self.kuavo5_leg_breakin_script_v52_80A
            else:
                # 根据版本选择roban2脚本
                try:
                    version_num = int(robot_version) if robot_version else 0
                    if version_num == 17:
                        target_script = self.roban2_v17_leg_breakin_script
                    else:
                        target_script = self.joint_breakin_script
                except (ValueError, TypeError):
                    target_script = self.joint_breakin_script

        self.print_colored(f"调试：ROBOT_VERSION = {robot_version or '(unknown)'}", Colors.BLUE)
        self.print_colored(f"调试：选择腿部磨线脚本: {target_script}", Colors.BLUE)
        self.print_colored(f"调试：脚本是否存在: {target_script.exists()}", Colors.BLUE)

        if not target_script.exists():
            self.print_colored(f"错误：未找到腿部磨线脚本 {target_script}", Colors.RED)
            return 1
        
        print()
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("      启动腿部磨线（ROS版本）", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        print()
        
        # 提示用户
        self.print_colored("请吊高机器人，将【腿部】摆到【零点位置】。", Colors.YELLOW)
        self.print_colored("最好将吊架的【万向环锁住】，避免机器旋转、腿部踢到移位机。", Colors.YELLOW)
        self.print_colored("确保无干涉、周围无障碍物。准备就绪后开始执行腿部磨线。", Colors.YELLOW)
        print()
        
        print()
        self.print_colored("开始执行腿部磨线！", Colors.GREEN)
        self.print_colored("提示：在执行过程中，按 Ctrl+C 可以安全停止程序", Colors.CYAN)
        self.print_colored("注意：运行时长由主程序通过 /breakin/can_start_new_round 话题控制", Colors.CYAN)
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
                    time.sleep(1)
                    break
                if i < 14:
                    print(".", end="", flush=True)
            else:
                self.print_colored("\n错误：无法启动ROS master，请手动启动roscore", Colors.RED)
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
        
        roscore_started_by_this_script = False
        try:
            import rospy
            from std_msgs.msg import Bool
            
            # 检查roscore是否已运行（主控制器可能已经启动）
            if not self._check_ros_master():
                if not self._start_roscore():
                    return 1
                roscore_started_by_this_script = True
                self.print_colored("✓ 已启动roscore", Colors.GREEN)
            else:
                self.print_colored("✓ ROS master已运行（由主控制器启动）", Colors.GREEN)
            
            # 初始化ROS节点（用于发布启动信号）
            rospy.init_node('leg_breakin_standalone_launcher', anonymous=True)
            
            # 检查standalone_mode是否已发布（主控制器可能已经发布）
            standalone_mode_already_published = False
            try:
                standalone_mode_received = [False]
                def standalone_mode_callback(msg):
                    standalone_mode_received[0] = True
                
                temp_sub = rospy.Subscriber('/breakin/standalone_mode', Bool, standalone_mode_callback)
                rospy.sleep(0.5)
                temp_sub.unregister()
                
                if standalone_mode_received[0]:
                    standalone_mode_already_published = True
                    self.print_colored("✓ standalone_mode已由主控制器发布", Colors.GREEN)
            except:
                pass
            
            # 创建发布者（100Hz发布）
            pub_allow_run = rospy.Publisher('/breakin/allow_run', Bool, queue_size=10)
            pub_can_start_new_round = rospy.Publisher('/breakin/can_start_new_round', Bool, queue_size=10)
            pub_leg_ready = rospy.Publisher('/breakin/leg_ready', Bool, queue_size=10, latch=True)
            # 注意：leg_started现在由C++层发布，这里不需要发布
            
            # 如果standalone_mode未发布，则发布它
            if not standalone_mode_already_published:
                pub_standalone_mode = rospy.Publisher('/breakin/standalone_mode', Bool, queue_size=10, latch=True)
                rospy.sleep(0.5)
                standalone_msg = Bool()
                standalone_msg.data = True
                pub_standalone_mode.publish(standalone_msg)
                self.print_colored("✓ 已发布 standalone_mode = True（单独运行模式）", Colors.GREEN)
            
            # 等待发布者注册
            rospy.sleep(0.5)
            
            # 发布leg_ready = True
            leg_ready_msg = Bool()
            leg_ready_msg.data = True
            pub_leg_ready.publish(leg_ready_msg)
            self.print_colored("✓ 已发布 leg_ready = True", Colors.GREEN)
            
            # 启动腿部磨线脚本
            leg_script_dir = self.joint_breakin_script.parent
            # ROS_BREAKIN_CONTROL_TIME=true: 告诉 joint_breakin.py "时长由ROS主控制器管理，不要再询问并读取时长"
            env = dict(os.environ, PYTHONUNBUFFERED='1', ROS_BREAKIN_CONTROL_TIME='true')
            
            self.print_colored("正在启动腿部磨线脚本...", Colors.BLUE)
            self.print_colored(f"脚本路径: {target_script}", Colors.BLUE)

            # 启动进程（ROS控时长）
            # Kuavo5V52 和 Roban2 都使用 ROS 话题控制
            if is_kuavo5:
                leg_script_dir = target_script.parent
            else:
                # 根据目标脚本确定工作目录
                leg_script_dir = target_script.parent
            
            self.leg_process = subprocess.Popen(
                ["python3", str(target_script)],
                cwd=str(leg_script_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                preexec_fn=os.setsid
            )
            
            self.print_colored(f"腿部磨线进程已启动，PID: {self.leg_process.pid}", Colors.GREEN)
            
            # 先启动100Hz发布线程，持续发布allow_run和can_start_new_round
            # 注意：can_start_new_round的值由主程序根据时长控制，这里暂时发布True
            # 实际应该由主程序（breakin_main_controller.py）发布
            stop_publish_flag = threading.Event()
            leg_started_detected = threading.Event()  # 用于线程间通信
            
            def publish_topics_loop():
                """100Hz发布话题的循环"""
                rate = rospy.Rate(100)  # 100Hz
                while not stop_publish_flag.is_set() and not rospy.is_shutdown():
                    try:
                        # 发布allow_run = True（持续运行）
                        allow_msg = Bool()
                        allow_msg.data = True
                        pub_allow_run.publish(allow_msg)
                    except rospy.exceptions.ROSException:
                        # ROS节点已关闭，退出循环
                        break
                    except Exception as e:
                        # 其他异常，记录但继续运行
                        pass
                    
                    # 注意：can_start_new_round应该由主程序发布，这里不发布
                    # 如果主程序未发布，C++层会使用默认值False
                    
                    try:
                        rate.sleep()
                    except rospy.exceptions.ROSException:
                        # ROS节点已关闭，退出循环
                        break
                    except Exception:
                        # 其他异常，退出循环
                        break
            
            publish_thread = threading.Thread(target=publish_topics_loop, daemon=True)
            publish_thread.start()
            
            self.print_colored("✓ 已启动100Hz话题发布线程", Colors.GREEN)
            
            # 订阅leg_started话题（由C++层发布）
            leg_started = False
            sub_leg_started = None
            def leg_started_callback(msg):
                nonlocal leg_started
                if msg.data and not leg_started:
                    leg_started = True
                    leg_started_detected.set()
                    self.print_colored("✓ 腿部磨线已开始运行", Colors.GREEN)
            sub_leg_started = rospy.Subscriber('/breakin/leg_started', Bool, leg_started_callback)
            
            # 启动输出读取线程（仅用于显示输出）
            def read_output():
                try:
                    for line in iter(self.leg_process.stdout.readline, ''):
                        if not line:
                            break
                        print(line.rstrip())
                        sys.stdout.flush()
                except Exception as e:
                    self.print_colored(f"读取输出出错: {e}", Colors.RED)
            
            output_thread = threading.Thread(target=read_output, daemon=True)
            output_thread.start()
            
            # 等待进程启动
            time.sleep(1.0)
            
            # 检查进程是否还在运行
            if self.leg_process.poll() is not None:
                self.print_colored(f"错误：腿部磨线脚本启动失败，已退出，返回码: {self.leg_process.returncode}", Colors.RED)
                return 1
            
            # 等待腿部磨线开始运行（通过订阅leg_started话题）
            self.print_colored("等待腿部磨线开始运行...", Colors.BLUE)
            start_wait_timeout = 30.0
            start_wait_start = time.time()
            while not leg_started and (time.time() - start_wait_start) < start_wait_timeout:
                if self.leg_process.poll() is not None:
                    return 1
                rospy.sleep(0.1)
            if not leg_started:
                self.print_colored("错误：等待腿部磨线开始运行超时", Colors.RED)
                if self.leg_process.poll() is None:
                    self.print_colored("提示：脚本仍在运行但未发布leg_started，请检查输出", Colors.YELLOW)
                return 1
            
            # 监控进程的循环（不再检查时间，由主程序通过can_start_new_round控制）
            while True:
                # 检查进程是否还在运行
                if self.leg_process.poll() is not None:
                    return_code = self.leg_process.returncode
                    break
                
                # 每0.1秒检查一次
                time.sleep(0.1)
            
            # 停止发布线程
            stop_publish_flag.set()
            publish_thread.join(timeout=1.0)
            
            # 发布停止信号（如果ROS节点还在运行）
            try:
                if not rospy.is_shutdown():
                    stop_msg = Bool()
                    stop_msg.data = False
                    pub_allow_run.publish(stop_msg)
                    pub_leg_ready.publish(stop_msg)
                    # 注意：leg_started由C++层发布，这里不需要发布
                    # 注意：can_start_new_round由主程序发布，这里不需要发布
                    rospy.sleep(0.5)
            except rospy.exceptions.ROSException:
                # ROS节点已关闭，忽略
                pass
            except Exception:
                # 其他异常，忽略
                pass
            
            if return_code == 0:
                self.print_colored("✓ 腿部磨线完成", Colors.GREEN)
            else:
                self.print_colored(f"腿部磨线进程退出，返回码: {return_code}", Colors.YELLOW)
            
            # 停止roscore（无论是否由本脚本启动）
            self._stop_roscore()
            
            return return_code
            
        except ImportError:
            self.print_colored("错误：需要ROS环境，请先source setup.bash", Colors.RED)
            return 1
        except Exception as e:
            self.print_colored(f"腿部磨线运行出错: {e}", Colors.RED)
            import traceback
            traceback.print_exc()
            return 1

def main():
    try:
        app = LegBreakinStandalone()
        exit_code = app.run_leg_breakin()
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
