#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import threading
import time
import select
import signal
from datetime import datetime

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)  # 获取父目录
sys.path.append(parent_dir)  # 添加父目录到路径

from common.common_utils import print_colored_text, kuavo_ros_control_path, develFound, installedFound

class HardwareSelfCheck:
    def __init__(self):
        """初始化硬件自检类"""
        self.process = None
        self.roslaunch_running = False
        # 保证输出路径始终相对于本脚本目录，且带时间戳
        script_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = time.strftime("%Y%m%d%H%M")
        self.output_file_path = os.path.join(script_dir, f"hardware_selfcheck_output_{timestamp}.txt")
        
        # 测试项目配置
        self.test_items = [
            {"name": "IMU检测", "status": "⏳ 等待中", "time": "0.0s", "detail": ""},
            {"name": "末端执行器", "status": "⏳ 等待中", "time": "0.0s", "detail": ""},
            {"name": "电机初始化", "status": "⏳ 等待中", "time": "0.0s", "detail": ""},
            {"name": "手臂电机", "status": "⏳ 等待中", "time": "0.0s", "detail": ""},
            {"name": "腿部电机", "status": "⏳ 等待中", "time": "0.0s", "detail": ""}
        ]
        
        # 测试计时器
        self.start_times = {}
        self.total_tests = len(self.test_items)
        self.completed_tests = set()
        
        # 重要警告收集
        self.important_warnings = []
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def clear_screen(self):
        """清屏并移动光标到开头"""
        print("\033[2J\033[H", end="")

    def print_header(self):
        """打印头部信息"""
        print_colored_text("====================== start self-check =====================", color="green", bold=True)
        print()
        
        # 生成支持点击的超链接
        clickable_link = f"\x1b]8;;file://{self.output_file_path}\x1b\\{self.output_file_path}\x1b]8;;\x1b\\"
        print(f"下位机自检已启动，输出已保存到 {clickable_link}")
        print("按Ctrl+C退出...")
        print()

    def print_table(self):
        """打印表格"""
        print("┌─────────────────────────────────────────────────────────────────────┐")
        print("│                       硬件自检进度表                                │")
        print("├─────────────────────┬─────────────────┬─────────────┬───────────────┤")
        print("│       测试项目      │      状态       │    耗时     │  详细信息     │")
        print("├─────────────────────┼─────────────────┼─────────────┼───────────────┤")
        
        for item in self.test_items:
            # 计算中文字符的实际显示宽度
            name = self.pad_string(item["name"], 19)
            status = self.pad_string(item["status"], 15)
            time_str = self.pad_string(item["time"], 11)
            detail = self.pad_string(item["detail"], 14)
            print(f"│ {name} │ {status} │ {time_str} │ {detail}│")
        
        print("└─────────────────────┴─────────────────┴─────────────┴───────────────┘")

    def pad_string(self, text, width):
        """填充字符串到指定宽度，正确处理中文字符"""
        # 计算字符串的实际显示宽度（中文字符算2个字符宽度）
        display_width = 0
        for char in text:
            if ord(char) > 127:  # 中文字符
                display_width += 2
            else:
                display_width += 1
        
        # 计算需要填充的空格数
        padding = width - display_width
        if padding > 0:
            return text + " " * padding
        else:
            return text

    def refresh_display(self):
        """刷新整个显示"""
        self.clear_screen()
        self.print_header()
        self.print_table()
        self.flush_log()

    def update_test_status(self, index, status, time_str=None, detail=None):
        """更新测试状态"""
        if 0 <= index < len(self.test_items):
            self.test_items[index]["status"] = status
            if time_str:
                self.test_items[index]["time"] = time_str
            if detail:
                self.test_items[index]["detail"] = detail
            
            # 记录已完成的测试项
            if "✅" in status or "❌" in status:
                self.completed_tests.add(index)
            
            # 刷新整个显示
            self.refresh_display()

    def start_test_timer(self, index):
        """开始测试计时"""
        self.start_times[index] = time.time()

    def get_test_time(self, index):
        """获取测试耗时"""
        if index in self.start_times:
            elapsed = time.time() - self.start_times[index]
            return f"{elapsed:.1f}s"
        return "0.0s"

    def run_hardware_self_check(self):
        """运行下位机自检"""
        try:
            # 获取项目根目录（相对于脚本位置）
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.abspath(os.path.join(script_dir, "../../../.."))
            
            # 构造启动命令
            if installedFound:
                bin_dir = os.path.join(project_root, "installed/bin")
                full_command = (
                    f"cd {bin_dir} && "
                    f"source {kuavo_ros_control_path}/devel/setup.bash && "
                    f"./hardwareSelfCheck"
                )
            elif develFound:
                bin_dir = os.path.join(project_root, "devel/lib/hardware_node")
                full_command = (
                    f"cd {bin_dir} && "
                    f"source {kuavo_ros_control_path}/devel/setup.bash && "
                    f"./hardwareSelfCheck"
                )
            else:
                print_colored_text("未找到编译产物！", color="red", bold=True)
                return 1

            # 定义输出文件路径
            os.makedirs(os.path.dirname(self.output_file_path), exist_ok=True)

            # 打开文件以写入模式，使用行缓冲模式
            self.output_file = open(self.output_file_path, 'w', buffering=1)

            # 启动进程
            self.process = subprocess.Popen(
                ['bash', '-c', full_command],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,  # 行缓冲
                universal_newlines=True,
            )

            self.roslaunch_running = True

            # 暂时禁用输入线程，避免与主线程输入冲突
            # thread = threading.Thread(target=self.input_thread, args=(self.process,), daemon=True)
            # thread.start()

            # 显示初始界面
            self.refresh_display()

            # 监控输出
            while self.process.poll() is None:
                # 主动刷新C++进程的输出缓冲区（不处理退出逻辑）
                self.force_flush_cpp_output_simple()
                
                rlist, _, _ = select.select([self.process.stdout], [], [], 0.1)
                if rlist:
                    line = self.process.stdout.readline()
                    if line:
                        self.output_file.write(line)
                        self.output_file.flush()
                        self.last_log_time = time.time()  # 更新最后日志时间
                        # 检查是否提前退出
                        if self.process_line(line.strip()):
                            # 如果process_line返回True，表示需要提前退出
                            break
            
            # 处理剩余输出
            self.process_remaining_output()

            # 等待进程结束（如果还没有结束）
            if self.process.poll() is None:
                self.process.wait()

            # 显示最终结果
            self.show_final_summary()
            return 0

        except Exception as e:
            print_colored_text(f"发生错误: {e}", color="red", bold=True)
            self.flush_log()
            return 1
        except KeyboardInterrupt:
            print_colored_text("\n接收到Ctrl+C，正在安全退出...", color="yellow")
            self.flush_log()
            return 0
        finally:
            # 确保进程已终止
            if self.process and self.process.poll() is None:
                self.ensure_cpp_process_terminated()
            self.roslaunch_running = False
            self.close_log_file()

    def process_remaining_output(self):
        """处理子进程剩余的输出"""
        if not self.process.stdout.closed:
            # 等待一小段时间，给子进程机会完成输出
            time.sleep(0.2)
            
            # 读取所有剩余的输出
            remaining_output = self.process.stdout.read()
            if remaining_output:
                self.output_file.write(remaining_output)
                self.output_file.flush()
                self.last_log_time = time.time()

    def process_line(self, line):
        """处理输出行"""
        # 调试信息：打印所有输出行（可选，用于调试）
        # print(f"DEBUG: {line.strip()}")
        
        # IMU初始化相关
        if "start init imu." in line:
            self.start_test_timer(0)
            self.update_test_status(0, "🔄 检测中...", "0.0s")
        elif "imu init failed" in line:
            self.update_test_status(0, "❌ 失败", self.get_test_time(0))
        elif "imu init success" in line:
            self.update_test_status(0, "✅ 成功", self.get_test_time(0))
        # IMU Yaw漂移检测失败
        elif "Yaw角漂移检测失败" in line:
            self.update_test_status(0, "❌ 失败", self.get_test_time(0), "Yaw漂移过大")
            print_colored_text(f"⚠️  IMU检测到重要警告: {line.strip()}", color="red")
            self.add_important_warning("IMU", line.strip())
        # IMU频率统计警告（非致命）
        elif "当前IMU（HIPNUC）不支持时间戳" in line:
            print_colored_text(f"ℹ️  IMU信息: {line.strip()}", color="yellow")

        # 末端执行器初始化相关
        elif "initEndEffector" in line and "init success" in line:
            self.start_test_timer(1)
            self.update_test_status(1, "🔄 检测中...", "0.0s")
        elif "末端执行器(灵巧手)初始化成功" in line:
            self.update_test_status(1, "✅ 成功", self.get_test_time(1))
        elif "末端执行器(灵巧手)初始化失败" in line:
            self.update_test_status(1, "❌ 失败", self.get_test_time(1))
        # 末端执行器端口分配失败
        elif "end effector port allocation failed" in line:
            self.update_test_status(1, "❌ 失败", self.get_test_time(1), "端口分配失败")
            print_colored_text(f"⚠️  末端执行器检测到重要警告: {line.strip()}", color="red")
            self.add_important_warning("末端执行器", line.strip())

        # 电机初始化相关
        elif "actuators_InterfaceSetup DONE" in line:
            self.start_test_timer(2)
            self.update_test_status(2, "🔄 检测中...", "0.0s")
        elif "电机初始化成功" in line:
            self.update_test_status(2, "✅ 成功", self.get_test_time(2))
        elif "电机初始化失败" in line:
            self.update_test_status(2, "❌ 失败", self.get_test_time(2))

        # 手臂电机测试相关
        elif "测试手臂电机>>" in line:
            self.start_test_timer(3)
            self.update_test_status(3, "🔄 检测中...", "0.0s")
        elif "测试手臂电机成功" in line:
            self.update_test_status(3, "✅ 成功", self.get_test_time(3))
        elif "测试手臂电机失败" in line:
            self.update_test_status(3, "❌ 失败", self.get_test_time(3))
        # 手臂关节移动失败
        elif "手臂关节" in line and "没有移动" in line:
            print_colored_text(f"⚠️  手臂电机检测到重要警告: {line.strip()}", color="red")
            self.add_important_warning("手臂电机", line.strip())
        # 运动超时
        elif "motion timeout" in line:
            print_colored_text(f"⚠️  运动检测到重要警告: {line.strip()}", color="red")
            self.add_important_warning("运动控制", line.strip())
        # 关节未到达目标位置
        elif "did not reach the tar pos" in line:
            print_colored_text(f"⚠️  运动检测到重要警告: {line.strip()}", color="red")
            self.add_important_warning("运动控制", line.strip())

        # 腿部电机测试相关
        elif "测试腿部电机>>" in line:
            self.start_test_timer(4)
            self.update_test_status(4, "🔄 检测中...", "0.0s")
        elif "移动到准备姿态" in line:
            print_colored_text(f"DEBUG: 检测到移动到准备姿态: {line.strip()}", color="cyan")
            self.update_test_status(4, "🔄 检测中...", self.get_test_time(4))
            # 继续监控后续输出，等待第二个提示
        elif "腿部电机测试完成" in line:
            # 这个消息由C++脚本输出，表示测试动作已完成
            pass
        elif "测试完毕" in line:
            # 移除自动成功判断，测试结果完全由用户输入决定
            pass
        
        # 处理C++代码中的具体用户输入提示
        elif "当前cali位置是否正确，[y/n] 输入[y]进入到准备姿态" in line:
            print_colored_text(f"DEBUG: 检测到第一个提示: {line.strip()}", color="cyan")
            return self.handle_cali_position_prompt(line)
        elif "准备姿态位置是否正确，[y/n] 输入[y]确认腿部电机测试成功" in line:
            print_colored_text(f"DEBUG: 检测到第二个提示: {line.strip()}", color="cyan")
            return self.handle_ready_pose_prompt(line)
        elif "输入x退出程序：" in line:
            # 这个提示现在由Python脚本直接处理，不需要等待C++输出
            pass
        
        # 检测C++脚本提前失败的情况
        elif "用户确认校准位置不正确，退出自检" in line:
            return self.handle_early_exit("用户确认校准位置不正确，退出自检", "用户取消了测试")
        elif "输入无效，请输入 'y' 或 'n'" in line:
            return self.handle_early_exit("输入无效，请输入 'y' 或 'n'", "用户输入无效")
        elif "用户确认校准位置正确，继续后续操作" in line:
            # 用户确认继续，不需要特殊处理
            pass
        # 移除基于C++响应的测试状态更新，测试结果完全由用户输入决定
        elif "用户确认准备姿态正确 - 腿部电机测试通过" in line:
            # 这个状态已经在用户输入时更新，这里不需要重复更新
            pass
        elif "用户确认准备姿态不正确 - 腿部电机测试失败" in line:
            # 这个状态已经在用户输入时更新，这里不需要重复更新
            pass

        return False

    def handle_cali_position_prompt(self, prompt_line):
        """处理校准位置确认提示"""
        print()
        print_colored_text("=" * 70, color="yellow")
        print_colored_text("🔵  腿部电机测试 - 校准位置确认", color="purple", bold=True)
        print_colored_text("C++脚本提示:", color="white")
        print_colored_text(f"  {prompt_line.strip()}", color="cyan")
        print_colored_text("请确认当前校准位置是否正确:", color="white")
        print_colored_text("  [y] - 位置正确，继续测试", color="green")
        print_colored_text("  [n] - 位置不正确，退出自检", color="red")
        print_colored_text("=" * 70, color="purple")
        
        try:
            # 等待用户输入，添加重试机制
            user_input = self.get_valid_user_input("请输入选择 (y/n): ", ['y', 'n', 'Y', 'N'])
            
            # 根据用户输入处理
            if user_input.lower() == 'y':
                print_colored_text("用户确认位置正确，继续测试...", color="green")
                
                # 发送输入到C++进程
                if not self.send_input_to_process('y'):
                    return False
                
                # 用户确认继续，腿部电机测试状态保持为"检测中"
                # 等待下一个提示（准备姿态确认）
                print_colored_text("等待移动到准备姿态...", color="yellow")
                
                return False
                
            elif user_input.lower() == 'n':
                # 立即更新腿部电机测试状态为失败
                self.update_test_status(4, "❌ 失败", self.get_test_time(4), "用户取消")
                print_colored_text("用户确认校准位置不正确，腿部电机测试失败！", color="red")
                
                return self.handle_negative_response("用户取消测试，正在终止进程...",
                                                   ["用户确认校准位置不正确", "退出自检"])
            else:
                # 这种情况理论上不会发生，因为get_valid_user_input已经验证了输入
                print_colored_text("输入验证失败，C++脚本将处理", color="yellow")
                return self.send_input_to_process(user_input)
                
        except KeyboardInterrupt:
            print_colored_text("\n接收到Ctrl+C，正在安全退出...", color="yellow")
            self.safe_exit("用户中断")
            raise
        except Exception as e:
            print_colored_text(f"处理用户输入时出错: {e}", color="red")
        
        print_colored_text("=" * 70, color="yellow")
        print()
        self.flush_log()
        return False

    def handle_ready_pose_prompt(self, prompt_line):
        """处理准备姿态位置确认提示"""
        print()
        print_colored_text("=" * 70, color="yellow")
        print_colored_text("🔵  腿部电机测试 - 准备姿态确认", color="purple", bold=True)
        print_colored_text("C++脚本提示:", color="white")
        print_colored_text(f"  {prompt_line.strip()}", color="cyan")
        print_colored_text("请确认当前准备姿态位置是否正确:", color="white")
        print_colored_text("  [y] - 位置正确，腿部电机测试通过", color="green")
        print_colored_text("  [n] - 位置不正确，腿部电机测试失败", color="red")
        print_colored_text("=" * 70, color="purple")
        
        try:
            # 等待用户输入，添加重试机制
            user_input = self.get_valid_user_input("请输入选择 (y/n): ", ['y', 'n', 'Y', 'N'])
            
            # 根据用户输入处理
            if user_input.lower() == 'y':
                # 立即更新腿部电机测试状态为成功
                self.update_test_status(4, "✅ 成功", self.get_test_time(4), "用户确认")
                print_colored_text("用户确认准备姿态正确，腿部电机测试通过！", color="green")
                
                # 发送输入到C++进程
                if not self.send_input_to_process('y'):
                    return False
                
                # 用户确认成功，腿部电机测试完成
                print_colored_text("腿部电机测试完成！", color="green")
                print()
                print_colored_text("=" * 60, color="blue")
                print_colored_text("🏁 测试即将完成", color="blue", bold=True)
                print_colored_text("正在安全退出程序...", color="white")
                print_colored_text("=" * 60, color="blue")
                
                # 直接发送 'x' 到C++进程并退出
                if not self.send_input_to_process('x'):
                    return False
                
                print_colored_text("=" * 60, color="blue")
                print()
                self.flush_log()
                
                # 标记测试完成，让主循环自然结束
                self.roslaunch_running = False
                return True
                
            elif user_input.lower() == 'n':
                # 立即更新腿部电机测试状态为失败
                self.update_test_status(4, "❌ 失败", self.get_test_time(4), "用户确认")
                print_colored_text("用户确认准备姿态不正确，腿部电机测试失败！", color="red")
                
                return self.handle_negative_response("用户取消测试，正在终止进程...",
                                                   ["用户确认准备姿态不正确", "腿部电机测试失败"])
            else:
                # 这种情况理论上不会发生，因为get_valid_user_input已经验证了输入
                print_colored_text("输入验证失败，C++脚本将处理", color="yellow")
                return self.send_input_to_process(user_input)
                
        except KeyboardInterrupt:
            print_colored_text("\n接收到Ctrl+C，正在安全退出...", color="yellow")
            self.safe_exit("用户中断")
            raise
        except Exception as e:
            print_colored_text(f"处理用户输入时出错: {e}", color="red")
            self.update_test_status(4, "❓ 未知", self.get_test_time(4), "输入错误")
        
        print_colored_text("=" * 70, color="yellow")
        print()
        self.flush_log()
        return False

    def handle_exit_prompt(self, prompt_line):
        """处理退出程序提示"""
        print()
        print_colored_text("=" * 60, color="blue")
        print_colored_text("🏁 测试即将完成", color="blue", bold=True)
        print_colored_text("C++脚本提示:", color="white")
        print_colored_text(f"  {prompt_line.strip()}", color="cyan")
        print_colored_text("请输入 'x' 退出程序:", color="white")
        print_colored_text("=" * 60, color="blue")
        
        try:
            # 等待用户输入，支持重试机制
            user_input = self.get_valid_user_input("请输入 x 退出: ", ['x', 'X'])
            
            # 发送输入到子进程
            if not self.send_input_to_process(user_input):
                return False
                
        except KeyboardInterrupt:
            print_colored_text("\n接收到Ctrl+C，正在安全退出...", color="yellow")
            self.safe_exit("用户中断")
            raise
        except Exception as e:
            print_colored_text(f"处理用户输入时出错: {e}", color="red")
        
        print_colored_text("=" * 60, color="blue")
        print()
        self.flush_log()
        return False

    def handle_early_exit(self, cpp_message, user_friendly_message):
        """处理C++脚本提前退出的情况"""
        print()
        print_colored_text("=" * 70, color="red")
        print_colored_text("❌ C++脚本提前退出", color="red", bold=True)
        print_colored_text("C++脚本消息:", color="white")
        print_colored_text(f"  {cpp_message}", color="yellow")
        print_colored_text("用户友好说明:", color="white")
        print_colored_text(f"  {user_friendly_message}", color="cyan")
        print_colored_text("=" * 70, color="red")
        
        # 更新相关测试项状态为失败
        if "校准位置不正确" in cpp_message:
            # 腿部电机测试失败
            self.update_test_status(4, "❌ 失败", self.get_test_time(4), "用户取消")
        elif "输入无效" in cpp_message:
            # 腿部电机测试失败
            self.update_test_status(4, "❌ 失败", self.get_test_time(4), "输入无效")
        
        # 使用安全退出方法
        self.safe_exit("C++脚本提前退出")
        self.flush_log()
        return True

    def handle_user_input_prompt(self, prompt_line):
        """处理通用用户输入提示（保留作为备用）"""
        print()
        print_colored_text("=" * 60, color="yellow")
        print_colored_text("⚠️  检测到用户输入提示", color="yellow", bold=True)
        print_colored_text(f"提示内容: {prompt_line.strip()}", color="white")
        print_colored_text("请在下方输入 y 或 n，然后按回车:", color="cyan")
        print_colored_text("=" * 60, color="yellow")
        
        try:
            # 等待用户输入
            user_input = input().strip().lower()
            
            # 发送输入到子进程
            if self.process and self.process.stdin:
                self.process.stdin.write(user_input + '\n')
                self.process.stdin.flush()
                print_colored_text(f"已发送输入: {user_input}", color="green")
            else:
                print_colored_text("无法发送输入到子进程", color="red")
                
        except Exception as e:
            print_colored_text(f"处理用户输入时出错: {e}", color="red")
        
        print_colored_text("=" * 60, color="yellow")
        print()
        self.flush_log()

    def input_thread(self, process):
        """输入处理线程"""
        try:
            while process.poll() is None:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = sys.stdin.readline().strip()
                    # 所有用户输入都由主线程处理，这里只处理Ctrl+C
                    # 避免与主线程的输入处理冲突
                    pass
        except KeyboardInterrupt:
            print_colored_text("\n输入线程接收到Ctrl+C", color="yellow")
            self.flush_log()
        except Exception as e:
            print(f"输入线程错误: {e}")
            self.flush_log()

    def show_final_summary(self):
        """显示最终总结"""
        print()
        print_colored_text("=" * 50, color="blue")
        print_colored_text("=== 下位机自检总结 ===", color="green", bold=True)
        
        # 检查是否有测试中断
        is_interrupted = self.check_test_interruption()
        
        success_count = sum(1 for item in self.test_items if "✅" in item["status"])
        fail_count = sum(1 for item in self.test_items if "❌" in item["status"])
        
        print_colored_text(f"总测试项目: {len(self.test_items)} 项", color="blue")
        print_colored_text(f"✅ 成功: {success_count} 项", color="green")
        print_colored_text(f"❌ 失败: {fail_count} 项", color="red")
        
        if is_interrupted:
            print_colored_text("⚠️  检测被中断，部分测试未完成", color="red", bold=True)
        elif fail_count == 0:
            print_colored_text("🎉 所有测试项目通过！", color="green", bold=True)
        else:
            print_colored_text(f"⚠️  有 {fail_count} 项测试失败，请检查相关硬件", color="red", bold=True)
        
        # 计算总耗时
        total_time = 0.0
        for i in range(len(self.test_items)):
            if i in self.start_times:
                total_time = max(total_time, time.time() - self.start_times[i])
        
        print_colored_text(f"⏱️  总耗时: {total_time:.1f} 秒", color="cyan")
        
        # 显示重要警告总结
        self.show_important_warnings()
        self.flush_log()

    def check_test_interruption(self):
        """检查测试是否被中断"""
        completed_count = len(self.completed_tests)
        if completed_count < self.total_tests:
            print()
            print_colored_text("=" * 60, color="red")
            print_colored_text("⚠️  检测中断警告", color="red", bold=True)
            print_colored_text(f"当前测试项：{self.total_tests} 项", color="white")
            print_colored_text(f"已完成测试：{completed_count} 项", color="yellow")
            print_colored_text(f"未完成测试：{self.total_tests - completed_count} 项", color="red")
            
            # 显示未完成的测试项
            incomplete_tests = []
            for i in range(self.total_tests):
                if i not in self.completed_tests:
                    incomplete_tests.append(self.test_items[i]["name"])
            
            if incomplete_tests:
                print_colored_text("未完成的测试项：", color="red")
                for test_name in incomplete_tests:
                    print_colored_text(f"  - {test_name}", color="yellow")
            
            print_colored_text("=" * 60, color="red")
            return True
        return False

    def signal_handler(self, signum, frame):
        """信号处理器，处理Ctrl+C等中断信号"""
        # 使用安全退出方法
        self.safe_exit(f"接收到信号 {signum}")
        self.flush_log()
        sys.exit(0)

    def ensure_cpp_process_terminated(self):
        """确保C++进程被正确终止"""
        if self.process and self.process.poll() is None:
            print_colored_text("正在主动关闭C++程序...", color="yellow")
            try:
                # 首先尝试优雅终止
                self.process.terminate()
                
                # 等待一段时间，给进程机会刷新输出
                wait_time = 0.5
                start_time = time.time()
                while time.time() - start_time < wait_time:
                    if self.process.poll() is not None:
                        break
                    time.sleep(0.1)
                
                # 读取可能剩余的输出
                self.process_remaining_output()
                self.flush_log()
                
                # 如果进程仍在运行，强制杀死
                if self.process.poll() is None:
                    print_colored_text("C++程序未响应，强制终止...", color="red")
                    self.process.kill()
                    
                    # 再次等待并读取输出
                    time.sleep(0.2)
                    self.process_remaining_output()
                    self.flush_log()
                    
                    # 最后一次等待
                    try:
                        self.process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        pass
                
                if self.process.poll() is not None:
                    print_colored_text("C++程序已成功关闭", color="green")
                else:
                    print_colored_text("警告：C++程序可能仍在运行", color="red")
                    
            except Exception as e:
                print_colored_text(f"关闭C++程序时出错: {e}", color="red")
                self.flush_log()
                # 最后的尝试：强制杀死
                try:
                    self.process.kill()
                except:
                    pass

    def safe_exit(self, reason="未知原因"):
        """安全退出方法，确保C++程序被关闭"""
        print()
        print_colored_text("=" * 60, color="yellow")
        print_colored_text(f"🔄 正在安全退出硬件自检 ({reason})", color="yellow", bold=True)
        print_colored_text("=" * 60, color="yellow")
        
        # 检查测试中断情况
        self.check_test_interruption()
        
        # 确保C++进程被终止
        self.ensure_cpp_process_terminated()
        
        # 更新运行状态
        self.roslaunch_running = False
        
        # 显示最终总结
        self.show_final_summary()
        self.flush_log()
        
        print_colored_text("硬件自检已安全退出", color="green", bold=True)
        print()

    def add_important_warning(self, warning_type, message):
        """添加重要警告到收集列表"""
        self.important_warnings.append({
            "type": warning_type,
            "message": message,
            "time": time.strftime("%H:%M:%S")
        })

    def show_important_warnings(self):
        """显示重要警告总结"""
        if self.important_warnings:
            print()
            print_colored_text("=" * 60, color="red")
            print_colored_text("⚠️  重要警告总结", color="red", bold=True)
            print_colored_text("=" * 60, color="red")
            
            for warning in self.important_warnings:
                print_colored_text(f"[{warning['time']}] {warning['type']}: {warning['message']}", color="yellow")
            
            print_colored_text("=" * 60, color="red")
            print()
    
    def flush_log(self):
        """刷新日志缓冲区"""
        if self.output_file:
            self.output_file.flush()
            # 可以考虑使用 os.fsync 确保数据写入磁盘
            # os.fsync(self.output_file.fileno())
    
    def close_log_file(self):
        """关闭日志文件"""
        if self.output_file:
            self.output_file.close()
            self.output_file = None

    def get_valid_user_input(self, prompt, valid_options, max_retries=3):
        """获取有效的用户输入，支持重试机制"""
        for attempt in range(max_retries):
            try:
                user_input = input(prompt).strip()
                if user_input in valid_options:
                    return user_input
                else:
                    remaining_attempts = max_retries - attempt - 1
                    if remaining_attempts > 0:
                        print_colored_text(f"输入无效，请重新输入。剩余尝试次数: {remaining_attempts}", color="yellow")
                        print_colored_text(f"有效选项: {', '.join(valid_options)}", color="cyan")
                    else:
                        print_colored_text("输入尝试次数已用完，使用默认值", color="red")
                        return valid_options[0]  # 返回第一个有效选项作为默认值
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print_colored_text(f"输入错误: {e}", color="red")
                if attempt < max_retries - 1:
                    print_colored_text("请重新输入", color="yellow")
        
        # 如果所有尝试都失败，返回默认值
        return valid_options[0]

    def handle_positive_response(self, success_message, expected_responses, next_prompt_keyword=None):
        """处理用户确认的响应"""
        # 发送输入到子进程
        if not self.send_input_to_process('y'):
            return False
        
        print_colored_text(success_message, color="green")
        
        # 等待C++进程处理输入并输出响应
        print_colored_text("等待C++进程处理输入...", color="yellow")
        
        # 主动刷新C++进程的输出缓冲区
        self.force_flush_cpp_output()
        
        # 等待C++进程的响应，最多等待5秒
        start_time = time.time()
        response_received = False
        while time.time() - start_time < 5.0:
            if self.process.poll() is not None:
                # C++进程已退出
                response_received = True
                print_colored_text("C++进程已退出", color="red")
                break
            
            # 主动刷新并检查输出
            self.force_flush_cpp_output()
            
            # 检查是否有新的输出
            rlist, _, _ = select.select([self.process.stdout], [], [], 0.1)
            if rlist:
                line = self.process.stdout.readline()
                if line:
                    # 将输出写入日志文件
                    self.output_file.write(line)
                    self.output_file.flush()
                    
                    # 检查是否是我们期望的响应
                    if any(response in line for response in expected_responses):
                        response_received = True
                        print_colored_text("收到C++进程响应，继续监控...", color="green")
                        break
                    # 检查是否出现了下一个提示
                    elif next_prompt_keyword and next_prompt_keyword in line:
                        print_colored_text("检测到下一个提示，立即处理...", color="green")
                        self.process_line(line.strip())
                        response_received = True
                        break
            
            time.sleep(0.1)
        
        if not response_received:
            print_colored_text("C++进程未响应，但继续监控...", color="yellow")
        
        return False

    def handle_negative_response(self, cancel_message, expected_responses):
        """处理用户取消的响应"""
        print_colored_text(cancel_message, color="red")
        
        # 先发送输入到C++进程，让它知道用户的选择
        if not self.send_input_to_process('n'):
            return True
        
        # 等待C++进程处理输入并输出响应
        print_colored_text("等待C++进程处理输入...", color="yellow")
        
        # 主动刷新C++进程的输出缓冲区
        self.force_flush_cpp_output()
        
        # 等待C++进程的响应，最多等待3秒
        start_time = time.time()
        response_received = False
        while time.time() - start_time < 3.0:
            if self.process.poll() is not None:
                # C++进程已退出
                response_received = True
                print_colored_text("C++进程已正常退出", color="green")
                break
            
            # 主动刷新并检查输出
            self.force_flush_cpp_output()
            
            # 检查是否有新的输出
            rlist, _, _ = select.select([self.process.stdout], [], [], 0.1)
            if rlist:
                line = self.process.stdout.readline()
                if line:
                    # 检查是否是我们期望的响应
                    if any(response in line for response in expected_responses):
                        response_received = True
                        print_colored_text("收到C++进程响应", color="green")
                        break
            
            time.sleep(0.1)
        
        if not response_received:
            print_colored_text("C++进程未响应，将强制终止...", color="yellow")
        
        # 更新测试状态为失败
        self.update_test_status(4, "❌ 失败", self.get_test_time(4), "用户取消")
        
        # 使用安全退出方法
        self.safe_exit("用户取消测试")
        return True

    def force_flush_cpp_output_simple(self):
        """强制刷新C++进程的输出缓冲区（简化版本，不处理退出逻辑）"""
        if self.process and self.process.poll() is None:
            try:
                # 尝试发送一个空信号来刷新输出缓冲区
                # 这里使用SIGCONT信号，它是一个安全的信号，不会中断进程
                self.process.send_signal(signal.SIGCONT)
                
                # 短暂等待，让进程有机会刷新输出
                time.sleep(0.05)
                
                # 尝试读取任何可用的输出
                while True:
                    rlist, _, _ = select.select([self.process.stdout], [], [], 0.01)
                    if not rlist:
                        break
                    
                    line = self.process.stdout.readline()
                    if not line:
                        break
                    
                    # 将输出写入日志文件
                    self.output_file.write(line)
                    self.output_file.flush()
                    
                    # 处理输出行（但不处理退出逻辑）
                    self.process_line(line.strip())
                        
            except Exception as e:
                # 忽略刷新过程中的错误，不影响主流程
                pass

    def force_flush_cpp_output(self):
        """强制刷新C++进程的输出缓冲区"""
        if self.process and self.process.poll() is None:
            try:
                # 尝试发送一个空信号来刷新输出缓冲区
                # 这里使用SIGCONT信号，它是一个安全的信号，不会中断进程
                self.process.send_signal(signal.SIGCONT)
                
                # 短暂等待，让进程有机会刷新输出
                time.sleep(0.05)
                
                # 尝试读取任何可用的输出
                while True:
                    rlist, _, _ = select.select([self.process.stdout], [], [], 0.01)
                    if not rlist:
                        break
                    
                    line = self.process.stdout.readline()
                    if not line:
                        break
                    
                    # 将输出写入日志文件
                    self.output_file.write(line)
                    self.output_file.flush()
                    
                    # 处理输出行
                    if self.process_line(line.strip()):
                        # 如果process_line返回True，表示需要提前退出
                        return True
                        
            except Exception as e:
                # 忽略刷新过程中的错误，不影响主流程
                pass
        
        return False

    def send_input_to_process(self, user_input):
        """发送输入到子进程，带错误处理"""
        if self.process and self.process.stdin:
            try:
                # 确保输入被正确发送
                self.process.stdin.write(user_input + '\n')
                self.process.stdin.flush()
                print_colored_text(f"已发送输入: {user_input}", color="green")
                return True
            except Exception as e:
                print_colored_text(f"发送输入到子进程时出错: {e}", color="red")
                return False
        else:
            print_colored_text("无法发送输入到子进程", color="red")
            return False

def run_hardware_self_check():
    """运行下位机自检的主函数"""
    checker = HardwareSelfCheck()
    return checker.run_hardware_self_check()

if __name__ == "__main__":
    run_hardware_self_check() 