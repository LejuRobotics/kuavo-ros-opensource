#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import threading
import time
import select
import argparse
from datetime import datetime
import signal

# 添加父目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from common.common_utils import print_colored_text, kuavo_ros_control_path, develFound, installedFound

class MotorFollowTest:
    def __init__(self, test_region="full"):
        """初始化电机跟随性测试类"""
        self.process = None
        self.roslaunch_running = False
        self.input_thread_enabled = True  # 控制输入线程的启用/禁用
        self.test_region = test_region
        
        # 根据测试区域设置电机对数量和名称
        self.setup_test_config()
        
        # 定义要打印的日志信息列表 - 匹配roslaunch的输出格式
        self.target_logs = [
            "电机对进度:",
            "当前测试对:",
            "测试完成标记",
            "保存测试对",
            "数据保存完成",
            "所有关节对测试完成",
            "测试对数量:",
            "关节对:",
            "测试模式:",
            "电机跟随测试开始",
            "电机跟随测试停止",
            "motorFollowTest",
            "motorFollowTestReal",
            "开始实物电机跟随测试主循环",
            "实物电机跟随测试结束",
            "测试进度:",
            "当前测试的关节",
            "电机测试对",
            "统计结果",
            "详细结果",
            "测试对"
        ]
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def setup_test_config(self):
        """根据测试区域设置配置 - 配置由C++程序处理"""
        if self.test_region == "upper":
            self.region_name = "上半身"
            self.test_mode = 2  # ARMS_ONLY
        elif self.test_region == "lower":
            self.region_name = "下半身"
            self.test_mode = 1  # LEGS_ONLY
        else:  # full
            self.region_name = "全身"
            self.test_mode = 0  # FULL_BODY
        
        # 测试对数量和具体配置由C++程序根据机器人版本和测试模式动态生成
        self.total_pairs = 0  # 将由C++程序输出中解析得到
        self.motor_pairs = []  # 将由C++程序输出中解析得到

    def clear_screen(self):
        """清屏并移动光标到开头"""
        print("\033[2J\033[H", end="")

    def print_header(self):
        """打印头部信息"""
        print_colored_text("====================== start motor following check =====================", color="green", bold=True)
        print_colored_text(f"测试区域: {self.region_name}", color="blue", bold=True)
        print_colored_text(f"电机对: {', '.join(self.motor_pairs)}", color="cyan")
        print_colored_text(f"总测试对数: {self.total_pairs}", color="blue")
        print()
        
        # 生成支持点击的超链接
        clickable_link = f"\x1b]8;;file://{self.output_file_path}\x1b\\{self.output_file_path}\x1b]8;;\x1b\\"
        print(f"电机跟随性测试已启动，输出已保存到 {clickable_link}")
        print("按Ctrl+C退出，或输入x结束测试...")
        print()

    def refresh_display(self):
        """刷新整个显示"""
        self.clear_screen()
        self.print_header()
        self.flush_log()

    def update_test_status(self, index, status, time_str=None, detail=None):
        """更新测试状态"""
        # 简单的状态更新，不再需要表格显示
        pass

    def start_test_timer(self, index):
        """开始测试计时"""
        self.start_times[index] = time.time()

    def get_test_time(self, index):
        """获取测试耗时"""
        if index in self.start_times:
            elapsed = time.time() - self.start_times[index]
            return f"{elapsed:.1f}s"
        return "0.0s"

    def input_thread(self, process):
        """输入处理线程"""
        import select
        import sys
        
        while process and process.poll() is None:
            try:
                # 只在启用时才处理输入
                if not self.input_thread_enabled:
                    time.sleep(0.1)
                    continue
                    
                # 使用select.select来检查是否有输入，避免阻塞
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = sys.stdin.readline().strip()  # 获取输入并清理空格
                    if process.stdin:
                        # 直接发送原始输入内容
                        process.stdin.write(user_input + "\n")
                        process.stdin.flush()
                        
                        # 如果输入x则退出
                        if user_input.lower() == 'x':
                            print("收到退出命令，正在结束测试...")
                            self.roslaunch_running = False
                            break
                        
            except (BrokenPipeError, AttributeError):
                break
            except Exception as e:
                print(f"输入线程错误: {e}", file=sys.stderr)
                break

    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        print("\n收到Ctrl+C, 正在关闭程序...")
        if self.process and self.process.poll() is None:
            # 发送退出命令
            if self.process.stdin:
                self.process.stdin.write('x\n')  # 发送退出命令
                self.process.stdin.flush()
            self.process.terminate()  # 发送SIGTERM
            try:
                self.process.wait(timeout=3)  # 等待3秒
            except subprocess.TimeoutExpired:
                self.process.kill()  # 强制终止
        self.roslaunch_running = False
        sys.exit(0)

    def register_signal_handler(self, process):
        """注册信号处理函数"""
        self.process = process
        import signal
        handler = lambda sig, frame: self.signal_handler(sig, frame)
        signal.signal(signal.SIGINT, handler)

    def check_package_compiled(self, package_names):
        """检查指定的ROS包是否已经编译"""
        if not develFound and not installedFound:
            print_colored_text("未找到编译产物！", color="red", bold=True)
            print_colored_text("请先运行 catkin build 编译项目", color="yellow")
            return False
        return True

    def run_motor_follow_test(self):
        """运行电机跟随性测试"""
        try:
            # 获取项目根目录（相对于脚本位置）
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.abspath(os.path.join(script_dir, "../../../.."))
            
            # 构造启动命令，使用launch文件启动
            launch_file = os.path.join(kuavo_ros_control_path, "src/kuavo-ros-control-lejulib/hardware_node/launch/motor_follow_test.launch")
            
            if not os.path.exists(launch_file):
                print_colored_text(f"Launch文件不存在: {launch_file}", color="red", bold=True)
                return 1
            
            full_command = (
                f"source {kuavo_ros_control_path}/devel/setup.bash && "
                f"echo '检查ROS环境...' && "
                f"echo '开始启动launch文件...' && "
                f"roslaunch hardware_node motor_follow_test.launch "
                f"mode:=real test_mode:={self.test_mode}"
            )

            # 定义输出文件路径，始终相对于本脚本目录，且带时间戳和区域标识
            script_dir = os.path.dirname(os.path.abspath(__file__))
            timestamp = time.strftime("%Y%m%d%H%M")
            self.output_file_path = os.path.join(script_dir, f"motor_follow_{self.test_region}_output_{timestamp}.log")

            # 确保输出目录存在
            os.makedirs(os.path.dirname(self.output_file_path), exist_ok=True)

            # 初始化进度跟踪变量
            current_pair = 0
            test_results = []
            test_details = []  # 存储每对电机的详细信息
            current_pair_progress = {"left": 0, "right": 0}  # 当前电机对的周期进度
            current_motor_pair = None  # 当前测试的电机对索引
            
            # 测试计时器
            self.start_times = {}
            
            # 初始化显示标记
            self.test_started_shown = False
            in_detail_result = False
            
            print_colored_text("=== 电机跟随性测试开始 ===", color="green", bold=True)
            print_colored_text(f"测试区域: {self.region_name}", color="blue", bold=True)
            print_colored_text("等待C++程序输出测试对信息...", color="blue", bold=True)
            print_colored_text("=" * 50, color="blue")
            
            # 初始化进度跟踪变量
            current_pair = 0
            self.test_started_shown = False

            # 打开文件以写入模式
            with open(self.output_file_path, 'w') as output_file:
                # 启动进程
                self.process = subprocess.Popen(
                    ['bash', '-c', full_command],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    universal_newlines=True,
                )

                # 注册信号处理
                self.register_signal_handler(self.process)
                self.roslaunch_running = True
                
                # 生成支持点击的超链接
                clickable_link = f"\x1b]8;;file://{self.output_file_path}\x1b\\{self.output_file_path}\x1b]8;;\x1b\\"
                print(f"电机跟随性测试已启动，输出已保存到 {clickable_link}")
                print("按Ctrl+C退出，或输入x结束测试...")
                print()

                # 启动输入线程处理用户输入
                motor_thread = threading.Thread(target=self.input_thread, args=(self.process,), daemon=True)
                motor_thread.start()

                # 实时监控输出并过滤目标日志
                while self.process.poll() is None:
                    line = self.process.stdout.readline()
                    if line:
                        output_file.write(line)
                        output_file.flush()
                        
                        # 检查是否为目标日志信息
                        for target_log in self.target_logs:
                            if target_log in line:
                                line_stripped = line.strip()
                                
                                # 处理不同的输出类型
                                if "测试对数量:" in line_stripped:
                                    # 解析测试对数量
                                    try:
                                        parts = line_stripped.split(":")
                                        if len(parts) >= 2:
                                            self.total_pairs = int(parts[1].strip())
                                            print_colored_text(f"检测到 {self.total_pairs} 对电机需要测试", color="blue", bold=True)
                                    except:
                                        pass
                                elif "关节对:" in line_stripped:
                                    # 解析关节对信息
                                    try:
                                        parts = line_stripped.split(":")
                                        if len(parts) >= 2:
                                            joint_info = parts[1].strip()
                                            self.motor_pairs.append(joint_info)
                                    except:
                                        pass
                                elif "电机对进度:" in line_stripped:
                                    # 解析电机对进度
                                    try:
                                        # 格式: "电机对进度: 1 | 左周期: 4/10 | 右周期: 6/10"
                                        parts = line_stripped.split("|")
                                        if len(parts) >= 3:
                                            pair_info = parts[0].split(":")
                                            left_info = parts[1].split(":")
                                            right_info = parts[2].split(":")
                                            
                                            try:
                                                pair_num = int(pair_info[1].strip()) + 1  # 修复：转换为1-based索引
                                            except (ValueError, IndexError):
                                                pair_num = 1  # 默认值
                                            
                                            # 安全解析周期数据
                                            left_cycle_str = left_info[1].strip()
                                            right_cycle_str = right_info[1].strip()
                                            
                                            try:
                                                left_cycle = int(left_cycle_str.split("/")[0]) if "/" in left_cycle_str else 0
                                                right_cycle = int(right_cycle_str.split("/")[0]) if "/" in right_cycle_str else 0
                                            except (ValueError, IndexError):
                                                left_cycle = 0
                                                right_cycle = 0
                                            
                                            # 更新当前电机对进度
                                            current_pair_progress["left"] = left_cycle
                                            current_pair_progress["right"] = right_cycle
                                            
                                            # 计算当前电机对的完成百分比
                                            left_progress = (left_cycle / 10) * 100 if 10 > 0 else 0
                                            right_progress = (right_cycle / 10) * 100 if 10 > 0 else 0
                                            avg_progress = (left_progress + right_progress) / 2
                                            
                                            # 显示当前电机对进度
                                            left_bar = "█" * int(left_progress / 5) + "░" * (20 - int(left_progress / 5)) if 5 > 0 else "░" * 20
                                            right_bar = "█" * int(right_progress / 5) + "░" * (20 - int(right_progress / 5)) if 5 > 0 else "░" * 20
                                            
                                            # 只有在显示过开始信息后才显示进度条
                                            if hasattr(self, 'test_started_shown') and self.test_started_shown:
                                                # 直接使用print函数，不使用print_colored_text，避免时间戳影响\r的效果
                                                print(f"\r🔄 电机对 {pair_num}: 左电机 [{left_bar}] {left_cycle}/10 | 右电机 [{right_bar}] {right_cycle}/10 | 平均: {avg_progress:.1f}%", end="", flush=True)
                                    except:
                                        pass
                                
                                elif "当前测试对:" in line_stripped:
                                    # 这是测试开始的信号，清除当前进度条并更新进度
                                    print()  # 换行，结束当前进度条
                                    progress = ((current_pair - 1) / self.total_pairs) * 100 if self.total_pairs > 0 else 0  # 修改：保持一致的进度计算
                                    progress_bar = "█" * int(progress / 2) + "░" * (50 - int(progress / 2)) if 2 > 0 else "░" * 50
                                    print_colored_text(f"🔄 开始测试第 {(current_pair-1)} 对电机", color="purple", bold=True)
                                    print_colored_text(f"总进度: [{progress_bar}] {progress:.1f}% ({current_pair-1}/{self.total_pairs})", color="cyan")
                                    # 重置当前电机对进度
                                    current_pair_progress = {"left": 0, "right": 0}
                                    # 重新启用进度条显示
                                    self.test_started_shown = True
                                
                                elif "测试完成标记" in line_stripped:
                                    # 测试完成标记
                                    print_colored_text(f"✅ {line_stripped}", color="green", bold=True)
                                
                                elif "电机跟随测试开始" in line_stripped:
                                    print_colored_text(f"🚀 {line_stripped}", color="green", bold=True)
                                    current_pair = 1
                                    test_count = 0
                                
                                elif "电机跟随测试停止" in line_stripped:
                                    print_colored_text(f"🛑 {line_stripped}", color="yellow", bold=True)
                                
                                elif "保存测试对" in line_stripped:
                                    print_colored_text(f"💾 {line_stripped}", color="blue")
                                
                                elif "数据保存完成" in line_stripped:
                                    print_colored_text(f"✅ {line_stripped}", color="green")
                                    current_pair += 1

                                
                                elif "所有关节对测试完成" in line_stripped:
                                    print()  # 确保换行
                                    # 显示最终100%进度
                                    progress = 100.0
                                    progress_bar = "█" * 50
                                    print_colored_text(f"总进度: [{progress_bar}] {progress:.1f}% ({self.total_pairs}/{self.total_pairs})", color="cyan")
                                    print_colored_text(f"🎉 {line_stripped}", color="green", bold=True)
                                    print_colored_text("=" * 50, color="blue")
                                    normal_count = 0
                                    abnormal_count = 0
                                    

                                elif "详细结果" in line_stripped:
                                    in_detail_result = True
                                    continue

                                elif in_detail_result and line_stripped.startswith("测试对"):
                                    test_count += 1
                                    # 统计测试结果正常/异常的电机对数
                                    try:
                                        result = line_stripped.split(":")[-1].strip()
                                        if result == "正常":
                                            normal_count += 1
                                        elif result == "异常":
                                            abnormal_count += 1
                                    except:
                                        pass 

                                elif "统计结果" in line_stripped:
                                    # 显示测试总结
                                    print_colored_text("=== 电机跟随性测试总结 ===", color="green", bold=True)
                                    print_colored_text(f"总测试对数: {test_count}", color="blue")
                                    print_colored_text(f"正常: {normal_count} 对", color="green")
                                    print_colored_text(f"异常: {abnormal_count} 对", color="red")
                                    
                                    if abnormal_count == 0:
                                        print_colored_text("🎉 所有电机对测试通过！", color="green", bold=True)
                                    else:
                                        print_colored_text(f"⚠️  有 {abnormal_count} 对电机测试异常，请根据输出日志，检查相关电机", color="red", bold=True)
                                        
                                    # 提示用户测试完成
                                    print_colored_text("=" * 50, color="blue")
                                    print_colored_text("📊 测试数据已保存并自动分析完成！", color="purple", bold=True)
                                    print_colored_text("📁 数据保存位置:", color="blue")
                                    print_colored_text("   - 测试数据: ./file/", color="cyan")
                                    print_colored_text("   - 分析报告: ./symmetry_analysis_report_*.txt", color="cyan")
                                    print_colored_text("   - 波形图像: ./pics/", color="cyan")
                                
                                else:
                                    # 其他目标日志信息
                                    print(line_stripped)
                                
                                break  # 找到目标日志后跳出内层循环
                
                # 等待进程结束
                self.process.wait()
                print_colored_text("电机跟随性测试已结束", color="green", bold=True)

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
                with open(self.output_file_path, 'a') as output_file:
                    output_file.write(remaining_output)
                    output_file.flush()

    def show_final_summary(self):
        """显示最终总结"""
        print()
        print_colored_text("=" * 50, color="blue")
        print_colored_text("=== 电机跟随性测试总结 ===", color="green", bold=True)
        print_colored_text(f"测试区域: {self.region_name}", color="blue", bold=True)
        print_colored_text("=" * 50, color="blue")

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
                
                # 如果进程仍在运行，强制杀死
                if self.process.poll() is None:
                    print_colored_text("C++程序未响应，强制终止...", color="red")
                    self.process.kill()
                    
                    # 再次等待并读取输出
                    time.sleep(0.2)
                    self.process_remaining_output()
                    
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
                # 最后的尝试：强制杀死
                try:
                    self.process.kill()
                except:
                    pass

    def flush_log(self):
        """刷新日志缓冲区"""
        pass  # 在这个实现中，日志直接写入文件，不需要额外刷新

    def close_log_file(self):
        """关闭日志文件"""
        pass  # 在这个实现中，文件在with语句中自动关闭

def run_motor_follow_test():
    """运行电机跟随性测试的主函数"""
    parser = argparse.ArgumentParser(description="运行电机跟随性测试")
    parser.add_argument("--region", choices=["full", "upper", "lower"], default="full", help="选择测试区域 (full, upper, lower)")
    args = parser.parse_args()

    tester = MotorFollowTest(test_region=args.region)
    return tester.run_motor_follow_test()

if __name__ == "__main__":
    run_motor_follow_test() 