#!/usr/bin/env python
import subprocess
import sys
import os

def check_and_install_dependencies():
    """检查并安装必要的依赖包"""
    import importlib.util

    # 检查 prompt-toolkit 版本
    try:
        import prompt_toolkit
        if prompt_toolkit.__version__ != "2.0.10":
            print(f"prompt-toolkit 版本为 {prompt_toolkit.__version__}，需要安装 2.0.10 版本...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "prompt-toolkit==2.0.10"])
            # 重新加载模块
            importlib.invalidate_caches()
    except ImportError:
        print("prompt-toolkit 未安装，正在安装 2.0.10 版本...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "prompt-toolkit==2.0.10"])

    # 检查 questionary
    try:
        import questionary
        if questionary.__version__ != "2.1.0":
            print(f"questionary 版本为 {questionary.__version__}，需要安装 2.1.0 版本...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "questionary==2.1.0"])
            importlib.invalidate_caches()
    except ImportError:
        print("questionary 未安装，正在安装 2.1.0 版本...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "questionary==2.1.0"])

# 在导入其他依赖之前检查并安装
check_and_install_dependencies()

import rosbag
import argparse
from collections import defaultdict
import bisect
import numpy as np
import math
import json
from dataclasses import dataclass
import questionary
import rospy
from questionary import Choice, Separator
import rich.console

# 初始化 console（需要在导入 RosbagToBezierPlanner 之前，因为错误处理中会用到）
console = rich.console.Console()

# 导入 RosbagToBezierPlanner 类
# 假设两个脚本在同一目录下
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
try:
    from rosbag_to_bezier_planner import RosbagToBezierPlanner
except ImportError:
    # 如果直接导入失败，提示错误
    console.print("[red]错误：无法导入 RosbagToBezierPlanner 类[/red]")
    console.print("[yellow]请确保 rosbag_to_bezier_planner.py 在同一目录下[/yellow]")
    sys.exit(1)

@dataclass
class Point:
    x: float
    y: float

def get_point(x, y):
    return Point(x, y)

class CurveCalculator:
    def __init__(self):
        self.ab_curve_scaling = 0.5

    def get_target_k(self, prev_point, target_point, next_point):
        k = 0
        if ((prev_point.y < target_point.y < next_point.y) or
            (prev_point.y > target_point.y > next_point.y)):
            v1 = get_point(prev_point.x - target_point.x, prev_point.y - target_point.y)
            v2 = get_point(next_point.x - target_point.x, next_point.y - target_point.y)
            
            if v2.x != 0 and abs(v1.x / v2.x - v1.y / v2.y) < 1e-6:  # Use epsilon for float comparison
                k = v1.y / v1.x if v1.x != 0 else 0
            else:
                tmp_point_x = (prev_point.x + target_point.x) / 2.0 - (target_point.x + next_point.x) / 2.0
                tmp_point_y = (prev_point.y + target_point.y) / 2.0 - (target_point.y + next_point.y) / 2.0
                k = tmp_point_y / tmp_point_x if tmp_point_x != 0 else 0
        return k

    def get_control_point(self, prev_point, target_point, next_point):
        left_cp = [0, 0]
        right_cp = [0, 0]
        k = 0
        if prev_point and next_point:
            k = self.get_target_k(prev_point, target_point, next_point)
        
        if prev_point:
            ab_interval = -(target_point.x - prev_point.x)
            left_cp = [
                ab_interval * self.ab_curve_scaling,
                k * (ab_interval * self.ab_curve_scaling)
            ]
        
        if next_point:
            ab_interval = next_point.x - target_point.x
            right_cp = [
                ab_interval * self.ab_curve_scaling,
                k * (ab_interval * self.ab_curve_scaling)
            ]
        
        return [
            [round(left_cp[0], 1), round(left_cp[1], 1)],
            [round(right_cp[0], 1), round(right_cp[1], 1)]
        ]

# 以下函数和类已不再使用，保留 CurveCalculator 以防其他地方需要
# 转换逻辑已改为使用 RosbagToBezierPlanner

class Menu:
    def __init__(self, title, previous_menu_name=None):
        self.title = title
        self.previous_menu_name = previous_menu_name

    def ask(self):
        question = self.get_question()
        return question.ask()

    def get_question(self):
        return None

    def back(self):
        if self.previous_menu_name:
            return self.previous_menu_name
        return None

    @classmethod
    def set_current_menu(cls, menu):
        cls.current_menu = menu

    @classmethod
    def get_current_menu(cls):
        return cls.current_menu

class MainMenu(Menu):
    def __init__(self):
        super().__init__("主菜单")

    def get_question(self):
        console.print("[bold]欢迎使用Rosbag到Tact文件转换工具[/bold]")
        return questionary.select(
            self.title,
            choices=[
                "1. 录制手臂头部手指rosbag数据",
                "2. 将rosbag数据转成tact文件",
                Separator(),
                "3. 退出",
            ],
        )

    def handle_option(self, option):
        if option.startswith("1"):
            Menu.set_current_menu("RecordRosbagMenu")
        elif option.startswith("2"):
            Menu.set_current_menu("ConvertRosbagMenu")
        elif option.startswith("3") or option is None:
            Menu.set_current_menu("Exit")

class RecordRosbagMenu(Menu):
    def __init__(self):
        super().__init__("录制Rosbag", "MainMenu")

    def get_question(self):
        return questionary.text("请输入要保存的rosbag文件名（不包含.bag后缀）：")

    def handle_option(self, option):
        if option:
            topics = ['/sensors_data_raw', '/dexhand/state']    # 新增默认监听录制手指的话题数据
            #topics = ["/robot_arm_q_v_tau", "/robot_head_motor_position", "/robot_hand_position"]
            topics_str = " ".join(topics)
            command = f"rosbag record -O {option}.bag {topics_str}"
            
            console.print(f"[bold]开始录制Rosbag...[/bold]")
            console.print(f"[green]录制的话题: {topics_str}[/green]")
            console.print("[yellow]按Ctrl+C停止录制[/yellow]")
            
            try:
                subprocess.run(command, shell=True, check=True)
            except subprocess.CalledProcessError:
                console.print("[red]录制过程中发生错误[/red]")
            except KeyboardInterrupt:
                console.print("[green]录制已停止[/green]")
            
            console.print(f"[bold green]Rosbag文件已保存为: {option}.bag[/bold green]")
        
        Menu.set_current_menu(self.back())

class ConvertRosbagMenu(Menu):
    def __init__(self):
        super().__init__("转换Rosbag", "MainMenu")

    def get_question(self):
        rosbag_files = [f for f in os.listdir('.') if f.endswith('.bag')]
        if not rosbag_files:
            console.print("[red]当前目录下没有找到.bag文件[/red]")
            return questionary.text("请输入.bag文件的完整路径：")
        else:
            return questionary.select(
                "请选择要转换的rosbag文件：",
                choices=rosbag_files + [Separator(), "返回上级菜单"]
            )

    def handle_option(self, option):
        if option == "返回上级菜单" or option is None:
            Menu.set_current_menu(self.back())
            return

        if not option.endswith('.bag'):
            option += '.bag'

        if not os.path.exists(option):
            console.print(f"[red]文件 {option} 不存在[/red]")
            Menu.set_current_menu(self.back())
            return

        # 使用 RosbagToBezierPlanner 进行转换
        try:
            console.print("[blue]开始使用贝塞尔曲线规划器转换 rosbag...[/blue]")
            
            
            # 询问参数
            sampling_rate = float(questionary.text(
                "请输入数据预处理采样率（秒，默认0.1）:",
                default="0.1"
            ).ask() or "0.1")
            
            smoothing_factor = float(questionary.text(
                "请输入贝塞尔曲线平滑因子（默认0.3）:",
                default="0.3"
            ).ask() or "0.3")
            
            output_dir = questionary.text(
                "请输入输出目录（默认./bezier_results）:",
                default="./bezier_results"
            ).ask() or "./bezier_results"
            
            # 询问是否保存采样结果
            save_sampling_tact = questionary.confirm(
                "是否保存采样结果为TACT文件？",
                default=True
            ).ask()
            
            custom_sampling_rate = None
            if save_sampling_tact:
                custom_sampling_rate_str = questionary.text(
                    "请输入自定义采样率（秒，默认0.5，留空则不采样）:",
                    default="0.5"
                ).ask() or "0.5"
                if custom_sampling_rate_str:
                    custom_sampling_rate = float(custom_sampling_rate_str)
            
            # 创建处理器实例
            processor = RosbagToBezierPlanner()
            
            # 执行转换
            success = processor.run(
                bag_file_path=option,
                sampling_rate=sampling_rate,
                smoothing_factor=smoothing_factor,
                output_dir=output_dir,
                save_tact=True,
                custom_sampling_rate=custom_sampling_rate,
                get_raw_trajectory=False,
                save_sampling_tact=save_sampling_tact
            )
            
            if success:
                console.print(f"[bold green]转换完成！TACT文件已保存到 {output_dir}[/bold green]")
            else:
                console.print("[red]转换过程中发生错误[/red]")
                
        except KeyboardInterrupt:
            console.print("[yellow]转换被用户中断[/yellow]")
        except Exception as e:
            console.print(f"[red]转换过程中发生错误: {str(e)}[/red]")
            import traceback
            console.print(f"[red]{traceback.format_exc()}[/red]")

        Menu.set_current_menu(self.back())

# main 函数已移除，转换逻辑已改为使用 RosbagToBezierPlanner
# 所有转换相关的函数（read_rosbag, reorganize_timestamps, create_json_from_organized_data 等）
# 已不再使用，转换功能现在由 RosbagToBezierPlanner 类提供


if __name__ == "__main__":
    Menu.set_current_menu("MainMenu")
    main_menu = MainMenu()
    record_rosbag_menu = RecordRosbagMenu()
    convert_rosbag_menu = ConvertRosbagMenu()
    menu_map = {
        "MainMenu": main_menu,
        "RecordRosbagMenu": record_rosbag_menu,
        "ConvertRosbagMenu": convert_rosbag_menu,
        "Exit": None,
    }
    while True:
        current_menu_name = Menu.get_current_menu()
        current_menu = menu_map[current_menu_name]
        if current_menu is None:
            break
        option = current_menu.ask()
        current_menu.handle_option(option)