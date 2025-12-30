#!/usr/bin/env python
import rosbag
import argparse
from collections import defaultdict
import bisect
import numpy as np
import math
import json
from dataclasses import dataclass
import questionary
import subprocess
import os
from questionary import Choice, Separator
import rich.console

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

def parse_args():
    parser = argparse.ArgumentParser(description="Process rosbag and generate JSON for robot control")
    parser.add_argument("--input_bag", "-i", required=True, type=str, help="Input bag file")
    return parser.parse_args()

def radians_to_degrees(radians):
    return [math.degrees(rad) for rad in radians]

def round_to_one_decimal(values):
    return [round(value, 1) for value in values]

def read_rosbag(input_bag, topics):
    timestamps = defaultdict(list)
    message_data = defaultdict(list)
    message_counts = {topic: 0 for topic in topics}
    
    with rosbag.Bag(input_bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            timestamp = t.to_sec()
            timestamps[topic].append(timestamp)
            message_counts[topic] += 1
            
            if topic == "/sensors_data_raw":
                q_degrees = radians_to_degrees(msg.joint_data.joint_q)
                message_data[topic].append(round_to_one_decimal(q_degrees))
            elif topic == "/dexhand/state":
                hand_data = list(msg.position)
                message_data[topic].append(round_to_one_decimal(hand_data))
    
    return timestamps, message_counts, message_data, {topic: np.mean(np.diff(timestamps[topic])) for topic in topics}

def check_timestamp_alignment(timestamps, topics, tolerance=0.01):
    def find_start_indices(timestamps, topics, tolerance):
        indices = {topic: 0 for topic in topics}
        max_indices = {topic: len(timestamps[topic]) for topic in topics}
        while all(indices[topic] < max_indices[topic] for topic in topics):
            times = {topic: timestamps[topic][indices[topic]] for topic in topics}
            if max(times.values()) - min(times.values()) <= tolerance:
                return indices
            min_time_topic = min(times, key=times.get)
            indices[min_time_topic] += 1
        return None

    start_indices = find_start_indices(timestamps, topics, tolerance)
    if start_indices is None:
        return 0.0

    aligned_count = 0
    total_count = min(len(timestamps[topic]) - start_indices[topic] for topic in topics)

    for i in range(total_count):
        times = [timestamps[topic][start_indices[topic] + i] for topic in topics]
        if max(times) - min(times) <= tolerance:
            aligned_count += 1

    return (aligned_count / total_count) * 100

def reorganize_timestamps(timestamps, topics, message_data, interval_ms):
    interval_sec = interval_ms / 1000.0

    aligned_start_time = max(timestamps[topic][0] for topic in topics)
    aligned_end_time = min(timestamps[topic][-1] for topic in topics)

    new_timestamps = np.arange(aligned_start_time, aligned_end_time + interval_sec, interval_sec)

    new_data_dict = {topic: [] for topic in topics}
    for topic in topics:
        for new_time in new_timestamps:
            closest_time = bisect.bisect_left(timestamps[topic], new_time)
            if closest_time < len(timestamps[topic]) and abs(timestamps[topic][closest_time] - new_time) <= interval_sec / 2:
                new_data_dict[topic].append(message_data[topic][closest_time])
            else:
                new_data_dict[topic].append([0.0] * len(message_data[topic][0]))

    organized_data = {}
    for i, timestamp in enumerate(new_timestamps):
        arm_data = new_data_dict["/sensors_data_raw"][i][12:26]
        hand_data = new_data_dict["/dexhand/state"][i][:12]
        head_data = new_data_dict["/sensors_data_raw"][i][-2:]
        organized_data[round(timestamp, 3)] = arm_data + hand_data + head_data

    return organized_data, check_timestamp_alignment({topic: new_timestamps for topic in topics}, topics)

def create_json_from_organized_data(organized_data, interval_ms, threshold=1.0):
    frames = []
    calculator = CurveCalculator()

    data_list = list(organized_data.items())
    
    for i in range(len(data_list[0][1])):  # For each servo
        prev_value = None
        prev_point = None
        for index, (timestamp, data) in enumerate(data_list):
            current_value = data[i]
            current_point = Point(int(index * interval_ms), current_value)
            
            if index >= len(frames):
                frames.append({
                    "servos": [None] * len(data),
                    "keyframe": int(index * interval_ms),
                    "attribute": {}
                })
            
            if (index == 0 or index == len(data_list) - 1 or 
                prev_value is None or abs(current_value - prev_value) >= threshold):
                next_point = Point(int((index+1) * interval_ms), data_list[index+1][1][i]) if index < len(data_list) - 1 else None
                cp = calculator.get_control_point(prev_point, current_point, next_point)
                
                frames[index]["servos"][i] = int(round(current_value))
                frames[index]["attribute"][str(i+1)] = {
                    "CP": cp,
                    "CPType": ["AUTO", "AUTO"],
                    "select": False
                }
                prev_value = current_value
                prev_point = current_point
            else:
                frames[index]["servos"][i] = None

    # Remove frames where all servos are None
    frames = [frame for frame in frames if any(servo is not None for servo in frame["servos"])]

    # Update keyframes
    for index, frame in enumerate(frames):
        frame["keyframe"] = int(index * interval_ms)

    return {
        "frames": frames,
        "musics": [],
        "finish": int((len(frames) - 1) * interval_ms),
        "first": 0,
        "version": "9a3101e_beta"
    }

console = rich.console.Console()

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
            topics = ['/sensors_data_raw', '/dexhand/state']
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

        try:
            args = argparse.Namespace(input_bag=option)
            main(args)
            console.print(f"[bold green]转换完成！tact文件已生成。[/bold green]")
        except Exception as e:
            console.print(f"[red]转换过程中发生错误: {str(e)}[/red]")

        Menu.set_current_menu(self.back())

def main(args=None):
    if args is None:
        args = parse_args()
    
    topics = ['/sensors_data_raw', '/dexhand/state']
    # topics = ["/robot_arm_q_v_tau", "/robot_head_motor_position", "/robot_hand_position"]

    timestamps, message_counts, message_data, topic_hz = read_rosbag(args.input_bag, topics)
    
    console.print("原始消息数量和频率：")
    for topic, count in message_counts.items():
        console.print(f"  {topic}: {count} 条消息, {1/topic_hz[topic]:.2f} Hz")

    alignment_percentage = check_timestamp_alignment(timestamps, topics, 0.05)
    console.print(f"\n原始对齐百分比: {alignment_percentage:.2f}%")

    if alignment_percentage == 100.00:
        max_hz = math.ceil(max(1/hz for hz in topic_hz.values()))
        min_allowed_interval = math.ceil(1000 / max_hz)
        console.print(f"\n原始数据中的最大频率: {max_hz} Hz")
        console.print(f"最小允许间隔: {min_allowed_interval} 毫秒")
        
        while True:
            try:
                interval_ms = float(questionary.text(f"请输入所需的时间间隔（毫秒，必须 >= {min_allowed_interval:.2f}）: ").ask())
                if interval_ms >= min_allowed_interval:
                    break
                console.print(f"错误：间隔必须至少为 {min_allowed_interval:.2f} 毫秒。")
            except ValueError:
                console.print("错误：请输入有效的数字。")
        
        organized_data, new_alignment_percentage = reorganize_timestamps(timestamps, topics, message_data, interval_ms)

        json_data = create_json_from_organized_data(organized_data, interval_ms)

        output_file = args.input_bag.rsplit('.', 1)[0] + '_output.tact'
        with open(output_file, 'w') as f:
            json.dump(json_data, f, indent=4)

        console.print(f"\nJSON数据已保存到 {output_file}")


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