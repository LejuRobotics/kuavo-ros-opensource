#!/usr/bin/env python3

import os
import subprocess
import sys
import threading
import time
import select
import tty
import termios

try:
    import questionary
    from rich.console import Console
    from rich.table import Table
except ImportError:
    print("Installing required packages...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "questionary", "rich"])
    import questionary
    from rich.console import Console
    from rich.table import Table

import re


console = Console()
# 默认路径设置为脚本所在目录
rosbag_log_save_path = os.path.dirname(os.path.abspath(__file__))

def get_bag_info(bag_path):
    """获取bag文件的详细信息"""
    try:
        result = subprocess.run(
            ["rosbag", "info", bag_path],
            capture_output=True,
            text=True,
            check=True
        )
        info_text = result.stdout

        info = {
            "path": bag_path,
            "topics": [],
            "duration": 0.0,
            "start_time": None,
            "end_time": None,
            "message_count": 0,
            "size": 0
        }

        # 解析话题信息
        topic_pattern = r"(\S+)\s+(\d+)\s+msgs\s+:\s+(\S+)"
        for match in re.finditer(topic_pattern, info_text):
            topic_name = match.group(1)
            msg_count = int(match.group(2))
            msg_type = match.group(3)
            info["topics"].append({
                "name": topic_name,
                "count": msg_count,
                "type": msg_type
            })
            info["message_count"] += msg_count

        # 解析时长
        duration_match = re.search(r"duration:\s+([\d.]+)s", info_text)
        if duration_match:
            info["duration"] = float(duration_match.group(1))

        # 解析开始和结束时间
        start_match = re.search(r"start:\s+([\d.]+)", info_text)
        end_match = re.search(r"end:\s+([\d.]+)", info_text)
        if start_match:
            info["start_time"] = float(start_match.group(1))
        if end_match:
            info["end_time"] = float(end_match.group(1))

        # 解析文件大小
        size_match = re.search(r"size:\s+([\d.]+)\s+([KMGT]?B)", info_text)
        if size_match:
            size_value = float(size_match.group(1))
            size_unit = size_match.group(2)
            info["size"] = f"{size_value} {size_unit}"

        return info
    except subprocess.CalledProcessError as e:
        console.print(f"[red]获取bag文件信息失败: {e}[/red]")
        return None
    except Exception as e:
        console.print(f"[red]解析bag文件信息失败: {e}[/red]")
        return None

def display_bag_info(info):
    """显示bag文件信息"""
    table = Table(title="Bag文件信息", show_header=True, header_style="bold magenta")
    table.add_column("属性", style="cyan")
    table.add_column("值", style="green")

    table.add_row("文件路径", info["path"])
    table.add_row("时长", f"{info['duration']:.2f} 秒")
    table.add_row("消息总数", str(info["message_count"]))
    if info["size"]:
        table.add_row("文件大小", info["size"])

    console.print(table)
    
    # 显示话题列表
    if info["topics"]:
        topic_table = Table(title="话题列表", show_header=True, header_style="bold blue")
        topic_table.add_column("话题名称", style="yellow")
        topic_table.add_column("消息数量", style="green")
        topic_table.add_column("消息类型", style="cyan")
        
        for topic in info["topics"]:
            topic_table.add_row(
                topic["name"],
                str(topic["count"]),
                topic["type"]
            )
        
        console.print(topic_table)

def select_bag_file():
    """选择bag文件"""
    # 首先询问是否使用默认路径或自定义路径
    path_choice = questionary.select(
        "选择bag文件来源",
        choices=["从默认目录选择", "输入自定义路径"]
    ).ask()
    
    if path_choice is None:
        return None
    
    if path_choice == "从默认目录选择":
        base_dir = rosbag_log_save_path
        if not os.path.exists(base_dir):
            console.print(f"[red]默认目录不存在: {base_dir}[/red]")
            return None
        
        # 检查当前目录下是否有直接的.bag文件
        direct_bag_files = [file for file in os.listdir(base_dir) 
                           if file.endswith(".bag") and os.path.isfile(os.path.join(base_dir, file))]
        
        # 检查是否有日期文件夹
        date_folders = [folder for folder in os.listdir(base_dir) 
                       if os.path.isdir(os.path.join(base_dir, folder))]
        
        # 如果当前目录下有直接的.bag文件，优先显示这些文件
        if direct_bag_files:
            if date_folders:
                # 两者都有，让用户选择
                file_choice = questionary.select(
                    "选择文件来源",
                    choices=["当前目录下的bag文件", "日期文件夹中的bag文件"]
                ).ask()
                if file_choice is None:
                    return None
                
                if file_choice == "当前目录下的bag文件":
                    selected_rosbag = questionary.select("请选择一个rosbag文件", choices=direct_bag_files).ask()
                    if selected_rosbag is None:
                        return None
                    return os.path.join(base_dir, selected_rosbag)
                # 否则继续处理日期文件夹
            else:
                # 只有直接文件，直接选择
                selected_rosbag = questionary.select("请选择一个rosbag文件", choices=direct_bag_files).ask()
                if selected_rosbag is None:
                    return None
                return os.path.join(base_dir, selected_rosbag)
        
        # 处理日期文件夹
        if not date_folders:
            console.print("[red]未找到日期文件夹或bag文件[/red]")
            return None

        selected_date = questionary.select("请选择一个日期文件夹", choices=date_folders).ask()
        if selected_date is None:
            return None
        
        date_folder_path = os.path.join(base_dir, selected_date)
        rosbag_files = [file for file in os.listdir(date_folder_path) 
                       if file.endswith(".bag")]

        if not rosbag_files:
            console.print("[red]所选日期文件夹中未找到rosbag文件[/red]")
            return None

        selected_rosbag = questionary.select("请选择一个rosbag文件", choices=rosbag_files).ask()
        if selected_rosbag is None:
            return None
        
        return os.path.join(date_folder_path, selected_rosbag)
    else:
        # 自定义路径 - 允许重新输入直到文件存在或用户取消
        while True:
            custom_path = questionary.text("请输入bag文件的完整路径 (输入 'q' 退出):").ask()
            if custom_path is None or not custom_path.strip():
                return None
            
            # 检查是否要退出
            if custom_path.strip().lower() == 'q':
                return None
            
            custom_path = os.path.expanduser(custom_path.strip())
            if not os.path.exists(custom_path):
                console.print(f"[red]文件不存在: {custom_path}[/red]")
                console.print("[yellow]请重新输入路径，或输入 'q' 退出[/yellow]\n")
                continue  # 继续循环，允许重新输入
            
            if not custom_path.endswith(".bag"):
                console.print("[yellow]警告: 文件不是.bag格式[/yellow]")
                # 询问是否继续
                continue_choice = questionary.confirm("文件不是.bag格式，是否继续?", default=False).ask()
                if not continue_choice:
                    continue  # 重新输入
            
            # 文件存在，返回路径
            return custom_path

def check_node_running(node_name):
    """检查ROS节点是否运行"""
    try:
        result = subprocess.run(
            ["rosnode", "list"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5
        )
        return node_name in result.stdout
    except Exception:
        return False

def check_topic_exists(topic_name):
    """检查ROS话题是否存在"""
    try:
        result = subprocess.run(
            ["rostopic", "list"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5
        )
        return topic_name in result.stdout
    except Exception:
        return False

def select_playback_target():
    """选择回放目标：mujoco仿真或真机"""
    target_choice = questionary.select(
        "选择回放目标",
        choices=["Mujoco仿真", "机器人真机"]
    ).ask()
    
    if target_choice is None:
        return None
    
    playback_target = {
        "type": "mujoco" if target_choice == "Mujoco仿真" else "real_robot",
        "name": target_choice
    }
    
    # 检查环境是否准备好
    console.print(f"\n[bold cyan]检查 {target_choice} 环境...[/bold cyan]")
    
    if playback_target["type"] == "mujoco":
        # 检查mujoco相关节点
        mujoco_nodes = ["/mujoco_simulator", "/humanoid_mpc_observation"]
        mujoco_topics = ["/kuavo_arm_traj", "/control_robot_hand_position"]
        
        nodes_ok = any(check_node_running(node) for node in mujoco_nodes)
        topics_ok = any(check_topic_exists(topic) for topic in mujoco_topics)
        
        if not nodes_ok and not topics_ok:
            console.print("[yellow]警告: 未检测到Mujoco仿真环境运行[/yellow]")
            console.print("[yellow]请确保已启动: roslaunch humanoid_controllers load_kuavo_mujoco_sim_with_vr.launch[/yellow]")
            continue_choice = questionary.confirm("是否继续回放?", default=True).ask()
            if not continue_choice:
                return None
        else:
            console.print("[green]✓ Mujoco仿真环境检测正常[/green]")
    else:
        # 检查真机相关节点
        real_nodes = ["/humanoid_mpc_observation", "/humanoid_quest_control_with_arm"]
        real_topics = ["/kuavo_arm_traj", "/control_robot_hand_position"]
        
        nodes_ok = any(check_node_running(node) for node in real_nodes)
        topics_ok = any(check_topic_exists(topic) for topic in real_topics)
        
        if not nodes_ok and not topics_ok:
            console.print("[yellow]警告: 未检测到真机控制环境运行[/yellow]")
            console.print("[yellow]请确保已启动: roslaunch humanoid_controllers load_kuavo_real_wheel_vr.launch[/yellow]")
            console.print("[red]注意: 在真机上回放前，请确保机器人处于安全状态！[/red]")
            continue_choice = questionary.confirm("是否继续回放?", default=False).ask()
            if not continue_choice:
                return None
        else:
            console.print("[green]✓ 真机控制环境检测正常[/green]")
            # 真机回放需要额外确认
            safety_confirm = questionary.confirm(
                "[bold red]确认: 机器人已处于安全状态，可以开始回放?[/bold red]",
                default=False
            ).ask()
            if not safety_confirm:
                console.print("[red]已取消回放[/red]")
                return None
    
    return playback_target

def playback_rosbag():
    """回放rosbag文件（增强版）"""
    # 选择bag文件
    bag_path = select_bag_file()
    if bag_path is None:
        return
    
    # 获取bag文件信息
    console.print("\n[bold cyan]正在读取bag文件信息...[/bold cyan]")
    bag_info = get_bag_info(bag_path)
    if bag_info is None:
        console.print("[red]无法读取bag文件信息[/red]")
        return
    
    # 显示bag文件信息
    console.print("\n")
    display_bag_info(bag_info)
    console.print("\n")
    
    # 选择回放目标
    playback_target = select_playback_target()
    if playback_target is None:
        return
    
    # 询问回放选项
    playback_options = {}
    playback_options["target"] = playback_target
    
    # 定义VR上肢动作的关键话题
    vr_arm_topics = [
        # 核心控制话题（必需）
        "/kuavo_arm_traj",              # 手臂轨迹 - 核心控制话题
        "/humanoid_mpc_target_arm",     # MPC手臂目标轨迹（重要！MPC需要这个来执行动作）
        "/humanoid_mpc_arm_commanded",  # MPC手臂命令轨迹
        
        # 辅助轨迹话题
        "/kuavo_arm_traj_filtered",     # 滤波后的手臂轨迹
        "/kuavo_arm_traj_origin",       # 原始手臂轨迹
        
        # 手部和头部控制
        "/control_robot_hand_position", # 手部位置控制
        "/robot_head_motion_data",      # 头部运动数据
        
        # IK相关
        "/drake_ik/eef_pose",           # 末端执行器位置（逆运动学解算结果）
        "/drake_ik/ik_solve_error",     # IK解算误差
        "/humanoid_ee_State",           # 末端执行器状态
        
        # 控制模式（通常排除，手动设置）
        # "/humanoid/mpc/arm_control_mode", # 手臂控制模式（通常排除，避免模式切换）
        "/modeSchedule",                 # 模式调度（可能包含控制模式信息）
    ]
    
    # 选择回放模式
    playback_mode = questionary.select(
        "选择回放模式",
        choices=[
            "VR原始数据回放（回放 /leju_quest_bone_poses 和 /quest_joystick_data）",
            "VR上肢动作（仅回放VR控制的上肢相关话题）",
            "自定义选择话题",
            "回放所有话题"
        ]
    ).ask()
    
    if playback_mode is None:
        return
    
    selected_topics = []
    
    if playback_mode == "VR原始数据回放（回放 /leju_quest_bone_poses 和 /quest_joystick_data）":
        # 回放VR原始数据话题
        vr_raw_topics = [
            "/leju_quest_bone_poses",    # Quest3骨骼姿态数据
            "/quest_joystick_data"       # Quest3手柄数据
        ]
        
        # 检查bag文件中是否存在这些话题
        available_vr_raw_topics = []
        for topic in bag_info["topics"]:
            if topic["name"] in vr_raw_topics:
                available_vr_raw_topics.append(topic["name"])
        
        if not available_vr_raw_topics:
            console.print("[red]错误: bag文件中未找到VR原始数据话题[/red]")
            console.print("[yellow]需要的话题: /leju_quest_bone_poses, /quest_joystick_data[/yellow]")
            return
        
        selected_topics = available_vr_raw_topics
        
        console.print(f"\n[bold green]✓ 已选择VR原始数据回放模式[/bold green]")
        console.print(f"[bold green]将回放 {len(selected_topics)} 个VR原始数据话题:[/bold green]")
        for topic_name in selected_topics:
            topic_info = next((t for t in bag_info["topics"] if t["name"] == topic_name), None)
            if topic_info:
                console.print(f"  - {topic_name:<50} ({topic_info['count']:,} msgs)")
        
        # 检查IK节点是否运行
        console.print("\n[bold cyan]检查IK节点状态...[/bold cyan]")
        ik_nodes = [
            "ik_ros_uni",                    # Python IK节点
            "ik_ros_uni_cpp_node",           # C++ IK节点
            "quest3_node"                    # Quest3节点
        ]
        
        running_ik_nodes = []
        for node_name in ik_nodes:
            if check_node_running(f"/{node_name}"):
                running_ik_nodes.append(node_name)
        
        if running_ik_nodes:
            console.print(f"[green]✓ 检测到IK节点运行中: {', '.join(running_ik_nodes)}[/green]")
            console.print("[green]  IK节点将接收VR原始数据并转换为机器人控制命令[/green]")
        else:
            console.print("[bold red]⚠️  警告: 未检测到IK节点运行！[/bold red]")
            console.print("[yellow]  IK节点（ik_ros_uni 或 ik_ros_uni_cpp_node）必须运行才能将VR数据转换为控制命令[/yellow]")
            console.print("[yellow]  请确保已启动: roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch[/yellow]")
            continue_choice = questionary.confirm("IK节点未运行，是否继续回放?", default=False).ask()
            if not continue_choice:
                return
        
        # 检查IK节点订阅的话题
        console.print("\n[bold cyan]检查话题订阅情况...[/bold cyan]")
        for topic_name in selected_topics:
            try:
                result = subprocess.run(
                    ["rostopic", "info", topic_name],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                if "Subscribers:" in result.stdout:
                    lines = result.stdout.split('\n')
                    subscribers = []
                    in_subscribers = False
                    for line in lines:
                        if "Subscribers:" in line:
                            in_subscribers = True
                            continue
                        if in_subscribers and line.strip() and not line.startswith("Publishers:"):
                            subscribers.append(line.strip())
                        if "Publishers:" in line:
                            break
                    
                    if subscribers:
                        console.print(f"[green]✓ {topic_name} 有订阅者:[/green]")
                        for sub in subscribers[:3]:  # 只显示前3个
                            console.print(f"    └─ {sub}")
                    else:
                        console.print(f"[yellow]⚠ {topic_name} 没有订阅者[/yellow]")
                        console.print(f"[yellow]    IK节点可能未订阅此话题，数据可能不会被处理[/yellow]")
            except Exception:
                console.print(f"[yellow]⚠ 无法检查 {topic_name} 的订阅情况[/yellow]")
        
        # 检查IK节点输出的关键话题订阅情况
        console.print("\n[bold cyan]检查IK输出话题订阅情况...[/bold cyan]")
        ik_output_topics = {
            "/kuavo_arm_traj": "手臂轨迹（IK节点输出）",
            "/control_robot_hand_position": "手部位置控制（IK节点输出）"
        }
        
        for topic_name, description in ik_output_topics.items():
            try:
                result = subprocess.run(
                    ["rostopic", "info", topic_name],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                if "Subscribers:" in result.stdout:
                    lines = result.stdout.split('\n')
                    subscribers = []
                    in_subscribers = False
                    for line in lines:
                        if "Subscribers:" in line:
                            in_subscribers = True
                            continue
                        if in_subscribers and line.strip() and not line.startswith("Publishers:"):
                            subscribers.append(line.strip())
                        if "Publishers:" in line:
                            break
                    
                    if subscribers:
                        console.print(f"[green]✓ {topic_name} 有订阅者 ({description}):[/green]")
                        for sub in subscribers[:3]:
                            console.print(f"    └─ {sub}")
                        # 特别检查 humanoid_quest_control_with_arm 是否订阅了 /kuavo_arm_traj
                        if topic_name == "/kuavo_arm_traj":
                            has_quest_control = any("humanoid_quest_control_with_arm" in sub for sub in subscribers)
                            if not has_quest_control:
                                console.print(f"[bold red]⚠️  警告: humanoid_quest_control_with_arm 节点未订阅 /kuavo_arm_traj！[/bold red]")
                                console.print(f"[yellow]  这会导致手臂轨迹无法转换为MPC目标轨迹，上肢将不会动作！[/yellow]")
                                console.print(f"[yellow]  解决方案: 检查 humanoid_quest_control_with_arm 节点配置，确保它订阅了 /kuavo_arm_traj[/yellow]")
                    else:
                        console.print(f"[bold red]⚠️  {topic_name} 没有订阅者 ({description})[/bold red]")
                        console.print(f"[red]    警告: 没有节点订阅此话题，IK节点输出的数据不会被处理！[/red]")
                        if topic_name == "/kuavo_arm_traj":
                            console.print(f"[yellow]    这会导致上肢无法动作！请检查 humanoid_quest_control_with_arm 节点是否运行并订阅了此话题[/yellow]")
            except Exception as e:
                console.print(f"[yellow]⚠ 无法检查 {topic_name} 的订阅情况: {e}[/yellow]")
        
        console.print("\n[bold cyan]数据流说明:[/bold cyan]")
        console.print("  1. 回放 /leju_quest_bone_poses → IK节点接收 → 计算逆运动学 → 发布 /kuavo_arm_traj")
        console.print("  2. 回放 /quest_joystick_data → IK节点接收 → 处理手柄数据 → 控制手部/头部")
        console.print("  3. /kuavo_arm_traj → humanoid_quest_control_with_arm 节点 → 转换为 /humanoid_mpc_target_arm")
        console.print("  4. /humanoid_mpc_target_arm → MPC节点 → 执行机器人动作")
        console.print("\n[yellow]注意: 需要确保以下节点都在运行:[/yellow]")
        console.print("  - IK节点 (ik_ros_uni 或 ik_ros_uni_cpp_node)")
        console.print("  - humanoid_quest_control_with_arm 节点")
        console.print("  - MPC节点 (humanoid_sqp_mpc)")
        
        # 重要警告：关于IK失败和节点订阅问题
        console.print("\n[bold red]⚠️  重要提示:[/bold red]")
        console.print("[yellow]如果回放时只有手指动，上肢不动，可能的原因:[/yellow]")
        console.print("  1. IK节点第二阶段求解失败率过高（检查IK节点日志）")
        console.print("  2. humanoid_quest_control_with_arm 节点未订阅 /kuavo_arm_traj")
        console.print("  3. 手臂控制模式未设置为外部控制模式 (2)")
        console.print("\n[yellow]如果看到 '[TorsoIK] 第二阶段IK求解失败' 错误:[/yellow]")
        console.print("  - IK成功率低于30%时，大部分VR数据无法转换为有效的手臂轨迹")
        console.print("  - 可能原因: VR数据超出机器人工作空间，或机器人当前姿态不匹配")
        console.print("  - 建议: 检查IK节点日志，调整IK参数或机器人初始姿态")
        
    elif playback_mode == "VR上肢动作（仅回放VR控制的上肢相关话题）":
        # 优先使用 /humanoid_mpc_target_arm + /control_robot_hand_position 组合
        priority_topics = [
            "/humanoid_mpc_target_arm",     # MPC手臂目标轨迹（优先）
            "/control_robot_hand_position"  # 手部位置控制（优先）
        ]
        
        # 检查bag文件中是否存在优先话题
        available_priority_topics = []
        for topic in bag_info["topics"]:
            if topic["name"] in priority_topics:
                available_priority_topics.append(topic["name"])
        
        # 如果优先话题都存在，使用优先组合
        if len(available_priority_topics) == len(priority_topics):
            selected_topics = available_priority_topics
            console.print(f"\n[bold green]✓ 使用高保真回放模式（最接近录制时动作）[/bold green]")
            console.print(f"[bold green]已选择 {len(selected_topics)} 个核心话题:[/bold green]")
            for topic_name in selected_topics:
                topic_info = next((t for t in bag_info["topics"] if t["name"] == topic_name), None)
                if topic_info:
                    console.print(f"  - {topic_name:<50} ({topic_info['count']:,} msgs)")
            
            console.print("\n[bold cyan]数据流说明:[/bold cyan]")
            console.print("  - 将直接回放 /humanoid_mpc_target_arm 话题（MPC实际执行的目标轨迹）")
            console.print("  - 将回放 /control_robot_hand_position 话题（手部位置控制）")
            console.print("  - MPC 节点直接接收目标轨迹并执行动作，最接近录制时的实际动作")
        
        # 如果缺少优先话题，回退到备选方案
        else:
            console.print(f"\n[bold yellow]⚠ 未找到完整的优先话题组合[/bold yellow]")
            missing_topics = [t for t in priority_topics if t not in available_priority_topics]
            if missing_topics:
                console.print(f"[yellow]缺少话题: {', '.join(missing_topics)}[/yellow]")
            
            # 从bag文件中筛选出存在的VR上肢话题作为备选
            available_vr_topics = []
            for topic in bag_info["topics"]:
                if topic["name"] in vr_arm_topics:
                    available_vr_topics.append(topic["name"])
            
            # 排除控制模式话题，避免模式切换导致的问题
            available_vr_topics = [t for t in available_vr_topics if "/humanoid/mpc/arm_control_mode" not in t]
            
            if available_vr_topics:
                console.print(f"\n[bold yellow]使用备选方案: 已选择 {len(available_vr_topics)} 个VR上肢动作话题[/bold yellow]")
                for topic_name in available_vr_topics:
                    topic_info = next((t for t in bag_info["topics"] if t["name"] == topic_name), None)
                    if topic_info:
                        console.print(f"  - {topic_name:<50} ({topic_info['count']:,} msgs)")
                
                # 检查是否包含关键话题
                has_kuavo_arm_traj = any("/kuavo_arm_traj" in t for t in available_vr_topics)
                has_mpc_target = any("/humanoid_mpc_target_arm" in t for t in available_vr_topics)
                has_hand_position = any("/control_robot_hand_position" in t for t in available_vr_topics)
                has_head_motion = any("/robot_head_motion_data" in t for t in available_vr_topics)
                
                console.print("\n[bold yellow]话题说明:[/bold yellow]")
                console.print("  - 已排除 /humanoid/mpc/arm_control_mode 话题，避免控制模式切换")
                console.print("  - 将在回放前手动设置控制模式为外部控制模式 (2)")
                
                # 检查头部运动数据话题
                if has_head_motion:
                    console.print("\n[bold yellow]⚠ 注意: 检测到 /robot_head_motion_data 话题[/bold yellow]")
                    console.print("[yellow]  如果回放时出现 'Invalid robot head motion data' 警告，[/yellow]")
                    console.print("[yellow]  说明该话题数据格式不正确，可以选择排除该话题[/yellow]")
                    exclude_head_choice = questionary.confirm(
                        "是否排除 /robot_head_motion_data 话题（避免警告信息）?",
                        default=False
                    ).ask()
                    if exclude_head_choice:
                        available_vr_topics = [t for t in available_vr_topics if "/robot_head_motion_data" not in t]
                        console.print("[green]✓ 已排除 /robot_head_motion_data 话题[/green]")
                
                if has_mpc_target:
                    console.print("\n[bold cyan]数据流说明:[/bold cyan]")
                    console.print("  - 将直接回放 /humanoid_mpc_target_arm 话题")
                    console.print("  - MPC 节点直接接收目标轨迹并执行动作")
                    if has_hand_position:
                        console.print("  - 将回放 /control_robot_hand_position 话题（手部控制）")
                elif has_kuavo_arm_traj:
                    console.print("\n[bold cyan]数据流说明:[/bold cyan]")
                    console.print("  - 将回放 /kuavo_arm_traj 话题")
                    console.print("  - humanoid_quest_control_with_arm 节点会将其转换为 /humanoid_mpc_target_arm")
                    console.print("  - MPC 节点接收 /humanoid_mpc_target_arm 并执行动作")
                    console.print("  - [yellow]确保 humanoid_quest_control_with_arm 节点正在运行！[/yellow]")
                    if has_hand_position:
                        console.print("  - 将回放 /control_robot_hand_position 话题（手部控制）")
                
                selected_topics = available_vr_topics
            else:
                console.print("[yellow]警告: 未找到VR上肢动作相关话题，将回放所有话题[/yellow]")
                selected_topics = None
    
    elif playback_mode == "自定义选择话题":
        # 选择回放的话题
        if bag_info["topics"]:
            topic_names = [topic["name"] for topic in bag_info["topics"]]
            selected_topics = questionary.checkbox(
                "选择要回放的话题（不选择则回放所有话题）",
                choices=topic_names
            ).ask()
            
            if selected_topics is None:
                return
    else:
        # 回放所有话题
        selected_topics = None
    
    if selected_topics:
        playback_options["topics"] = selected_topics
    
    # 回放速度
    rate_choice = questionary.select(
        "选择回放速度",
        choices=["0.25x (慢速)", "0.5x (半速)", "1.0x (正常)", "2.0x (倍速)", "4.0x (快速)", "自定义"]
    ).ask()
    
    if rate_choice is None:
        return
    
    if rate_choice == "自定义":
        rate_input = questionary.text("请输入回放速度倍数 (例如: 0.5, 1.0, 2.0):").ask()
        if rate_input:
            try:
                playback_options["rate"] = float(rate_input)
            except ValueError:
                console.print("[red]无效的速度值，使用默认速度 1.0x[/red]")
                playback_options["rate"] = 1.0
        else:
            playback_options["rate"] = 1.0
    else:
        rate_map = {
            "0.25x (慢速)": 0.25,
            "0.5x (半速)": 0.5,
            "1.0x (正常)": 1.0,
            "2.0x (倍速)": 2.0,
            "4.0x (快速)": 4.0
        }
        playback_options["rate"] = rate_map[rate_choice]
    
    # 是否循环播放
    loop = questionary.confirm("是否循环播放?", default=False).ask()
    if loop:
        playback_options["loop"] = True
    
    # 起始时间
    start_time = questionary.text(
        f"从第几秒开始播放? (0-{bag_info['duration']:.1f}, 留空则从开始):"
    ).ask()
    
    if start_time and start_time.strip():
        try:
            start_sec = float(start_time.strip())
            if 0 <= start_sec <= bag_info["duration"]:
                playback_options["start"] = start_sec
            else:
                console.print(f"[yellow]起始时间超出范围，将从开始播放[/yellow]")
        except ValueError:
            console.print("[yellow]无效的起始时间，将从开始播放[/yellow]")
    
    # 播放时长
    duration = questionary.text(
        f"播放多少秒? (留空则播放到结束):"
    ).ask()
    
    if duration and duration.strip():
        try:
            duration_sec = float(duration.strip())
            if duration_sec > 0:
                playback_options["duration"] = duration_sec
        except ValueError:
            console.print("[yellow]无效的播放时长，将播放到结束[/yellow]")
    
    # 构建rosbag play命令
    command = ["rosbag", "play"]
    
    # 添加话题过滤
    if "topics" in playback_options:
        command.append("--topics")
        command.extend(playback_options["topics"])
    
    # 添加回放速度
    if "rate" in playback_options:
        command.extend(["-r", str(playback_options["rate"])])
    
    # 添加循环播放
    if playback_options.get("loop", False):
        command.append("-l")
    
    # 添加起始时间
    if "start" in playback_options:
        command.extend(["-s", str(playback_options["start"])])
    
    # 添加播放时长
    if "duration" in playback_options:
        command.extend(["-u", str(playback_options["duration"])])
    
    # 不添加安静模式，以便看到回放进度
    # command.append("-q")  # 注释掉，以便调试
    
    # 添加bag文件路径
    command.append(bag_path)
    
    # 检查手臂控制相关节点
    console.print("\n[bold cyan]检查手臂控制节点...[/bold cyan]")
    arm_control_nodes = [
        "humanoid_quest_control_with_arm",      # VR控制节点（重要！将/kuavo_arm_traj转换为MPC目标轨迹）
        "humanoid_Arm_time_target_control",     # 手臂时间目标控制
        "humanoid_plan_arm_trajectory_node"     # 手臂轨迹规划节点
    ]
    
    running_arm_nodes = []
    for node_name in arm_control_nodes:
        if check_node_running(f"/{node_name}"):
            running_arm_nodes.append(node_name)
    
    if running_arm_nodes:
        console.print(f"[green]✓ 检测到手臂控制节点运行中: {', '.join(running_arm_nodes)}[/green]")
        
        # 特别检查VR控制节点
        if "humanoid_quest_control_with_arm" in running_arm_nodes:
            console.print("[green]  ✓ humanoid_quest_control_with_arm 节点运行中[/green]")
            console.print("[green]    此节点会将 /kuavo_arm_traj 转换为 /humanoid_mpc_target_arm[/green]")
        else:
            console.print("[yellow]⚠ 未检测到 humanoid_quest_control_with_arm 节点[/yellow]")
            console.print("[yellow]  此节点负责将 /kuavo_arm_traj 转换为 MPC 目标轨迹[/yellow]")
            console.print("[yellow]  如果此节点未运行，需要直接回放 /humanoid_mpc_target_arm 话题[/yellow]")
    else:
        console.print("[yellow]⚠ 未检测到手臂控制节点运行[/yellow]")
        console.print("[yellow]  这可能是手臂没有动作的原因！[/yellow]")
        console.print("[yellow]  请确保已启动包含手臂控制的launch文件[/yellow]")
        console.print("[yellow]  特别是 humanoid_quest_control_with_arm 节点（VR控制节点）[/yellow]")
    
    # 检查手臂控制模式话题
    console.print("\n[bold cyan]检查手臂控制模式...[/bold cyan]")
    arm_mode_topic = "/humanoid/mpc/arm_control_mode"
    try:
        result = subprocess.run(
            ["rostopic", "echo", arm_mode_topic, "-n", "1"],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0 and result.stdout.strip():
            console.print(f"[green]✓ 当前手臂控制模式话题有数据[/green]")
            console.print(f"[dim]{result.stdout.strip()[:100]}...[/dim]")
        else:
            console.print(f"[yellow]⚠ 无法获取手臂控制模式，可能需要手动设置[/yellow]")
    except Exception:
        console.print(f"[yellow]⚠ 无法检查手臂控制模式话题[/yellow]")
    
    console.print("\n[bold red]重要提示:[/bold red]")
    console.print("  如果看到警告: 'armControlMode_ 1 != EXTERN_CONTROL'")
    console.print("  说明手臂控制模式未设置为外部控制模式，需要:")
    console.print("  1. 确保回放 /humanoid/mpc/arm_control_mode 话题")
    console.print("  2. 或者手动切换控制模式（如果bag文件中没有该话题）")
    
    # 检查关键话题的订阅者
    console.print("\n[bold cyan]检查话题订阅者...[/bold cyan]")
    critical_topics = {
        "/kuavo_arm_traj": "手臂轨迹控制（必需）",
        "/humanoid_mpc_target_arm": "MPC手臂目标轨迹（重要！MPC执行动作需要）",
        "/control_robot_hand_position": "手部位置控制（必需）",
        "/robot_head_motion_data": "头部运动数据"
    }
    
    topics_with_subscribers = []
    topics_without_subscribers = []
    
    for topic_name, description in critical_topics.items():
        try:
            result = subprocess.run(
                ["rostopic", "info", topic_name],
                capture_output=True,
                text=True,
                timeout=3
            )
            if "Subscribers:" in result.stdout and "None" not in result.stdout:
                # 提取订阅者信息
                lines = result.stdout.split('\n')
                subscribers = []
                in_subscribers = False
                for line in lines:
                    if "Subscribers:" in line:
                        in_subscribers = True
                        continue
                    if in_subscribers and line.strip() and not line.startswith("Publishers:"):
                        subscribers.append(line.strip())
                    if "Publishers:" in line:
                        break
                
                if subscribers:
                    topics_with_subscribers.append((topic_name, description, subscribers))
                else:
                    topics_without_subscribers.append((topic_name, description))
            else:
                topics_without_subscribers.append((topic_name, description))
        except Exception as e:
            topics_without_subscribers.append((topic_name, description))
    
    if topics_with_subscribers:
        console.print("[green]✓ 以下话题有订阅者（可以正常接收数据）:[/green]")
        for topic_name, description, subscribers in topics_with_subscribers:
            console.print(f"  - {topic_name:<40} {description}")
            for sub in subscribers[:2]:  # 只显示前2个订阅者
                console.print(f"    └─ {sub}")
    
    if topics_without_subscribers:
        console.print("[yellow]⚠ 以下话题没有订阅者（可能无法正常执行动作）:[/yellow]")
        for topic_name, description in topics_without_subscribers:
            console.print(f"  - {topic_name:<40} {description}")
            console.print(f"    [red]警告: 没有节点订阅此话题，回放的数据可能不会被处理！[/red]")
    
    # 特别检查手臂控制模式
    if "/humanoid/mpc/arm_control_mode" in [t[0] for t in topics_without_subscribers]:
        console.print("\n[bold red]⚠️  关键问题: 手臂控制模式话题没有订阅者！[/bold red]")
        console.print("[yellow]这会导致手臂控制模式无法切换，手臂将不会执行动作。[/yellow]")
        console.print("[yellow]解决方案: 确保 MPC 节点正在运行并订阅该话题。[/yellow]")
    
    # 显示回放配置
    console.print("\n[bold green]回放配置:[/bold green]")
    config_table = Table(show_header=False, box=None)
    config_table.add_column("选项", style="cyan")
    config_table.add_column("值", style="green")
    
    config_table.add_row("文件路径", bag_path)
    config_table.add_row("回放目标", playback_options.get("target", {}).get("name", "未知"))
    config_table.add_row("回放速度", f"{playback_options.get('rate', 1.0)}x")
    config_table.add_row("循环播放", "是" if playback_options.get("loop", False) else "否")
    if "start" in playback_options:
        config_table.add_row("起始时间", f"{playback_options['start']:.1f} 秒")
    if "duration" in playback_options:
        config_table.add_row("播放时长", f"{playback_options['duration']:.1f} 秒")
    if "topics" in playback_options:
        topics_list = playback_options["topics"]
        config_table.add_row("回放话题数量", str(len(topics_list)))
        # 显示关键话题
        arm_traj_included = any("/kuavo_arm_traj" in t for t in topics_list)
        hand_pos_included = any("/control_robot_hand_position" in t for t in topics_list)
        head_motion_included = any("/robot_head_motion_data" in t for t in topics_list)
        
        mpc_target_included = any("/humanoid_mpc_target_arm" in t for t in topics_list)
        
        config_table.add_row("包含手臂轨迹", "✓ 是" if arm_traj_included else "✗ 否")
        config_table.add_row("包含MPC目标轨迹", "✓ 是" if mpc_target_included else "✗ 否 [red]重要！[/red]")
        config_table.add_row("包含手部位置", "✓ 是" if hand_pos_included else "✗ 否")
        config_table.add_row("包含头部运动", "✓ 是" if head_motion_included else "✗ 否")
        
        if not mpc_target_included:
            console.print("\n[bold red]⚠️  警告: 未包含 /humanoid_mpc_target_arm 话题！[/bold red]")
            console.print("[yellow]MPC需要此话题来接收目标轨迹，否则会出现 'target not updated' 警告[/yellow]")
            console.print("[yellow]建议: 在VR上肢动作模式下，确保包含此话题[/yellow]")
        
        # 显示所有话题（如果数量不多）
        if len(topics_list) <= 10:
            config_table.add_row("回放话题", ", ".join(topics_list))
        else:
            config_table.add_row("回放话题", f"{len(topics_list)} 个话题（前5个: {', '.join(topics_list[:5])}...）")
    else:
        config_table.add_row("回放话题", "所有话题")
    
    console.print(config_table)
    
    # 根据回放目标显示不同的提示
    target_name = playback_options.get("target", {}).get("name", "")
    if "真机" in target_name:
        console.print("\n[bold red]⚠️  真机回放警告:[/bold red]")
        console.print("  - 请确保机器人周围安全，有足够的空间")
        console.print("  - 随时准备按 Ctrl+C 停止回放")
        console.print("  - 建议先以慢速回放测试")
    
    # 操作提示将在回放开始时显示（包含循环模式信息）
    
    # 检查并设置手臂控制模式
    console.print("\n[bold cyan]设置手臂控制模式...[/bold cyan]")
    console.print("[yellow]重要: 需要将控制模式设置为外部控制模式 (2)，手臂才能执行动作[/yellow]")
    
    # 检查是否回放了控制模式话题（如果回放了，可能会覆盖我们的设置）
    has_arm_mode_topic = False
    if "topics" in playback_options:
        has_arm_mode_topic = any("/humanoid/mpc/arm_control_mode" in t for t in playback_options["topics"])
    
    if has_arm_mode_topic:
        console.print("[yellow]⚠ 警告: 检测到将回放控制模式话题[/yellow]")
        console.print("[yellow]  这可能导致控制模式在回放过程中被切换，建议排除此话题[/yellow]")
        exclude_mode_choice = questionary.confirm(
            "是否排除控制模式话题，改为手动设置?",
            default=True
        ).ask()
        
        if exclude_mode_choice:
            playback_options["topics"] = [t for t in playback_options["topics"] if "/humanoid/mpc/arm_control_mode" not in t]
            console.print("[green]✓ 已排除控制模式话题[/green]")
    
    # 尝试调用服务设置控制模式为外部控制（2）
    set_mode_choice = questionary.confirm(
        "是否设置手臂控制模式为外部控制模式 (2)?",
        default=True
    ).ask()
    
    if set_mode_choice:
        try:
            console.print("[cyan]正在设置手臂控制模式为外部控制模式 (2)...[/cyan]")
            
            # 首先尝试使用Python rospy直接调用服务（更可靠）
            try:
                # 添加ROS工作空间路径，确保可以导入kuavo_msgs
                import sys
                import os
                ros_ws_path = "/home/lab/kuavo-ros-control"
                if os.path.exists(ros_ws_path):
                    devel_python_path = os.path.join(ros_ws_path, "devel", "lib", "python3", "dist-packages")
                    if devel_python_path not in sys.path:
                        sys.path.insert(0, devel_python_path)
                
                import rospy
                from kuavo_msgs.srv import changeArmCtrlMode
                
                # 如果rospy已经初始化，使用现有的节点，否则初始化新节点
                try:
                    rospy.get_node_uri()
                except:
                    rospy.init_node('vr_rosbag_playback_set_mode', anonymous=True)
                
                rospy.wait_for_service('/humanoid_change_arm_ctrl_mode', timeout=5)
                change_arm_mode = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
                
                # 调用服务
                response = change_arm_mode(control_mode=2)
                
                if response.result:
                    console.print("[green]✓ 手臂控制模式已成功设置为外部控制模式 (2)[/green]")
                    console.print("[green]✓ 现在可以正常接收并执行手臂轨迹了[/green]")
                else:
                    console.print("[yellow]⚠ 服务调用成功，但返回结果为False[/yellow]")
                    console.print("[yellow]  请手动验证控制模式是否已设置[/yellow]")
                    # 即使返回False，也尝试通过话题验证
                    try:
                        import std_msgs.msg
                        pub = rospy.Publisher('/humanoid/mpc/arm_control_mode', std_msgs.msg.Float64MultiArray, queue_size=1)
                        rospy.sleep(0.5)
                        msg = std_msgs.msg.Float64MultiArray()
                        msg.data = [2.0, 2.0]
                        pub.publish(msg)
                        console.print("[yellow]  已尝试通过话题发布控制模式，请验证是否生效[/yellow]")
                    except:
                        pass
                    
            except ImportError as e:
                # 如果无法导入，使用命令行方式
                console.print(f"[yellow]无法使用Python rospy: {e}[/yellow]")
                console.print("[yellow]尝试使用命令行方式...[/yellow]")
                raise
            except rospy.ROSException as e:
                console.print(f"[yellow]ROS服务不可用: {e}[/yellow]")
                console.print("[yellow]尝试使用命令行方式...[/yellow]")
                raise
            except Exception as e:
                console.print(f"[yellow]Python服务调用失败: {e}[/yellow]")
                console.print("[yellow]尝试使用命令行方式...[/yellow]")
                raise
                
        except Exception:
            # 如果Python方式失败，尝试命令行方式
            try:
                console.print("[yellow]尝试使用命令行方式设置控制模式...[/yellow]")
                
                # 尝试多种格式
                result = None
                for cmd_format in [
                    ["rosservice", "call", "/humanoid_change_arm_ctrl_mode", "{control_mode: 2}"],
                    ["rosservice", "call", "/humanoid_change_arm_ctrl_mode", "control_mode: 2"],
                ]:
                    try:
                        result = subprocess.run(
                            cmd_format,
                            capture_output=True,
                            text=True,
                            timeout=5
                        )
                        if result.returncode == 0:
                            break
                    except Exception:
                        continue
                
                if result and result.returncode == 0:
                    if "result: True" in result.stdout or "result: 1" in result.stdout or "Success" in result.stdout:
                        console.print("[green]✓ 手臂控制模式已成功设置为外部控制模式 (2)[/green]")
                    else:
                        console.print("[yellow]⚠ 服务调用成功，但返回结果不确定[/yellow]")
                        console.print(f"[dim]服务响应: {result.stdout[:200]}[/dim]")
                else:
                    # 如果服务调用失败，尝试通过话题发布
                    console.print("[yellow]服务调用失败，尝试通过话题发布控制模式...[/yellow]")
                    try:
                        topic_result = subprocess.run(
                            ["rostopic", "pub", "-1", "/humanoid/mpc/arm_control_mode", 
                             "std_msgs/Float64MultiArray", "data: [2.0, 2.0]"],
                            capture_output=True,
                            text=True,
                            timeout=5
                        )
                        if topic_result.returncode == 0:
                            console.print("[green]✓ 已通过话题发布控制模式为外部控制模式 (2)[/green]")
                        else:
                            raise Exception("话题发布失败")
                    except Exception as topic_e:
                        console.print(f"[red]✗ 无法通过服务或话题设置控制模式[/red]")
                        console.print(f"[dim]服务错误: {result.stderr if result else 'N/A'}[/dim]")
                        console.print(f"[dim]话题错误: {topic_e}[/dim]")
                        console.print("[yellow]  请手动运行以下命令设置控制模式:[/yellow]")
                        console.print("[yellow]  python3 /home/lab/set_arm_mode.py[/yellow]")
                        console.print("[yellow]  或者: rostopic pub -1 /humanoid/mpc/arm_control_mode std_msgs/Float64MultiArray 'data: [2.0, 2.0]'[/yellow]")
            except Exception as e:
                console.print(f"[red]✗ 设置控制模式失败: {e}[/red]")
                console.print("[yellow]  请手动确保手臂控制模式已设置为外部控制模式 (2)[/yellow]")
                console.print("[yellow]  这是机器人不动的关键原因！[/yellow]")
                console.print("[yellow]  可以运行: python3 /home/lab/set_arm_mode.py[/yellow]")
    else:
        console.print("[yellow]⚠ 跳过控制模式设置，请确保控制模式已正确设置，否则手臂可能不会执行动作[/yellow]")
        console.print("[yellow]  可以运行: python3 /home/lab/set_arm_mode.py[/yellow]")
    
    # 倒计时提示
    countdown_choice = questionary.confirm("准备开始回放，是否继续?", default=True).ask()
    if not countdown_choice:
        console.print("[yellow]已取消回放[/yellow]")
        return
    
    console.print("\n[bold green]开始回放...[/bold green]\n")
    
    # 显示实际执行的命令（用于调试）
    console.print(f"[dim]执行命令: {' '.join(command)}[/dim]\n")
    
    # 检查是否循环播放
    is_loop_mode = playback_options.get("loop", False)
    loop_count = 0
    is_paused = False
    should_exit = False
    bag_duration = bag_info.get("duration", 0) if bag_info else 0
    
    # 保存终端设置
    old_settings = None
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    except:
        # 如果无法设置终端（例如在非交互式环境中），使用标准输入
        pass
    
    def get_key():
        """非阻塞获取键盘输入"""
        if old_settings is None:
            return None
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key
        except:
            pass
        return None
    
    # 执行回放
    process = None
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        def keyboard_listener():
            """键盘监听线程"""
            nonlocal is_paused, should_exit
            while process.poll() is None and not should_exit:
                key = get_key()
                if key == ' ':  # 空格键：暂停/继续
                    is_paused = not is_paused
                    if is_paused:
                        console.print("\n[yellow]⏸  已暂停 (按空格键继续)[/yellow]")
                        process.send_signal(subprocess.signal.SIGSTOP)
                    else:
                        console.print("\n[green]▶  继续播放[/green]")
                        process.send_signal(subprocess.signal.SIGCONT)
                elif key == 'q' or key == 'Q':  # Q键：退出
                    should_exit = True
                    console.print("\n[yellow]收到退出信号，正在停止回放...[/yellow]")
                    process.terminate()
                    break
                time.sleep(0.1)
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
        keyboard_thread.start()
        
        # 显示操作提示
        console.print("[bold cyan]操作提示:[/bold cyan]")
        console.print("  - 按 [bold]空格键[/bold] 暂停/继续播放")
        console.print("  - 按 [bold]Q键[/bold] 退出播放")
        if is_loop_mode:
            console.print("  - 循环播放模式已启用，将显示循环次数\n")
        else:
            console.print("")
        
        # 实时显示输出并跟踪循环次数
        line_count = 0
        bag_duration = bag_info.get("duration", 0) if bag_info else 0
        playback_start_time = time.time()
        last_loop_time = playback_start_time
        loop_detection_enabled = is_loop_mode and bag_duration > 0
        
        # 循环检测：通过监控时间来判断（如果播放时间超过bag文件时长，说明开始了新的循环）
        def check_loop_completion():
            """检查是否完成一次循环"""
            nonlocal loop_count, last_loop_time
            if loop_detection_enabled and not is_paused:
                elapsed = time.time() - last_loop_time
                # 如果经过的时间超过bag文件时长（考虑回放速度），可能是新循环
                playback_rate = playback_options.get("rate", 1.0)
                expected_duration = bag_duration / playback_rate
                # 使用85%阈值，避免误判，同时能及时检测到新循环
                if elapsed >= expected_duration * 0.85:
                    loop_count += 1
                    last_loop_time = time.time()
                    console.print(f"\n[bold green]━━━ 第 {loop_count} 次循环开始 ━━━[/bold green]")
                    return True
            return False
        
        # 启动循环检测线程
        def loop_monitor():
            """循环监控线程"""
            while process.poll() is None and not should_exit:
                check_loop_completion()
                time.sleep(0.3)  # 每0.3秒检查一次，更及时
        
        if loop_detection_enabled:
            loop_monitor_thread = threading.Thread(target=loop_monitor, daemon=True)
            loop_monitor_thread.start()
        
        # 第一次循环
        if is_loop_mode:
            loop_count = 1
            console.print(f"\n[bold green]━━━ 第 {loop_count} 次循环开始 ━━━[/bold green]")
        
        for line in process.stdout:
            if should_exit:
                break
                
            line = line.strip()
            if not line:
                continue
            
            # 检测rosbag play的输出中的循环标记
            if is_loop_mode:
                # rosbag play在循环模式下，每次重新开始时可能会输出特定信息
                # 检测常见的循环标记
                if ("Reading" in line and "bag" in line.lower()) or \
                   ("Replaying" in line) or \
                   ("Starting" in line and "playback" in line.lower()):
                    # 这可能是新循环的开始，但我们已经通过时间检测了
                    pass
                elif "Done" in line or "finished" in line.lower() or "complete" in line.lower():
                    # 循环完成标记
                    if loop_count > 0:
                        console.print(f"[bold green]✓ 第 {loop_count} 次循环完成[/bold green]")
            
            # 显示前几行输出（帮助调试）
            if line_count < 5:
                console.print(f"[dim]{line}[/dim]")
                line_count += 1
            
            # 检查进程是否结束
            if process.poll() is not None:
                break
        
        # 等待进程结束
        process.wait()
        
        # 恢复终端设置
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        if should_exit:
            console.print("\n[yellow]回放已由用户退出[/yellow]")
        elif is_loop_mode and loop_count > 0:
            console.print(f"\n[bold green]回放完成! 共循环 {loop_count} 次[/bold green]")
        else:
            console.print("\n[bold green]回放完成![/bold green]")
        
    except KeyboardInterrupt:
        should_exit = True
        if process:
            process.terminate()
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        console.print("\n[red]回放已停止[/red]")
    except Exception as e:
        should_exit = True
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        console.print(f"\n[red]回放出错: {e}[/red]")

def main():
    """主函数：直接启动回放功能"""
    playback_rosbag()

if __name__ == "__main__":
    main()