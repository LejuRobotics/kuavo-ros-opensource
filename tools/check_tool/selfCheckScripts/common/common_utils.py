#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import time
import threading
from datetime import datetime

# 获取当前脚本所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取kuavo_ros_control路径 (从common目录向上4级)
kuavo_ros_control_path = os.path.abspath(os.path.join(current_dir, "../../../.."))

# 检查编译产物
develFound = os.path.exists(os.path.join(kuavo_ros_control_path, "devel"))
installedFound = os.path.exists(os.path.join(kuavo_ros_control_path, "installed"))

def print_colored_text(text, color=None, bold=False, end="\n", level=None):
    """打印彩色文本，支持日志等级和时间戳"""
    level_colors = {
        "INFO": "cyan",
        "WARNING": "yellow",
        "ERROR": "red",
        "SUCCESS": "green",
        "DEBUG": "magenta"
    }
    prefix = ""
    if level:
        level = level.upper()
        color = level_colors.get(level, color)
        prefix = f"[{level}] "
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    prefix = f"[{timestamp}] {prefix}"
    colors = {
        "red": "\033[31m",
        "green": "\033[32m", 
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "white": "\033[37m",
        "purple": "\033[35m"
    }
    bold_code = "\033[1m" if bold else ""
    reset_code = "\033[0m"
    color_code = colors.get(color, "")
    print(f"{bold_code}{color_code}{prefix}{text}{reset_code}", end=end)

def check_package_compiled(package_names):
    """检查包是否已编译"""
    if not develFound and not installedFound:
        print_colored_text("未找到编译产物！", color="red", bold=True)
        print_colored_text("请先运行 catkin build 编译项目", color="yellow")
        return False
    return True

class SlaveTestLogger:
    """SlaveTest日志打印优化类 - 表格样式"""
    
    def __init__(self):
        self.test_results = {}
        self.current_test = None
        self.start_time = None
        self.test_items = {
            "1. 上位机连接测试": {
                "sub_items": ["SSH连接", "认证验证"],
                "status": "⏳",
                "details": "等待中...",
                "duration": 0.0
            },
            "2. 音响服务测试": {
                "sub_items": ["play_music检查", "音频播放"],
                "status": "⏳",
                "details": "等待中...",
                "duration": 0.0
            },
            "3. 麦克风服务测试": {
                "sub_items": ["record_music检查", "音频录制"],
                "status": "⏳",
                "details": "等待中...",
                "duration": 0.0
            },
            "4. 相机数据测试": {
                "sub_items": ["RGB图像检测", "深度图像检测"],
                "status": "⏳",
                "details": "等待中...",
                "duration": 0.0
            },
            "5. 雷达数据测试": {
                "sub_items": ["点云数据检测", "数据验证"],
                "status": "⏳",
                "details": "等待中...",
                "duration": 0.0
            }
        }
        self.sub_test_results = {}
        self.total_start_time = time.time()
        
    def print_header(self):
        """打印测试头部信息"""
        print_colored_text("┌" + "─" * 80 + "┐", color="cyan", bold=True)
        print_colored_text("│" + " " * 26 + "KUAVO SLAVE TEST SUITE" + " " * 26 + "│", color="cyan", bold=True)
        print_colored_text("└" + "─" * 80 + "┘", color="cyan", bold=True)
        print_colored_text(f"测试开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", color="white")
        print()
        
    def print_table_header(self):
        """打印表格头部"""
        print_colored_text("┌" + "─" * 80 + "┐", color="cyan")
        print_colored_text("│ 测试项目                │ 状态  │ 详细信息                │ 耗时   │", color="cyan")
        print_colored_text("├" + "─" * 80 + "┤", color="cyan")
        
    def print_table_footer(self):
        """打印表格底部"""
        print_colored_text("└" + "─" * 80 + "┘", color="cyan")
        
    def print_test_row(self, test_name, status="⏳", details="等待中...", duration=0.0, is_sub_item=False):
        """打印测试行"""
        prefix = "│ ├─ " if is_sub_item else "│ "
        test_name_padded = test_name.ljust(22)
        status_padded = status.ljust(6)
        details_padded = details.ljust(22)
        duration_str = f"{duration:.1f}s".ljust(8)
        
        # 根据状态选择颜色
        status_color = "white"
        if status == "✅":
            status_color = "green"
        elif status == "⚠️":
            status_color = "yellow"
        elif status == "❌":
            status_color = "red"
        elif status == "🔄":
            status_color = "blue"
            
        print_colored_text(f"{prefix}{test_name_padded} │ ", color="white", end="")
        print_colored_text(f"{status_padded} │ ", color=status_color, end="")
        print_colored_text(f"{details_padded} │ ", color="white", end="")
        print_colored_text(f"{duration_str} │", color="white")
        
    def print_separator(self):
        """打印分隔线"""
        print_colored_text("├" + "─" * 80 + "┤", color="cyan")
        
    def print_initial_table(self):
        """打印初始表格"""
        self.print_table_header()
        
        for i, (test_name, test_info) in enumerate(self.test_items.items()):
            self.print_test_row(test_name, test_info["status"], test_info["details"], test_info["duration"])
            
            # 打印子测试项
            for sub_item in test_info["sub_items"]:
                self.print_test_row(sub_item, "⏳", "等待中...", 0.0, is_sub_item=True)
            
            if i < len(self.test_items) - 1:
                self.print_separator()
                
        self.print_table_footer()
        self.print_progress_footer()
        
    def initialize_display(self):
        """初始化显示 - 显示完整的表格"""
        self.print_header()
        self.print_initial_table()
        
    def print_progress_footer(self):
        """打印进度底部"""
        total_tests = sum(len(test_info["sub_items"]) + 1 for test_info in self.test_items.values())
        success_count = sum(1 for result in self.sub_test_results.values() if result.get("status") == "✅")
        warning_count = sum(1 for result in self.sub_test_results.values() if result.get("status") == "⚠️")
        error_count = sum(1 for result in self.sub_test_results.values() if result.get("status") == "❌")
        
        print_colored_text("┌" + "─" * 80 + "┐", color="cyan")
        progress_text = f"测试进度: {success_count + warning_count + error_count}/{total_tests}"
        progress_text += f"    │ ✅ 成功: {success_count}    │ ⚠️ 警告: {warning_count}    │ ❌ 失败: {error_count}"
        if success_count + warning_count + error_count == total_tests:
            total_duration = time.time() - self.total_start_time
            progress_text += f"    │ 通过率: {int((success_count/total_tests)*100)}%"
            print_colored_text(f"│ 测试完成 - 总耗时: {total_duration:.1f}s" + " " * (80 - len(f"测试完成 - 总耗时: {total_duration:.1f}s") - 1) + "│", color="cyan")
        else:
            print_colored_text(f"│ {progress_text}" + " " * (80 - len(progress_text) - 1) + "│", color="cyan")
        print_colored_text("└" + "─" * 80 + "┘", color="cyan")
        
    def start_test(self, test_name, description=""):
        """开始一个测试项"""
        self.current_test = test_name
        self.start_time = time.time()
        
        # 更新主测试项状态
        if test_name in self.test_items:
            self.test_items[test_name]["status"] = "🔄"
            self.test_items[test_name]["details"] = "正在测试..."
            self.test_items[test_name]["start_time"] = self.start_time
            
        self.refresh_table()
        
    def update_test_status(self, status, message=""):
        """更新测试状态"""
        if not self.current_test:
            return
            
        # 更新主测试项状态
        if self.current_test in self.test_items:
            duration = time.time() - self.start_time
            self.test_items[self.current_test]["status"] = status
            self.test_items[self.current_test]["details"] = message
            self.test_items[self.current_test]["duration"] = duration
            
        self.refresh_table()
        
    def update_sub_test_status(self, main_test, sub_test, status, message="", duration=0.0):
        """更新子测试项状态"""
        sub_test_key = f"{main_test}_{sub_test}"
        self.sub_test_results[sub_test_key] = {
            "status": status,
            "message": message,
            "duration": duration
        }
        self.refresh_table()
        
    def refresh_table(self):
        """刷新表格显示"""
        # 清屏（可选，或者使用\r来覆盖）
        print("\033[2J\033[H")  # 清屏并回到顶部
        self.print_header()
        self.print_table_header()
        
        for i, (test_name, test_info) in enumerate(self.test_items.items()):
            self.print_test_row(test_name, test_info["status"], test_info["details"], test_info["duration"])
            
            # 打印子测试项
            for sub_item in test_info["sub_items"]:
                sub_test_key = f"{test_name}_{sub_item}"
                sub_result = self.sub_test_results.get(sub_test_key, {})
                sub_status = sub_result.get("status", "⏳")
                sub_details = sub_result.get("message", "等待中...")
                sub_duration = sub_result.get("duration", 0.0)
                self.print_test_row(sub_item, sub_status, sub_details, sub_duration, is_sub_item=True)
            
            if i < len(self.test_items) - 1:
                self.print_separator()
                
        self.print_table_footer()
        self.print_progress_footer()
        
    def print_connection_status(self, host, port, status, details=""):
        """打印连接状态"""
        if status == "success":
            self.update_sub_test_status("1. 上位机连接测试", "SSH连接", "✅", f"连接正常 {host}:{port}")
            self.update_sub_test_status("1. 上位机连接测试", "认证验证", "✅", "认证通过")
        elif status == "error":
            self.update_sub_test_status("1. 上位机连接测试", "SSH连接", "❌", f"连接失败 {host}:{port}")
            if details:
                self.update_sub_test_status("1. 上位机连接测试", "认证验证", "❌", details)
                
    def print_service_test_result(self, service_name, test_type, result, details=""):
        """打印服务测试结果"""
        if "play_music" in service_name:
            self.update_sub_test_status("2. 音响服务测试", "play_music检查", "✅" if result == "success" else "❌", details)
            if result == "success":
                self.update_sub_test_status("2. 音响服务测试", "音频播放", "✅", "播放成功")
        elif "record_music" in service_name:
            self.update_sub_test_status("3. 麦克风服务测试", "record_music检查", "✅" if result == "success" else "❌", details)
            if result == "success":
                self.update_sub_test_status("3. 麦克风服务测试", "音频录制", "✅", "录制成功")
        
    def print_sensor_data_status(self, sensor_type, data_type, status, count=0):
        """打印传感器数据状态"""
        if "相机" in sensor_type or "camera" in sensor_type.lower():
            if "RGB" in data_type or "rgb" in data_type.lower():
                if status == "success":
                    self.update_sub_test_status("4. 相机数据测试", "RGB图像检测", "✅", "数据有效")
                elif status == "error":
                    self.update_sub_test_status("4. 相机数据测试", "RGB图像检测", "❌", "数据无效")
            elif "深度" in data_type or "depth" in data_type.lower():
                if status == "success":
                    self.update_sub_test_status("4. 相机数据测试", "深度图像检测", "✅", "数据有效")
                elif status == "error":
                    self.update_sub_test_status("4. 相机数据测试", "深度图像检测", "❌", "数据无效")
        elif "雷达" in sensor_type or "lidar" in sensor_type.lower():
            if status == "success":
                self.update_sub_test_status("5. 雷达数据测试", "点云数据检测", "✅", "数据有效")
                self.update_sub_test_status("5. 雷达数据测试", "数据验证", "✅", "验证通过")
            elif status == "error":
                self.update_sub_test_status("5. 雷达数据测试", "点云数据检测", "❌", "数据无效")
                self.update_sub_test_status("5. 雷达数据测试", "数据验证", "❌", "验证失败") 