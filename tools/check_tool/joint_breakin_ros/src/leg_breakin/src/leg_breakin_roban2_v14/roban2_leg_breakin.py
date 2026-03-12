#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import signal
import threading
import glob
from pathlib import Path
from EcMasterConfig import EcMasterConfig

# 设置Python为无缓冲模式，确保实时输出
os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.flush()
sys.stderr.flush()

# 全局停止标志
stop_program = False

# 颜色定义
class Colors:
    RED = '\033[1;31m'
    YELLOW = '\033[1;33m'
    MAGENTA = '\033[1;35m'
    NC = '\033[0m'  # No Color

class ECLogMonitor:
    """EtherCAT 日志监控类，实时读取并打印错误日志"""
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.stop_monitoring = False
        self.monitor_thread = None
        self.file_handles = {}  # 存储文件句柄和已读取位置
        self.processed_files = set()  # 已处理的文件集合
        
    def find_log_files(self):
        """查找所有错误日志文件"""
        err_log_pattern = str(self.log_dir / "EC_log_err*.log")
        log_files = glob.glob(err_log_pattern)
        return log_files
    
    def parse_log_line(self, line):
        """解析日志行，提取关键信息"""
        line = line.strip()
        if not line:
            return None, None
        
        # 尝试解析时间戳（格式：0000010429: ERROR: ...）
        parts = line.split(':', 2)
        if len(parts) >= 3:
            timestamp = parts[0]
            level = parts[1].strip()
            message = parts[2].strip()
            return level, message
        elif len(parts) == 2:
            timestamp = parts[0]
            message = parts[1].strip()
            # 检查是否是错误信息
            if 'ERROR' in message.upper() or 'out of sync' in message.lower():
                return 'ERROR', message
            return None, message
        
        return None, line
    
    def format_log_message(self, level, message, filename):
        """格式化日志消息并添加颜色"""
        file_basename = Path(filename).name
        timestamp = time.strftime('%H:%M:%S', time.localtime())
        
        if level and 'ERROR' in level.upper():
            return f"{Colors.RED}[{timestamp}] [{file_basename}] {level}: {message}{Colors.NC}"
        elif 'out of sync' in message.lower() or 'out-of-sync' in message.lower():
            return f"{Colors.YELLOW}[{timestamp}] [{file_basename}] {message}{Colors.NC}"
        elif 'Redundancy' in message or 'break' in message.lower():
            return f"{Colors.YELLOW}[{timestamp}] [{file_basename}] {message}{Colors.NC}"
        else:
            return f"{Colors.MAGENTA}[{timestamp}] [{file_basename}] {message}{Colors.NC}"
    
    def monitor_file(self, filepath):
        """监控单个日志文件的新增内容"""
        try:
            if filepath in self.file_handles:
                file_handle, last_pos = self.file_handles[filepath]
                # 检查文件句柄是否仍然有效
                try:
                    file_handle.tell()
                except (ValueError, IOError):
                    # 文件句柄已失效，重新打开
                    try:
                        file_handle.close()
                    except:
                        pass
                    file_handle = open(filepath, 'r', encoding='utf-8', errors='ignore')
                    file_handle.seek(0, 2)  # 移动到文件末尾
                    last_pos = file_handle.tell()
                    self.file_handles[filepath] = (file_handle, last_pos)
            else:
                # 打开文件，从文件末尾开始读取新内容
                file_handle = open(filepath, 'r', encoding='utf-8', errors='ignore')
                file_handle.seek(0, 2)  # 移动到文件末尾
                last_pos = file_handle.tell()
                self.file_handles[filepath] = (file_handle, last_pos)
            
            # 检查文件是否有新内容
            file_handle.seek(0, 2)  # 移动到文件末尾
            file_size = file_handle.tell()
            
            if file_size > last_pos:
                # 读取新内容
                file_handle.seek(last_pos)
                new_content = file_handle.read()
                lines = new_content.split('\n')
                
                for line in lines:
                    if line.strip():
                        level, message = self.parse_log_line(line)
                        formatted_msg = self.format_log_message(level, message, filepath)
                        print(formatted_msg, flush=True)
                
                self.file_handles[filepath] = (file_handle, file_size)
            else:
                # 恢复到上次位置
                file_handle.seek(last_pos)
                
        except Exception as e:
            # 文件可能被重新创建或删除，重置句柄
            if filepath in self.file_handles:
                try:
                    self.file_handles[filepath][0].close()
                except:
                    pass
                del self.file_handles[filepath]
    
    def monitor_loop(self):
        """监控循环，在后台线程中运行"""
        while not self.stop_monitoring:
            try:
                # 查找所有错误日志文件
                log_files = self.find_log_files()
                
                # 监控每个文件
                for log_file in log_files:
                    if os.path.exists(log_file):
                        self.monitor_file(log_file)
                
                # 休眠一段时间再检查
                time.sleep(0.1)  # 100ms 检查一次
                
            except Exception as e:
                # 静默处理异常，避免影响主程序
                pass
    
    def start(self):
        """启动监控线程"""
        if self.log_dir.exists():
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
    
    def stop(self):
        """停止监控"""
        self.stop_monitoring = True
        # 关闭所有文件句柄
        for file_handle, _ in self.file_handles.values():
            try:
                file_handle.close()
            except:
                pass
        self.file_handles.clear()

# 信号处理函数
def signal_handler(signum, frame):
    """处理中断信号"""
    global stop_program
    print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 收到停止信号，正在安全停止Roban2腿部磨线程序...")
    stop_program = True
    
    # 尝试清理EC Master资源
    try:
        if 'ec_master_wrap' in sys.modules:
            ec_master_wrap.cleanupEcMaster()
            print(f"{time.strftime('%H:%M:%S', time.localtime())} EC Master资源已清理")
    except Exception as e:
        print(f"{time.strftime('%H:%M:%S', time.localtime())} [警告] 清理EC Master资源时出错: {e}")
    
    # 立即退出程序
    print(f"{time.strftime('%H:%M:%S', time.localtime())} Roban2腿部磨线程序正在退出...")
    sys.exit(0)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 检查是否有root权限
if os.geteuid() != 0:
    print("\033[31merror: 请使用root权限运行\033[0m")
    sys.exit(1)

# 获取当前脚本所在目录（而非工作目录）
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "build_lib"
target_path = os.path.join(script_dir, relative_path)
sys.path.append(target_path)

import ec_master_wrap
g_EcMasterConfig = EcMasterConfig()

# 获取机器人型号并设置到C++层
robot_type, _ = g_EcMasterConfig.get_robot_type_and_slave_num(g_EcMasterConfig.robot_version)
ec_master_wrap.set_robot_model(robot_type)

# 设置驱动器类型到C++层
ec_master_wrap.set_driver_type(g_EcMasterConfig.driver_type)
# 设置C++层的全局驱动器类型
ec_master_wrap.set_global_driver_type(g_EcMasterConfig.driver_type)
print(f"\033[1;33m机器人型号: {robot_type}, 驱动器类型: {g_EcMasterConfig.driver_type}\033[0m")

def main():
    # 注意：运行时长不再从stdin读取，而是由主程序通过ROS话题 /breakin/can_start_new_round 控制
    
    # 获取当前脚本所在目录
    script_dir = Path(__file__).parent.absolute()
    ec_log_dir = script_dir / "EC_log"
    
    # 启动 EC 日志监控
    ec_monitor = ECLogMonitor(ec_log_dir)
    ec_monitor.start()
    
    # 初始化编码器范围和命令参数
    # 注意：get_encoder_range 现在使用关节ID，而不是从站ID
    for joint_id in range(1, g_EcMasterConfig.slave_num+1):
        encoder_range = g_EcMasterConfig.get_encoder_range(joint_id)
        if encoder_range is not None:
            # 将关节ID转换为从站ID
            slave_id = g_EcMasterConfig.getSlaveIdByJointId(joint_id)
            if slave_id is not None:
                ec_master_wrap.setEncoderRange(slave_id, encoder_range)
    ec_master_wrap.set_command_args(g_EcMasterConfig.command_args)
    
    # 设置从站ID到关节ID的映射关系（如果需要）
    for slave_id, joint_id in g_EcMasterConfig.slave2joint.items():
        # 注意：如果C++层需要这个映射，需要实现setSlave2Joint函数
        # 目前C++层直接使用从站ID，所以这里暂时不需要
        pass

    # 兼容性参数（已废弃，不再使用心跳文件检查）
    # 保留参数以保持接口兼容性
    check_arm_heartbeat = False

    # 自动查找配置文件路径
    # 从脚本目录向上查找，找到joint_breakin_ros工作空间根目录
    action_config_file = ""
    script_dir = Path(__file__).parent.absolute()
    # 脚本目录通常是 .../joint_breakin_ros/src/leg_breakin/src/leg_breakin_roban2_v14
    # 需要向上4级找到工作空间根目录
    workspace_root = script_dir
    for _ in range(5):  # 最多向上5级
        config_file_path = workspace_root / "config" / "leg_breakin" / "leg_breakin_roban2_v14_config.yaml"
        if config_file_path.exists():
            action_config_file = str(config_file_path)
            print(f"[信息] 找到配置文件: {action_config_file}")
            break
        # 向上查找
        parent = workspace_root.parent
        if parent == workspace_root:  # 已到达根目录
            break
        workspace_root = parent
    
    # 如果还没找到，尝试一些常见路径
    if not action_config_file:
        possible_paths = [
            script_dir / ".." / ".." / ".." / ".." / "config" / "leg_breakin" / "leg_breakin_roban2_v14_config.yaml",
            script_dir / ".." / ".." / ".." / ".." / ".." / "config" / "leg_breakin" / "leg_breakin_roban2_v14_config.yaml",
        ]
        for path in possible_paths:
            abs_path = path.resolve()
            if abs_path.exists():
                action_config_file = str(abs_path)
                print(f"[信息] 找到配置文件: {action_config_file}")
                break

    success = False
    try:
        # 直接调用C++层实现的Roban2腿部磨线函数，所有逻辑都在C++层完成
        # C++层会阻塞直到运动完成并自动退出
        # C++层会处理同步和停止信号
        # 运行时长由主程序通过ROS话题 /breakin/can_start_new_round 控制
        # 传递配置文件路径给C++函数
        success = ec_master_wrap.Roban2LegBreakin(check_arm_heartbeat, action_config_file)
        if not success:
            print("\033[1;31m✘ Roban2磨线运动失败或被中断\033[0m")
    except KeyboardInterrupt:
        print(f"\n{time.strftime('%H:%M:%S', time.localtime())} 用户中断，正在安全停止...")
        stop_program = True
    except Exception as e:
        print(f"\033[1;31m✘ 程序运行时发生异常: {e}\033[0m")
        import traceback
        traceback.print_exc()
    finally:
        # 确保清理EC Master资源
        try:
            print(f"{time.strftime('%H:%M:%S', time.localtime())} 正在清理EC Master资源...")
            ec_master_wrap.cleanupEcMaster()
            print(f"{time.strftime('%H:%M:%S', time.localtime())} ✓ EC Master资源已清理")
        except Exception as e:
            print(f"{time.strftime('%H:%M:%S', time.localtime())} [警告] 清理EC Master资源时出错: {e}")
        
        # 停止 EC 日志监控
        try:
            ec_monitor.stop()
            print(f"{time.strftime('%H:%M:%S', time.localtime())} EC日志监控已停止")
        except Exception as e:
            print(f"{time.strftime('%H:%M:%S', time.localtime())} [警告] 停止EC日志监控时出错: {e}")
        
        if success:
            print("\033[1;32m✓ Roban2磨线运动完成\033[0m")

if __name__ == "__main__":
    main()
