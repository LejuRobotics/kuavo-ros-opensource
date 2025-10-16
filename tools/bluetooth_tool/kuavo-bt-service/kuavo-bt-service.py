#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import subprocess
import signal
import sys
import os
import logging
import re
from datetime import datetime
from bluetooth_device_scanner import ConnectedDevicesScanner

# 配置
CONFIG_FILE = "/opt/lejurobot/kuavo-bt-service/config.conf"
RETRY_DELAY = 3

class BluetoothAutoConnect:
    def __init__(self):
        self.running = True
        self.target_devices = []
        self.device_scanner = ConnectedDevicesScanner()
        self.connected_check_count = {}  # 记录每个已连接设备的检查次数
        self.last_controller_check = 0   # 上次检查控制器的时间
        self.controller_check_interval = 30  # 控制器检查间隔（秒）
        
        # 配置日志
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.logger = logging.getLogger(__name__)
        
        # 信号处理
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """处理终止信号"""
        self.logger.info("接收到终止信号，正在停止服务...")
        self.running = False
        # 执行清理
        self.cleanup()
        
    def load_config(self):
        """加载配置文件"""
        self.target_devices = []
        self.default_controller = None
        
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith('TARGET_DEVICES='):
                            devices_str = line.split('=', 1)[1].strip('"\'')
                            if devices_str:
                                self.target_devices = [d.strip() for d in devices_str.split(',')]
                        elif line.startswith('DEFAULT_CONTROLLER='):
                            controller_str = line.split('=', 1)[1].strip('"\'')
                            if controller_str:
                                self.default_controller = controller_str
                
                self.logger.info(f"加载配置文件: {CONFIG_FILE}")
                if self.default_controller:
                    self.logger.info(f"默认控制器: {self.default_controller}")
            else:
                self.logger.info("配置文件不存在，使用空配置")
        except Exception as e:
            self.logger.info(f"加载配置文件失败: {e}")
            
    def check_controller_exists(self, controller_mac):
        """检查指定的控制器是否存在"""
        try:
            result = subprocess.run(['bluetoothctl', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if controller_mac.upper() in line.upper():
                        return True
            return False
        except Exception as e:
            self.logger.error(f"检查控制器存在性失败: {e}")
            return False
    
    def get_available_controllers(self):
        """获取所有可用的控制器列表"""
        try:
            result = subprocess.run(['bluetoothctl', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                controllers = []
                for line in result.stdout.split('\n'):
                    match = re.search(r'Controller\ ([0-9A-F:]+)\ (.+)', line)
                    if match:
                        controllers.append({
                            'mac': match.group(1),
                            'name': match.group(2).strip()
                        })
                return controllers
            return []
        except Exception as e:
            self.logger.error(f"获取可用控制器失败: {e}")
            return []
    
    def get_current_default_controller(self):
        """获取当前默认的控制器MAC地址"""
        try:
            result = subprocess.run(['bluetoothctl', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if '[default]' in line:
                        match = re.search(r'Controller\ ([0-9A-F:]+)\ ', line)
                        if match:
                            return match.group(1)
            return None
        except Exception as e:
            self.logger.error(f"获取当前默认控制器失败: {e}")
            return None
    
    def monitor_controller_changes(self):
        """监控控制器变化，如果默认控制器被改变则自动切换回来"""
        try:
            current_time = time.time()
            
            # 检查是否到了检查时间
            if current_time - self.last_controller_check < self.controller_check_interval:
                return
            
            self.last_controller_check = current_time
            
            # 如果没有配置默认控制器，跳过监控
            if not self.default_controller:
                return
            
            # 获取当前默认控制器
            current_default = self.get_current_default_controller()
            
            if current_default:
                # 检查控制器是否发生了变化
                if current_default.upper() != self.default_controller.upper():
                    self.logger.warning(f"检测到控制器变化！期望: {self.default_controller}, 当前: {current_default}")
                    self.logger.info("正在切换回配置的默认控制器...")
                    
                    # 尝试切换回配置的控制器
                    if self.set_default_controller():
                        self.logger.info("控制器切换成功")
                    else:
                        self.logger.error("控制器切换失败")
                        
                        # 如果普通切换失败，尝试强制切换
                        self.logger.info("尝试强制切换控制器...")
                        if self.force_controller_selection():
                            self.logger.info("强制控制器切换成功")
                        else:
                            self.logger.error("强制控制器切换也失败")
                else:
                    # 控制器没有变化，定期确认设置
                    if hasattr(self, '_controller_verification_count'):
                        self._controller_verification_count += 1
                    else:
                        self._controller_verification_count = 0
                    
                    # 每10次检查（约5分钟）进行一次确认性设置
                    if self._controller_verification_count >= 10:
                        self.logger.info("定期确认控制器设置")
                        self.set_default_controller()
                        self._controller_verification_count = 0
            else:
                # 没有当前默认控制器，尝试设置
                self.logger.warning("没有检测到默认控制器，尝试设置")
                if self.set_default_controller():
                    self.logger.info("默认控制器设置成功")
                else:
                    self.logger.error("默认控制器设置失败")
                    
        except Exception as e:
            self.logger.error(f"监控控制器变化失败: {e}")
    
    def set_default_controller(self):
        """设置默认蓝牙控制器"""
        try:
            # 如果没有配置默认控制器，尝试自动选择
            if not self.default_controller:
                self.logger.info("未配置默认控制器，尝试自动选择")
                available_controllers = self.get_available_controllers()
                if available_controllers:
                    self.default_controller = available_controllers[0]['mac']
                    self.logger.info(f"自动选择控制器: {self.default_controller}")
                else:
                    self.logger.info("没有可用的蓝牙控制器，跳过设置")
                    return False
            
            self.logger.info(f"设置默认控制器: {self.default_controller}")
            
            # 检查控制器是否存在
            if not self.check_controller_exists(self.default_controller):
                self.logger.warning(f"指定的控制器不存在: {self.default_controller}")
                
                # 尝试获取可用控制器
                available_controllers = self.get_available_controllers()
                if available_controllers:
                    self.logger.warning("可用的控制器:")
                    for controller in available_controllers:
                        self.logger.warning(f"  - {controller['name']} ({controller['mac']})")
                    
                    # 如果有可用的控制器，使用第一个作为默认
                    first_controller = available_controllers[0]['mac']
                    self.logger.warning(f"使用第一个可用控制器作为默认: {first_controller}")
                    self.default_controller = first_controller
                else:
                    self.logger.info("没有可用的蓝牙控制器，跳过设置")
                    return False
            
            # 首先检查当前默认控制器
            current_default = self.get_current_default_controller()
            if current_default and current_default.upper() == self.default_controller.upper():
                self.logger.info("控制器已经是默认状态，无需重新设置")
                return True
            
            # 使用bluetoothctl设置默认控制器
            result = subprocess.run(['bluetoothctl', 'select', self.default_controller], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.logger.info("默认控制器设置成功")
                # 尝试系统级配置
                self._set_system_default_controller()
                return True
            else:
                self.logger.error(f"设置默认控制器失败: {result.stderr}")
                return False
        except Exception as e:
            self.logger.error(f"设置默认控制器异常: {e}")
            return False
    
    def _set_system_default_controller(self):
        """设置系统级默认控制器"""
        try:
            self.logger.info("系统级控制器设置暂未实现，依赖定期检查机制")
            return True
        except Exception as e:
            self.logger.error(f"系统级控制器设置失败: {e}")
            return False
    
    def force_controller_selection(self):
        """强制设置控制器，包括重置蓝牙服务"""
        try:
            if not self.default_controller:
                return False
                
            self.logger.info(f"强制设置控制器: {self.default_controller}")
            
            # 重启蓝牙服务
            self.logger.info("重启蓝牙服务以应用控制器设置")
            try:
                subprocess.run(['systemctl', 'restart', 'bluetooth'], 
                             capture_output=True, text=True, timeout=30)
                time.sleep(3)  # 等待服务重启
            except Exception as e:
                self.logger.warning(f"重启蓝牙服务失败: {e}")
            
            # 设置默认控制器
            return self.set_default_controller()
            
        except Exception as e:
            self.logger.error(f"强制设置控制器失败: {e}")
            return False
            
    def ensure_bluetooth_audio_modules(self):
        """确保蓝牙音频模块已加载"""
        try:
            self.logger.info("检查蓝牙音频模块...")
            
            # 检查模块是否已加载
            result = subprocess.run(['pactl', 'list', 'short', 'modules'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                if 'module-bluetooth-discover' in result.stdout:
                    self.logger.info("蓝牙音频模块已加载")
                    return True
                else:
                    self.logger.info("蓝牙音频模块未加载，正在加载...")
                    # 加载蓝牙音频模块
                    load_result = subprocess.run(['pactl', 'load-module', 'module-bluetooth-discover'], 
                                               capture_output=True, text=True, timeout=10)
                    if load_result.returncode == 0:
                        self.logger.info("蓝牙音频模块加载成功")
                        return True
                    else:
                        self.logger.error(f"蓝牙音频模块加载失败: {load_result.stderr}")
                        return False
                
        except Exception as e:
            self.logger.error(f"确保蓝牙音频模块失败: {e}")
            return False

    def ensure_scan_off(self):
        """确保蓝牙扫描是关闭的"""
        try:
            self.logger.info("确保蓝牙扫描处于关闭状态")
            subprocess.run(['bluetoothctl', 'scan', 'off'], 
                         capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.logger.error(f"关闭扫描失败: {e}")
            
    def cleanup(self):
        """服务清理"""
        try:
            self.logger.info("执行服务清理，关闭蓝牙扫描")
            self.ensure_scan_off()
        except Exception as e:
            self.logger.error(f"服务清理失败: {e}")
            
    def get_device_name(self, mac):
        """获取设备名称"""
        try:
            # 使用 ConnectedDevicesScanner 获取设备名称
            result = subprocess.run(['bluetoothctl', 'info', mac], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if line.strip().startswith('Name:'):
                        return line.split(':', 1)[1].strip()
            
            return mac
        except Exception as e:
            self.logger.error(f"获取设备名称失败: {e}")
            return mac
            
    def is_device_connected(self, mac):
        """检查设备是否已连接"""
        try:
            # 使用 ConnectedDevicesScanner 获取已连接设备列表
            connected_devices = self.device_scanner.get_connected_devices()
            # 检查指定MAC地址是否在已连接设备列表中
            for device in connected_devices:
                if device['mac_address'].upper() == mac.upper():
                    return True
            return False
        except Exception as e:
            self.logger.error(f"检查连接状态失败: {e}")
            return False
            
    def check_a2dp_errors_and_load_module(self):
        """检查是否有 a2dp-sink 错误并加载模块"""
        try:
            # journalctl -u bluetooth --since '1 minute ago' -g 'Protocol not available'
            result = subprocess.run(['journalctl', '-u', 'bluetooth', '--since', '1 minute ago', '-g', 'Protocol not available'], 
                                  capture_output=True, text=True, timeout=10)
            
            # 检查输出中是否包含 Protocol not available
            if 'Protocol not available' in result.stdout:
                self.logger.warning("检测到 Protocol not available 错误，正在加载蓝牙音频模块...")
                return self.ensure_bluetooth_audio_modules()
            
            return False
        except Exception as e:
            self.logger.error(f"检查 a2dp 错误失败: {e}")
            return False

    def connect_device(self, mac, name):
        """连接设备"""
        try:
            # 直接尝试连接，如果设备已配对且可信，应该能直接连接
            result = subprocess.run(['bluetoothctl', 'connect', mac], 
                                  capture_output=True, text=True, timeout=15)
            
            # 检查连接结果
            if result.returncode == 0:
                # 连接成功后，检查并设置信任
                self.trust_device(mac)
                return True
            else:
                # 记录第一次连接失败的详细信息
                error_msg = result.stderr.strip()
                self.logger.error(f"直接连接失败 {name} ({mac}): {error_msg}")
                
                # 如果直接连接失败，尝试开启扫描后重连
                try:
                    self.logger.info(f"开启扫描模式寻找设备 {name} ({mac})")
                    # 开启扫描
                    subprocess.run(['bluetoothctl', 'scan', 'on'], 
                                 capture_output=True, text=True, timeout=1)
                    
                    # 等待设备被发现
                    import time
                    time.sleep(5)
                    
                    # 再次尝试连接
                    result = subprocess.run(['bluetoothctl', 'connect', mac], 
                                          capture_output=True, text=True, timeout=15)
                    
                    # 如果连接成功，检查并设置信任，并关闭扫描
                    if result.returncode == 0:
                        self.logger.info(f"设备连接成功，关闭扫描模式 {name} ({mac})")
                        self.trust_device(mac)
                        # 连接成功后关闭扫描
                        subprocess.run(['bluetoothctl', 'scan', 'off'], 
                                     capture_output=True, text=True, timeout=1)
                        return True
                    else:
                        # 记录第二次连接失败的详细信息
                        self.logger.error(f"扫描后连接失败 {name} ({mac}): {result.stderr.strip()}")
                        # 连接失败也要关闭扫描
                        subprocess.run(['bluetoothctl', 'scan', 'off'], 
                                     capture_output=True, text=True, timeout=1)
                        return False
                except Exception as scan_error:
                    # 如果扫描过程出错，记录错误并确保关闭扫描
                    self.logger.error(f"扫描过程出错 {name} ({mac}): {scan_error}")
                    try:
                        subprocess.run(['bluetoothctl', 'scan', 'off'], 
                                     capture_output=True, text=True, timeout=1)
                    except:
                        pass
                    return False
                    
        except Exception as e:
            self.logger.error(f"连接设备异常失败 {name} ({mac}): {e}")
            return False
    
    def trust_device(self, mac):
        """检查并设置设备信任"""
        try:
            # 检查设备是否已信任
            result = subprocess.run(['bluetoothctl', 'info', mac], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                if "Trusted: yes" not in result.stdout:
                    # 如果设备未被信任，设置信任
                    trust_result = subprocess.run(['bluetoothctl', 'trust', mac], 
                                                capture_output=True, text=True, timeout=5)
                    if trust_result.returncode == 0:
                        self.logger.info(f"设备已设置信任: {mac}")
                    else:
                        self.logger.info(f"设置设备信任失败: {mac}")
                else:
                    self.logger.info(f"设备已信任: {mac}")
            else:
                self.logger.info(f"无法检查设备信任状态: {mac}")
                
        except Exception as e:
            self.logger.info(f"检查设备信任状态失败: {e}")
            
    def monitor_devices(self):
        """监控设备"""
        self.logger.info(f"开始监控 {len(self.target_devices)} 个目标设备")
        
        # 存储设备状态
        device_states = {}
        
        # 检查并打印所有设备的初始状态，但不存入device_states
        for mac in self.target_devices:
            name = self.get_device_name(mac)
            is_connected = self.is_device_connected(mac)
            status = "已连接" if is_connected else "未连接"
            self.logger.info(f"设备: {name} ({mac}) - {status}")
            # 不在这里存入device_states，让下面的逻辑处理初始连接
        
        while self.running:
            for mac in self.target_devices:
                if not self.running:
                    break
                    
                name = self.get_device_name(mac)
                current_state = self.is_device_connected(mac)
                
                # 调试信息
                previous_state = device_states.get(mac, None)
                if previous_state is not None and previous_state != current_state:
                    self.logger.info(f"调试: {name} 状态变更: {previous_state} -> {current_state}")
                
                # 简化逻辑：只要设备未连接就尝试连接
                if mac not in device_states:
                    # 首次检测到此设备
                    device_states[mac] = current_state
                    # 初始化检查计数
                    self.connected_check_count[mac] = 0
                
                # 如果设备未连接，尝试连接
                if not current_state:
                    self.logger.info(f"设备未连接，尝试连接: {name} ({mac})")
                    if self.connect_device(mac, name):
                        self.logger.info(f"连接成功: {name} ({mac})")
                        device_states[mac] = True  # 更新状态为已连接
                        self.connected_check_count[mac] = 0  # 重置计数
                    else:
                        self.logger.error(f"连接失败: {name} ({mac}) - 请检查设备是否开机并处于配对状态")
                        device_states[mac] = False  # 更新状态为断开
                else:
                    # 设备已连接，更新状态
                    device_states[mac] = True
                    # 增加检查计数
                    self.connected_check_count[mac] += 1
                    # 每10次打印一次已连接设备的日志
                    if self.connected_check_count[mac] % 10 == 0:
                        self.logger.info(f"设备保持连接: {name} (第{self.connected_check_count[mac]}次检查)")
            
            # 动态调整检查频率
            # 检查是否所有设备都已连接
            all_connected = True
            for mac in self.target_devices:
                if mac in device_states and not device_states[mac]:
                    all_connected = False
                    break
            
            # 根据连接状态设置检查间隔
            check_delay = 10 if all_connected else 3
            
            # 监控控制器变化
            self.monitor_controller_changes()
            
            # 如果有设备未连接，每隔5次检查一次是否有 a2dp-sink 错误并加载模块
            if not all_connected:
                if not hasattr(self, '_a2dp_check_count'):
                    self._a2dp_check_count = 0
                
                self._a2dp_check_count += 1
                if self._a2dp_check_count >= 5:
                    self.check_a2dp_errors_and_load_module()
                    self._a2dp_check_count = 0
            
            # 等待下次检查
            for _ in range(check_delay):
                if not self.running:
                    break
                time.sleep(1)
                
    def wait_for_bluetooth(self):
        """等待蓝牙服务启动"""
        self.logger.info("等待蓝牙服务启动...")
        timeout = 30
        count = 0
        
        while count < timeout and self.running:
            try:
                # 检查蓝牙服务状态
                result = subprocess.run(['systemctl', 'is-active', 'bluetooth'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    self.logger.info("蓝牙服务已启动")
                    return True
            except:
                pass
                
            if count % 5 == 0:
                self.logger.info(f"等待蓝牙服务启动... ({count}/{timeout})")
                
            time.sleep(1)
            count += 1
            
        self.logger.info("蓝牙服务启动超时")
        return False
        
    def run(self):
        """主运行函数"""
        self.logger.info("启动蓝牙自动连接服务")
        
        # 加载配置
        self.load_config()
        
        # 等待蓝牙服务
        if not self.wait_for_bluetooth():
            self.logger.info("蓝牙服务未就绪，退出")
            return
            
        # 检查是否有可用的蓝牙控制器
        available_controllers = self.get_available_controllers()
        if not available_controllers:
            self.logger.warning("没有检测到任何蓝牙控制器，将等待控制器出现")
        else:
            self.logger.info(f"检测到 {len(available_controllers)} 个蓝牙控制器:")
            for controller in available_controllers:
                is_default = '[default]' in controller['name']
                status = " (默认)" if is_default else ""
                self.logger.info(f"  - {controller['name'].replace('[default]', '').strip()}{status}: {controller['mac']}")
            
        # 设置默认控制器（如果有控制器的话）
        if available_controllers:
            if not self.set_default_controller():
                self.logger.warning("设置默认控制器失败，但服务将继续运行")
        else:
            self.logger.info("暂无可用控制器，跳过控制器设置")
        
        # 确保初始状态下扫描是关闭的
        self.ensure_scan_off()
            
        # 检查目标设备
        if not self.target_devices:
            self.logger.info("配置文件中没有目标设备，等待配置...")
            while self.running and not self.target_devices:
                self.load_config()
                if self.target_devices:
                    self.logger.info("发现目标设备，开始监控")
                    break
                self.logger.info("配置文件中仍无目标设备，继续等待...")
                time.sleep(30)
                
        if not self.running:
            return
            
        try:
            # 开始监控
            self.monitor_devices()
        finally:
            # 服务停止时执行清理
            self.cleanup()
        
        self.logger.info("服务已停止")

if __name__ == "__main__":
    service = BluetoothAutoConnect()
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        if command == "start":
            service.run()
        elif command == "stop":
            service.logger.info("停止蓝牙自动连接服务")
        elif command == "restart":
            service.logger.info("重启蓝牙自动连接服务")
            service.run()
        elif command == "status":
            try:
                result = subprocess.run(['systemctl', 'is-active', 'bluetooth'], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    print("蓝牙服务运行中")
                else:
                    print("蓝牙服务未运行")
            except:
                print("无法检查蓝牙服务状态")
        else:
            print(f"用法: {sys.argv[0]} {{start|stop|restart|status}}")
            sys.exit(1)
    else:
        service.run()