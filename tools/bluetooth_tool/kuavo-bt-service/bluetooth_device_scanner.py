#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import dbus
import subprocess
import sys
import json
from datetime import datetime

class ConnectedDevicesScanner:
    def __init__(self):
        self.bus = dbus.SystemBus()
        
    def get_connected_devices(self):
        """获取已连接的蓝牙设备列表"""
        connected_devices = []
        
        try:
            # 使用DBus获取所有设备
            obj_manager = self.bus.get_object('org.bluez', '/')
            manager = dbus.Interface(obj_manager, 'org.freedesktop.DBus.ObjectManager')
            
            objects = manager.GetManagedObjects()
            
            # 遍历所有对象，查找已连接的设备
            for path, interfaces in objects.items():
                if 'org.bluez.Device1' in interfaces:
                    device_props = interfaces['org.bluez.Device1']
                    
                    # 检查设备是否已连接
                    if 'Connected' in device_props and device_props['Connected']:
                        device_info = {
                            'mac_address': device_props.get('Address', ''),
                            'name': device_props.get('Name', 'Unknown'),
                            'path': path,
                            'connected': True,
                            'rssi': device_props.get('RSSI', 0),
                            'paired': device_props.get('Paired', False)
                        }
                        connected_devices.append(device_info)
                        
        except Exception as e:
            print(f"DBus查询失败: {e}")
            # 备用方法：使用bluetoothctl
            return self.get_connected_devices_fallback()
            
        return connected_devices
        
    def get_connected_devices_fallback(self):
        """备用方法：使用bluetoothctl获取已连接设备"""
        connected_devices = []
        
        try:
            # 获取已配对设备列表
            result = subprocess.run(['bluetoothctl', 'devices', 'Paired'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    line = line.strip()
                    if line.startswith('Device '):
                        parts = line.split()
                        if len(parts) >= 3:
                            mac = parts[1]
                            name = ' '.join(parts[2:])
                            
                            # 检查设备是否已连接（使用bluetoothctl）
                            connected = False
                            try:
                                result = subprocess.run(['bluetoothctl', 'info', mac], 
                                                      capture_output=True, text=True, timeout=5)
                                if result.returncode == 0:
                                    for line in result.stdout.split('\n'):
                                        if 'Connected: yes' in line:
                                            connected = True
                                            break
                            except:
                                pass
                            
                            if connected:
                                device_info = {
                                    'mac_address': mac,
                                    'name': name,
                                    'path': f"/org/bluez/hci0/dev_{mac.replace(':', '_').upper()}",
                                    'connected': True,
                                    'rssi': 0,
                                    'paired': True
                                }
                                connected_devices.append(device_info)
                                
        except Exception as e:
            print(f"bluetoothctl查询失败: {e}")
            
        return connected_devices
        
                
    def print_devices(self, devices):
        """打印设备列表"""
        if not devices:
            print("没有已连接的蓝牙设备")
            return
            
        print(f"已连接的蓝牙设备 ({len(devices)} 个):")
        print("-" * 60)
        
        for i, device in enumerate(devices, 1):
            print(f"{i}. 设备名称: {device['name']}")
            print(f"   MAC地址: {device['mac_address']}")
            print(f"   信号强度: {device['rssi']} dBm")
            print(f"   已配对: {'是' if device['paired'] else '否'}")
            print(f"   设备路径: {device['path']}")
            print()
            
            
    def print_simple(self, devices):
        """简单格式输出"""
        if not devices:
            print("0")
            return
            
        for device in devices:
            print(f"{device['mac_address']} {device['name']}")
            
    def print_json(self, devices):
        """以JSON格式输出设备列表"""
        output = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'total_devices': len(devices),
            'devices': devices
        }
        print(json.dumps(output, indent=2, ensure_ascii=False))
            
    def main(self):
        # 解析命令行参数
        output_format = 'table'  # 默认表格格式
        
        if len(sys.argv) > 1:
            if sys.argv[1] in ['--json', '-j']:
                output_format = 'json'
            elif sys.argv[1] in ['--simple', '-s']:
                output_format = 'simple'
            elif sys.argv[1] in ['--help', '-h']:
                print("用法: bluetooth_device_scanner [选项]")
                print("选项:")
                print("  --json, -j    以JSON格式输出")
                print("  --simple, -s  简单格式输出")
                print("  --help, -h    显示帮助信息")
                return
                
        # 获取已连接设备
        devices = self.get_connected_devices()
        
        # 根据格式输出
        if output_format == 'json':
            self.print_json(devices)
        elif output_format == 'simple':
            self.print_simple(devices)
        else:
            self.print_devices(devices)

if __name__ == "__main__":
    scanner = ConnectedDevicesScanner()
    scanner.main()