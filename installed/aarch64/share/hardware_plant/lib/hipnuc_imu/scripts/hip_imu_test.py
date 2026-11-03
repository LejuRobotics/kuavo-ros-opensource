#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

class HipnucIMUTester:
    def __init__(self):
        self.running = True
        self.test_results = {
            'device_found': False,
            'data_reading': False
        }
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        print("\n\n正在退出测试...")
        self.running = False
        sys.exit(0)
    
    def print_header(self):
        print("=" * 50)
        print("           hipnuc IMU 连接检查程序")
        print("=" * 50)
        print("按 Ctrl+C 退出测试")
        print()
    
    def check_device_connection(self):
        # 检查串口设备
        serial_devices = []
        for i in range(10):
            device = f"/dev/ttyUSB{i}"
            if os.path.exists(device):
                serial_devices.append(device)
        
        if not serial_devices:
            print("   ❌ 未找到串口设备")
            return False
        
        print(f"   ✅ 找到串口设备: {', '.join(serial_devices)}")
        
        # 检查hipnuc设备
        hipnuc_devices = []
        for device in serial_devices:
            try:
                result = subprocess.run(['udevadm', 'info', '--name=' + device], 
                                      capture_output=True, text=True, timeout=5)
                # 严格匹配hipnuc设备标识
                if 'hipnuc' in result.stdout.lower():
                    hipnuc_devices.append(device)
            except Exception as e:
                print(f"   ⚠️  检查设备 {device} 时出错: {e}")
                continue
        
        if hipnuc_devices:
            print(f"   ✅ 找到hipnuc设备: {', '.join(hipnuc_devices)}")
            self.test_results['device_found'] = True
            return True
        else:
            # 未找到hipnuc设备，直接报错并终止程序
            print("   ❌ 未找到hipnuc设备，测试终止")
            print("   可能原因:")
            print("   - 设备未正确连接")
            print("   - 设备驱动未安装")
            print("   - 非hipnuc品牌IMU设备")
            self.test_results['device_found'] = False
            # 终止程序
            self.running = False
            sys.exit(1)
    
    def run_all_tests(self):
        """运行所有测试"""
        self.print_header()
        tests = [
            self.check_device_connection,
        ]
        
        for test in tests:
            if not self.running:
                break
            test()
            print()
            time.sleep(1)

def main():
    tester = HipnucIMUTester()
    tester.run_all_tests()

if __name__ == "__main__":
    main()