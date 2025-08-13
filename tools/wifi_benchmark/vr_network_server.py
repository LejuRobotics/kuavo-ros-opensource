#!/usr/bin/env python3
"""
VR网络测试工具 - 服务端
基于iperf3的VR网络性能测试服务端脚本
"""

import subprocess
import argparse
import sys
import time

class VRNetworkServer:
    def __init__(self, port=9019, bind_ip="0.0.0.0"):
        self.port = port
        self.bind_ip = bind_ip
        self.process = None
        
    def start_server(self):
        """启动iperf3服务端"""
        try:
            print(f"🚀 启动VR网络测试服务端...")
            print(f"📡 监听地址: {self.bind_ip}:{self.port}")
            print(f"⚡ 等待客户端连接...")
            print(f"🛑 按 Ctrl+C 停止服务")
            
            # 启动iperf3服务端，支持JSON输出
            cmd = [
                "iperf3", 
                "-s",                    # 服务端模式
                "-p", str(self.port),    # 端口
                "-B", self.bind_ip,      # 绑定IP
                "-J",                    # JSON输出
                "-1",                    # 单次连接后退出
                "-V"                     # 详细输出
            ]
            
            while True:
                print(f"\n⏳ [{time.strftime('%H:%M:%S')}] 等待新的测试连接...")
                self.process = subprocess.Popen(
                    cmd, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE,
                    text=True
                )
                
                stdout, stderr = self.process.communicate()
                
                if self.process.returncode == 0:
                    print(f"✅ [{time.strftime('%H:%M:%S')}] 测试完成，数据已发送至客户端")
                elif stderr:
                    print(f"❌ 测试错误: {stderr}")
                    
        except KeyboardInterrupt:
            print(f"\n\n🛑 服务端停止")
            sys.exit(0)
        except Exception as e:
            print(f"❌ 启动失败: {e}")
            sys.exit(1)
            


def main():
    parser = argparse.ArgumentParser(description="VR网络测试服务端")
    parser.add_argument("-p", "--port", type=int, default=9019, 
                       help="监听端口 (默认: 9019)")
    parser.add_argument("-B", "--bind", default="0.0.0.0", 
                       help="绑定IP地址 (默认: 0.0.0.0)")
    
    args = parser.parse_args()
    
    # 检查iperf3是否安装
    try:
        subprocess.run(["iperf3", "--version"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ 错误: 请先安装 iperf3")
        print("   Ubuntu/Debian: sudo apt install iperf3")
        print("   CentOS/RHEL: sudo yum install iperf3")
        sys.exit(1)
    
    server = VRNetworkServer(args.port, args.bind)
    server.start_server()

if __name__ == "__main__":
    main() 