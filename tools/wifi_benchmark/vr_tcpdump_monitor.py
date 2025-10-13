#!/usr/bin/env python3
"""
VR端口流量监控工具（基于tcpdump）
监测本地连接到VR端特定端口的数据包，并统计时间间隔
"""

import subprocess
import sys
import time
import signal
import argparse
from datetime import datetime
from collections import defaultdict
import re
import os
import statistics

# VR相关端口定义（基于项目分析）
VR_PORTS = [
    # Quest3核心数据端口和Noitom手势数据端口
    10019,    # Quest3主要数据端口/左手手势数据服务端口
    
    # Quest3广播发现端口
    11000, 11001, 11002, 11003, 11004, 11005, 11006, 11007, 11008, 11009, 11010,
]

# 端口描述映射
PORT_DESCRIPTIONS = {
    10019: "Quest3主数据",
    11000: "Quest3广播发现",
}

# 为广播端口添加描述
for port in range(11000, 11011):
    PORT_DESCRIPTIONS[port] = f"Quest3广播发现-{port-11000}"
for port in range(10030, 10041):
    PORT_DESCRIPTIONS[port] = f"视频流广播-{port-10030}"

class VRTcpdumpMonitor:
    def __init__(self, interface='wlo1'):
        self.interface = interface
        self.port_stats = defaultdict(lambda: {
            'packet_count': 0,
            'total_bytes': 0,
            'timestamps': [],
            'intervals': [],
            'last_timestamp': None,
            'connections': set(),
            'direction_stats': {'sent': 0, 'recv': 0}
        })
        self.start_time = time.time()
        self.tcpdump_process = None
        self.monitoring = False
        
    def check_interface(self):
        """检查网络接口是否存在"""
        try:
            result = subprocess.run(['ip', 'link', 'show', self.interface], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print(f"❌ 网络接口 {self.interface} 不存在")
                print("可用接口:")
                subprocess.run(['ip', 'link', 'show'], text=True)
                return False
            return True
        except Exception as e:
            print(f"❌ 检查网络接口失败: {e}")
            return False
    
    def check_permissions(self):
        """检查是否有运行tcpdump的权限"""
        if os.geteuid() != 0:
            print("❌ 需要root权限来运行tcpdump")
            print("请使用: sudo python3 vr_tcpdump_monitor.py")
            return False
        return True
    
    def build_tcpdump_filter(self):
        """构建tcpdump过滤器"""
        # 构建端口过滤器，监听所有VR端口（作为源端口或目标端口）
        port_filters = []
        for port in VR_PORTS:
            port_filters.append(f"port {port}")
        
        return " or ".join(port_filters)
    
    def parse_tcpdump_line(self, line):
        """解析tcpdump输出行"""
        try:
            # 匹配时间戳 (支持多种格式)
            # 格式1: 2024-01-15 19:22:11.241746 (完整日期时间)
            # 格式2: 19:22:11.241746 (仅时间)
            time_match = re.search(r'(\d{4}-\d{2}-\d{2}\s+)?(\d{2}:\d{2}:\d{2}\.\d+)', line)
            if not time_match:
                return None
            
            timestamp_str = time_match.group(2)  # 获取时间部分
            
            # 转换为秒数
            h, m, s = timestamp_str.split(':')
            timestamp = int(h) * 3600 + int(m) * 60 + float(s)
            
            # 匹配IP地址和端口
            # 格式1: IP 192.168.1.100.12345 > 192.168.1.200.10019: UDP, length 100
            pattern1 = r'IP (\S+)\.(\d+) > (\S+)\.(\d+):[^,]*, length (\d+)'
            match = re.search(pattern1, line)
            
            if not match:
                # 格式2: 尝试其他可能的格式
                pattern2 = r'(\S+)\.(\d+) > (\S+)\.(\d+):'
                match = re.search(pattern2, line)
                if not match:
                    return None
                
                # 尝试提取length
                length_match = re.search(r'length (\d+)', line)
                length = int(length_match.group(1)) if length_match else 0
                
                return {
                    'timestamp': timestamp,
                    'timestamp_str': timestamp_str,
                    'src_ip': match.group(1),
                    'src_port': int(match.group(2)),
                    'dst_ip': match.group(3),
                    'dst_port': int(match.group(4)),
                    'length': length
                }
            else:
                return {
                    'timestamp': timestamp,
                    'timestamp_str': timestamp_str,
                    'src_ip': match.group(1),
                    'src_port': int(match.group(2)),
                    'dst_ip': match.group(3),
                    'dst_port': int(match.group(4)),
                    'length': int(match.group(5))
                }
                
        except Exception as e:
            # 调试信息
            if "length" in line and ("10019" in line or "10029" in line):
                print(f"⚠️ 解析失败的行: {line.strip()}")
                print(f"   错误: {e}")
            return None
    
    def update_stats(self, packet_info):
        """更新统计信息"""
        # 检查是否涉及我们监控的端口
        for port in VR_PORTS:
            if packet_info['src_port'] == port or packet_info['dst_port'] == port:
                stats = self.port_stats[port]
                
                # 更新基本统计
                stats['packet_count'] += 1
                stats['total_bytes'] += packet_info['length']
                
                # 记录时间戳
                current_timestamp = packet_info['timestamp']
                stats['timestamps'].append(current_timestamp)
                
                # 计算时间间隔
                if stats['last_timestamp'] is not None:
                    interval = current_timestamp - stats['last_timestamp']
                    # 只记录合理的间隔（小于10秒）
                    if interval > 0 and interval < 10:
                        stats['intervals'].append(interval * 1000)  # 转换为毫秒
                
                stats['last_timestamp'] = current_timestamp
                
                # 记录连接信息
                connection = f"{packet_info['src_ip']}:{packet_info['src_port']}<->{packet_info['dst_ip']}:{packet_info['dst_port']}"
                stats['connections'].add(connection)
                
                # 统计方向
                if packet_info['dst_port'] == port:
                    stats['direction_stats']['recv'] += 1
                else:
                    stats['direction_stats']['sent'] += 1
    
    def monitor_traffic(self):
        """监控流量的主函数"""
        # 构建tcpdump命令
        filter_expr = self.build_tcpdump_filter()
        cmd = [
            'tcpdump',
            '-i', self.interface,
            '-nn',              # 不解析主机名和端口名
            '-l',               # 行缓冲
            '-tttt',            # 打印完整时间戳
            filter_expr
        ]
        
        print(f"🚀 启动tcpdump监控...")
        print(f"📡 网络接口: {self.interface}")
        print(f"🎯 监控端口: {len(VR_PORTS)} 个")
        print(f"📋 监控端口列表: {', '.join(map(str, sorted(VR_PORTS)))}")
        print(f"🔧 tcpdump过滤器长度: {len(filter_expr)} 字符")
        print("="*80)
        
        try:
            self.tcpdump_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )
            
            self.monitoring = True
            
            # 启动线程监控stderr
            def monitor_stderr():
                for line in self.tcpdump_process.stderr:
                    if line.strip():
                        print(f"⚠️ tcpdump: {line.strip()}")
            
            import threading
            stderr_thread = threading.Thread(target=monitor_stderr)
            stderr_thread.daemon = True
            stderr_thread.start()
            
            # 监控stdout输出
            import select
            
            # 读取tcpdump输出
            while self.monitoring:
                # 检查stdout是否有数据
                ready, _, _ = select.select([self.tcpdump_process.stdout], [], [], 0.1)
                if ready:
                    line = self.tcpdump_process.stdout.readline()
                    if not line:
                        break
                    
                    line = line.strip()
                    if line:
                        packet_info = self.parse_tcpdump_line(line)
                        if packet_info:
                            self.update_stats(packet_info)
                            # 实时显示捕获的数据包
                            for port in VR_PORTS:
                                if packet_info['src_port'] == port or packet_info['dst_port'] == port:
                                    print(f"📦 [{packet_info['timestamp_str']}] 端口 {port}: "
                                          f"{packet_info['src_ip']}:{packet_info['src_port']} -> "
                                          f"{packet_info['dst_ip']}:{packet_info['dst_port']} "
                                          f"({packet_info['length']} bytes)")
                
                # 检查进程是否结束
                if self.tcpdump_process.poll() is not None:
                    break
                    
        except Exception as e:
            print(f"❌ tcpdump错误: {e}")
        finally:
            if self.tcpdump_process:
                self.tcpdump_process.terminate()
    
    def stop(self):
        """停止监控"""
        self.monitoring = False
        if self.tcpdump_process:
            self.tcpdump_process.terminate()
            self.tcpdump_process.wait()
    
    def run(self, duration=None):
        """运行监控"""
        import threading
        
        # 启动流量监控线程
        monitor_thread = threading.Thread(target=self.monitor_traffic)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        try:
            if duration:
                print(f"⏱️  监控将在 {duration} 秒后自动停止\n")
                time.sleep(duration)
            else:
                print("📌 按 Ctrl+C 停止监控\n")
                monitor_thread.join()
        except KeyboardInterrupt:
            print("\n\n🛑 正在停止监控...")
        finally:
            self.stop()
            self.print_final_summary()
    
    def print_final_summary(self):
        """打印最终总结"""
        runtime = time.time() - self.start_time
        
        print("\n" + "="*80)
        print(f"📊 VR流量监控总结 (运行时间: {int(runtime)}秒)")
        print("="*80)
        
        # 统计活跃端口
        active_ports = []
        for port, stats in self.port_stats.items():
            if stats['packet_count'] > 0:
                active_ports.append(port)
        
        print(f"\n🎮 VR端口活跃情况:")
        print(f"  监控端口总数: {len(VR_PORTS)}")
        print(f"  活跃端口数: {len(active_ports)}")
        print(f"  活跃率: {len(active_ports)/len(VR_PORTS)*100:.1f}%")
        
        if active_ports:
            print(f"\n📈 端口详细统计:")
            print("-" * 80)
            
            for port in sorted(active_ports):
                stats = self.port_stats[port]
                desc = PORT_DESCRIPTIONS.get(port, "未知服务")
                
                print(f"\n🔹 端口 {port} - {desc}")
                print(f"   数据包总数: {stats['packet_count']}")
                print(f"   数据总量: {stats['total_bytes']/1024:.2f} KB")
                print(f"   连接数: {len(stats['connections'])}")
                print(f"   方向: 发送 {stats['direction_stats']['sent']} 包, "
                      f"接收 {stats['direction_stats']['recv']} 包")
                
                # 时间间隔统计
                if stats['intervals']:
                    intervals = stats['intervals']
                    print(f"\n   ⏱️  数据包时间间隔统计 (毫秒):")
                    print(f"      最小间隔: {min(intervals):.2f} ms")
                    print(f"      最大间隔: {max(intervals):.2f} ms")
                    print(f"      平均间隔: {statistics.mean(intervals):.2f} ms")
                    print(f"      中位数: {statistics.median(intervals):.2f} ms")
                    
                    # 计算标准差（如果有足够数据）
                    if len(intervals) > 1:
                        print(f"      标准差: {statistics.stdev(intervals):.2f} ms")
                    
                    # 计算包速率
                    avg_interval_sec = statistics.mean(intervals) / 1000
                    if avg_interval_sec > 0:
                        packet_rate = 1 / avg_interval_sec
                        print(f"      平均包速率: {packet_rate:.1f} 包/秒")
                    
                    # VR性能评估
                    avg_interval = statistics.mean(intervals)
                    if avg_interval < 20:
                        rating = "🟢 优秀 - 高频率数据传输"
                    elif avg_interval < 50:
                        rating = "🟡 良好 - 适合大多数VR应用"
                    elif avg_interval < 100:
                        rating = "🟠 一般 - 可能影响实时性"
                    else:
                        rating = "🔴 较差 - 数据传输频率过低"
                    print(f"      VR实时性评估: {rating}")
        else:
            print("\n❌ 未检测到任何VR端口流量")
            print("   请检查:")
            print("   1. VR设备是否已连接并正在传输数据")
            print("   2. 网络接口是否正确（当前使用: " + self.interface + "）")
            print("   3. 端口配置是否正确")

def main():
    parser = argparse.ArgumentParser(
        description="VR端口流量监控工具（基于tcpdump）- 统计数据包时间间隔",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  实时监控:     sudo python3 vr_tcpdump_monitor.py
  限时监控:     sudo python3 vr_tcpdump_monitor.py -d 60
  指定网卡:     sudo python3 vr_tcpdump_monitor.py -i eth0
        """
    )
    
    parser.add_argument('-i', '--interface', default='wlo1',
                       help='网络接口 (默认: wlo1)')
    parser.add_argument('-d', '--duration', type=int, default=None,
                       help='监控时长（秒），不指定则持续监控')
    
    args = parser.parse_args()
    
    # 创建监控器
    monitor = VRTcpdumpMonitor(interface=args.interface)
    
    # 检查权限
    if not monitor.check_permissions():
        sys.exit(1)
    
    # 检查网络接口
    if not monitor.check_interface():
        sys.exit(1)
    
    # 设置信号处理
    def signal_handler(sig, frame):
        monitor.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 运行监控
    try:
        monitor.run(duration=args.duration)
    except Exception as e:
        print(f"❌ 运行错误: {e}")
        monitor.stop()

if __name__ == "__main__":
    main() 