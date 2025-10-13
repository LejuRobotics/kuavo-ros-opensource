#!/usr/bin/env python3
"""
VR网络测试工具 - 客户端
基于iperf3的VR网络性能测试客户端脚本
支持自定义数据包大小和流量控制
"""

import subprocess
import argparse
import json
import sys
import time

class VRNetworkClient:
    def __init__(self, server_ip, port=9019):
        self.server_ip = server_ip
        self.port = port
        
    def run_test(self, packet_size=512, bandwidth="5M", duration=30, protocol="udp"):
        """运行网络测试"""
        try:
            print(f"🎯 VR网络性能测试")
            print(f"{'='*50}")
            print(f"🔗 目标服务器: {self.server_ip}:{self.port}")
            print(f"📦 数据包大小: {packet_size} bytes")
            print(f"🚀 目标带宽: {bandwidth}")
            print(f"⏱️  测试时长: {duration} 秒")
            print(f"🔧 协议类型: {protocol.upper()}")
            print(f"{'='*50}")
            
            # 1. 先测试延迟 (RTT)
            print(f"🔍 测试延迟...")
            self.test_latency()
            
            # 2. 再进行吞吐量测试
            print(f"🚀 测试吞吐量...")
            
            # 构建iperf3命令
            cmd = [
                "iperf3",
                "-c", self.server_ip,      # 客户端模式，连接到服务器
                "-p", str(self.port),      # 端口
                "-l", str(packet_size),    # 数据包大小
                "-b", bandwidth,           # 带宽限制
                "-t", str(duration),       # 测试时长
                "-J"                       # JSON输出
            ]
            
            # 添加协议选项
            if protocol.lower() == "udp":
                cmd.append("-u")           # UDP协议
                cmd.append("--get-server-output")  # 获取服务端输出，修复UDP统计问题
            
            # 执行测试
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.parse_results(result.stdout)
            else:
                print(f"❌ 测试失败: {result.stderr}")
                
        except Exception as e:
            print(f"❌ 测试错误: {e}")
    
    def test_latency(self):
        """测试网络延迟"""
        try:
            # 使用ping测试延迟
            ping_cmd = ["ping", "-c", "10", "-i", "0.2", self.server_ip]
            result = subprocess.run(ping_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                # 解析ping结果
                lines = result.stdout.split('\n')
                for line in lines:
                    if "rtt min/avg/max/mdev" in line or "round-trip" in line:
                        # 提取延迟数据
                        parts = line.split('=')
                        if len(parts) > 1:
                            times = parts[1].strip().split('/')
                            if len(times) >= 4:
                                min_rtt = float(times[0])
                                avg_rtt = float(times[1])
                                max_rtt = float(times[2])
                                mdev_rtt = float(times[3].split()[0])
                                
                                print(f"📊 网络延迟 (RTT):")
                                print(f"   平均延迟: {avg_rtt:.2f} ms")
                                print(f"   最小延迟: {min_rtt:.2f} ms")
                                print(f"   最大延迟: {max_rtt:.2f} ms")
                                print(f"   延迟抖动: {mdev_rtt:.2f} ms")
                                
                                # 延迟质量评估
                                if avg_rtt < 20:
                                    status = "🟢 优秀"
                                elif avg_rtt < 50:
                                    status = "🟡 良好"
                                elif avg_rtt < 100:
                                    status = "🟠 一般"
                                else:
                                    status = "🔴 较差"
                                print(f"   延迟质量: {status}")
                                
                                # 保存延迟数据供后续使用
                                self._last_latency = avg_rtt
                                self._last_jitter = mdev_rtt
                                return avg_rtt, mdev_rtt
                        break
            else:
                print(f"⚠️ 延迟测试失败: {result.stderr}")
                
        except Exception as e:
            print(f"⚠️ 延迟测试错误: {e}")
        
        return None, None
    
    def parse_results(self, json_output):
        """解析并显示测试结果"""
        try:
            data = json.loads(json_output)
            
            print(f"\n📊 VR网络测试结果:")
            print(f"{'='*60}")
            
            # 基本信息
            if "start" in data:
                start_info = data["start"]
                if "test_start" in start_info:
                    test_info = start_info["test_start"]
                    print(f"🔧 协议: {test_info.get('protocol', 'N/A').upper()}")
                    print(f"📦 数据包大小: {test_info.get('blksize', 'N/A')} bytes")
                    print(f"⏱️  测试时长: {test_info.get('duration', 'N/A')} 秒")
            
            # 性能指标 - 修复UDP数据解析
            print(f"\n📈 性能指标:")
            print(f"{'-'*40}")
            
            sent_data = {}
            recv_data = {}
            
            # 尝试多种路径来获取数据
            if "end" in data:
                end_data = data["end"]
                
                # 获取发送数据
                if "sum_sent" in end_data:
                    sent_data = end_data["sum_sent"]
                elif "sum" in end_data:
                    sent_data = end_data["sum"]
                
                # 获取接收数据
                if "sum_received" in end_data:
                    recv_data = end_data["sum_received"]
                elif "server_output_json" in end_data and "end" in end_data["server_output_json"]:
                    # UDP模式下从服务端输出获取
                    server_end = end_data["server_output_json"]["end"]
                    if "sum" in server_end:
                        recv_data = server_end["sum"]
                elif "sum" in end_data:
                    recv_data = end_data["sum"]
            
            # 显示延迟数据 (如果有ping测试结果)
            if hasattr(self, '_last_latency') and self._last_latency:
                print(f"⚡ 平均延迟: {self._last_latency:.2f} ms")
            
            # 显示吞吐量
            recv_throughput = 0
            if "bits_per_second" in recv_data:
                recv_throughput = recv_data["bits_per_second"] / 1_000_000
                print(f"🚀 接收吞吐量: {recv_throughput:.2f} Mbps")
            
            if "bits_per_second" in sent_data:
                sent_throughput = sent_data["bits_per_second"] / 1_000_000
                print(f"📤 发送吞吐量: {sent_throughput:.2f} Mbps")
                
            # 显示UDP专用指标（抖动和丢包率）
            if "jitter_ms" in recv_data:
                jitter = recv_data["jitter_ms"]
                print(f"📈 网络抖动: {jitter:.3f} ms")
                
                # 抖动质量评估
                if jitter < 1:
                    status = "🟢 优秀"
                elif jitter < 5:
                    status = "🟡 良好"
                elif jitter < 20:
                    status = "🟠 一般"
                else:
                    status = "🔴 较差"
                print(f"   抖动质量: {status}")
            
            # 丢包率计算和显示
            loss_rate = 0
            if "lost_packets" in recv_data:
                lost = recv_data["lost_packets"]
                if "packets" in sent_data:
                    total = sent_data["packets"]
                elif "packets" in recv_data:
                    total = recv_data["packets"]
                else:
                    total = 0
                    
                if total > 0:
                    loss_rate = (lost / total * 100)
                    print(f"📦 丢包率: {loss_rate:.3f}% ({lost}/{total})")
                    
                    # 丢包质量评估
                    if loss_rate == 0:
                        status = "🟢 完美"
                    elif loss_rate < 0.1:
                        status = "🟡 优秀"
                    elif loss_rate < 0.5:
                        status = "🟠 可接受"
                    else:
                        status = "🔴 需优化"
                    print(f"   丢包质量: {status}")
            
            # 数据传输量
            if "bytes" in sent_data:
                total_mb = sent_data["bytes"] / 1_000_000
                print(f"📊 传输数据: {total_mb:.2f} MB")
                
                # VR应用综合评估
                print(f"\n🎮 VR应用适用性评估:")
                print(f"{'-'*40}")
                
                vr_suitable = True
                issues = []
                recommendations = []
                
                # 检查延迟 (如果有ping数据)
                avg_latency = getattr(self, '_last_latency', None)
                if avg_latency:
                    if avg_latency > 50:
                        vr_suitable = False
                        issues.append("延迟过高")
                        recommendations.append("优化网络路由")
                    elif avg_latency > 20:
                        recommendations.append("考虑使用有线连接")
                
                # 检查抖动
                if "jitter_ms" in recv_data:
                    jitter = recv_data["jitter_ms"]
                    if jitter > 20:
                        vr_suitable = False
                        issues.append("抖动过高")
                        recommendations.append("启用QoS优化")
                    elif jitter > 5:
                        recommendations.append("检查网络拥塞")
                
                # 检查丢包率
                if "lost_packets" in recv_data:
                    if hasattr(self, '_packets_sent'):
                        loss_rate = (recv_data["lost_packets"] / self._packets_sent * 100) if self._packets_sent > 0 else 0
                    else:
                        loss_rate = (recv_data.get("lost_packets", 0) / recv_data.get("packets", 1) * 100)
                        
                    if loss_rate > 1.0:
                        vr_suitable = False
                        issues.append("丢包严重")
                        recommendations.append("检查硬件连接")
                    elif loss_rate > 0.5:
                        vr_suitable = False
                        issues.append("丢包率过高")
                        recommendations.append("减少网络负载")
                
                # 检查吞吐量
                actual_throughput = 0
                if "bits_per_second" in recv_data:
                    actual_throughput = recv_data["bits_per_second"]
                elif "bits_per_second" in sent_data:
                    actual_throughput = sent_data["bits_per_second"]
                
                if actual_throughput < 1_000_000:  # 1Mbps
                    vr_suitable = False
                    issues.append("带宽不足")
                    recommendations.append("升级网络带宽")
                elif actual_throughput < 5_000_000:  # 5Mbps
                    recommendations.append("考虑增加带宽以提升体验")
                
                # 显示评估结果
                if vr_suitable:
                    print(f"✅ 网络质量: 适合VR应用")
                    if recommendations:
                        print(f"💡 优化建议: {', '.join(recommendations)}")
                else:
                    print(f"❌ 网络质量: 不适合VR应用")
                    print(f"   问题: {', '.join(issues)}")
                    if recommendations:
                        print(f"🔧 修复建议: {', '.join(recommendations)}")
            
            print(f"{'='*60}")
            
        except json.JSONDecodeError:
            print(f"❌ 无法解析测试结果")
        except Exception as e:
            print(f"❌ 结果解析错误: {e}")

def main():
    parser = argparse.ArgumentParser(
        description="VR网络测试客户端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  基础测试:     python3 vr_network_client.py 192.168.1.100
  小包测试:     python3 vr_network_client.py 192.168.1.100 -l 256 -b 1M
  大包测试:     python3 vr_network_client.py 192.168.1.100 -l 8192 -b 50M
  长时间测试:   python3 vr_network_client.py 192.168.1.100 -t 300
  TCP测试:      python3 vr_network_client.py 192.168.1.100 --tcp
        """
    )
    
    parser.add_argument("server_ip", help="服务器IP地址")
    parser.add_argument("-p", "--port", type=int, default=9019, 
                       help="服务器端口 (默认: 9019)")
    parser.add_argument("-l", "--length", type=int, default=512, 
                       help="数据包大小 (bytes, 默认: 512)")
    parser.add_argument("-b", "--bandwidth", default="5M", 
                       help="目标带宽 (如: 1M, 10M, 100M, 默认: 5M)")
    parser.add_argument("-t", "--time", type=int, default=30, 
                       help="测试时长 (秒, 默认: 30)")
    parser.add_argument("--tcp", action="store_true", 
                       help="使用TCP协议 (默认UDP)")
    
    args = parser.parse_args()
    
    # 检查iperf3是否安装
    try:
        subprocess.run(["iperf3", "--version"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ 错误: 请先安装 iperf3")
        print("   Ubuntu/Debian: sudo apt install iperf3")
        print("   CentOS/RHEL: sudo yum install iperf3")
        sys.exit(1)
    
    protocol = "tcp" if args.tcp else "udp"
    
    client = VRNetworkClient(args.server_ip, args.port)
    client.run_test(
        packet_size=args.length,
        bandwidth=args.bandwidth,
        duration=args.time,
        protocol=protocol
    )

if __name__ == "__main__":
    main() 