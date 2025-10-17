# 网络性能测试以及流量监测

# 1. 快速开始

### 安装依赖

```bash
# Ubuntu/Debian
sudo apt install iperf3 sockperf python3 python3-psutil

# CentOS/RHEL  
sudo yum install iperf3 sockperf python3 python3-psutil
```

### 基本使用

1. **启动服务端**（在VR设备或目标服务器上）
   
   ```bash
   python3 vr_network_server.py
   ```
   
   - 服务端只负责启动监听，不显示性能数据
   - 测试完成后显示简单的完成提示

2. **运行客户端测试**（在测试机器上）
   
   ```bash
   python3 vr_network_client.py <VR设备IP>
   ```
   
   - 客户端负责数据分析和结果展示
   - 显示完整的性能指标和VR适用性评估

# 2.  网络性能测试

## 📊 测试指标

**核心四大指标**（仅客户端显示）：

- **⚡ 延迟**: 通过ping测试获取RTT延迟数据
- **📈 抖动**: 网络延迟波动（iperf3 jitter）  
- **📦 丢包率**: 数据包丢失百分比
- **🚀 吞吐量**: 实际数据传输速率
- **🎮 VR适用性**: 综合评估是否适合VR应用

## 🛠️ 高级配置

### 自定义数据包大小

```bash
# 小包测试 (模拟手势数据)
python3 vr_network_client.py 192.168.1.100 -l 256 -b 1M

# 中包测试 (模拟复合数据)  
python3 vr_network_client.py 192.168.1.100 -l 2048 -b 10M

# 大包测试 (模拟视频流)
python3 vr_network_client.py 192.168.1.100 -l 8192 -b 50M
```

### 调整流量大小

```bash
# 低流量测试
python3 vr_network_client.py 192.168.1.100 -b 1M

# 高流量测试  
python3 vr_network_client.py 192.168.1.100 -b 100M
```

### 长时间稳定性测试

```bash
python3 vr_network_client.py 192.168.1.100 -t 300  # 5分钟
python3 vr_network_client.py 192.168.1.100 -t 1800 # 30分钟
```

## 📋 参数说明

### 服务端 (vr_network_server.py)

- `-p, --port`: 监听端口 (默认: 9019)
- `-B, --bind`: 绑定IP地址 (默认: 0.0.0.0)

### 客户端 (vr_network_client.py)

- `server_ip`: 服务器IP地址 (必需)
- `-p, --port`: 服务器端口 (默认: 9019)
- `-l, --length`: 数据包大小 (bytes, 默认: 512)
- `-b, --bandwidth`: 目标带宽 (如: 1M, 10M, 100M, 默认: 5M)
- `-t, --time`: 测试时长 (秒, 默认: 30)
- `--tcp`: 使用TCP协议 (默认UDP)

## 🎯 VR测试场景推荐

### Quest3 手势数据测试

```bash
python3 vr_network_client.py <quest3_ip> -l 512 -b 2M -t 60
```

### 高频传感器数据测试

```bash
python3 vr_network_client.py <quest3_ip> -l 256 -b 5M -t 120
```

### 视频流质量测试

```bash
python3 vr_network_client.py <quest3_ip> -l 16384 -b 50M -t 300
```

## 🛠️ 设计原理

### 分工明确

- **服务端**: 纯粹的数据接收器，只负责启动iperf3监听，不做数据分析
- **客户端**: 数据分析器，集成ping延迟测试和iperf3吞吐量测试，提供完整分析

### 数据流程

1. 客户端先用ping测试延迟
2. 客户端连接服务端进行iperf3测试
3. 客户端获取并解析所有测试数据
4. 客户端显示完整的VR网络评估报告

### 抖动说明

- **网络抖动**: iperf3测量的UDP数据包传输延迟变化，反映网络稳定性
- 只显示一个抖动指标，避免混淆

# 3. 本地端口流量监测工具

## 3.1 基于tcpdump的流量监控（推荐）

由于原始的 `vr_traffic_monitor.py` 监控的是本地端口，而实际上是本地作为客户端连接到VR端的端口，因此提供了基于tcpdump的新监控脚本 `vr_tcpdump_monitor.py`。

### 使用方法

**实时监控所有VR端口流量：**
```bash
sudo python3 vr_tcpdump_monitor.py
```

**限时监控（例如60秒）：**
```bash
sudo python3 vr_tcpdump_monitor.py -d 60
```

**指定网卡设备（默认wlo1）：**
```bash
sudo python3 vr_tcpdump_monitor.py -i eth0
```

**自定义统计显示间隔：**
```bash
sudo python3 vr_tcpdump_monitor.py -s 5  # 每5秒显示一次统计
```

### 功能特点

- ✅ 正确监控本地连接到VR端的端口流量
- ✅ 基于tcpdump实现，精确捕获网络数据包
- ✅ 实时显示每个端口的发送/接收数据量和速率
- ✅ 显示活跃连接数和端口使用情况
- ✅ 提供详细的流量统计和汇总报告

### 注意事项

- 需要root权限运行（使用sudo）
- 确保tcpdump已安装：`sudo apt install tcpdump`
- 默认监控wlo1网卡，可通过-i参数指定其他网卡

## 3.2 原始监控工具（仅供参考）

### 基本用法

**1. 监控VR端口流量**

```bash
python3 vr_traffic_monitor.py monitor -d 60 -i 2
```

**2. 测试VR端口延迟**

```bash
python3 vr_traffic_monitor.py latency <VR设备IP> -d 10
```

**⚠️ 注意：** 此工具监控的是本地端口，当本地作为客户端连接VR设备时可能无法正确显示流量。建议使用上述基于tcpdump的监控工具。

### 监控的VR端口 (基于实际代码分析)

脚本会自动监控以下VR相关端口 (共28个)：

**核心数据端口:**
| 端口 | 用途 | 来源文件 |
|------|------|----------|
| 10019 | Quest3主要数据端口/左手手势数据 | monitor_quest3.py, noitom_hand_publish.py |
| 10029 | 右手手势数据服务端口 | noitom_hand_publish.py |
| 10050 | Noitom本地左手端口 | noitom_hand_publish.py |
| 10060 | Noitom本地右手端口 | noitom_hand_publish.py |

**Quest3广播发现端口:**
| 端口 | 用途 | 来源文件 |
|------|------|----------|
| 11000-11010 | Quest3广播发现端口 (11个) | monitor_quest3.py |

**WebRTC视频流端口:**
| 端口 | 用途 | 来源文件 |
|------|------|----------|
| 8765 | WebRTC信令服务器 | webrtc_singaling_server.py |
| 10030-10040 | 视频流广播端口 (11个) | webrtc_videostream.py |

**其他服务端口:**
| 端口 | 用途 | 来源文件 |
|------|------|----------|
| 8031 | Flask HTTP服务端口 | flask_server.py |
| 8080 | UDP测试服务端 | test_upd_server.py |
| 8443 | VR广播端口 | planarmwebsocketservice, pico-body-tracking |
| 12345 | Body Tracking UDP Server | body_tracking_udp_server.py |

> 📝 **端口验证**: 所有端口均基于 `src/manipulation_nodes/noitom_hi5_hand_udp_python` 项目的实际代码分析确定，详见 `VR端口分析报告.md`

# 最后： 注意：

### **一、带宽限制 `-b` 的实现原理**

1. **目标速率控制**  
   `-b` 参数设定的是 **理论发送带宽上限**（如 `-b 100M` 表示目标速率为 100 Mbps）。iperf3 客户端会尝试以该速率发送数据，但实际速率可能受网络拥塞、设备性能等因素影响而低于设定值。

2. **发包频率计算**  
   实际发包频率由以下公式决定：  
   \[
   \text{发包频率 (pps)} = \frac{\text{带宽 (bps)}}{\text{有效数据包大小 (bit)}}
   \]  
   
   - **有效数据包大小** = 应用层数据包大小（`-l` 参数） + 协议头部开销  
     - TCP 开销：IP 头（20B） + TCP 头（20B） = **40B**  
     - UDP 开销：IP 头（20B） + UDP 头（8B） = **28B**  
   - **示例**：  
     - 若 `-l 1000 -b 100M`（UDP 协议）：  
       有效包大小 = 1000B + 28B = 1028B = 8224 bit  
       发包频率 = \(100 \times 10^6 \, \text{bps} / 8224 \, \text{bit} \approx 12,153 \, \text{包/秒}\)

3. **工具行为差异**  
   
   - **TCP 模式**：  
     通过滑动窗口和拥塞控制算法动态调整速率，`-b` 仅作为**软性上限**（实际速率可能略低）。  
   - **UDP 模式**：  
     严格按计算出的发包频率发送数据包，`-b` 为**硬性限制**（客户端强制按此速率发包）。

---
