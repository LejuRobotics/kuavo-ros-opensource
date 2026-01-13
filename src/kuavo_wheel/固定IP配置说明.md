# Kuavo 固定IP配置说明

本文档说明如何为 Kuavo 系统配置固定IP地址，确保下位机(kuavo_master)和上位机(kuavo_slave)之间的稳定通信。

## 概述

在 Kuavo 轮式底盘系统中：
- **上位机 (kuavo_slave)**: 执行高级控制逻辑，连接到下位机的 ROS Master
- **下位机 (kuavo_master)**: 运行 ROS Master，控制机器人本体
- **轮式底盘**: 独立的移动平台，有自己的控制系统
- **网络交换机**: 连接所有设备的中心节点

## IP地址规划

### 默认配置
- **上位机 (kuavo_slave) IP**: `169.254.128.136`
- **下位机 (kuavo_master) IP**: `169.254.128.130`
- **轮式底盘 (Wheel Master) IP**: `169.254.128.2`
- **子网掩码**: `255.255.255.0` (/24)

### 网络拓扑
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│     上位机      │    │   网络交换机    │    │   轮式底盘      │
│ (kuavo_slave)   │    │               │    │ (Wheel Master)  │
│                │    │               │    │                │
│ 169.254.128.136 │◄──►│   Switch      │◄──►│ 169.254.128.2  │
└─────────────────┘    │               │    └─────────────────┘
                       │               │
┌─────────────────┐    │               │
│     下位机      │    │               │
│ (kuavo_master)  │    │               │
│                │    │               │
│ 169.254.128.130 │◄──►│               │
└─────────────────┘    └─────────────────┘
                           以太网连接
```

## 使用脚本配置

### 1. 查看帮助信息
```bash
cd kuavo-ros-control_3/src/kuavo_wheel/scripts
chmod +x setup_fixed_ips.sh
./setup_fixed_ips.sh --help
```

### 2. 配置上位机 (kuavo_slave)
```bash
sudo ./setup_fixed_ips.sh --upper
```

### 3. 配置下位机 (kuavo_master)
```bash
sudo ./setup_fixed_ips.sh --lower
```

### 4. 自定义子网掩码
```bash
sudo ./setup_fixed_ips.sh --upper --netmask 255.255.0.0
sudo ./setup_fixed_ips.sh --lower --netmask 255.255.0.0
```

### 5. 脚本运行示例
运行脚本时，会显示类似以下的交互界面：

```
======================================
    Kuavo 固定IP设置脚本
======================================

[INFO] 配置参数:
  机器类型:        下位机 (kuavo_master)
  目标IP:          169.254.128.130
  子网掩码:        255.255.255.0

[INFO] 检测可用网络接口...

可用网络接口:
============================================
 1) docker0         [UP] - 当前IP: 172.17.0.1
 2) enp6s0          [UP] - 当前IP: 192.168.50.111
 3) wlx6c1ff7324aff [UP] - 当前IP: 10.10.20.67
============================================

请选择网络接口 (1-3): 2
[INFO] 已选择接口: enp6s0
```

## 脚本功能

### 自动检测
- **网络接口检测**: 自动显示可用的网络接口列表，支持以太网和Wi-Fi接口
- **兼容性检测**: 优先使用 `ifconfig` 命令，备选 `ip` 命令或 `/sys/class/net`
- **接口状态显示**: 显示接口状态(UP/DOWN)和当前IP地址
- **IP冲突检查**: 验证目标IP地址是否已被占用

### 配置持久化
- 创建 `/etc/netplan/99-kuavo-fixed-ip.yaml` 配置文件
- 确保重启后配置保持有效
- 使用 `netplan apply` 应用配置

### 用户交互
- **交互式选择**: 显示所有可用网络接口，让用户选择
- **权限检查**: 需要 sudo 权限才能修改系统配置
- **配置验证**: 配置完成后验证IP设置是否正确

## 手动配置方法

### 临时配置 (重启后失效)
```bash
# 查看网络接口
ip link show

# 配置IP地址
sudo ip addr flush dev eth0
sudo ip addr add 169.254.128.130/24 dev eth0
sudo ip link set eth0 up

# 验证配置
ip addr show eth0
```

### 永久配置 (Netplan)
创建文件 `/etc/netplan/99-kuavo-fixed-ip.yaml`:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # 替换为实际网络接口名
      dhcp4: false
      addresses:
        - 169.254.128.130/24
      optional: true
```

应用配置:
```bash
sudo netplan apply
```
