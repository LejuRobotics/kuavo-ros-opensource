# 五代机器人网络拓扑适配（26/28 双网口 + DHCP + 跨网段互通 + Wifi/5G 自动切换）

本功能用于研扬工控机环境下，进行网络拓扑适配与优化：在 **26/28 两个网段提供 DHCP**，实现 **26↔28 跨网段互通**，并提供 **Wifi/5G 双出口自动切换**，保证内网与外设接入稳定可靠。

## 一、26/28 双网口 + DHCP + 跨网段互通

### 功能说明

- **双网口 DHCP**：下位机 `enp3s0` 为 26 网段、`eno1` 为 28 网段，两个网口均监听并分配地址池（26 地址池避开 `26.1/26.12/26.22`）
- **跨网段互通**：下位机开启转发与规则，实现 26/28 互通，不影响原有通信与 ROS 功能
- **26 网段出网**：支持 26 网段通过 Wifi/5G 出网（NAT）
- **CPU 亲和性**：DHCP 服务绑定到 CPU 15

### IP 规划

- **26 网段**：
  - 下位机网关：`192.168.26.1/24`（`enp3s0`）
  - DHCP 地址池：`192.168.26.100` - `192.168.26.200`
  - 上位机：`192.168.26.12`（静态）
  - 底盘：`192.168.26.22`（静态）（可选，若为轮臂时存在）
- **28 网段**：
  - 下位机网关：`192.168.28.1/24`（`eno1`）
  - DHCP 地址池：`192.168.28.100` - `192.168.28.200`

### 部署

#### 下位机执行脚本

```bash
sudo bash setup_network_26_28.sh
```

**脚本功能**：配置双网口静态 IP、DHCP 服务、IP 转发、iptables 规则（26↔28 互通 + 26 出网）、wait-for-eno1 服务、DHCP CPU 亲和性

#### 上位机配置路由转发

**临时生效**：

```bash
sudo ip route add 192.168.28.0/24 via 192.168.26.1
```

**永久化配置**：

```bash
# 查看连接
nmcli connection show

# 选择承载 26.12 的网卡连接（示例以 eth0 为设备名，实际可能是 eth0/eth1/eno*/enp*）
CONN_NAME=$(nmcli -t -f NAME,DEVICE connection show | grep -E ":eth0$" | cut -d: -f1 | head -1)

# 添加静态路由
sudo nmcli connection modify "$CONN_NAME" ipv4.routes "192.168.28.0/24 192.168.26.1"
sudo nmcli connection down "$CONN_NAME" && sudo nmcli connection up "$CONN_NAME"
```

### 验证

```bash
# 检查下位机双网口 IP
ip addr show enp3s0 | grep 192.168.26.1
ip addr show eno1   | grep 192.168.28.1

# 检查 DHCP 服务状态
journalctl -u isc-dhcp-server -n 30 --no-pager
```

### 常见问题

- **DHCP 报 "No subnet declaration for eno1 (no IPv4 addresses)"**
  - 检查 `wait-for-eno1.service` 是否启用：`systemctl status wait-for-eno1.service`
  - 查看日志：`journalctl -u wait-for-eno1 -n 50 --no-pager`

- **外设能拿到 IP 但跨网段不通**
  - 检查转发：`sysctl net.ipv4.ip_forward`（应为 1）
  - 检查 iptables：`iptables -S FORWARD` / `iptables -t nat -S`

- **接口名不一致导致脚本无效**
  - 先确认：`ip link` 输出，按实际接口名调整脚本

---

## 二、Wifi/5G 自动切换

> **注意**：若没有 5G 模块，可跳过此脚本（仅 Wifi 场景无需自动切换功能）

### 功能说明

- **自动切换**：基于路由 metric + 链路质量检测（延迟）实现优先级与故障切换
- **防抖机制**：避免频繁切换，需连续 4 次相同决策才切换
- **单网口模式**：若只有 Wifi 或只有 5G，固定走可用链路（不做 ping）
- **CPU 亲和性**：网络质量检测服务绑定到 CPU 15
- **日志轮转**：自动日志轮转，保留 3 天

### 部署

#### 下位机执行脚本

```bash
sudo bash setup_wifi_5g_switch.sh
```

**脚本功能**：部署 Wifi/5G 网络质量检测与自动切换（systemd timer，每 5 秒检测一次，带防抖、CPU 亲和性、日志轮转）

### 验证

```bash
# 检查 Wifi/5G 切换服务
systemctl status network-quality-check.timer
tail -f /var/log/network-quality-check.log

# 检查默认路由
ip route | grep default
```

### 常见问题

- **服务未启动**
  - 检查定时器：`systemctl status network-quality-check.timer`
  - 检查服务：`systemctl status network-quality-check.service`

- **切换不生效**
  - 查看日志：`tail -f /var/log/network-quality-check.log`
  - 检查接口是否存在：`ip link show wlp* wwan*`
