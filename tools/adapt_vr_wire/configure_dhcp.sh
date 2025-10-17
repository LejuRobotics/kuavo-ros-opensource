#!/bin/bash

set -e

# 日志函数
log() {
    echo "[`date '+%F %T'`] $1"
}

# 检查 root 权限
if [[ $EUID -ne 0 ]]; then
    log "错误：此脚本必须以 root 权限运行！"
    log "请使用 sudo 执行: sudo $0"
    exit 1
fi


# 检查isc-dhcp-server是否安装
if ! dpkg -s isc-dhcp-server &> /dev/null; then
    log "未发现isc-dhcp-server，准备安装..."
    log "更新软件源..."
    apt update -y
    log "安装isc-dhcp-server..."
    apt install -y isc-dhcp-server
else
    log "isc-dhcp-server已安装"
fi

# 检查network-manager是否安装
if ! dpkg -s network-manager &> /dev/null; then
    log "未发现network-manager，准备安装..."
    log "更新软件源..."
    apt update -y
    log "安装network-manager..."
    apt install -y network-manager
else
    log "network-manager已安装"
fi

# 根据udev规则重命名后的网卡名
INTERFACE_ASIX="enxasix"  # 对应ASIX网卡名
INTERFACE_REALTEK="enx00e04c68345c"  # 对应Realtek网卡名

# 检查网卡是否存在
if ! ip link show "$INTERFACE_ASIX" &> /dev/null; then
    log "错误：ASIX网卡 $INTERFACE_ASIX 不存在，请检查udev规则是否生效"
    exit 1
fi

if ! ip link show "$INTERFACE_REALTEK" &> /dev/null; then
    log "错误：Realtek网卡 $INTERFACE_REALTEK 不存在，请检查udev规则是否生效"
    exit 1
fi

# 分配接口到子网（ASIX->28网段，Realtek->26网段）
INTERFACE_28="$INTERFACE_ASIX"
INTERFACE_26="$INTERFACE_REALTEK"

log "已确定网卡分配:"
log "Realtek 网卡 -> $INTERFACE_26 -> 192.168.26.0/24"
log "ASIX 网卡 -> $INTERFACE_28 -> 192.168.28.0/24"

# 固定配置参数
SUBNET_26="192.168.26.0"
NETMASK_26="255.255.255.0"
ROUTER_26="192.168.26.1"
POOL_START_26="192.168.26.12"
POOL_END_26="192.168.26.12"

SUBNET_28="192.168.28.0"
NETMASK_28="255.255.255.0"
ROUTER_28="192.168.28.1"
POOL_START_28="192.168.28.6"
POOL_END_28="192.168.28.6"

LEASE_TIME="600"

# 配置DHCP服务监听网卡
log "配置 DHCP 服务监听网卡: $INTERFACE_26 和 $INTERFACE_28"
sudo sed -i "s/^INTERFACESv4=\".*\"/INTERFACESv4=\"$INTERFACE_26 $INTERFACE_28\"/" /etc/default/isc-dhcp-server

# 配置DHCP服务
log "配置 DHCP 服务为两个子网分配 IP..."
DHCP_CONF="/etc/dhcp/dhcpd.conf"
sudo tee $DHCP_CONF > /dev/null <<EOF
default-lease-time $LEASE_TIME;
max-lease-time $((LEASE_TIME * 2));
authoritative;

subnet $SUBNET_26 netmask $NETMASK_26 {
  range $POOL_START_26 $POOL_END_26;
  option routers $ROUTER_26;
  option subnet-mask $NETMASK_26;
  option domain-name-servers 8.8.8.8, 114.114.114.114;
}

subnet $SUBNET_28 netmask $NETMASK_28 {
  range $POOL_START_28 $POOL_END_28;
  option routers $ROUTER_28;
  option subnet-mask $NETMASK_28;
  option domain-name-servers 8.8.8.8, 114.114.114.114;
}
EOF

# 清理现有网络配置
log "清理现有网络配置..."
sudo nmcli connection delete $INTERFACE_26-wired 2>/dev/null || true
sudo nmcli connection delete $INTERFACE_28-wired 2>/dev/null || true

# 配置网卡静态IP
log "配置网卡 $INTERFACE_26 静态 IP 为 $ROUTER_26/24"
sudo nmcli connection add type ethernet ifname $INTERFACE_26 con-name $INTERFACE_26-wired ip4 $ROUTER_26/24

log "配置网卡 $INTERFACE_28 静态 IP 为 $ROUTER_28/24"
sudo nmcli connection add type ethernet ifname $INTERFACE_28 con-name $INTERFACE_28-wired ip4 $ROUTER_28/24

# 激活配置
log "应用网络配置..."
sudo nmcli connection down $INTERFACE_26-wired || true
sudo nmcli connection up $INTERFACE_26-wired
sudo nmcli connection down $INTERFACE_28-wired || true
sudo nmcli connection up $INTERFACE_28-wired

# 重启DHCP服务
log "重启 DHCP 服务..."
sudo systemctl restart isc-dhcp-server
sudo systemctl enable isc-dhcp-server

# 显示最终配置结果
log "=== DHCP配置完成，当前网络接口状态 ==="
ip addr show | grep -E '^[0-9]+:|inet ' | grep -E '^[0-9]+:|'"$INTERFACE_26|$INTERFACE_28"
