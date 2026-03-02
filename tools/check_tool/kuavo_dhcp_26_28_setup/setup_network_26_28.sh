#!/bin/bash
set -e

# 彩色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok()      { echo -e "${GREEN}[OK]${NC}   $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERR]${NC}  $1"; }

#------------------------------
# 0. 权限检查
#------------------------------
if [ "$EUID" -ne 0 ]; then
  log_error "请用 root 或 sudo 运行本脚本"
  exit 1
fi

log_info "开始配置 26/28 网段网络与 DHCP 服务..."

#------------------------------
# 1. 备份关键配置文件
#------------------------------
BACKUP_DIR="/root/net_26_28_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

log_info "备份现有配置到: $BACKUP_DIR"

# 备份 netplan
if ls /etc/netplan/*.yaml >/dev/null 2>&1; then
  cp /etc/netplan/*.yaml "$BACKUP_DIR"/ 2>/dev/null || true
  log_ok "已备份 netplan 配置"
else
  log_warn "未找到 /etc/netplan/*.yaml"
fi

# 备份 DHCP 配置
if [ -f /etc/dhcp/dhcpd.conf ]; then
  cp /etc/dhcp/dhcpd.conf "$BACKUP_DIR"/dhcpd.conf.bak 2>/dev/null || true
  log_ok "已备份 /etc/dhcp/dhcpd.conf"
fi

if [ -f /etc/default/isc-dhcp-server ]; then
  cp /etc/default/isc-dhcp-server "$BACKUP_DIR"/isc-dhcp-server.bak 2>/dev/null || true
  log_ok "已备份 /etc/default/isc-dhcp-server"
fi

# 备份 sysctl
if [ -f /etc/sysctl.conf ]; then
  cp /etc/sysctl.conf "$BACKUP_DIR"/sysctl.conf.bak 2>/dev/null || true
  log_ok "已备份 /etc/sysctl.conf"
fi

# 备份 iptables 规则（如果存在）
if command -v iptables-save >/dev/null 2>&1; then
  iptables-save > "$BACKUP_DIR"/iptables_rules.v4 2>/dev/null || true
  log_ok "已备份当前 iptables 规则"
fi

#------------------------------
# 2. 配置 netplan 静态 IP（enp3s0 / eno1，renderer: NetworkManager）
#------------------------------
NETPLAN_FILE="/etc/netplan/01-network-manager-all.yaml"

log_info "写入 netplan 配置到 $NETPLAN_FILE（renderer: NetworkManager）"

cat > "$NETPLAN_FILE" <<EOF
# 让 NetworkManager 管理所有设备
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp3s0:
      addresses:
        - 192.168.26.1/24
      dhcp4: false
      dhcp6: false
    eno1:
      addresses:
        - 192.168.28.1/24
      dhcp4: false
      dhcp6: false
      optional: true   # 即使无网线/NO-CARRIER，也应用配置
EOF

log_ok "netplan 配置已写入（由 NetworkManager 渲染/接管）"

log_info "应用 netplan 配置..."
netplan apply
sleep 2

log_ok "netplan 已应用（由 NetworkManager 接管）"

#------------------------------
# 3. 安装并配置 isc-dhcp-server
#------------------------------
if ! command -v dhcpd >/dev/null 2>&1; then
  log_info "安装 isc-dhcp-server..."
  apt-get update
  apt-get install -y isc-dhcp-server
  log_ok "isc-dhcp-server 已安装"
else
  log_ok "isc-dhcp-server 已存在"
fi

log_info "写入 /etc/dhcp/dhcpd.conf..."

cat > /etc/dhcp/dhcpd.conf <<'EOF'
# DHCP 服务器配置 (26 / 28 网段)
option domain-name-servers 8.8.8.8, 8.8.4.4;
default-lease-time 600;
max-lease-time 7200;
ddns-update-style none;
authoritative;

# 26网段DHCP配置（enp3s0）
# 排除地址：26.1（下位机）、26.12（上位机）、26.22（底盘）
subnet 192.168.26.0 netmask 255.255.255.0 {
    range 192.168.26.100 192.168.26.200;
    option routers 192.168.26.1;
    option subnet-mask 255.255.255.0;
    option broadcast-address 192.168.26.255;
}

# 28网段DHCP配置（eno1）
subnet 192.168.28.0 netmask 255.255.255.0 {
    range 192.168.28.100 192.168.28.200;
    option routers 192.168.28.1;
    option subnet-mask 255.255.255.0;
    option broadcast-address 192.168.28.255;
}
EOF

log_ok "/etc/dhcp/dhcpd.conf 配置完成"

log_info "配置 DHCP 监听接口..."

cat > /etc/default/isc-dhcp-server <<EOF
INTERFACESv4="enp3s0 eno1"
INTERFACESv6=""
EOF

log_ok "/etc/default/isc-dhcp-server 配置完成"

#------------------------------
# 4. 配置 DHCP PID 目录自动创建
#------------------------------
log_info "配置 DHCP PID 目录自动创建..."

cat > /etc/tmpfiles.d/dhcp-server.conf <<EOF
d /run/dhcp-server 0755 dhcpd dhcpd -
EOF

systemd-tmpfiles --create /etc/tmpfiles.d/dhcp-server.conf
log_ok "/run/dhcp-server 目录已配置并创建"

#------------------------------
# 5. 开启 IP 转发 + iptables 规则
#------------------------------
log_info "开启 IPv4 转发..."

# 运行时启用
echo 1 > /proc/sys/net/ipv4/ip_forward

# 永久启用 - 使用 /etc/sysctl.d/ 目录（更可靠，优先级更高）
cat > /etc/sysctl.d/99-ip-forward.conf <<EOF
# 启用 IPv4 转发（26/28 网段互通需要）
net.ipv4.ip_forward=1
EOF

# 同时在 /etc/sysctl.conf 中添加（向后兼容）
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf 2>/dev/null; then
  echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
fi

# 立即应用配置
sysctl -p /etc/sysctl.d/99-ip-forward.conf 2>/dev/null || true

# 验证配置
if [ "$(sysctl -n net.ipv4.ip_forward)" = "1" ]; then
  log_ok "IPv4 转发已启用并验证成功"
else
  log_warn "IPv4 转发配置可能未生效，请检查系统配置"
fi

log_info "配置 iptables 规则，实现 26/28 网段互通，以及 26 网段通过 wwan0 / wlp4s0 上网..."

# 清除相关规则（谨慎）
iptables -t nat -F || true
iptables -F FORWARD || true

# 设置 FORWARD 链默认策略为 ACCEPT（根据安全策略自行调整）
iptables -P FORWARD ACCEPT

# 添加 26 <-> 28 互访规则（局域网互通）
iptables -A FORWARD -s 192.168.26.0/24 -d 192.168.28.0/24 -j ACCEPT
iptables -A FORWARD -s 192.168.28.0/24 -d 192.168.26.0/24 -j ACCEPT

#------------------------------------------------------------------
# 26 网段通过 wwan0 上网（主蜂窝出口）
#------------------------------------------------------------------

# 对从 26 网段经 wwan0 出口的流量做地址伪装
iptables -t nat -A POSTROUTING -s 192.168.26.0/24 -o wwan0 -j MASQUERADE

# 允许内网 -> wwan0 的转发（新连接 + 回程）
iptables -A FORWARD -i enp3s0 -o wwan0 -s 192.168.26.0/24 \
  -m conntrack --ctstate NEW,ESTABLISHED,RELATED -j ACCEPT

# 允许 wwan0 回包 -> 内网
iptables -A FORWARD -i wwan0 -o enp3s0 -d 192.168.26.0/24 \
  -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

#------------------------------------------------------------------
# 可选：26 网段通过 wlp4s0 上网（Wi‑Fi 出口）
# 注：即使当前没有 wlp4s0 接口，这些规则也不会报错。
#------------------------------------------------------------------

iptables -t nat -A POSTROUTING -s 192.168.26.0/24 -o wlp4s0 -j MASQUERADE

iptables -A FORWARD -i enp3s0 -o wlp4s0 -s 192.168.26.0/24 \
  -m conntrack --ctstate NEW,ESTABLISHED,RELATED -j ACCEPT

iptables -A FORWARD -i wlp4s0 -o enp3s0 -d 192.168.26.0/24 \
  -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

# 保存规则（使用 /etc/iptables/rules.v4）
if command -v iptables-save >/dev/null 2>&1; then
  mkdir -p /etc/iptables
  iptables-save > /etc/iptables/rules.v4
  log_ok "iptables 规则已保存到 /etc/iptables/rules.v4"
  
  # 尝试安装 iptables-persistent 以实现自动加载
  if ! dpkg -l | grep -q "iptables-persistent"; then
    log_info "尝试安装 iptables-persistent 以实现规则自动加载..."
    DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent 2>/dev/null || \
    log_warn "无法安装 iptables-persistent，将创建 systemd 服务来加载规则"
  fi
  
  # 如果 iptables-persistent 不可用，创建 systemd 服务来加载规则
  if ! systemctl is-enabled netfilter-persistent &>/dev/null 2>&1; then
    log_info "创建 systemd 服务以在启动时自动加载 iptables 规则..."
    
    cat > /etc/systemd/system/iptables-restore.service <<'EOF'
[Unit]
Description=Restore iptables rules
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/sbin/iptables-restore /etc/iptables/rules.v4
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    
    systemctl daemon-reload
    systemctl enable iptables-restore.service
    log_ok "已创建并启用 iptables-restore.service"
  fi
else
  log_warn "未找到 iptables-save，规则不会持久化，请自行安装 iptables-persistent"
fi

#------------------------------
# 6. 创建 wait-for-eno1.service：等待 eno1 UP + 有 IP
#------------------------------
log_info "创建 wait-for-eno1.service（在 DHCP 启动前确保 eno1 自动配置为 192.168.28.1）..."

# 使用独立脚本，避免 systemd unit 文件中的引号嵌套问题
cat > /usr/local/bin/wait-for-eno1.sh <<'EOS'
#!/bin/bash

set -e

# 检查 eno1 接口是否存在
if ! ip link show eno1 &>/dev/null; then
  echo "警告: eno1 接口不存在，跳过配置"
  exit 0
fi

# 如果 NetworkManager 正在运行，优先使用 NetworkManager 配置
if command -v nmcli &>/dev/null && systemctl is-active NetworkManager &>/dev/null; then
  echo "使用 NetworkManager 配置 eno1..."
  
  # 查找 eno1 的连接
  ENO1_CONN=$(nmcli -t -f NAME,DEVICE connection show | grep ":eno1$" | cut -d: -f1 | head -1)
  
  if [ -n "$ENO1_CONN" ]; then
    # 如果连接存在，激活它
    nmcli connection up "$ENO1_CONN" 2>/dev/null || true
  else
    # 如果连接不存在，创建一个新的
    nmcli connection add type ethernet ifname eno1 con-name eno1-static 2>/dev/null || true
    nmcli connection modify eno1-static ipv4.addresses 192.168.28.1/24 2>/dev/null || true
    nmcli connection modify eno1-static ipv4.method manual 2>/dev/null || true
    nmcli connection modify eno1-static ipv4.dhcp no 2>/dev/null || true
    nmcli connection up eno1-static 2>/dev/null || true
  fi
fi

# 最多等待30秒，确保 eno1 有 IP 地址
for i in {1..30}; do
  # 确保接口是 UP 状态
  ip link set eno1 up 2>/dev/null || true
  
  # 如果接口还没有 IP，尝试添加
  if ! ip addr show eno1 2>/dev/null | grep -q "192.168.28.1"; then
    # 先删除可能存在的旧地址（避免冲突）
    ip addr flush dev eno1 2>/dev/null || true
    # 添加新地址
    ip addr add 192.168.28.1/24 dev eno1 2>/dev/null || true
  fi
  
  # 验证 IP 地址是否存在
  if ip addr show eno1 2>/dev/null | grep -q "inet.*192.168.28.1"; then
    echo "eno1 接口已配置 IP: 192.168.28.1/24"
    exit 0
  fi
  
  sleep 1
done

# 即使超时也返回成功，避免阻塞系统启动
echo "警告: eno1 接口配置超时，但继续启动"
exit 0
EOS

chmod +x /usr/local/bin/wait-for-eno1.sh
log_ok "wait-for-eno1.sh 脚本已创建"

cat > /etc/systemd/system/wait-for-eno1.service <<'EOF'
[Unit]
Description=Wait for eno1 interface to be UP with IP before DHCP
Before=isc-dhcp-server.service
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/wait-for-eno1.sh

[Install]
WantedBy=multi-user.target
EOF

log_ok "wait-for-eno1.service 已创建"

#------------------------------
# 7. 配置 DHCP 服务依赖 wait-for-eno1
#------------------------------
log_info "为 isc-dhcp-server 配置依赖 wait-for-eno1.service..."

mkdir -p /etc/systemd/system/isc-dhcp-server.service.d

cat > /etc/systemd/system/isc-dhcp-server.service.d/wait-for-eno1.conf <<'EOF'
[Unit]
After=wait-for-eno1.service network-online.target NetworkManager.service
Wants=wait-for-eno1.service network-online.target
Requires=wait-for-eno1.service

[Service]
# 确保 PID 目录在服务启动前存在
ExecStartPre=/bin/mkdir -p /run/dhcp-server
ExecStartPre=/bin/chown dhcpd:dhcpd /run/dhcp-server
ExecStartPre=/bin/chmod 755 /run/dhcp-server
# 最后检查 eno1 接口状态（等待一小段时间确保 wait-for-eno1 完成）
ExecStartPre=/bin/sleep 1
EOF

# CPU 亲和性：将 dhcpd 绑定到核心 15（避免与 network-quality-check 等配置混淆）
cat > /etc/systemd/system/isc-dhcp-server.service.d/cpu-affinity.conf <<'EOF'
[Service]
CPUAffinity=15
EOF

log_ok "isc-dhcp-server 依赖配置完成（含 CPU 亲和性绑定核心 15）"

#------------------------------
# 8. 重新加载 systemd 并启动/启用服务
#------------------------------
log_info "重新加载 systemd 配置并启用相关服务..."

systemctl daemon-reload

# 启用 wait-for-eno1 和 DHCP
systemctl enable wait-for-eno1.service
systemctl enable isc-dhcp-server.service

# 先手动执行一次 wait-for-eno1，确保当前会话 eno1 就绪
systemctl start wait-for-eno1.service || true

# 验证 eno1 接口状态
log_info "验证 eno1 接口配置..."
sleep 2  # 给 NetworkManager/系统一点时间应用配置

if ip link show eno1 &>/dev/null; then
  if ip addr show eno1 2>/dev/null | grep -q "inet.*192.168.28.1"; then
    log_ok "eno1 接口已配置 IP: 192.168.28.1/24"
  else
    log_warn "eno1 接口没有 IP 地址，尝试手动配置..."
    ip link set eno1 up 2>/dev/null || true
    ip addr add 192.168.28.1/24 dev eno1 2>/dev/null || true
    sleep 1
    if ip addr show eno1 2>/dev/null | grep -q "inet.*192.168.28.1"; then
      log_ok "eno1 接口已手动配置 IP"
    else
      log_warn "eno1 接口配置失败，DHCP 可能无法监听该接口"
    fi
  fi
else
  log_warn "eno1 接口不存在，跳过配置"
fi

# 确保 PID 目录存在（双重保险）
mkdir -p /run/dhcp-server
chown dhcpd:dhcpd /run/dhcp-server 2>/dev/null || chmod 755 /run/dhcp-server

# 启动 DHCP 服务
systemctl restart isc-dhcp-server.service

log_ok "isc-dhcp-server 已启动"

#------------------------------
# 9. 验证与提示
#------------------------------
log_info "验证 eno1 接口与 DHCP 状态..."

ip addr show eno1 | sed 's/^/  /'
echo ""

log_info "最近 DHCP 日志："
journalctl -u isc-dhcp-server -n 10 --no-pager || true

echo ""
log_ok "全部配置完成！"

log_info "重启后检查要点："
log_info "  1. eno1 是否自动配置为 192.168.28.1："
log_info "       ip addr show eno1 | grep 192.168.28.1"
log_info "  2. DHCP 是否监听 eno1："
log_info "       journalctl -u isc-dhcp-server -n 20 | grep eno1"
log_info "  3. 设备插入 eno1 后是否自动获取 28 网段 IP"

echo ""
log_warn "注意：上位机 (192.168.26.12) 仍需添加到 28 网段的静态路由，例如："
log_warn "  在上位机上执行：sudo ip route add 192.168.28.0/24 via 192.168.26.1"
echo ""