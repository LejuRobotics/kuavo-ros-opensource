#!/bin/bash
#
# setup_network_26_28.sh
#
# 作用：
#   1. 检测网口是否存在，根据实际情况动态配置：
#      - enp3s0: 192.168.26.1/24（26网段，对上位机/内部设备）
#      - eno1 : 192.168.28.1/24（28网段，对外部设备）
#   2. 部署 DHCP 服务（isc-dhcp-server），为存在的网段分配 IP
#   3. 开启 IP 转发 + iptables 规则，实现 26 <-> 28 网段互通
#   4. 解决 DHCP 启动时 eno1 还未就绪的问题（仅 eno1 存在时）
#   5. 创建 /run/dhcp-server 目录自动创建配置，避免 PID 文件错误
#
# 适用环境：
#   - Ubuntu 18.04/20.04/22.04（使用 netplan + NetworkManager）
#   - 网卡名：enp3s0（26网口），eno1（28网口）
#
# 使用方法：
#   sudo bash setup_network_26_28.sh
#
# 注意：
#   - 脚本会覆盖部分系统配置文件（netplan、dhcpd.conf 等），执行前自行备份。
#   - 上位机（192.168.26.12）仍需单独添加一条到 28 网段的静态路由：
#       sudo ip route add 192.168.28.0/24 via 192.168.26.1
#

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
# 网口配置定义
#------------------------------
INTERNAL_IF="enp3s0"      # 内网接口 (26网段)
INTERNAL_IP="192.168.26.1"
INTERNAL_NET="192.168.26.0/24"
INTERNAL_DHCP_RANGE="192.168.26.100 192.168.26.200"

EXTERNAL_IF="eno1"        # 外网接口 (28网段)
EXTERNAL_IP="192.168.28.1"
EXTERNAL_NET="192.168.28.0/24"
EXTERNAL_DHCP_RANGE="192.168.28.100 192.168.28.200"

# 接口存在标志
INTERNAL_IF_EXISTS=false
EXTERNAL_IF_EXISTS=false

# WiFi 和 5G 接口名（动态检测）
# WiFi: 可能是 wlp4s0（无5G模块）或 wlp5s0（有5G模块）
# 5G:   可能是 wwan0、wwx* 等
WIFI_IF=""
WWAN_IF=""

#------------------------------
# 0. 权限检查
#------------------------------
if [ "$EUID" -ne 0 ]; then
  log_error "请用 root 或 sudo 运行本脚本"
  exit 1
fi

#------------------------------
# 1. 检测网口是否存在
#------------------------------
log_info "检测网络接口..."

# 显示当前所有网络接口
log_info "当前系统网络接口："
ip link show | grep -E "^[0-9]+:" | awk '{print "  " $2}' | tr -d ':'

# 检测内网接口
if ip link show "$INTERNAL_IF" &>/dev/null; then
  INTERNAL_IF_EXISTS=true
  log_ok "检测到内网接口: $INTERNAL_IF"
else
  log_warn "未检测到内网接口: $INTERNAL_IF"
fi

# 检测外网接口
if ip link show "$EXTERNAL_IF" &>/dev/null; then
  EXTERNAL_IF_EXISTS=true
  log_ok "检测到外网接口: $EXTERNAL_IF"
else
  log_warn "未检测到外网接口: $EXTERNAL_IF"
fi

# 至少需要一个网口
if [ "$INTERNAL_IF_EXISTS" = false ] && [ "$EXTERNAL_IF_EXISTS" = false ]; then
  log_error "未检测到任何网口 ($INTERNAL_IF 或 $EXTERNAL_IF)，无法继续配置"
  exit 1
fi

# 动态检测 WiFi 接口（wlp* 或 wlan* 开头的无线网卡）
# 有5G模块时可能是 wlp5s0，无5G模块时可能是 wlp4s0，也可能是传统命名 wlan*
WIFI_IF=$(ip link show 2>/dev/null | grep -E "wlp|wlan" | head -1 | awk -F: '{print $2}' | tr -d ' ')
if [ -n "$WIFI_IF" ]; then
  log_ok "检测到 WiFi 接口: $WIFI_IF"
else
  log_info "未检测到 WiFi 接口 (wlp*/wlan*)"
fi

# 动态检测 5G 接口（优先匹配 wwan* 或 wwx2*，否则回退 wwan0）
WWAN_IF=$(ip link show 2>/dev/null | grep -E "^[0-9]+: (wwan|wwx2)" | head -1 | awk -F: '{print $2}' | tr -d ' ')
if [ -z "$WWAN_IF" ]; then
  if ip link show wwan0 &>/dev/null 2>&1; then
    WWAN_IF="wwan0"
  fi
fi
if [ -n "$WWAN_IF" ]; then
  log_ok "检测到 5G 接口: $WWAN_IF"
else
  log_info "未检测到 5G 接口 (wwan*/wwx*)"
fi

log_info "开始配置网络与 DHCP 服务..."

#------------------------------
# 2. 备份关键配置文件
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
# 3. 配置 netplan 静态 IP（根据实际存在的接口动态配置）
#------------------------------
NETPLAN_FILE="/etc/netplan/01-network-manager-all.yaml"

log_info "写入 netplan 配置到 $NETPLAN_FILE（renderer: NetworkManager）"

# 动态生成 netplan 配置
{
  echo "# Kuavo 网络配置 - 由 setup_network_26_28.sh 自动生成"
  echo "# 生成时间: $(date)"
  echo "network:"
  echo "  version: 2"
  echo "  renderer: NetworkManager"
  echo "  ethernets:"

  if [ "$INTERNAL_IF_EXISTS" = true ]; then
    echo "    $INTERNAL_IF:"
    echo "      addresses:"
    echo "        - $INTERNAL_IP/24"
    echo "      dhcp4: false"
    echo "      dhcp6: false"
  fi

  if [ "$EXTERNAL_IF_EXISTS" = true ]; then
    echo "    $EXTERNAL_IF:"
    echo "      addresses:"
    echo "        - $EXTERNAL_IP/24"
    echo "      dhcp4: false"
    echo "      dhcp6: false"
    echo "      optional: true   # 即使无网线/NO-CARRIER，也应用配置"
  fi
} > "$NETPLAN_FILE"

log_ok "netplan 配置已写入（由 NetworkManager 渲染/接管）"
[ "$INTERNAL_IF_EXISTS" = true ] && log_info "  - $INTERNAL_IF: $INTERNAL_IP/24"
[ "$EXTERNAL_IF_EXISTS" = true ] && log_info "  - $EXTERNAL_IF: $EXTERNAL_IP/24"

log_info "应用 netplan 配置..."
netplan apply
sleep 2

log_ok "netplan 已应用（由 NetworkManager 接管）"

#------------------------------
# 4. 安装并配置 isc-dhcp-server
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

# 动态生成 DHCP 配置
{
  echo "# DHCP 服务器配置 - 由 setup_network_26_28.sh 自动生成"
  echo "# 生成时间: $(date)"
  echo "option domain-name-servers 8.8.8.8, 8.8.4.4;"
  echo "default-lease-time 600;"
  echo "max-lease-time 7200;"
  echo "ddns-update-style none;"
  echo "authoritative;"
  echo ""

  if [ "$INTERNAL_IF_EXISTS" = true ]; then
    echo "# 26网段DHCP配置（$INTERNAL_IF）"
    echo "# 排除地址：26.1（下位机）、26.12（上位机）、26.22（底盘）"
    echo "subnet 192.168.26.0 netmask 255.255.255.0 {"
    echo "    range $INTERNAL_DHCP_RANGE;"
    echo "    option routers 192.168.26.1;"
    echo "    option subnet-mask 255.255.255.0;"
    echo "    option broadcast-address 192.168.26.255;"
    echo "}"
    echo ""
  fi

  if [ "$EXTERNAL_IF_EXISTS" = true ]; then
    echo "# 28网段DHCP配置（$EXTERNAL_IF）"
    echo "subnet 192.168.28.0 netmask 255.255.255.0 {"
    echo "    range $EXTERNAL_DHCP_RANGE;"
    echo "    option routers 192.168.28.1;"
    echo "    option subnet-mask 255.255.255.0;"
    echo "    option broadcast-address 192.168.28.255;"
    echo "}"
  fi
} > /etc/dhcp/dhcpd.conf

log_ok "/etc/dhcp/dhcpd.conf 配置完成"

log_info "配置 DHCP 监听接口..."

# 动态生成监听接口列表
DHCP_INTERFACES=""
[ "$INTERNAL_IF_EXISTS" = true ] && DHCP_INTERFACES="$INTERNAL_IF"
[ "$EXTERNAL_IF_EXISTS" = true ] && DHCP_INTERFACES="$DHCP_INTERFACES $EXTERNAL_IF"
DHCP_INTERFACES=$(echo "$DHCP_INTERFACES" | xargs)  # 去除首尾空格

cat > /etc/default/isc-dhcp-server <<EOF
# DHCP 监听接口 - 由 setup_network_26_28.sh 自动生成
INTERFACESv4="$DHCP_INTERFACES"
INTERFACESv6=""
EOF

log_ok "/etc/default/isc-dhcp-server 配置完成"
log_info "  - 监听接口: $DHCP_INTERFACES"

#------------------------------
# 5. 配置 DHCP PID 目录自动创建
#------------------------------
log_info "配置 DHCP PID 目录自动创建..."

cat > /etc/tmpfiles.d/dhcp-server.conf <<EOF
d /run/dhcp-server 0755 dhcpd dhcpd -
EOF

systemd-tmpfiles --create /etc/tmpfiles.d/dhcp-server.conf
log_ok "/run/dhcp-server 目录已配置并创建"

#------------------------------
# 6. 开启 IP 转发 + iptables 规则
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

log_info "配置 iptables 规则，实现 26/28 网段互通，以及 26 网段上网出口..."

# 清除相关规则（谨慎）
iptables -t nat -F || true
iptables -F FORWARD || true

# 设置 FORWARD 链默认策略为 ACCEPT（根据安全策略自行调整）
iptables -P FORWARD ACCEPT

# 添加 26 <-> 28 互访规则（只有两个接口都存在时才配置）
if [ "$INTERNAL_IF_EXISTS" = true ] && [ "$EXTERNAL_IF_EXISTS" = true ]; then
  iptables -A FORWARD -s 192.168.26.0/24 -d 192.168.28.0/24 -j ACCEPT
  iptables -A FORWARD -s 192.168.28.0/24 -d 192.168.26.0/24 -j ACCEPT
  log_info "  - 已配置 26/28 网段互通规则"
fi

#------------------------------------------------------------------
# 26 网段上网出口 - 只有内网接口存在时才配置
# WiFi 和 5G 接口名是动态检测的，不写死
#------------------------------------------------------------------
if [ "$INTERNAL_IF_EXISTS" = true ]; then
  # 5G 出口（wwan* 或 wwx*）
  if [ -n "$WWAN_IF" ]; then
    iptables -t nat -A POSTROUTING -s 192.168.26.0/24 -o $WWAN_IF -j MASQUERADE
    iptables -A FORWARD -i $INTERNAL_IF -o $WWAN_IF -s 192.168.26.0/24 \
      -m conntrack --ctstate NEW,ESTABLISHED,RELATED -j ACCEPT
    iptables -A FORWARD -i $WWAN_IF -o $INTERNAL_IF -d 192.168.26.0/24 \
      -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    log_info "  - 已配置 26 网段通过 $WWAN_IF (5G) 上网"
  else
    log_info "  - 未检测到 5G 接口，跳过 5G 出口配置"
  fi

  # WiFi 出口（wlp*）
  if [ -n "$WIFI_IF" ]; then
    iptables -t nat -A POSTROUTING -s 192.168.26.0/24 -o $WIFI_IF -j MASQUERADE
    iptables -A FORWARD -i $INTERNAL_IF -o $WIFI_IF -s 192.168.26.0/24 \
      -m conntrack --ctstate NEW,ESTABLISHED,RELATED -j ACCEPT
    iptables -A FORWARD -i $WIFI_IF -o $INTERNAL_IF -d 192.168.26.0/24 \
      -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    log_info "  - 已配置 26 网段通过 $WIFI_IF (WiFi) 上网"
  else
    log_info "  - 未检测到 WiFi 接口，跳过 WiFi 出口配置"
  fi
fi

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
# 7. 创建 wait-for-external-if.service（只有外网接口存在时才创建）
#------------------------------
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  log_info "创建 wait-for-$EXTERNAL_IF.service（在 DHCP 启动前确保 $EXTERNAL_IF 自动配置为 $EXTERNAL_IP）..."

  # 使用独立脚本，避免 systemd unit 文件中的引号嵌套问题
  cat > /usr/local/bin/wait-for-external-if.sh <<EOS
#!/bin/bash

set -e

# 检查 $EXTERNAL_IF 接口是否存在
if ! ip link show $EXTERNAL_IF &>/dev/null; then
  echo "警告: $EXTERNAL_IF 接口不存在，跳过配置"
  exit 0
fi

# 如果 NetworkManager 正在运行，优先使用 NetworkManager 配置
if command -v nmcli &>/dev/null && systemctl is-active NetworkManager &>/dev/null; then
  echo "使用 NetworkManager 配置 $EXTERNAL_IF..."

  # 查找 $EXTERNAL_IF 的连接
  EXT_CONN=\$(nmcli -t -f NAME,DEVICE connection show | grep ":$EXTERNAL_IF\$" | cut -d: -f1 | head -1)

  if [ -n "\$EXT_CONN" ]; then
    # 如果连接存在，激活它
    nmcli connection up "\$EXT_CONN" 2>/dev/null || true
  else
    # 清理旧的连接（避免重复创建）
    OLD_CONNECTIONS=\$(nmcli -t -f NAME connection show | grep -E "^${EXTERNAL_IF}-static" || true)
    if [ -n "\$OLD_CONNECTIONS" ]; then
      echo "发现旧的 ${EXTERNAL_IF}-static 连接，正在清理..."
      echo "\$OLD_CONNECTIONS" | while IFS= read -r conn; do
        if [ -n "\$conn" ]; then
          nmcli connection delete "\$conn" 2>/dev/null || true
          echo "已删除连接: \$conn"
        fi
      done
      sleep 1
    fi

    # 创建新的连接
    nmcli connection add type ethernet ifname $EXTERNAL_IF con-name ${EXTERNAL_IF}-static 2>/dev/null || true
    nmcli connection modify ${EXTERNAL_IF}-static ipv4.addresses $EXTERNAL_IP/24 2>/dev/null || true
    nmcli connection modify ${EXTERNAL_IF}-static ipv4.method manual 2>/dev/null || true
    nmcli connection modify ${EXTERNAL_IF}-static ipv4.dhcp no 2>/dev/null || true
    nmcli connection up ${EXTERNAL_IF}-static 2>/dev/null || true
  fi
fi

# 最多等待30秒，确保 $EXTERNAL_IF 有 IP 地址
for i in {1..30}; do
  # 确保接口是 UP 状态
  ip link set $EXTERNAL_IF up 2>/dev/null || true

  # 如果接口还没有 IP，尝试添加
  if ! ip addr show $EXTERNAL_IF 2>/dev/null | grep -q "$EXTERNAL_IP"; then
    # 先删除可能存在的旧地址（避免冲突）
    ip addr flush dev $EXTERNAL_IF 2>/dev/null || true
    # 添加新地址
    ip addr add $EXTERNAL_IP/24 dev $EXTERNAL_IF 2>/dev/null || true
  fi

  # 验证 IP 地址是否存在
  if ip addr show $EXTERNAL_IF 2>/dev/null | grep -q "inet.*$EXTERNAL_IP"; then
    echo "$EXTERNAL_IF 接口已配置 IP: $EXTERNAL_IP/24"
    exit 0
  fi

  sleep 1
done

# 即使超时也返回成功，避免阻塞系统启动
echo "警告: $EXTERNAL_IF 接口配置超时，但继续启动"
exit 0
EOS

  chmod +x /usr/local/bin/wait-for-external-if.sh
  log_ok "wait-for-external-if.sh 脚本已创建"

  cat > /etc/systemd/system/wait-for-external-if.service <<EOF
[Unit]
Description=Wait for $EXTERNAL_IF interface to be UP with IP before DHCP
Before=isc-dhcp-server.service
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/wait-for-external-if.sh

[Install]
WantedBy=multi-user.target
EOF

  log_ok "wait-for-external-if.service 已创建"
else
  log_info "外网接口 $EXTERNAL_IF 不存在，跳过创建 wait-for-external-if.service"
fi

#------------------------------
# 8. 配置 DHCP 服务依赖（只有外网接口存在时才添加 wait-for-external-if 依赖）
#------------------------------
log_info "为 isc-dhcp-server 配置依赖..."

mkdir -p /etc/systemd/system/isc-dhcp-server.service.d

if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  # 外网接口存在时，添加 wait-for-external-if 依赖
  cat > /etc/systemd/system/isc-dhcp-server.service.d/wait-for-external-if.conf <<EOF
[Unit]
After=wait-for-external-if.service network-online.target NetworkManager.service
Wants=wait-for-external-if.service network-online.target
Requires=wait-for-external-if.service

[Service]
# 确保 PID 目录在服务启动前存在
ExecStartPre=/bin/mkdir -p /run/dhcp-server
ExecStartPre=/bin/chown dhcpd:dhcpd /run/dhcp-server
ExecStartPre=/bin/chmod 755 /run/dhcp-server
# 最后检查接口状态（等待一小段时间确保 wait-for-external-if 完成）
ExecStartPre=/bin/sleep 1
EOF

  log_ok "isc-dhcp-server 依赖配置完成（包含 wait-for-external-if.service）"
else
  # 外网接口不存在时，只配置基础依赖
  cat > /etc/systemd/system/isc-dhcp-server.service.d/wait-for-external-if.conf <<EOF
[Unit]
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
# 确保 PID 目录在服务启动前存在
ExecStartPre=/bin/mkdir -p /run/dhcp-server
ExecStartPre=/bin/chown dhcpd:dhcpd /run/dhcp-server
ExecStartPre=/bin/chmod 755 /run/dhcp-server
EOF

  log_ok "isc-dhcp-server 依赖配置完成（无 wait-for-external-if.service，外网接口不存在）"
fi

# CPU 亲和性：不绑定特定核心，由系统自由调度
# dhcpd 是轻量服务，几乎不占 CPU，自由调度对控制进程影响可忽略
if [ -f /etc/systemd/system/isc-dhcp-server.service.d/cpu-affinity.conf ]; then
  rm -f /etc/systemd/system/isc-dhcp-server.service.d/cpu-affinity.conf
  log_info "已移除旧的 CPU 亲和性配置"
fi

log_ok "isc-dhcp-server 不绑定 CPU 核心，由系统自由调度"

#------------------------------
# 9. 重新加载 systemd 并启动/启用服务
#------------------------------
log_info "重新加载 systemd 配置并启用相关服务..."

systemctl daemon-reload

# 启用并启动 wait-for-external-if（如果存在）
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  systemctl enable wait-for-external-if.service
  systemctl start wait-for-external-if.service || true
fi

# 启用 DHCP
systemctl enable isc-dhcp-server.service

# 验证外网接口状态（如果存在）
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  log_info "验证 $EXTERNAL_IF 接口配置..."
  sleep 2  # 给 NetworkManager/系统一点时间应用配置

  if ip link show "$EXTERNAL_IF" &>/dev/null; then
    if ip addr show "$EXTERNAL_IF" 2>/dev/null | grep -q "inet.*$EXTERNAL_IP"; then
      log_ok "$EXTERNAL_IF 接口已配置 IP: $EXTERNAL_IP/24"
    else
      log_warn "$EXTERNAL_IF 接口没有 IP 地址，尝试手动配置..."
      ip link set "$EXTERNAL_IF" up 2>/dev/null || true
      ip addr add "$EXTERNAL_IP"/24 dev "$EXTERNAL_IF" 2>/dev/null || true
      sleep 1
      if ip addr show "$EXTERNAL_IF" 2>/dev/null | grep -q "inet.*$EXTERNAL_IP"; then
        log_ok "$EXTERNAL_IF 接口已手动配置 IP"
      else
        log_warn "$EXTERNAL_IF 接口配置失败，DHCP 可能无法监听该接口"
      fi
    fi
  fi
fi

# 确保 PID 目录存在（双重保险）
mkdir -p /run/dhcp-server
chown dhcpd:dhcpd /run/dhcp-server 2>/dev/null || chmod 755 /run/dhcp-server

# 启动 DHCP 服务
systemctl restart isc-dhcp-server.service

log_ok "isc-dhcp-server 已启动"

#------------------------------
# 10. 验证与提示
#------------------------------
log_info "验证 DHCP 状态..."

log_info "最近 DHCP 日志："
journalctl -u isc-dhcp-server -n 10 --no-pager || true

echo ""
log_ok "全部配置完成！"
echo ""

# 输出配置总结
log_info "========== 配置总结 =========="
log_info "检测到的接口："
[ "$INTERNAL_IF_EXISTS" = true ] && log_ok "  - $INTERNAL_IF (内网): $INTERNAL_IP/24"
[ "$EXTERNAL_IF_EXISTS" = true ] && log_ok "  - $EXTERNAL_IF (外网): $EXTERNAL_IP/24"
[ "$INTERNAL_IF_EXISTS" = false ] && log_warn "  - $INTERNAL_IF (内网): 未检测到"
[ "$EXTERNAL_IF_EXISTS" = false ] && log_warn "  - $EXTERNAL_IF (外网): 未检测到"

log_info "DHCP 监听接口: $DHCP_INTERFACES"

log_info "已配置的功能："
log_ok "  - IPv4 转发已启用"
log_ok "  - DHCP CPU 由系统自由调度"
[ "$INTERNAL_IF_EXISTS" = true ] && [ "$EXTERNAL_IF_EXISTS" = true ] && log_ok "  - 26/28 网段互通规则已配置"
[ "$INTERNAL_IF_EXISTS" = true ] && [ -n "$WWAN_IF" ] && log_ok "  - 26 网段可通过 $WWAN_IF (5G) 上网"
[ "$INTERNAL_IF_EXISTS" = true ] && [ -n "$WIFI_IF" ] && log_ok "  - 26 网段可通过 $WIFI_IF (WiFi) 上网"
[ "$EXTERNAL_IF_EXISTS" = true ] && log_ok "  - wait-for-external-if.service 已创建"

echo ""
log_info "重启后检查要点："
if [ "$INTERNAL_IF_EXISTS" = true ]; then
  log_info "  1. $INTERNAL_IF 是否自动配置为 $INTERNAL_IP："
  log_info "       ip addr show $INTERNAL_IF | grep $INTERNAL_IP"
fi
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  log_info "  2. $EXTERNAL_IF 是否自动配置为 $EXTERNAL_IP："
  log_info "       ip addr show $EXTERNAL_IF | grep $EXTERNAL_IP"
fi
log_info "  3. DHCP 服务状态："
log_info "       journalctl -u isc-dhcp-server -n 20"

if [ "$INTERNAL_IF_EXISTS" = true ] && [ "$EXTERNAL_IF_EXISTS" = true ]; then
  echo ""
  log_warn "注意：上位机 (192.168.26.12) 仍需添加到 28 网段的静态路由，例如："
  log_warn "  在上位机上执行：sudo ip route add 192.168.28.0/24 via 192.168.26.1"
fi
echo ""
