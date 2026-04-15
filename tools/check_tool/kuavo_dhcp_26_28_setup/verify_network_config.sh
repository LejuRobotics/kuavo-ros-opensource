#!/bin/bash
#
# verify_network_config.sh
#
# 网络配置自动化验证脚本（7 项验证）
# 每完成一项后询问用户是否继续下一项
#
# 使用方法：
#   sudo bash verify_network_config.sh
#

set -e

# 彩色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

log_info()  { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}   $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERR]${NC}  $1"; }

log_step()  { echo -e "${CYAN}========================================${NC}"; }
log_title() { echo -e "${CYAN}$1${NC}"; }

check_pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; PASS_COUNT=$((PASS_COUNT + 1)); }
check_fail() { echo -e "  ${RED}[FAIL]${NC} $1"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
check_warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; WARN_COUNT=$((WARN_COUNT + 1)); }

#------------------------------
# 权限检查
#------------------------------
if [ "$EUID" -ne 0 ]; then
  log_error "请用 root 或 sudo 运行本脚本"
  exit 1
fi

#------------------------------
# 接口定义（与 setup 脚本一致）
#------------------------------
INTERNAL_IF="enp3s0"
INTERNAL_IP="192.168.26.1"
EXTERNAL_IF="eno1"
EXTERNAL_IP="192.168.28.1"

INTERNAL_IF_EXISTS=false
EXTERNAL_IF_EXISTS=false

ip link show "$INTERNAL_IF" &>/dev/null && INTERNAL_IF_EXISTS=true
ip link show "$EXTERNAL_IF" &>/dev/null && EXTERNAL_IF_EXISTS=true

# 询问是否继续下一项
ask_continue() {
  local next="$1"
  echo ""
  echo -e "${CYAN}-------------------------------------------${NC}"
  read -p "下一个验证: ${YELLOW}${next}${NC}，是否继续? [Y/n]: " choice
  if [ "$choice" = "n" ] || [ "$choice" = "N" ]; then
    echo ""
    log_info "用户终止验证"
    echo ""
    log_step
    log_title "验证完成 - 总结（已跳过后续项目）"
    log_step
    echo ""
    echo -e "${GREEN}通过: $PASS_COUNT${NC}"
    echo -e "${RED}失败: $FAIL_COUNT${NC}"
    echo -e "${YELLOW}警告: $WARN_COUNT${NC}"
    echo ""
    exit 0
  fi
}

#==========================================================
# 验证 1: 网络 IP 配置 + DHCP 服务状态
#==========================================================
log_step
log_title "验证 1: 网络 IP 配置 + DHCP 服务状态"
log_step
echo ""

# 检查内网接口 IP
if [ "$INTERNAL_IF_EXISTS" = true ]; then
  if ip addr show "$INTERNAL_IF" 2>/dev/null | grep -q "$INTERNAL_IP"; then
    check_pass "$INTERNAL_IF 已配置 $INTERNAL_IP/24"
  else
    check_fail "$INTERNAL_IF 未配置 $INTERNAL_IP"
  fi
else
  check_warn "$INTERNAL_IF 接口不存在"
fi

# 检查外网接口 IP
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  if ip addr show "$EXTERNAL_IF" 2>/dev/null | grep -q "$EXTERNAL_IP"; then
    check_pass "$EXTERNAL_IF 已配置 $EXTERNAL_IP/24"
  else
    check_fail "$EXTERNAL_IF 未配置 $EXTERNAL_IP"
  fi
else
  check_warn "$EXTERNAL_IF 接口不存在"
fi

# 检查 DHCP 服务状态
if systemctl is-active --quiet isc-dhcp-server; then
  check_pass "isc-dhcp-server 服务运行中"
else
  check_fail "isc-dhcp-server 服务未运行"
  log_info "最近日志："
  journalctl -u isc-dhcp-server -n 5 --no-pager 2>/dev/null || true
fi

# 检查 DHCP 监听接口
DHCP_INTERFACES=$(grep "^INTERFACESv4=" /etc/default/isc-dhcp-server 2>/dev/null | cut -d'"' -f2)
if [ -n "$DHCP_INTERFACES" ]; then
  check_pass "DHCP 监听接口: $DHCP_INTERFACES"
else
  check_warn "未找到 DHCP 监听接口配置"
fi

#==========================================================
# 验证 2: DHCP 进程运行状态
#==========================================================
if ! ask_continue "DHCP 进程运行状态"; then exit 0; fi

log_step
log_title "验证 2: DHCP 进程运行状态"
log_step
echo ""

DHCP_PID=$(pgrep -x dhcpd 2>/dev/null)
if [ -n "$DHCP_PID" ]; then
  check_pass "dhcpd 进程正在运行 (PID: $DHCP_PID)"

  # 显示进程调度信息
  CPU_ALLOWED=$(cat /proc/$DHCP_PID/status 2>/dev/null | grep "Cpus_allowed_list" | awk '{print $2}')
  CURRENT_CPU=$(ps -p $DHCP_PID -o psr --no-headers 2>/dev/null | tr -d ' ')
  log_info "  Cpus_allowed_list: $CPU_ALLOWED（由系统自由调度）"
  log_info "  当前运行在 CPU: $CURRENT_CPU"
else
  check_fail "dhcpd 进程未运行"
fi

#==========================================================
# 验证 3: 外部设备通过 DHCP 获取 IP
#==========================================================
if ! ask_continue "外部设备 DHCP 获取 IP（需手动确认）"; then exit 0; fi

log_step
log_title "验证 3: 外部设备通过 DHCP 获取 IP"
log_step
echo ""

log_info "请确认以下内容："
log_info "  1. 将外部设备（如笔记本电脑）连接到对应网口"
log_info "  2. 设备网络设置为 DHCP 自动获取"
log_info "  3. 检查设备是否获取到了正确网段的 IP"
echo ""

# 检查 DHCP 租约文件
if [ -f /var/lib/dhcp/dhcpd.leases ]; then
  LEASE_COUNT=$(grep -c "lease " /var/lib/dhcp/dhcpd.leases 2>/dev/null || echo "0")
  log_info "当前 DHCP 租约记录数: $LEASE_COUNT"
  if [ "$LEASE_COUNT" -gt 0 ]; then
    log_info "最近的租约："
    tail -20 /var/lib/dhcp/dhcpd.leases | grep -E "lease |hardware|starts" | sed 's/^/    /'
  fi
else
  log_warn "未找到 DHCP 租约文件"
fi

echo ""
read -p "外部设备是否成功通过 DHCP 获取到 IP? [y/N]: " result
if [ "$result" = "y" ] || [ "$result" = "Y" ]; then
  check_pass "外部设备成功获取 DHCP IP"
else
  check_fail "外部设备未能获取 DHCP IP"
fi

#==========================================================
# 验证 4: 26/28 跨网段通信 + ROS 不受影响
#==========================================================
if ! ask_continue "26/28 跨网段通信 + ROS 不受影响"; then exit 0; fi

log_step
log_title "验证 4: 26/28 跨网段通信 + ROS 不受影响"
log_step
echo ""

# 检查 IP 转发
FORWARD_STATUS=$(sysctl -n net.ipv4.ip_forward 2>/dev/null)
if [ "$FORWARD_STATUS" = "1" ]; then
  check_pass "IPv4 转发已启用"
else
  check_fail "IPv4 转发未启用"
fi

# 检查 iptables FORWARD 规则（使用 iptables-save 避免接口名被截断）
if iptables-save 2>/dev/null | grep -q "192.168.26.0/24"; then
  check_pass "iptables 中存在 26/28 网段互通规则"
else
  check_warn "iptables 中未找到 26/28 网段互通规则"
fi

# 手动验证跨网段通信
echo ""
log_info "请手动验证："
log_info "  1. 从 26 网段设备 ping 28 网段设备（如: ping 192.168.28.100）"
log_info "  2. 从 28 网段设备 ping 26 网段设备（如: ping 192.168.26.12）"
log_info "  3. 确认 ROS 通信（话题、服务）正常工作"
echo ""
read -p "26/28 跨网段通信正常且 ROS 不受影响? [y/N]: " result
if [ "$result" = "y" ] || [ "$result" = "Y" ]; then
  check_pass "26/28 跨网段通信正常，ROS 不受影响"
else
  check_fail "26/28 跨网段通信或 ROS 存在问题"
fi

#==========================================================
# 验证 5: Wifi/5G 自动切换 + 上位机上网
#==========================================================
if ! ask_continue "Wifi/5G 自动切换 + 上位机上网（需手动确认）"; then exit 0; fi

log_step
log_title "验证 5: Wifi/5G 自动切换 + 上位机上网"
log_step
echo ""

# 检查 NAT 规则（使用 iptables-save 避免接口名被截断）
# 动态检测 WiFi 和 5G 接口名（与 setup_wifi_5g_switch.sh 保持一致）
WIFI_IF=$(ip link show 2>/dev/null | grep -E "wlp|wlan" | head -1 | awk -F: '{print $2}' | tr -d ' ')
WWAN_IF=$(ip link show 2>/dev/null | grep -E "^[0-9]+: (wwan|wwx2)" | head -1 | awk -F: '{print $2}' | tr -d ' ')
[ -z "$WWAN_IF" ] && { ip link show wwan0 &>/dev/null && WWAN_IF="wwan0" || true; }

if [ -n "$WWAN_IF" ] && iptables-save -t nat 2>/dev/null | grep -q "$WWAN_IF"; then
  check_pass "iptables NAT 规则包含 $WWAN_IF (5G)"
else
  check_warn "iptables NAT 规则中未找到 5G 接口"
fi

if [ -n "$WIFI_IF" ] && iptables-save -t nat 2>/dev/null | grep -q "$WIFI_IF"; then
  check_pass "iptables NAT 规则包含 $WIFI_IF (WiFi)"
else
  check_warn "iptables NAT 规则中未找到 WiFi 接口"
fi

echo ""
log_info "请手动验证："
log_info "  1. 上位机能否通过下位机上网（如: ping 8.8.8.8）"
log_info "  2. 拔掉 5G 模块后 WiFi 是否自动切换"
log_info "  3. 重新插入 5G 模块后是否切回 5G"
echo ""
read -p "Wifi/5G 自动切换和上位机上网正常? [y/N]: " result
if [ "$result" = "y" ] || [ "$result" = "Y" ]; then
  check_pass "Wifi/5G 自动切换和上位机上网正常"
else
  check_fail "Wifi/5G 自动切换或上位机上网存在问题"
fi

#==========================================================
# 验证 6: 外部网络无法修改机器人内部静态 IP
#==========================================================
if ! ask_continue "外部网络无法修改内部静态 IP"; then exit 0; fi

log_step
log_title "验证 6: 外部网络无法修改内部静态 IP"
log_step
echo ""

log_info "说明: 内网静态 IP 通过 netplan/NetworkManager 配置文件写入，"
log_info "      存储在本地 /etc/netplan/ 中，外部网络设备无法修改。"

# 检查内网静态 IP 是否通过 netplan 配置
STATIC_26=false
if [ "$INTERNAL_IF_EXISTS" = true ]; then
  if ip addr show "$INTERNAL_IF" 2>/dev/null | grep -q "$INTERNAL_IP"; then
    if grep -q "$INTERNAL_IP" /etc/netplan/*.yaml 2>/dev/null; then
      check_pass "$INTERNAL_IF 的 $INTERNAL_IP 为 netplan 静态配置，外部无法修改"
      STATIC_26=true
    else
      check_warn "$INTERNAL_IF 有 $INTERNAL_IP 但未在 netplan 中找到静态配置"
    fi
  else
    check_warn "$INTERNAL_IF 未配置 $INTERNAL_IP"
  fi
else
  log_info "$INTERNAL_IF 不存在，跳过检查"
fi

# 检查外网静态 IP 是否通过 netplan 配置
if [ "$EXTERNAL_IF_EXISTS" = true ]; then
  if ip addr show "$EXTERNAL_IF" 2>/dev/null | grep -q "$EXTERNAL_IP"; then
    if grep -q "$EXTERNAL_IP" /etc/netplan/*.yaml 2>/dev/null; then
      check_pass "$EXTERNAL_IF 的 $EXTERNAL_IP 为 netplan 静态配置，外部无法修改"
    else
      check_warn "$EXTERNAL_IF 有 $EXTERNAL_IP 但未在 netplan 中找到静态配置"
    fi
  else
    check_warn "$EXTERNAL_IF 未配置 $EXTERNAL_IP"
  fi
else
  log_info "$EXTERNAL_IF 不存在，跳过检查"
fi

#==========================================================
# 验证 7: 高负载下 DHCP 服务 + 跨网段转发稳定性
#==========================================================
if ! ask_continue "高负载下 DHCP 服务 + 跨网段转发稳定性"; then exit 0; fi

log_step
log_title "验证 7: 高负载下 DHCP 服务 + 跨网段转发稳定性"
log_step
echo ""

# 检查 stress-ng 是否安装
if ! command -v stress-ng &>/dev/null; then
  log_info "安装 stress-ng..."
  apt-get update -qq && apt-get install -y -qq stress-ng 2>/dev/null
  if ! command -v stress-ng &>/dev/null; then
    check_fail "无法安装 stress-ng，跳过自动化压测"
    log_warn "请手动安装: sudo apt install stress-ng"
    read -p "高负载下 DHCP 稳定且转发无延迟? [y/N]: " result
    if [ "$result" = "y" ] || [ "$result" = "Y" ]; then
      check_pass "手动验证通过"
    else
      check_fail "手动验证未通过"
    fi
    # 跳到总结
    echo ""
    log_step
    log_title "验证完成 - 总结"
    log_step
    echo ""
    echo -e "${GREEN}通过: $PASS_COUNT${NC}"
    echo -e "${RED}失败: $FAIL_COUNT${NC}"
    echo -e "${YELLOW}警告: $WARN_COUNT${NC}"
    if [ $FAIL_COUNT -eq 0 ]; then
      log_ok "所有关键验证通过！"
    else
      log_error "有 $FAIL_COUNT 项验证失败，请检查配置"
    fi
    echo ""
    exit 0
  fi
fi

DHCP_PID=$(pgrep -x dhcpd 2>/dev/null)
if [ -z "$DHCP_PID" ]; then
  check_fail "DHCP 进程未运行，无法进行稳定性测试"
else
  # 获取可用 CPU 核心数
  CPU_CORES=$(nproc)
  STRESS_DURATION=60  # 压测持续 60 秒

  #--- 7a: DHCP 服务稳定性测试 ---
  log_info "--- 7a: 高负载下 DHCP 服务稳定性 ---"
  log_info "使用 stress-ng 对所有 CPU 核心施压，验证 DHCP 服务在高负载下是否正常"
  echo ""

  log_info "启动 stress-ng 压测 (${CPU_CORES} 个工作线程, 持续 ${STRESS_DURATION}s)..."
  stress-ng --cpu ${CPU_CORES} --cpu-method all --timeout ${STRESS_DURATION}s &>/dev/null &
  STRESS_PID=$!
  sleep 1

  # 压测期间检查 dhcpd 是否存活
  DHCP_ALIVE=true
  for i in $(seq 1 $((STRESS_DURATION / 5))); do
    if ! kill -0 $DHCP_PID 2>/dev/null; then
      DHCP_ALIVE=false
      break
    fi
    sleep 5
  done

  wait $STRESS_PID 2>/dev/null || true

  if [ "$DHCP_ALIVE" = true ]; then
    check_pass "高负载下 dhcpd 进程始终存活"
  else
    check_fail "高负载下 dhcpd 进程异常退出"
  fi

  #--- 7b: 跨网段转发稳定性测试 ---
  echo ""
  log_info "--- 7b: 高负载下跨网段转发稳定性 ---"
  log_info "请确保 26 网段和 28 网段都有设备在线，用于 ping 测试"
  echo ""

  read -p "请输入 28 网段中可用于 ping 测试的 IP（留空跳过）: " PING_TARGET

  if [ -n "$PING_TARGET" ]; then
    log_info "启动 stress-ng 压测，同时测试跨网段 ping..."

    stress-ng --cpu ${CPU_CORES} --cpu-method all --timeout ${STRESS_DURATION}s &>/dev/null &
    STRESS_PID=$!
    sleep 1

    log_info "在高负载下 ping $PING_TARGET (${STRESS_DURATION}s)..."
    PING_RESULT=$(ping -c 30 -i 2 $PING_TARGET 2>/dev/null)
    PING_EXIT=$?

    wait $STRESS_PID 2>/dev/null || true

    if [ $PING_EXIT -eq 0 ]; then
      PACKET_LOSS=$(echo "$PING_RESULT" | tail -1 | grep -oP '\d+(?=% packet loss)' || echo "N/A")
      AVG_RTT=$(echo "$PING_RESULT" | tail -1 | grep -oP '\d+\.\d+(?= ms)' | tail -1 || echo "N/A")
      check_pass "跨网段 ping 成功: 丢包率 ${PACKET_LOSS}%, 平均延迟 ${AVG_RTT}ms"
    else
      check_fail "跨网段 ping 失败: $PING_TARGET 不可达"
    fi
  else
    log_info "跳过自动 ping 测试"
    read -p "高负载下跨网段通信是否正常? [y/N]: " result
    if [ "$result" = "y" ] || [ "$result" = "Y" ]; then
      check_pass "手动验证通过"
    else
      check_fail "手动验证未通过"
    fi
  fi
fi

#==========================================================
# 验证完成 - 总结
#==========================================================
echo ""
log_step
log_title "验证完成 - 总结"
log_step
echo ""
echo -e "${GREEN}通过: $PASS_COUNT${NC}"
echo -e "${RED}失败: $FAIL_COUNT${NC}"
echo -e "${YELLOW}警告: $WARN_COUNT${NC}"
if [ $FAIL_COUNT -eq 0 ]; then
  log_ok "所有关键验证通过！"
else
  log_error "有 $FAIL_COUNT 项验证失败，请检查配置"
fi
echo ""
