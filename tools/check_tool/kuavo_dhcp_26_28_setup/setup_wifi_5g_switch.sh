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

log_info "开始配置 Wifi/5G 自动切换机制..."

#------------------------------
# 1. 检测接口
#------------------------------
WIFI_IFACE=$(ip link show | grep -E "wlp|wlan" | head -1 | awk -F: '{print $2}' | tr -d ' ')
# 优先匹配名为 wwan* 或 wwx2* 的 5G 接口
WWAN_IFACE=$(ip link show | grep -E "^[0-9]+: (wwan|wwx2)" | head -1 | awk -F: '{print $2}' | tr -d ' ')
# 如果没检测到，尝试直接使用 wwan0
if [ -z "$WWAN_IFACE" ]; then
  if ip link show wwan0 &>/dev/null; then
    WWAN_IFACE="wwan0"
  fi
fi

if [ -z "$WIFI_IFACE" ]; then
  log_warn "未检测到 Wifi 接口，退出"
  exit 1
elif [ -z "$WWAN_IFACE" ]; then
  log_warn "未检测到 5G 接口，退出"
  exit 1
fi

log_info "检测到接口：Wifi=$WIFI_IFACE, 5G=$WWAN_IFACE"

#------------------------------
# 2. 创建网络质量检测脚本
#------------------------------
log_info "创建网络质量检测脚本..."

cat > /usr/local/bin/network_quality_check.sh <<'EOS'
#!/bin/bash
# Wifi/5G 网络质量检测与自动切换脚本

LOG_FILE="/var/log/network-quality-check.log"
WIFI_IFACE=""
WWAN_IFACE=""

# 日志函数
# 注意：输出到 stderr，避免被 $() 捕获影响返回值
log_msg() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE" >&2
}

# 自动检测 Wifi 接口名（wlp*/wlan*）
detect_wifi_iface() {
  ip link show | grep -E "wlp|wlan" | head -1 | awk -F: '{print $2}' | tr -d ' '
}

# 自动检测 5G 接口名：优先匹配 wwan* 或 wwx2*，否则退回 wwan0（如果存在）
detect_wwan_iface() {
  local iface
  iface=$(ip link show | grep -E "^[0-9]+: (wwan|wwx2)" | head -1 | awk -F: '{print $2}' | tr -d ' ')
  if [ -z "$iface" ] && ip link show wwan0 &>/dev/null; then
    iface="wwan0"
  fi
  echo "$iface"
}

# 如果未通过环境变量传入，则在脚本运行时动态检测接口名
if [ -z "$WIFI_IFACE" ]; then
  WIFI_IFACE=$(detect_wifi_iface)
fi

if [ -z "$WWAN_IFACE" ]; then
  WWAN_IFACE=$(detect_wwan_iface)
fi

if [ -z "$WIFI_IFACE" ] || [ -z "$WWAN_IFACE" ]; then
  log_msg "无法自动检测到 Wifi 或 5G 接口名 (WIFI_IFACE='$WIFI_IFACE', WWAN_IFACE='$WWAN_IFACE')，本次跳过检测与切换。"
  exit 0
fi

# 检测接口网络质量并返回延迟（毫秒）
# 返回格式：延迟值（毫秒），如果失败返回 -1
# 使用方式：LATENCY=$(check_interface_quality "<iface_name>")
check_interface_quality() {
  local iface=$1
  
  # 检查接口是否存在
  if [ -z "$iface" ]; then
    log_msg "接口 $iface 名称为空"
    printf "-1"
    return
  fi
  
  if ! ip link show "$iface" &>/dev/null; then
    log_msg "接口 $iface 不存在"
    printf "-1"
    return
  fi
  
  # 检查接口状态
  # 注意：wwan/5G 接口可能显示 UNKNOWN 状态但实际可用，需要特殊处理
  local state=$(ip link show "$iface" 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}' || echo "UNKNOWN")
  
  # 对于 wwan 接口，允许 UNKNOWN 状态（5G 模块可能显示此状态但实际可用）
  if echo "$iface" | grep -qE "wwan|wwx"; then
      if [ "$state" != "UP" ] && [ "$state" != "UNKNOWN" ]; then
        log_msg "接口 $iface (5G) 状态异常: $state，不允许继续"
        printf "-1"
        return
      fi
    else
      # 对于其他接口（如 Wifi），必须 UP
      if [ "$state" != "UP" ]; then
        log_msg "接口 $iface 未 UP，当前状态: $state"
        printf "-1"
        return
      fi
  fi
  
  # 检查接口是否有 IP 地址
  if ! ip addr show "$iface" 2>/dev/null | grep -q "inet "; then
    log_msg "接口 $iface 没有 IP 地址"
    printf "-1"
    return
  fi
  
  # 尝试 ping 网关（如果存在），先检查网关连通性
  local gateway=$(ip route | grep "$iface" | grep default | awk '{print $3}' | head -1)
  if [ -n "$gateway" ]; then
  # 指定接口 ping 网关，避免多默认路由时走错出口
  local gateway_ping_result=$(ping -c 1 -W 2 -I "$iface" "$gateway" 2>&1)
    local gateway_ping_code=$?
    if [ $gateway_ping_code -ne 0 ]; then
      log_msg "接口 $iface ping 网关失败（返回码: $gateway_ping_code）:"
      echo "$gateway_ping_result" | while IFS= read -r line; do
        log_msg "  $line"
      done
      printf "-1"
      return
    fi
  fi
  
  # 尝试 ping 外网（8.8.8.8）并获取延迟
  # 使用 -I 指定接口，-c 1 发送1个包，-W 3 超时3秒
  # 从输出中提取时间（time=XX.X ms）
  local ping_result
  local ping_exit_code
  ping_result=$(ping -c 1 -W 3 -I "$iface" 8.8.8.8 2>&1)
  ping_exit_code=$?
  
  if [ $ping_exit_code -ne 0 ]; then
    log_msg "接口 $iface ping 失败，返回码: $ping_exit_code"
    printf "-1"
    return
  fi
  
  # 提取延迟值（毫秒）
  # ping 输出格式可能是：
  #   64 bytes from 8.8.8.8: icmp_seq=1 ttl=111 time=288 ms
  #   或 time=25.3 ms
  #   或 time<1ms
  # 注意：ping 输出的 time=XX.X 单位是毫秒
  local latency
  
  # 方法1：匹配 time=XXX ms 或 time=XX.X ms 格式
  # 使用 sed 提取 time= 后面的数字（整数或小数），去掉可能的空格
  latency=$(echo "$ping_result" | grep -oE "time=[0-9.]+" | sed 's/time=//' | head -1)
  
  # 如果没匹配到，尝试 time<1ms 格式
  if [ -z "$latency" ]; then
      latency=$(echo "$ping_result" | grep -oE "time<[0-9.]+" | sed 's/time<//' | head -1)
      if [ -z "$latency" ]; then
        log_msg "接口 $iface 无法从 ping 输出中提取延迟值，尝试提取的格式: time=[0-9.]+ 和 time<[0-9.]+"
        log_msg "grep 结果: $(echo "$ping_result" | grep -oE "time=[0-9.]+" || echo '无匹配')"
        printf "-1"
        return
      fi
      # 如果延迟小于1ms，返回1
    printf "1"
    return
  fi
  
  # 如果提取到延迟值，转换为整数毫秒（四舍五入）
  if [ -n "$latency" ]; then
    # 验证是否为有效数字（整数或小数）
    if echo "$latency" | grep -qE '^[0-9]+\.?[0-9]*$'; then
      # 转换为整数（四舍五入）
      local latency_ms=$(echo "$latency" | awk '{printf "%.0f", $1}')
      # 确保返回的是纯数字（去除所有非数字字符）
      latency_ms=$(echo "$latency_ms" | tr -d '\n\r ' | grep -oE '^[0-9]+$' || echo "-1")
      # 使用 printf 确保输出纯数字，不换行
      printf "%d" "$latency_ms"
      return
    else
      log_msg "接口 $iface 提取到的延迟值格式无效: '$latency'"
    fi
  fi
  
  # 如果都失败了，返回 -1
  log_msg "接口 $iface 延迟提取完全失败"
  printf "-1"
}

# 设置默认路由的 metric（不删除旧路由，避免断开现有连接）
set_default_route_metric() {
  local iface=$1
  local metric=$2
  
  # 获取该接口的网关
  local gateway=$(ip route | grep "$iface" | grep default | awk '{print $3}' | head -1)
  
  # 幂等：如果默认路由已是相同 dev+gateway+metric，则跳过，避免重复写入
  if [ -n "$gateway" ]; then
    if ip route show default via "$gateway" dev "$iface" metric "$metric" 2>/dev/null | grep -q "^default"; then
      return 0
    fi
  else
    if ip route show default dev "$iface" metric "$metric" 2>/dev/null | grep -q "^default"; then
      return 0
    fi
  fi
  
  if [ -n "$gateway" ]; then
    # 使用 replace 或 add，不删除旧路由
    # 这样已建立的连接（如 SSH）不会断开，只是新连接会使用新的路由
    if ip route show default dev "$iface" &>/dev/null; then
      # 如果该接口已有默认路由，使用 replace 更新 metric
      ip route replace default via "$gateway" dev "$iface" metric "$metric" 2>/dev/null || true
      log_msg "更新 $iface 默认路由，metric=$metric, gateway=$gateway"
    else
      # 如果该接口没有默认路由，添加新的（保留其他接口的路由）
      ip route add default via "$gateway" dev "$iface" metric "$metric" 2>/dev/null || true
      log_msg "添加 $iface 默认路由，metric=$metric, gateway=$gateway"
    fi
  else
    # 如果没有网关，尝试直接通过接口
    if ip route show default dev "$iface" &>/dev/null; then
      ip route replace default dev "$iface" metric "$metric" 2>/dev/null || true
      log_msg "更新 $iface 默认路由，metric=$metric（无网关，不影响现有连接）"
    else
      ip route add default dev "$iface" metric "$metric" 2>/dev/null || true
      log_msg "添加 $iface 默认路由，metric=$metric（无网关，保留其他路由）"
    fi
  fi
}

# 主逻辑
log_msg "---"

# 配置参数
LATENCY_THRESHOLD=500        # 延迟阈值（毫秒）：超过此值认为质量差
HYSTERESIS_THRESHOLD=200     # 防抖阈值（毫秒）：只有当新接口延迟比当前接口低超过此值时才切换
STATE_FILE="/var/run/network-primary-interface.state"
DECISION_HISTORY_FILE="/var/run/network-decision-history.txt"
REQUIRED_CONSECUTIVE=4       # 需要连续多少次相同决策才切换

# 确保状态文件目录存在
mkdir -p "$(dirname "$STATE_FILE")" 2>/dev/null || true

# 单网口模式（不做 ping）：
# 如果 Wifi/5G 其中一个接口不存在或没有 IP，则只走当前可用的那个，不再持续 ping 另一个
WIFI_AVAILABLE=false
WWAN_AVAILABLE=false
if ip link show "$WIFI_IFACE" &>/dev/null && ip addr show "$WIFI_IFACE" 2>/dev/null | grep -q "inet "; then
  WIFI_AVAILABLE=true
fi
if ip link show "$WWAN_IFACE" &>/dev/null && ip addr show "$WWAN_IFACE" 2>/dev/null | grep -q "inet "; then
  WWAN_AVAILABLE=true
fi

if [ "$WIFI_AVAILABLE" = true ] && [ "$WWAN_AVAILABLE" != true ]; then
  set_default_route_metric "$WIFI_IFACE" 100
  echo "$WIFI_IFACE" > "$STATE_FILE"
  > "$DECISION_HISTORY_FILE" 2>/dev/null || true
  log_msg "仅检测到 Wifi ($WIFI_IFACE)，5G 不可用（接口不存在或无 IP）。单网口模式：不做 ping，固定走 Wifi。"
  log_msg "---"
  exit 0
elif [ "$WWAN_AVAILABLE" = true ] && [ "$WIFI_AVAILABLE" != true ]; then
  set_default_route_metric "$WWAN_IFACE" 100
  echo "$WWAN_IFACE" > "$STATE_FILE"
  > "$DECISION_HISTORY_FILE" 2>/dev/null || true
  log_msg "仅检测到 5G ($WWAN_IFACE)，Wifi 不可用（接口不存在或无 IP）。单网口模式：不做 ping，固定走 5G。"
  log_msg "---"
  exit 0
elif [ "$WWAN_AVAILABLE" != true ] && [ "$WIFI_AVAILABLE" != true ]; then
  log_msg "Wifi/5G 都不可用（接口不存在或无 IP），跳过 ping 与切换。"
  log_msg "---"
  exit 0
fi

# 检查 Wifi 延迟
WIFI_LATENCY=$(check_interface_quality "$WIFI_IFACE")
# 清理返回值：去除换行符、回车符和空格
WIFI_LATENCY=$(echo "$WIFI_LATENCY" | tr -d '\n\r ' | head -1)

WIFI_OK=false
# 检查是否为有效数字且 >= 0
if [ -n "$WIFI_LATENCY" ] && echo "$WIFI_LATENCY" | grep -qE '^[0-9]+$' && [ "$WIFI_LATENCY" -ge 0 ] 2>/dev/null; then
  WIFI_OK=true
  WIFI_LATENCY_SEC=$(awk "BEGIN {printf \"%.2f\", $WIFI_LATENCY/1000}")
  log_msg "Wifi ($WIFI_IFACE) 网络质量正常，延迟: ${WIFI_LATENCY_SEC}s (${WIFI_LATENCY}ms)"
else
  log_msg "Wifi ($WIFI_IFACE) 网络质量异常（无法连接或ping失败），延迟值: '${WIFI_LATENCY}'"
  WIFI_LATENCY=999999
fi

# 检查 5G 延迟
WWAN_LATENCY=$(check_interface_quality "$WWAN_IFACE")
# 清理返回值：去除换行符、回车符和空格
WWAN_LATENCY=$(echo "$WWAN_LATENCY" | tr -d '\n\r ' | head -1)

WWAN_OK=false
# 检查是否为有效数字且 >= 0
if [ -n "$WWAN_LATENCY" ] && echo "$WWAN_LATENCY" | grep -qE '^[0-9]+$' && [ "$WWAN_LATENCY" -ge 0 ] 2>/dev/null; then
  WWAN_OK=true
  WWAN_LATENCY_SEC=$(awk "BEGIN {printf \"%.2f\", $WWAN_LATENCY/1000}")
  log_msg "5G ($WWAN_IFACE) 网络质量正常，延迟: ${WWAN_LATENCY_SEC}s (${WWAN_LATENCY}ms)"
else
  log_msg "5G ($WWAN_IFACE) 网络质量异常（无法连接或ping失败），延迟值: '${WWAN_LATENCY}'"
  # 尝试数值比较看具体错误
  if [ "$WWAN_LATENCY" -ge 0 ] 2>&1; then
    log_msg "数值比较成功，但条件判断失败，可能是逻辑问题"
  else
    log_msg "数值比较失败: $?"
  fi
  WWAN_LATENCY=999999
fi

# 读取上一次的主路由接口
CURRENT_PRIMARY=""
if [ -f "$STATE_FILE" ]; then
  CURRENT_PRIMARY=$(cat "$STATE_FILE" 2>/dev/null | head -1)
fi

# 确定应该使用哪个接口作为主路由
SELECTED_IFACE=""
SELECTED_LATENCY=999999
SELECTED_OK=false

# 策略1：如果只有一个接口正常，使用它
# 注意：这里必须显式加括号分组，避免 bash 的 &&/|| 优先级导致误判
if [[ "$WIFI_OK" == true && "$WIFI_LATENCY" -lt "$LATENCY_THRESHOLD" && ( "$WWAN_OK" != true || "$WWAN_LATENCY" -ge "$LATENCY_THRESHOLD" ) ]]; then
  SELECTED_IFACE="$WIFI_IFACE"
  SELECTED_LATENCY="$WIFI_LATENCY"
  SELECTED_OK=true
  log_msg "决策：只有 Wifi 正常，选择 Wifi"
elif [[ "$WWAN_OK" == true && "$WWAN_LATENCY" -lt "$LATENCY_THRESHOLD" && ( "$WIFI_OK" != true || "$WIFI_LATENCY" -ge "$LATENCY_THRESHOLD" ) ]]; then
  SELECTED_IFACE="$WWAN_IFACE"
  SELECTED_LATENCY="$WWAN_LATENCY"
  SELECTED_OK=true
  log_msg "决策：只有 5G 正常，选择 5G"
# 策略2：如果两个都正常，选择延迟更低的
elif [ "$WIFI_OK" = true ] && [ "$WIFI_LATENCY" -lt "$LATENCY_THRESHOLD" ] && \
     [ "$WWAN_OK" = true ] && [ "$WWAN_LATENCY" -lt "$LATENCY_THRESHOLD" ]; then
  if [ "$WIFI_LATENCY" -lt "$WWAN_LATENCY" ]; then
    SELECTED_IFACE="$WIFI_IFACE"
    SELECTED_LATENCY="$WIFI_LATENCY"
    log_msg "决策：两者都正常，Wifi 延迟更低 (${WIFI_LATENCY}ms < ${WWAN_LATENCY}ms)，选择 Wifi"
  else
    SELECTED_IFACE="$WWAN_IFACE"
    SELECTED_LATENCY="$WWAN_LATENCY"
    log_msg "决策：两者都正常，5G 延迟更低 (${WWAN_LATENCY}ms < ${WIFI_LATENCY}ms)，选择 5G"
  fi
  SELECTED_OK=true
fi

# 如果都没有选择，说明都异常
if [ -z "$SELECTED_IFACE" ]; then
  log_msg "警告：Wifi 和 5G 都异常或延迟过高，无法切换"
  log_msg "---"
  exit 0
fi

# 判断本次应该使用哪个接口（决策逻辑）
DECISION_IFACE=""

# 如果当前没有主路由，直接使用选择的接口
if [ -z "$CURRENT_PRIMARY" ]; then
  DECISION_IFACE="$SELECTED_IFACE"
  log_msg "当前无主路由，决策使用 $SELECTED_IFACE"
# 如果当前主路由就是选择的接口，不需要切换
elif [ "$CURRENT_PRIMARY" = "$SELECTED_IFACE" ]; then
  DECISION_IFACE="$SELECTED_IFACE"
  log_msg "当前主路由已是 $SELECTED_IFACE，决策保持不变"
# 如果当前主路由异常，决策切换到新接口
elif [ "$CURRENT_PRIMARY" = "$WIFI_IFACE" ] && ([ "$WIFI_OK" != true ] || [ "$WIFI_LATENCY" -ge "$LATENCY_THRESHOLD" ]); then
  DECISION_IFACE="$SELECTED_IFACE"
  log_msg "当前主路由 Wifi 异常，决策切换到 $SELECTED_IFACE"
elif [ "$CURRENT_PRIMARY" = "$WWAN_IFACE" ] && ([ "$WWAN_OK" != true ] || [ "$WWAN_LATENCY" -ge "$LATENCY_THRESHOLD" ]); then
  DECISION_IFACE="$SELECTED_IFACE"
  log_msg "当前主路由 5G 异常，决策切换到 $SELECTED_IFACE"
# 如果新接口明显更好（延迟差超过防抖阈值），决策切换
else
  # 获取当前主路由的延迟
  CURRENT_LATENCY=999999
  if [ "$CURRENT_PRIMARY" = "$WIFI_IFACE" ]; then
    CURRENT_LATENCY="$WIFI_LATENCY"
  elif [ "$CURRENT_PRIMARY" = "$WWAN_IFACE" ]; then
    CURRENT_LATENCY="$WWAN_LATENCY"
  fi
  
  # 只有当新接口延迟比当前接口低超过防抖阈值时才决策切换
  LATENCY_DIFF=$((CURRENT_LATENCY - SELECTED_LATENCY))
  if [ "$LATENCY_DIFF" -gt "$HYSTERESIS_THRESHOLD" ]; then
    DECISION_IFACE="$SELECTED_IFACE"
    log_msg "新接口延迟明显更好 (差 ${LATENCY_DIFF}ms > ${HYSTERESIS_THRESHOLD}ms)，决策切换到 $SELECTED_IFACE"
  else
    DECISION_IFACE="$CURRENT_PRIMARY"
    log_msg "新接口延迟优势不明显 (差 ${LATENCY_DIFF}ms <= ${HYSTERESIS_THRESHOLD}ms)，决策保持当前路由 $CURRENT_PRIMARY"
  fi
fi

# 记录本次决策到历史文件
echo "$DECISION_IFACE" >> "$DECISION_HISTORY_FILE"

# 只保留最近 REQUIRED_CONSECUTIVE 次记录
tail -n "$REQUIRED_CONSECUTIVE" "$DECISION_HISTORY_FILE" > "$DECISION_HISTORY_FILE.tmp" 2>/dev/null
mv "$DECISION_HISTORY_FILE.tmp" "$DECISION_HISTORY_FILE" 2>/dev/null || true

# 检查最近 REQUIRED_CONSECUTIVE 次是否都是同一个决策
CONSECUTIVE_COUNT=$(tail -n "$REQUIRED_CONSECUTIVE" "$DECISION_HISTORY_FILE" 2>/dev/null | grep -c "^${DECISION_IFACE}$" || echo "0")

log_msg "本次决策: $DECISION_IFACE，最近 ${REQUIRED_CONSECUTIVE} 次中相同决策次数: $CONSECUTIVE_COUNT"

# 只有当连续 REQUIRED_CONSECUTIVE 次都是同一个决策，且与当前主路由不同时，才执行切换
SHOULD_SWITCH=false
if [ "$CONSECUTIVE_COUNT" -ge "$REQUIRED_CONSECUTIVE" ] && [ "$DECISION_IFACE" != "$CURRENT_PRIMARY" ]; then
  SHOULD_SWITCH=true
  log_msg "连续 ${REQUIRED_CONSECUTIVE} 次决策都是 $DECISION_IFACE，执行切换"
else
  SHOULD_SWITCH=false
  if [ "$DECISION_IFACE" = "$CURRENT_PRIMARY" ]; then
    log_msg "决策与当前路由相同，无需切换"
  else
    log_msg "需要连续 ${REQUIRED_CONSECUTIVE} 次相同决策才切换，当前: $CONSECUTIVE_COUNT/${REQUIRED_CONSECUTIVE}（防抖）"
  fi
fi

# 执行路由切换
if [ "$SHOULD_SWITCH" = true ]; then
  # 设置主路由
  set_default_route_metric "$DECISION_IFACE" 100
  
  # 设置备用路由（如果另一个接口也正常）
  if [ "$DECISION_IFACE" = "$WIFI_IFACE" ] && [ "$WWAN_OK" = true ] && [ "$WWAN_LATENCY" -lt "$LATENCY_THRESHOLD" ]; then
    set_default_route_metric "$WWAN_IFACE" 200
    log_msg "5G 设置为备用路由（metric 200）"
  elif [ "$DECISION_IFACE" = "$WWAN_IFACE" ] && [ "$WIFI_OK" = true ] && [ "$WIFI_LATENCY" -lt "$LATENCY_THRESHOLD" ]; then
    set_default_route_metric "$WIFI_IFACE" 200
    log_msg "Wifi 设置为备用路由（metric 200）"
  fi
  
  # 保存当前主路由到状态文件
  echo "$DECISION_IFACE" > "$STATE_FILE"
  # 切换后清空历史记录，重新开始计数
  > "$DECISION_HISTORY_FILE"
  log_msg "已切换到 $DECISION_IFACE 作为主路由（metric 100，延迟 ${SELECTED_LATENCY}ms）"
else
  # 即使不切换，也确保路由配置正确（可能只是更新 metric）
  # 使用当前主路由，而不是决策的接口
  if [ -n "$CURRENT_PRIMARY" ]; then
    if [ "$CURRENT_PRIMARY" = "$WIFI_IFACE" ]; then
      set_default_route_metric "$WIFI_IFACE" 100
      if [ "$WWAN_OK" = true ] && [ "$WWAN_LATENCY" -lt "$LATENCY_THRESHOLD" ]; then
        set_default_route_metric "$WWAN_IFACE" 200
      fi
    elif [ "$CURRENT_PRIMARY" = "$WWAN_IFACE" ]; then
      set_default_route_metric "$WWAN_IFACE" 100
      if [ "$WIFI_OK" = true ] && [ "$WIFI_LATENCY" -lt "$LATENCY_THRESHOLD" ]; then
        set_default_route_metric "$WIFI_IFACE" 200
      fi
    fi
  fi
fi
EOS

chmod +x /usr/local/bin/network_quality_check.sh
log_ok "网络质量检测脚本已创建: /usr/local/bin/network_quality_check.sh"

#------------------------------
# 3. 创建 systemd 服务
#------------------------------
log_info "创建 systemd 服务..."

cat > /etc/systemd/system/network-quality-check.service <<'EOF'
[Unit]
Description=Network Quality Check and Auto Switch (Wifi/5G)
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/network_quality_check.sh
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# 创建 systemd 定时器（每5秒检查一次）
cat > /etc/systemd/system/network-quality-check.timer <<'EOF'
[Unit]
Description=Network Quality Check Timer (Wifi/5G)
Requires=network-quality-check.service

[Timer]
OnBootSec=1min
OnUnitActiveSec=5s
AccuracySec=1s

[Install]
WantedBy=timers.target
EOF

log_ok "systemd 服务已创建"

#------------------------------
# 3.1. 配置 CPU 亲和性（绑定到 CPU 15）
#------------------------------
log_info "配置 network-quality-check 服务 CPU 亲和性（绑定到 CPU 15）..."

mkdir -p /etc/systemd/system/network-quality-check.service.d/

cat > /etc/systemd/system/network-quality-check.service.d/cpu-affinity.conf <<'EOF'
[Service]
CPUAffinity=15
EOF

log_ok "CPU 亲和性配置完成（绑定到 CPU 15）"

#------------------------------
# 4. 创建日志目录和日志轮转配置
#------------------------------
log_info "创建日志目录..."
mkdir -p /var/log
touch /var/log/network-quality-check.log
chmod 644 /var/log/network-quality-check.log
log_ok "日志目录已创建"

log_info "配置日志轮转（自动清理旧日志）..."
cat > /etc/logrotate.d/network-quality-check <<'EOF'
/var/log/network-quality-check.log {
    daily
    rotate 3
    compress
    delaycompress
    missingok
    notifempty
    create 644 root root
    copytruncate
}
EOF

log_ok "日志轮转配置完成（保留最近3天，自动压缩）"

#------------------------------
# 5. 启用并启动服务
#------------------------------
log_info "启用并启动服务..."
systemctl daemon-reload
systemctl enable network-quality-check.timer
systemctl start network-quality-check.timer
log_ok "服务已启用并启动（开机自启已配置）"

#------------------------------
# 6. 验证与提示
#------------------------------
log_info "验证服务状态..."
if systemctl is-active --quiet network-quality-check.timer; then
  log_ok "定时器运行正常"
else
  log_warn "定时器可能未正常启动，请检查: systemctl status network-quality-check.timer"
fi

echo ""
log_ok "Wifi/5G 自动切换机制配置完成！"
echo ""
log_info "配置信息："
log_info "  - 检测脚本: /usr/local/bin/network_quality_check.sh"
log_info "  - 日志文件: /var/log/network-quality-check.log（自动轮转，保留3天）"
log_info "  - 检查间隔: 5秒"
log_info "  - 接口: Wifi=$WIFI_IFACE, 5G=$WWAN_IFACE"
log_info "  - 延迟阈值: 500ms（超过此值认为质量差）"
log_info "  - 防抖阈值: 200ms（延迟差超过此值才切换）"
log_info "  - 连续决策: 需要连续4次相同决策才切换"
log_info "  - CPU 亲和性: 绑定到 CPU 15（与 DHCP 服务相同）"
log_info "  - 开机自启: 已启用（开机后1分钟开始检查）"
log_info "  - 优先级: 主路由 metric=100，备用路由 metric=200"
echo ""
log_info "常用命令："
log_info "  查看定时器状态: systemctl status network-quality-check.timer"
log_info "  查看实时日志: tail -f /var/log/network-quality-check.log"
log_info "  查看当前路由: ip route | grep default"
log_info "  手动触发检查: sudo /usr/local/bin/network_quality_check.sh"
log_info "  停止服务: sudo systemctl stop network-quality-check.timer"
log_info "  禁用服务: sudo systemctl disable network-quality-check.timer"
echo ""
