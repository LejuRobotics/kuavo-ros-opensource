#!/usr/bin/env bash
# ============================================================================
# X1 上位机（Thor）网络环境一键恢复脚本
#
# 用途：Thor（NVIDIA Jetson AGX Thor）重刷镜像后，一键恢复与移远板网关对接
#       所需的网络配置。
#
# 运行位置：Thor 上位机
# 用法：
#   sudo bash setup-thor-network.sh                       # 自动识别网口
#   sudo bash setup-thor-network.sh --lan-iface eth0 --dds-iface eth1
#
# 网络约定（两端一致，见移远板 setup-x1-network.sh）：
#   移远板 eth0 = 192.168.26.1（接交换机，网关 + DHCP）
#   移远板 eth1 = 192.168.28.1（直连 Thor/RK3588，DDS 专用，无 DHCP）
#
#   Thor 接交换机的那条链路 → 192.168.26.12/24 + 网关 26.1（走这里上网）
#   Thor 直连移远板那条链路 → 192.168.28.12/24（DDS，不配网关）
#
# 【本脚本要解决的核心问题】
#   与 RK3588 同源：网卡内核名（eth0/eth1）不一定对应你插线的物理口，
#   而 NetworkManager 的 profile 是按【内核口名】绑定 IP 的。IP 落到错误的
#   物理口上就会导致：数据包从错误链路发出 → 移远板 FORWARD 只放行
#   eth0→wlan0，其余 DROP → 上不了外网、端口转发（:23/:5902）失败。
#
#   所以【不要按口名判断接线】，本脚本用 DHCP 探测识别哪个口接交换机，
#   也可以用 --lan-iface/--dds-iface 显式指定。
#
# 特性：
#   - 幂等：可重复执行
#   - 自动识别哪个口接交换机（DHCP 探测：只有交换机侧有 DHCP 服务）
#   - 显式固定路由 metric，避免 NetworkManager 的自动惩罚导致优先级随机翻转
#   - 执行前备份现有 NM 连接配置
# ============================================================================

set -eo pipefail

# --------------------------- 用户配置区 ---------------------------

ROLE_NAME="Thor"

# 接交换机那条链路（26 网段）：静态 IP + 网关，负责上网
LAN_IP="192.168.26.12"
LAN_PREFIX="24"
LAN_GW="192.168.26.1"
LAN_METRIC="100"                 # 必须显著优于 wlan0，否则回包走错口

# 直连移远板那条链路（28 网段）：DDS 专用，不配网关
DDS_IP="192.168.28.12"
DDS_PREFIX="24"
DDS_METRIC="600"

# 无线上网口（测试台 WiFi）：降到最低优先级，只作兜底，不抢 26 段
WLAN_METRIC="30000"

# DNS：走移远板 NAT 出去查公共 DNS（26.1 本身不提供 DNS 服务）
DNS_SERVERS="223.5.5.5,114.114.114.114"

# NM 连接名（脚本自建，与出厂 profile 区分开）
LAN_CONN="x1-lan"
DDS_CONN="x1-dds"

# 显式指定网口（留空 = 自动识别）
FORCE_LAN_IFACE=""
FORCE_DDS_IFACE=""

# --------------------------- 颜色/日志 ---------------------------

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# --------------------------- 基础检查 ---------------------------

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "请用 root 运行: sudo bash $0"
        exit 1
    fi
}

# Thor（Jetson）出厂镜像可能用 systemd-networkd 而非 NetworkManager。
# 这里只做检测提示，不擅自改动网络栈。
check_platform() {
    if command -v nmcli &>/dev/null; then
        if ! systemctl is-active NetworkManager &>/dev/null; then
            log_warn "NetworkManager 未运行，正在启动"
            systemctl enable --now NetworkManager &>/dev/null || true
            sleep 2
        fi
        return 0
    fi

    log_error "未找到 nmcli（NetworkManager）。本脚本依赖 NetworkManager 配置 IP。"
    if systemctl is-active systemd-networkd &>/dev/null; then
        log_error "检测到本机使用 systemd-networkd，请改用 netplan 配置，或先安装并启用 NetworkManager："
        log_error "  sudo apt-get install -y network-manager && sudo systemctl enable --now NetworkManager"
    fi
    exit 1
}

# 列出所有【有线物理网口】（排除 lo / wlan / bridge / docker / veth / rmnet 等）
list_eth_ifaces() {
    local d name
    for d in /sys/class/net/*; do
        name=$(basename "$d")
        [[ -e "$d/device" ]] || continue          # 必须有物理设备（排除 lo/bridge/dummy）
        [[ -e "$d/wireless" ]] && continue         # 排除无线
        [[ "$name" == rmnet* ]] && continue
        echo "$name"
    done
}

# 取网口驱动名（比口名可靠地反映物理硬件）
iface_driver() {
    local d
    d=$(readlink -f "/sys/class/net/$1/device/driver" 2>/dev/null) || { echo ""; return; }
    basename "$d" 2>/dev/null || echo ""
}

# 判断是否 USB 网卡
is_usb_iface() {
    local path
    path=$(readlink -f "/sys/class/net/$1/device" 2>/dev/null) || return 1
    [[ "$path" == *"/usb"* ]]
}

# --------------------------- 网口识别 ---------------------------
#
# 策略（按可靠性排序）：
#   1. 显式指定 --lan-iface / --dds-iface
#   2. DHCP 探测：只有【交换机侧】有 DHCP 服务（移远板 dhcpd 只监听 eth0），
#      直连链路（28 段）不提供 DHCP。能拿到 192.168.26.x 租约的就是接交换机的口。
#   3. 驱动名兜底：USB 网卡 = 直连（28），非 USB = 接交换机（26）
#
detect_ifaces() {
    local ifaces
    ifaces=$(list_eth_ifaces)

    if [[ -z "$ifaces" ]]; then
        log_error "未找到任何有线物理网口"
        exit 1
    fi
    log_info "检测到有线网口: $(echo $ifaces | tr '\n' ' ')"

    # --- 显式指定优先 ---
    if [[ -n "$FORCE_LAN_IFACE" && -n "$FORCE_DDS_IFACE" ]]; then
        LAN_IFACE="$FORCE_LAN_IFACE"
        DDS_IFACE="$FORCE_DDS_IFACE"
        log_info "使用显式指定的网口：接交换机=${LAN_IFACE}，直连=${DDS_IFACE}"
        return 0
    fi

    # --- DHCP 探测 ---
    local found
    found=$(dhcp_probe_lan "$ifaces" || true)
    if [[ -n "$found" ]]; then
        LAN_IFACE="$found"
        log_info "DHCP 探测：${LAN_IFACE} 拿到了 26 网段租约 → 判定为【接交换机】的口"
    else
        # --- 驱动名兜底 ---
        log_warn "DHCP 探测未得出结论（移远板可能未开机），改用驱动名判断"
        local usb_ifaces="" nonusb_ifaces="" i
        for i in $ifaces; do
            if is_usb_iface "$i"; then
                usb_ifaces="$usb_ifaces $i"
            else
                nonusb_ifaces="$nonusb_ifaces $i"
            fi
        done
        usb_ifaces=${usb_ifaces# }
        nonusb_ifaces=${nonusb_ifaces# }

        if [[ -n "$usb_ifaces" && -n "$nonusb_ifaces" ]]; then
            LAN_IFACE=$(echo "$nonusb_ifaces" | awk '{print $1}')
            DDS_IFACE=$(echo "$usb_ifaces" | awk '{print $1}')
            log_info "驱动名判断：${LAN_IFACE} 非 USB（板载）→ 接交换机；${DDS_IFACE} 是 USB → 直连"
        else
            log_error "无法自动识别网口，请显式指定："
            log_error "  sudo bash $0 --lan-iface <接交换机的口> --dds-iface <直连移远板的口>"
            log_error "  提示：用下面的命令看哪个口是 USB / 板载"
            log_error "    for i in $ifaces; do echo \"\$i \$(cat /sys/class/net/\$i/address) \$(basename \$(readlink -f /sys/class/net/\$i/device/driver))\"; done"
            exit 1
        fi
    fi

    # 自动识别时，另一个口即为直连口
    if [[ -z "$FORCE_DDS_IFACE" ]]; then
        DDS_IFACE=""
        local i
        for i in $ifaces; do
            [[ "$i" == "$LAN_IFACE" ]] && continue
            DDS_IFACE="$i"
            break
        done
    else
        DDS_IFACE="$FORCE_DDS_IFACE"
    fi

    if [[ -z "$DDS_IFACE" ]]; then
        log_error "只找到一个有线网口，无法同时配置 26/28 两段。若确实只有单口，请用 --dds-iface 指定或改用单臂方案"
        exit 1
    fi

    log_info "最终分配：接交换机=${LAN_IFACE}（${LAN_IP}），直连=${DDS_IFACE}（${DDS_IP}）"
}

# 对每个网口临时起 DHCP，看谁能拿到 192.168.26.x 租约。
# 用完立刻删除临时连接，不污染现场。
dhcp_probe_lan() {
    local ifaces="$1" i tmp_conn got=""

    for i in $ifaces; do
        tmp_conn="x1-probe-$i"

        # 先把该口上已有的连接停掉，避免抢口
        local active
        active=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v d="$i" '$NF==d {print $1}')
        if [[ -n "$active" ]]; then
            while IFS= read -r c; do
                [[ -z "$c" ]] && continue
                nmcli con down "$c" &>/dev/null || true
            done <<< "$active"
            sleep 2
        fi

        nmcli con delete "$tmp_conn" &>/dev/null || true
        if ! nmcli con add type ethernet ifname "$i" con-name "$tmp_conn" ipv4.method auto &>/dev/null; then
            continue
        fi

        nmcli con up "$tmp_conn" &>/dev/null || true
        # 等 DHCP（最多 12s）
        local w=0 addr=""
        while [[ $w -lt 12 ]]; do
            sleep 1; w=$((w + 1))
            addr=$(ip -4 addr show "$i" 2>/dev/null | grep -oP 'inet \K192\.168\.26\.[0-9]+' | head -1)
            [[ -n "$addr" ]] && break
        done

        nmcli con down "$tmp_conn" &>/dev/null || true
        nmcli con delete "$tmp_conn" &>/dev/null || true

        if [[ -n "$addr" ]]; then
            got="$i"
            log_info "  DHCP 探测：${i} → ${addr}"
            break
        else
            log_info "  DHCP 探测：${i} → 未拿到 26 网段租约"
        fi
    done

    [[ -n "$got" ]] && echo "$got"
    return 0
}

# --------------------------- 配置 ---------------------------

backup_connections() {
    local f=/tmp/nm-connections-backup.$(date +%Y%m%d%H%M%S).txt
    nmcli -f ALL con show > "$f" 2>/dev/null || true
    log_info "现有 NM 连接配置已备份到 ${f}"
}

configure_lan() {
    log_info "配置接交换机链路（26 网段）：${LAN_IFACE} = ${LAN_IP}/${LAN_PREFIX}，网关 ${LAN_GW}，metric ${LAN_METRIC}"

    # 清掉可能抢这个口的其它连接
    local c
    while IFS= read -r c; do
        [[ -z "$c" ]] && continue
        [[ "$c" == "$LAN_CONN" ]] && continue
        log_warn "  删除抢占 ${LAN_IFACE} 的连接: ${c}"
        nmcli con delete "$c" &>/dev/null || true
    done <<< "$(nmcli -t -f NAME,DEVICE con show 2>/dev/null | awk -F: -v d="$LAN_IFACE" '$NF==d {print $1}')"

    nmcli con delete "$LAN_CONN" &>/dev/null || true
    nmcli con add type ethernet ifname "$LAN_IFACE" con-name "$LAN_CONN" \
        ipv4.method manual \
        ipv4.addresses "${LAN_IP}/${LAN_PREFIX}" \
        ipv4.gateway "$LAN_GW" \
        ipv4.dns "$DNS_SERVERS" \
        ipv4.ignore-auto-dns yes \
        ipv4.route-metric "$LAN_METRIC" \
        ipv4.never-default no \
        ipv6.method ignore \
        connection.autoconnect yes >/dev/null

    nmcli con up "$LAN_CONN" >/dev/null 2>&1 || log_warn "  ${LAN_IFACE} 激活失败（可能未接网线），配置已保存"
    log_info "  完成（默认路由经 ${LAN_GW}，metric ${LAN_METRIC}）"
}

configure_dds() {
    log_info "配置直连链路（28 网段）：${DDS_IFACE} = ${DDS_IP}/${DDS_PREFIX}，无网关，metric ${DDS_METRIC}"

    local c
    while IFS= read -r c; do
        [[ -z "$c" ]] && continue
        [[ "$c" == "$DDS_CONN" ]] && continue
        log_warn "  删除抢占 ${DDS_IFACE} 的连接: ${c}"
        nmcli con delete "$c" &>/dev/null || true
    done <<< "$(nmcli -t -f NAME,DEVICE con show 2>/dev/null | awk -F: -v d="$DDS_IFACE" '$NF==d {print $1}')"

    nmcli con delete "$DDS_CONN" &>/dev/null || true
    nmcli con add type ethernet ifname "$DDS_IFACE" con-name "$DDS_CONN" \
        ipv4.method manual \
        ipv4.addresses "${DDS_IP}/${DDS_PREFIX}" \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.ignore-auto-dns yes \
        ipv4.route-metric "$DDS_METRIC" \
        ipv4.never-default yes \
        ipv6.method ignore \
        connection.autoconnect yes >/dev/null

    nmcli con up "$DDS_CONN" >/dev/null 2>&1 || log_warn "  ${DDS_IFACE} 激活失败（可能未接网线），配置已保存"
    log_info "  完成（DDS 专用，不参与外网路由）"
}

# 把 WiFi 降到最低优先级，避免它抢 26 段的默认路由。
# NetworkManager 会给「非默认连接」的默认路由加 20000 惩罚，加给谁取决于激活
# 顺序 —— 这就是同一台机器两次开机 metric 会翻转的原因。
# 把两条路的 metric 拉开 >20000，惩罚加给谁都改变不了优先级。
lower_wlan_priority() {
    local found=0 name dev
    while IFS=: read -r name dev; do
        [[ "$dev" == "wlan"* || "$dev" == "wlp"* ]] || continue
        [[ -z "$name" ]] && continue
        log_info "降低无线连接优先级：${name}（${dev}）metric → ${WLAN_METRIC}"
        nmcli con mod "$name" ipv4.route-metric "$WLAN_METRIC" &>/dev/null || true
        nmcli con up "$name" &>/dev/null || true
        found=1
    done <<< "$(nmcli -t -f NAME,DEVICE con show 2>/dev/null)"
    [[ $found -eq 0 ]] && log_info "未发现无线连接，跳过"
    return 0
}

# --------------------------- 验证 ---------------------------

verify() {
    echo ""
    log_info "========== 验证 =========="
    echo "  接口地址:"
    ip -o -4 addr show 2>/dev/null | awk '$2!="lo:" {print "    " $2, $4}'
    echo ""
    echo "  默认路由:"
    ip route show default 2>/dev/null | sed 's/^/    /' || echo "    (无)"
    echo ""
    echo "  网口 → 物理硬件映射（判断接线是否配对）:"
    local i
    for i in $(list_eth_ifaces); do
        printf "    %-8s mac=%s  driver=%s  %s\n" \
            "$i" "$(cat /sys/class/net/$i/address 2>/dev/null)" \
            "$(iface_driver "$i")" \
            "$(is_usb_iface "$i" && echo 'USB' || echo '板载')"
    done
    echo ""
    if ping -c1 -W3 "$LAN_GW" &>/dev/null; then
        log_info "  网关 ${LAN_GW} 可达 ✓"
    else
        log_warn "  网关 ${LAN_GW} 不可达（移远板未开机 / 接线未通）"
    fi
    if ping -c1 -W3 223.5.5.5 &>/dev/null; then
        log_info "  外网可达 ✓"
    elif ping -c1 -W3 www.baidu.com &>/dev/null; then
        log_info "  外网（域名）可达 ✓"
    else
        log_warn "  外网暂不可达"
    fi
}

# --------------------------- 主流程 ---------------------------

main() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --lan-iface) LAN_IFACE="$2"; FORCE_LAN_IFACE="$2"; shift 2 ;;
            --dds-iface) DDS_IFACE="$2"; FORCE_DDS_IFACE="$2"; shift 2 ;;
            -h|--help)
                sed -n '1,40p' "$0" | grep '^#' | sed 's/^# \{0,1\}//'
                exit 0 ;;
            *) log_error "未知参数: $1"; exit 1 ;;
        esac
    done

    check_root
    check_platform
    log_info "开始恢复 ${ROLE_NAME} 网络配置..."
    echo ""

    backup_connections
    detect_ifaces
    echo ""
    configure_lan
    configure_dds
    lower_wlan_priority
    nmcli con reload &>/dev/null || true
    verify

    log_info "完成！配置已持久化，重启后依然生效。"
}

main "$@"
