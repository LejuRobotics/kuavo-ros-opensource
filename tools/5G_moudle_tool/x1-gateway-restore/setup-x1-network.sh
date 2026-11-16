#!/usr/bin/env bash
# ============================================================================
# Kuavo X1 移远板网络环境一键恢复脚本
#
# 用途：移远板（QCS6490 / SG560D-WF）重刷新镜像或更换新板子后，一键恢复
#       我们在工单 #2169(DHCP) / #2171(iptables NAT) / #2173(SSH/VNC 端口转发)
#       / #2172(联调) 中完成的全部网络配置，并把出厂用户改名为交付用户。
#
# 运行位置：移远板（头部控制器）
# 用法：
#   sudo bash setup-x1-network.sh            # 全量恢复（含改用户 + VNC + 射频自启）
#   sudo bash setup-x1-network.sh --no-vnc   # 跳过 VNC 配置
#   sudo bash setup-x1-network.sh --no-rf    # 跳过蜂窝射频开机自启
#
# 用户/密码写死为 leju_x1 / leju_x1（交付约定），无需交互输入。
#
# 特性：
#   - 幂等：可重复执行，不会重复叠加规则
#   - 安全：只新增/修改本方案涉及的配置，不碰用户其他数据
#   - 自动清理旧网络规划残留（eth1 / 192.168.18.x / 出厂 bridge0/ppp0 转发残留）
#   - 出厂用户 quectel 直接【改名】为 leju_x1（uid / 家目录全部内容 / 附加组原样保留，
#     改名后系统中不再存在 quectel），并设置密码；VNC 复用该用户
#   - VNC 依赖（TigerVNC + xfce4 + dbus-x11）缺失时自动 apt 安装
#
# 注意：
#   - 出口接口在运行时自动检测（WAN_CANDIDATES：wlan0 → rmnet_data0），只认【真正
#     拿到 IP】的口。按设计「默认走 WIFI，WIFI 不可用时才走 5G」：wlan0（WiFi）优先，
#     5G（rmnet_data0）兜底。出口变了请重跑本脚本重建 NAT/DNAT（规则绑在具体出口上）。
#   - 【重要】5G 拨号成功后，rmnet_data0 会被 QCMAP/quectel-CM 自动加上
#     `default via ... dev rmnet_data0 metric 500`，而 wlan0 是 metric 334（DHCP 派生）。
#     334 < 500，所以默认路由本来就走 wlan0，与上面的优先级一致，无需额外调 metric。
#     但若某天 WiFi 的 metric 变得比 500 大，5G 会抢走默认路由 —— 用
#     `ip route show default` 确认后再跑本脚本。
#   - 重刷镜像后首次运行，请先确认 WiFi（wlan0）已联网，否则出口检测可能失败。
#   - apt 源不可达时自动切换到国内镜像（装 VNC 依赖用）。
#   - 登录用户默认 leju_x1 / leju_x1，VNC 密码默认与登录密码一致（均为前 8 字符）。
#   - 蜂窝射频：模块重启后 AT+CFUN 停在 0（射频关），脚本会装一个 oneshot 服务
#     开机下发 AT+CFUN=1（幂等，已是 1 则跳过），让 5G 能作为兜底出口自动就绪。
#
# 约定（两端一致，见 x1-network-port-convention）：
#   设备      IP               SSH             VNC
#   移远板    192.168.26.1     自身 22         自身 5901
#   Thor      192.168.26.12    移远板 23 转发   移远板 5902 转发
#   RK3588    192.168.26.13    移远板 24 转发   移远板 5903 转发
#
#   28 网段（DDS 专用，独立链路）：
#   移远板 eth1 = 192.168.28.1   Thor = 192.168.28.12   RK3588 = 192.168.28.13
#   28 网段流量只在内部转发（DDS），不转发到 26 网段/外网
#
#   网络设计确认（2026-08-19 联调验证）：
#   - 26 网段：控制/SSH/VNC/外网。移远板 eth0 = 26.1（网关+DHCP），
#     Thor=26.12、RK3588=26.13（静态），DHCP 池 26.100~200。
#   - 28 网段：DDS 专用。移远板 eth1 = 28.1，Thor=28.12、RK3588=28.13（静态），
#     不提供 DHCP（dhcpd.conf 已声明空 subnet 隔离，否则 dhcpd 绑定 0.0.0.0:67
#     会在 eth1 上响应 DHCP 分配 26 网段地址）。
#   - 设备接入：机器连交换机 → DHCP 自动分配 26.x；连移远板 28 网段需静态配 28.x。
#   - 28 网段数据只在 28 网段内部转发，不转发到 26/外网（FORWARD DROP + 无 eth1→eth0 规则）。
#   - 静态 IP(26.12/26.13/28.12/28.13) 与 DHCP 池(26.100~200) 不重叠，互不冲突。
# ============================================================================

set -eo pipefail
set +u   # detect_wan_iface 可能遇到未定义

# --------------------------- 用户配置区 ---------------------------

LAN_IFACE="eth0"                 # LAN 口（连接交换机 → Thor/RK3588）
LAN_IP="192.168.26.1"            # 移远板 LAN 侧固定 IP（网关）
LAN_NETMASK="255.255.255.0"
LAN_PREFIX="24"                  # CIDR 前缀长度（nmcli ipv4.addresses 只接受 1-32 的数字）
LAN_SUBNET="192.168.26.0/24"
DHCP_RANGE_START="192.168.26.100"
DHCP_RANGE_END="192.168.26.200"

# 出口候选（按优先级）：WiFi 优先，5G 兜底（设计约定「默认走 WIFI，WIFI 不可用时才走 5G」）
WAN_CANDIDATES=("wlan0" "rmnet_data0")

# Thor / RK3588 固定 IP（静态约定，非 DHCP 分配）
IP_THOR="192.168.26.12"
IP_RK="192.168.26.13"

# 端口转发：外部端口:目标IP:目标端口
#   SSH: 23→Thor:22, 24→RK3588:22
#   VNC: 5902→Thor:5902, 5903→RK3588:5903（直连，非中转）
DNAT_RULES=(
    "23:${IP_THOR}:22"
    "24:${IP_RK}:22"
    "5902:${IP_THOR}:5902"
    "5903:${IP_RK}:5903"
)

# --------------------------- 28 网段（DDS 专用） ---------------------------
# DDS 数据专用链路，与 26 网段（控制/SSH/VNC/外网）物理与逻辑隔离：
#   - 3588 eth0 直连移远板 eth1（或经交换机），独立链路
#   - 28 网段流量只在内部转发，不转发到 26 网段，也不出外网
DDS_IFACE="eth1"                 # 移远板 DDS 口
DDS_IP="192.168.28.1"            # 移远板 28 网段 IP
DDS_NETMASK="255.255.255.0"
DDS_PREFIX="24"                  # CIDR 前缀长度（nmcli ipv4.addresses 只接受 1-32 的数字）
DDS_SUBNET="192.168.28.0/24"
DDS_CONN="dds-lan"               # NM 连接名
DDS_IP_THOR="192.168.28.12"      # Thor 28 网段 IP（约定，需两端一致）
DDS_IP_RK="192.168.28.13"        # RK3588 28 网段 IP（约定，需两端一致）

# --------------------------- 登录用户（写死 leju_x1，交付约定） ---------------------------
# 换板/重刷镜像后，把出厂用户【改名】为 leju_x1 并设置密码。
# 用改名而非新建：uid / 家目录全部内容 / 附加组（tty,sudo,video,render）原样保留，
# 改名后系统中不再存在出厂用户。
# 用户名与密码写死为 leju_x1（交付固定约定，不做环境变量覆盖/交互输入）。
LOGIN_USER="leju_x1"
LOGIN_PASSWORD="leju_x1"
# 出厂用户名（改名来源）
LEGACY_USER="quectel"
# 家目录根路径（改名后家目录 = ${HOME_ROOT}/${LOGIN_USER}）
HOME_ROOT="/home"

# VNC（移远板自身桌面，复用登录用户 LOGIN_USER，密码同为 leju_x1）
VNC_USER="$LOGIN_USER"
VNC_PASSWORD="leju_x1"
VNC_DISPLAY=":2"                 # :1 被 gdm 占用
VNC_PORT="5901"
VNC_GEOMETRY="1280x720"
VNC_ENABLE="yes"                 # --no-vnc 时置 no

# 蜂窝射频开机自启（--no-rf 时置 no）
# 模块重启后 CFUN 停在 0（射频关），此服务开机下发 AT+CFUN=1。
# 5G 仅作 WiFi 的兜底出口，射频没开起来不阻塞开机。
RF_ENABLE="yes"

# --------------------------- 颜色/日志 ---------------------------

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# 检查 TCP 端口是否监听（优先 ss，回退 netstat；镜像可能只装其一）
check_port_listen() {
    local port="$1"
    if command -v ss &>/dev/null; then
        ss -tln 2>/dev/null | grep -q ":${port} "
    elif command -v netstat &>/dev/null; then
        netstat -tln 2>/dev/null | grep -q ":${port} "
    else
        return 1
    fi
}

# --------------------------- 基础检查 ---------------------------

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "请用 root 运行: sudo bash $0"
        exit 1
    fi
}

check_platform() {
    # 仅在移远板（QCS6490 aarch64）上运行
    if [[ "$(uname -m)" != "aarch64" ]]; then
        log_error "本脚本仅用于移远板（QCS6490 aarch64），当前平台 $(uname -m)"
        exit 1
    fi
    if ! command -v nmcli &>/dev/null; then
        log_error "缺少 nmcli（NetworkManager），请确认镜像包含 NetworkManager"
        exit 1
    fi
}

detect_wan_iface() {
    # 只选【真正拿到 IPv4 地址】的出口，按 WAN_CANDIDATES 优先级（wlan0 → rmnet_data0）。
    #
    # 不要退化成"链路存在即选中"：rmnet_data0 等蜂窝口即使驱动已加载（ip link 可见），
    # 未拨号成功时并没有 IP，选中它会让 MASQUERADE/DNAT 全挂在空壳口上 → NAT 全废。
    local iface
    for iface in "${WAN_CANDIDATES[@]}"; do
        ip link show "$iface" &>/dev/null || continue
        if ip -4 addr show "$iface" 2>/dev/null | grep -q "inet "; then
            echo "$iface"; return 0
        fi
    done
    echo ""; return 1
}

# --------------------------- 0.5 登录用户（出厂用户改名为 leju_x1） ---------------------------

configure_user() {
    log_info "配置登录用户：${LEGACY_USER} → ${LOGIN_USER}"

    if ! getent passwd "$LOGIN_USER" >/dev/null; then
        # 出厂用户存在 → 改名（保留 uid / 家目录全部内容 / 附加组）
        rename_user "$LEGACY_USER" "$LOGIN_USER"
    else
        log_info "  ${LOGIN_USER} 已存在，跳过改名"
    fi

    # 设置密码
    if echo "${LOGIN_USER}:${LOGIN_PASSWORD}" | chpasswd 2>/dev/null; then
        log_info "  密码已设置"
    else
        log_error "  密码设置失败，请手动执行: echo '${LOGIN_USER}:${LOGIN_PASSWORD}' | chpasswd"
        exit 1
    fi

    log_info "  ${LOGIN_USER} 权限: $(id -nG "$LOGIN_USER")"
    return 0
}

# 把出厂用户改名为交付用户名，并同步改组名、家目录名。
#   groupmod -n 改同名主组（gid 不变）
#   usermod -l  改用户名（uid 不变）
#   usermod -d -m 改家目录并搬运内容
# 家目录里的配置文件（.config/.vnc/xstartup 等）含旧家目录绝对路径，需同步替换。
#
# 注意：usermod -l 要求该用户没有正在运行的进程，否则报
#   "user X is currently used by process N"
# 出厂用户默认是图形桌面登录态（gdm + gnome-session 一大串进程），必须先停掉。
rename_user() {
    local old="$1" new="$2" old_home new_home
    old_home=$(getent passwd "$old" | cut -d: -f6)
    new_home="${HOME_ROOT}/${new}"

    # 0) 停掉占用旧用户的会话：先停显示管理器，再清残留进程
    if pgrep -u "$old" >/dev/null 2>&1; then
        log_info "  停止 ${old} 的登录会话（gdm + 残留进程）"
        systemctl stop gdm3 2>/dev/null || systemctl stop gdm 2>/dev/null || \
            systemctl stop lightdm 2>/dev/null || true
        sleep 2
        pkill -u "$old" 2>/dev/null || true
        sleep 2
        pkill -9 -u "$old" 2>/dev/null || true
        sleep 1
        local left
        left=$(pgrep -u "$old" 2>/dev/null | wc -l)
        if [[ "$left" -gt 0 ]]; then
            log_error "  仍有 ${left} 个 ${old} 进程占用，请重启后重跑本脚本"
            exit 1
        fi
    fi

    log_info "  改名: ${old} → ${new}（保留 uid / 家目录内容）"

    # 1) 主组改名（若存在与旧用户同名的组）
    if getent group "$old" >/dev/null; then
        if getent group "$new" >/dev/null; then
            log_warn "    组 ${new} 已存在，跳过组改名"
        else
            groupmod -n "$new" "$old" && log_info "    组 ${old} → ${new}"
        fi
    fi

    # 2) 用户名 + 家目录
    if ! usermod -l "$new" -s /bin/bash "$old"; then
        log_error "    用户名改名失败，中止"
        exit 1
    fi
    if [[ "$old_home" != "$new_home" ]]; then
        if [[ -d "$new_home" ]]; then
            log_error "    目标家目录 ${new_home} 已存在，未搬运。请先清理后重跑"
            exit 1
        fi
        usermod -d "$new_home" -m "$new" && log_info "    家目录 ${old_home} → ${new_home}"
    fi

    # 3) 家目录内残留的旧绝对路径（桌面/VNC/session 配置里存在硬编码路径）
    local f
    while IFS= read -r -d '' f; do
        if grep -Il "$old_home" "$f" >/dev/null 2>&1; then
            sed -i "s|${old_home}|${new_home}|g" "$f" 2>/dev/null || true
        fi
    done < <(find "$new_home" -type f -size -1M -print0 2>/dev/null)
    log_info "    home 内旧路径引用已修正"

    # 4) 家目录归属（home 内容搬到新路径后属主已是旧 uid，这里统一为改名后的用户）
    chown -R "$new":"$new" "$new_home" 2>/dev/null || true
    return 0
}

# --------------------------- 1. eth0 固定 IP（NetworkManager） ---------------------------

configure_networkmanager() {
    log_info "配置 NetworkManager：${LAN_IFACE} = ${LAN_IP}（robot-lan，不干扰默认路由）"

    # 删除可能抢占 eth0 的默认连接，让 robot-lan 独占
    local dup
    dup=$(nmcli -t -f NAME,DEVICE connection show 2>/dev/null | awk -F: -v dev="$LAN_IFACE" '$2==dev && $1!="robot-lan"{print $1}' | head -1 || true)
    if [[ -n "$dup" ]]; then
        log_warn "  删除抢占 ${LAN_IFACE} 的连接: $dup"
        nmcli connection delete "$dup" 2>/dev/null || true
    fi

    if nmcli connection show robot-lan &>/dev/null; then
        nmcli connection modify robot-lan \
            connection.interface-name "$LAN_IFACE" \
            ipv4.method manual \
            ipv4.addresses "${LAN_IP}/${LAN_PREFIX}" \
            ipv4.gateway "" \
            ipv4.dns "" \
            ipv4.routes "" \
            ipv4.route-metric 600 \
            ipv4.never-default yes \
            ipv4.ignore-auto-dns yes \
            connection.autoconnect yes
    else
        nmcli connection add \
            type ethernet \
            ifname "$LAN_IFACE" \
            con-name robot-lan \
            ipv4.method manual \
            ipv4.addresses "${LAN_IP}/${LAN_PREFIX}" \
            ipv4.gateway "" \
            ipv4.dns "" \
            ipv4.routes "" \
            ipv4.route-metric 600 \
            ipv4.never-default yes \
            ipv4.ignore-auto-dns yes \
            connection.autoconnect yes
    fi

    nmcli connection reload
    nmcli connection up robot-lan 2>&1 | grep -v "^$" | tail -1 || log_warn "  激活失败（eth0 可能未接网线），配置已保存"

    # 确保 wlan0 提供默认路由（外网不被 26 网段抢）
    local wlan_conn
    wlan_conn=$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | grep ':wlan0$' | cut -d: -f1 || true)
    if [[ -n "$wlan_conn" ]]; then
        nmcli connection modify "$wlan_conn" ipv4.never-default no ipv4.ignore-auto-dns no 2>/dev/null || true
        nmcli connection up "$wlan_conn" 2>&1 | grep -v "^$" | tail -1 || true
        log_info "  wlan0 连接 '$wlan_conn' 已确保提供默认路由"
    fi
    log_info "  ${LAN_IFACE} = ${LAN_IP}（metric 600, never-default）"
}

# --------------------------- 2. dhcpcd 防抢地址 ---------------------------

configure_dhcpcd() {
    log_info "配置 dhcpcd：denyinterfaces ${LAN_IFACE} ${DDS_IFACE} bridge0（防止抢地址 + 抢默认路由）"
    local f=/etc/dhcpcd.conf
    touch "$f"
    for iface in "$LAN_IFACE" "$DDS_IFACE" bridge0; do
        if ! grep -q "denyinterfaces $iface" "$f" 2>/dev/null; then
            echo "denyinterfaces $iface" >> "$f"
        fi
    done

    # 重启 dhcpcd 让 denyinterfaces 立即生效（否则已持有的租约继续保留）
    systemctl restart dhcpcd 2>/dev/null || service dhcpcd restart 2>/dev/null || true
    sleep 1

    # 清理 LAN/DDS 口上除固定 IP 外的所有残留地址。
    # 注意：不能用 "dynamic" 关键字过滤——板子自家 dhcpd 服务 eth0 时，dhcpcd 客户端
    # 会从"自己"拿到 26.100 这类地址，内核里标记为 "secondary noprefixroute" 而非
    # "dynamic"，只匹配 dynamic 会漏掉，残留地址会带出 metric 更优的 26 网段路由，
    # 把外网默认路由压下去（实测会导致板子失联）。
    _strip_stray_addr() {
        local dev="$1" keep="$2" addr
        ip -4 -o addr show dev "$dev" 2>/dev/null | awk '{print $4}' | while read -r addr; do
            [[ "$addr" == "${keep}/"* ]] && continue
            log_warn "  删除 ${dev} 残留地址 ${addr}（非固定 IP ${keep}）"
            ip addr del "$addr" dev "$dev" 2>/dev/null || true
        done
    }
    _strip_stray_addr "$LAN_IFACE" "$LAN_IP"
    _strip_stray_addr "$DDS_IFACE" "$DDS_IP"

    # 删除 26 网段被误加的默认路由
    ip route del default via "${LAN_IP}" dev "$LAN_IFACE" 2>/dev/null || true
}

# --------------------------- 3. DHCP 服务（isc-dhcp-server） ---------------------------

configure_dhcpd() {
    log_info "配置 DHCP 服务（isc-dhcp-server）：${LAN_SUBNET} 池 ${DHCP_RANGE_START}~${DHCP_RANGE_END}"

    if ! command -v dhcpd &>/dev/null; then
        log_error "缺少 isc-dhcp-server，请先安装（apt-get install isc-dhcp-server）"
        exit 1
    fi

    # dhcpd.conf：只追加/替换我们的 subnet 段，保留其余
    # 用完整 subnet 声明做幂等判断，避免误匹配同网段其它子网
    if ! grep -qF "subnet 192.168.26.0 netmask 255.255.255.0 {" /etc/dhcp/dhcpd.conf 2>/dev/null; then
        cat >> /etc/dhcp/dhcpd.conf << EOF

# ===== Kuavo X1 恢复脚本生成 =====
subnet 192.168.26.0 netmask 255.255.255.0 {
    range ${DHCP_RANGE_START} ${DHCP_RANGE_END};
    option domain-name-servers 8.8.8.8, 8.8.4.4;
    option routers ${LAN_IP};
    option broadcast-address 192.168.26.255;
    default-lease-time 600;
    max-lease-time 7200;
}
EOF
    fi

    # 28 网段（DDS 专用）不提供 DHCP：声明空 subnet（无 range），
    # 阻止 dhcpd 在 eth1 上分配 26 网段地址（dhcpd 绑定 0.0.0.0:67 会响应所有接口的 DHCP 广播）
    if ! grep -qF "subnet 192.168.28.0 netmask 255.255.255.0 {" /etc/dhcp/dhcpd.conf 2>/dev/null; then
        cat >> /etc/dhcp/dhcpd.conf << EOF

# ===== Kuavo X1: 28 网段（DDS 专用）不提供 DHCP 服务 =====
subnet 192.168.28.0 netmask 255.255.255.0 {
}
EOF
        log_info "  28 网段空 subnet 声明已追加（禁止 DHCP 分配）"
    fi

    # 监听接口（文件可能不存在，先 touch 防 sed -i 报错中断）
    touch /etc/default/dhcp-server
    sed -i '/^INTERFACES=/d' /etc/default/dhcp-server
    echo "INTERFACES=\"${LAN_IFACE}\"" >> /etc/default/dhcp-server

    systemctl enable dhcpd 2>/dev/null || systemctl enable isc-dhcp-server 2>/dev/null || systemctl enable dhcp-server 2>/dev/null || true

    # 【重要】出厂 dhcpd.service 只有 After=network.target，但 eth0 的 IP 由
    # NetworkManager 在更晚的阶段配置。开机瞬间 eth0 还没地址 → dhcpd 找不到
    # 监听口直接退出（status=1/FAILURE），26 网段拿不到 DHCP。
    # 用 drop-in 补一条等待条件：等 NM 把 eth0 配好再启动。
    install_dhcpd_wait_dropin

    systemctl restart dhcpd 2>/dev/null || systemctl restart isc-dhcp-server 2>/dev/null || systemctl restart dhcp-server 2>/dev/null || \
        log_warn "  DHCP 服务重启失败，检查: journalctl -u dhcpd -n 20"
    sleep 1

    if systemctl is-active dhcpd &>/dev/null; then
        log_info "  DHCP 已监听 ${LAN_IFACE}"
    else
        log_warn "  DHCP 未在运行，检查: journalctl -u dhcpd -n 20"
    fi
}

# 给 dhcpd 加开机等待：等 ${LAN_IFACE} 拿到 IPv4 再启动，避免抢跑退出。
install_dhcpd_wait_dropin() {
    local unit="" u
    for u in dhcpd isc-dhcp-server dhcp-server; do
        if systemctl list-unit-files "${u}.service" 2>/dev/null | grep -q "^${u}.service"; then
            unit="$u"; break
        fi
    done
    [[ -z "$unit" ]] && { log_warn "  未找到 dhcpd 服务单元，跳过开机等待配置"; return 0; }

    local dir="/etc/systemd/system/${unit}.service.d"
    mkdir -p "$dir"

    # After=network-online.target 覆盖 NM 的等待机制；ExecStartPre 再兜底轮询
    # 接口地址（最多 30s），双保险。
    cat > "${dir}/10-wait-lan.conf" <<EOF
[Unit]
After=network-online.target NetworkManager-wait-online.service
Wants=network-online.target

[Service]
# 等 ${LAN_IFACE} 真正拿到 IPv4 再启动（最多 30s），避免开机抢跑
ExecStartPre=/bin/bash -c 'for i in \$(seq 1 30); do ip -4 addr show ${LAN_IFACE} 2>/dev/null | grep -q "inet " && exit 0; sleep 1; done; exit 0'
EOF

    systemctl daemon-reload
    log_info "  已配置 ${unit} 开机等待 ${LAN_IFACE} 就绪（${dir}/10-wait-lan.conf）"
}

# --------------------------- 4. IP 转发 ---------------------------

enable_ip_forward() {
    log_info "开启 IPv4 转发"
    sed -i '/^net.ipv4.ip_forward/d' /etc/sysctl.conf
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
    sysctl -p > /dev/null 2>&1 || true   # 个别参数可能只读返回非0，不中断
    log_info "  ip_forward = $(cat /proc/sys/net/ipv4/ip_forward)"
}

# --------------------------- 4.5 28 网段（DDS 专用） ---------------------------

configure_dds_28() {
    log_info "配置 28 网段（DDS 专用）：${DDS_IFACE} = ${DDS_IP}（${DDS_SUBNET}，与 26 网段隔离）"

    # 删除抢占 DDS_IFACE 的连接（保留 dds-lan）
    local dup
    dup=$(nmcli -t -f NAME,DEVICE connection show 2>/dev/null | awk -F: -v dev="$DDS_IFACE" -v keep="$DDS_CONN" '$2==dev && $1!=keep{print $1}' | head -1 || true)
    if [[ -n "$dup" ]]; then
        log_warn "  删除抢占 ${DDS_IFACE} 的连接: $dup"
        nmcli connection delete "$dup" 2>/dev/null || true
    fi

    # 创建/更新 dds-lan 连接（静态 28.1，never-default 防抢默认路由）
    if nmcli connection show "$DDS_CONN" &>/dev/null; then
        nmcli connection modify "$DDS_CONN" \
            connection.interface-name "$DDS_IFACE" \
            ipv4.method manual \
            ipv4.addresses "${DDS_IP}/${DDS_PREFIX}" \
            ipv4.gateway "" \
            ipv4.dns "" \
            ipv4.routes "" \
            ipv4.route-metric 600 \
            ipv4.never-default yes \
            ipv4.ignore-auto-dns yes \
            connection.autoconnect yes
    else
        nmcli connection add \
            type ethernet \
            ifname "$DDS_IFACE" \
            con-name "$DDS_CONN" \
            ipv4.method manual \
            ipv4.addresses "${DDS_IP}/${DDS_PREFIX}" \
            ipv4.gateway "" \
            ipv4.dns "" \
            ipv4.routes "" \
            ipv4.route-metric 600 \
            ipv4.never-default yes \
            ipv4.ignore-auto-dns yes \
            connection.autoconnect yes
    fi

    nmcli connection reload
    nmcli connection up "$DDS_CONN" 2>&1 | grep -v "^$" | tail -1 || log_warn "  激活失败（${DDS_IFACE} 可能未接网线），配置已保存"
    log_info "  ${DDS_IFACE} = ${DDS_IP}（${DDS_SUBNET}，metric 600, never-default）"

    # dhcpcd 防抢 DDS_IFACE 地址
    local f=/etc/dhcpcd.conf
    touch "$f"
    if ! grep -q "denyinterfaces $DDS_IFACE" "$f" 2>/dev/null; then
        echo "denyinterfaces $DDS_IFACE" >> "$f"
    fi

    # iptables：28 网段内部转发 + 与 26 网段/外网隔离
    #   - 放行 28 网段内部（eth1→eth1）互转，供 DDS 设备间通信
    #   - 不添加 28→26 / 28→外网 的转发规则（FORWARD policy DROP 天然隔离）
    iptables -C FORWARD -i "$DDS_IFACE" -o "$DDS_IFACE" -s "$DDS_SUBNET" -d "$DDS_SUBNET" -j ACCEPT 2>/dev/null || \
        iptables -A FORWARD -i "$DDS_IFACE" -o "$DDS_IFACE" -s "$DDS_SUBNET" -d "$DDS_SUBNET" -j ACCEPT

    log_info "  28 网段内部转发已放行（eth1→eth1），与 26 网段/外网隔离"
}

# --------------------------- 5. iptables（NAT + 端口转发） ---------------------------

flush_iptables() {
    # 清理旧网络规划残留（eth1 / 192.168.18.x / 出厂 bridge0/ppp0），保证干净
    # 先备份当前规则，防止误在已配置环境执行导致不可逆丢失
    if iptables-save > /tmp/iptables-rules.backup.$(date +%Y%m%d%H%M%S) 2>/dev/null; then
        log_info "  当前 iptables 规则已备份到 /tmp/iptables-rules.backup.*"
    fi
    log_info "清理旧 iptables 残留..."
    # 显式删除出厂 bridge0/ppp0 拨号残留（iptables -F 已清空全链，这里再兜底删，
    # 防止 -F 被修改/部分失效时残留污染镜像模板）
    for rule in \
        "-i bridge0 -o bridge+ -j ACCEPT" \
        "-i bridge0 -o ppp0 -j ACCEPT" \
        "-i ppp0 -o bridge0 -j ACCEPT" \
        "-i bridge0 -j DROP" \
        "-i ppp0 -j DROP"; do
        iptables -D FORWARD $rule 2>/dev/null || true
    done
    iptables -t nat -F
    iptables -t nat -X
    iptables -F
    iptables -X
    iptables -t mangle -F
    iptables -t mangle -X
    # 恢复默认 policy 后重建
    iptables -P INPUT ACCEPT
    iptables -P FORWARD DROP
    iptables -P OUTPUT ACCEPT
}

# 列出【链路存在】的出口候选（不要求已有 IP）。
# NAT/DNAT 规则要同时覆盖所有候选口，这样 WiFi 掉线走 5G、WiFi 恢复切回 WiFi
# 都由内核路由决定，无需重跑脚本。
list_wan_ifaces() {
    local iface
    for iface in "${WAN_CANDIDATES[@]}"; do
        ip link show "$iface" &>/dev/null && echo "$iface"
    done
}

# 列出【已拿到 IPv4】的出口候选（用于校验至少有一条路能出网）
list_active_wan_ifaces() {
    local iface
    for iface in "${WAN_CANDIDATES[@]}"; do
        ip link show "$iface" &>/dev/null || continue
        ip -4 addr show "$iface" 2>/dev/null | grep -q "inet " && echo "$iface"
    done
}

configure_iptables() {
    local wan_ifaces wan_iface
    wan_ifaces=$(list_wan_ifaces)
    if [[ -z "$wan_ifaces" ]]; then
        log_error "未检测到任何出口接口（${WAN_CANDIDATES[*]}），请确认硬件"
        exit 1
    fi

    # 至少有一条出口真正拿到 IP，否则配出来全是空规则
    wan_iface=$(detect_wan_iface)
    if [[ -z "$wan_iface" ]]; then
        log_error "出口接口存在但都没有 IPv4（请先连 WiFi 或等 5G 拨号），无法确定默认出口"
        exit 1
    fi
    log_info "配置 iptables（出口: $(echo $wan_ifaces | tr '\n' ' ')，当前生效: ${wan_iface}）"

    flush_iptables

    # NAT 出外网：对【所有】候选出口都挂 MASQUERADE。
    # 内核选路决定实际走谁 → WiFi 挂了自动走 5G，WiFi 回来自动切回，规则不用重建。
    for wan_iface in $wan_ifaces; do
        iptables -t nat -A POSTROUTING -o "$wan_iface" -j MASQUERADE
    done

    # DNAT 端口转发：入口同样覆盖所有候选出口（WiFi 或 5G 都能访问到内网设备）
    for wan_iface in $wan_ifaces; do
        for rule in "${DNAT_RULES[@]}"; do
            IFS=':' read -r ext_port dest_ip dest_port <<< "$rule"
            iptables -t nat -A PREROUTING -i "$wan_iface" -p tcp --dport "$ext_port" \
                -j DNAT --to-destination "${dest_ip}:${dest_port}"
        done
    done

    # FORWARD：policy DROP，明确放行 LAN↔WAN（覆盖所有候选出口）
    for wan_iface in $wan_ifaces; do
        iptables -A FORWARD -i "$LAN_IFACE" -o "$wan_iface" -m state --state NEW,RELATED,ESTABLISHED -j ACCEPT
        iptables -A FORWARD -i "$wan_iface" -o "$LAN_IFACE" -m state --state RELATED,ESTABLISHED -j ACCEPT
    done

    # 端口转发对应 FORWARD 放行（DNAT 命中后回程）
    for wan_iface in $wan_ifaces; do
        for rule in "${DNAT_RULES[@]}"; do
            IFS=':' read -r ext_port dest_ip dest_port <<< "$rule"
            iptables -A FORWARD -i "$wan_iface" -o "$LAN_IFACE" -p tcp --dport "$dest_port" -d "$dest_ip" \
                -m state --state NEW,RELATED,ESTABLISHED -j ACCEPT
        done
    done

    # 28 网段（DDS 专用）内部转发：flush 后重建
    #   放行 eth1→eth1 的 28 网段流量（DDS 设备间互转）
    #   不添加 28→26 / 28→外网 规则（FORWARD policy DROP 天然隔离）
    iptables -A FORWARD -i "$DDS_IFACE" -o "$DDS_IFACE" -s "$DDS_SUBNET" -d "$DDS_SUBNET" -j ACCEPT

    # 基础 INPUT（ICMP + 已建立连接）
    iptables -A INPUT -p icmp -j ACCEPT
    iptables -A INPUT -m state --state RELATED,ESTABLISHED -j ACCEPT

    persist_iptables
}

persist_iptables() {
    # 持久化 + 开机自动恢复。
    #
    # 【重要】出厂镜像自带的 iptables.service 读的是 /etc/iptables/iptables.rules
    # （不是 Debian 惯用的 rules.v4），且默认 disabled。此前只写了 rules.v4 又没
    # 启用任何服务 → 重启后规则全丢（NAT/DNAT 消失，端口转发和 26 段上网全断）。
    # 这里同时写两处路径，并启用镜像自带的 iptables.service，不依赖联网装包。
    local primary=/etc/iptables/iptables.rules
    local legacy=/etc/iptables/rules.v4
    mkdir -p /etc/iptables

    iptables-save > "$primary"
    iptables-save > "$legacy"

    if [[ -f /lib/systemd/system/iptables.service ]]; then
        systemctl enable iptables.service &>/dev/null || true
        log_info "  规则已写入 ${primary}（+ ${legacy}），并启用 iptables.service 开机恢复"
    elif command -v netfilter-persistent &>/dev/null; then
        netfilter-persistent save
        log_info "  规则已持久化（netfilter-persistent）"
    else
        log_warn "  规则已写入 ${primary}，但未找到开机恢复机制，重启后需重跑本脚本"
    fi
}

# --------------------------- 5.5 蜂窝射频开机自启 ---------------------------
# 模块重启后 AT+CFUN 停在 0（最小功能，射频关），不会自动注网/拨号。
# 这里装一个 oneshot 服务，开机经 port_bridge 的 AT 通道（127.0.0.1:9083）
# 下发 AT+CFUN=1 打开射频，让 5G 能作为兜底出口自动就绪。
#
# 说明：
#   - 只是"打开射频"，不主动拨号。数据连接由模块侧的自动拨号机制建立，
#     成功后 rmnet_data0 自动拿到 IPv4，脚本的出口检测即可选中它。
#   - 出口优先级仍按 WAN_CANDIDATES（wlan0 优先），5G 只在 WiFi 不可用时兜底。
#   - 幂等：已是 CFUN=1 直接跳过；AT 通道未就绪时轮询等待。
RF_SERVICE_NAME="x1-cellular-rf.service"

configure_cellular_rf() {
    if [[ "$RF_ENABLE" != "yes" ]]; then
        log_info "跳过蜂窝射频自启（--no-rf）"
        return 0
    fi

    # 出厂镜像没有这个模块时直接跳过，不影响其它配置
    if [[ ! -e /dev/at_mdm0 ]]; then
        log_warn "未检测到 /dev/at_mdm0，跳过蜂窝射频自启（本机可能无蜂窝模块）"
        return 0
    fi
    if ! command -v python3 &>/dev/null; then
        log_warn "缺少 python3，跳过蜂窝射频自启"
        return 0
    fi

    log_info "配置蜂窝射频开机自启（${RF_SERVICE_NAME}）"

    install -m 0755 /dev/stdin /usr/local/sbin/x1-cellular-rf-on.sh <<'RFON_EOF'
#!/bin/bash
# 开机把模块射频打开（AT+CFUN=1）。
# 模块重启后停在 CFUN=0（最小功能，射频关），不打开则无法注网/拨号。
# AT 通道由 port_bridge.service 提供（127.0.0.1:9083 <-> /dev/at_mdm0）。
AT_HOST=127.0.0.1
AT_PORT=9083
MAX_WAIT=60      # 等 AT 通道就绪的最长时间（秒）
RETRY=3          # CFUN=1 重试次数

at_cmd() {
    python3 - "$1" <<'PY'
import socket, sys, time
cmd = sys.argv[1]
try:
    s = socket.create_connection(("127.0.0.1", 9083), timeout=5)
    s.settimeout(2.0)
    s.sendall((cmd + "\r\n").encode())
    time.sleep(1.0)
    buf = b""
    try:
        while True:
            d = s.recv(4096)
            if not d:
                break
            buf += d
    except socket.timeout:
        pass
    s.close()
    print(buf.decode(errors="replace").replace("\r", "").replace("\x00", "").strip())
except Exception as e:
    print("ERR: %s" % e)
PY
}

# 1) 等 AT 通道就绪（port_bridge 起来需要时间）
waited=0
while [ "$waited" -lt "$MAX_WAIT" ]; do
    if at_cmd "AT" | grep -q "OK"; then break; fi
    sleep 2; waited=$((waited + 2))
done
if [ "$waited" -ge "$MAX_WAIT" ]; then
    echo "AT channel not ready after ${MAX_WAIT}s"; exit 1
fi

# 2) 已经是 CFUN=1 就跳过（幂等）
cur=$(at_cmd "AT+CFUN?")
if echo "$cur" | grep -q "+CFUN: 1"; then
    echo "already CFUN=1, nothing to do"; exit 0
fi

# 3) 下发 CFUN=1（带重试）
i=1
while [ "$i" -le "$RETRY" ]; do
    echo "AT+CFUN=1 attempt $i (current: $(echo "$cur" | grep -o '+CFUN: [0-9]*'))"
    out=$(at_cmd "AT+CFUN=1")
    echo "$out"
    if echo "$out" | grep -q "OK"; then
        sleep 3
        if at_cmd "AT+CFUN?" | grep -q "+CFUN: 1"; then
            echo "RF ON ok"; exit 0
        fi
    fi
    sleep 3; i=$((i + 1))
done
echo "failed to enable RF"; exit 1
RFON_EOF

    # ---- 5G 出口的 DNS ----
    # 蜂窝口 rmnet_data0 在 NM 里是 unmanaged，NM 不会把它的 DNS 交给
    # systemd-resolved。WiFi 正常时 resolv.conf 里有 WiFi 下发的 DNS，
    # 看不出问题；一旦 WiFi 掉线，NM 撤走那些 DNS，只剩 127.0.0.53 这个
    # stub，而 resolved 对 rmnet_data0 是 Current Scopes: none → 解析失败
    # （表现为 ping 域名报 unknown host，但 IP 直连正常）。
    # 这里补一个服务：等 rmnet_data0 拿到 IP 后，为它注册公共 DNS。
    cat > /usr/local/sbin/x1-cellular-dns.sh <<'DNS_EOF'
#!/bin/bash
# 给 5G 出口（rmnet_data0）注册 DNS，供 systemd-resolved 使用。
# 蜂窝口不受 NetworkManager 管理，不注册 DNS 的话 WiFi 掉线后无法解析域名。
CELL_IFACE=rmnet_data0
MAX_WAIT=90      # 等 rmnet_data0 拿到 IP 的最长时间（秒）

# 1) 等接口拿到 IPv4（拨号完成）
waited=0
while [ "$waited" -lt "$MAX_WAIT" ]; do
    if ip -4 addr show "$CELL_IFACE" 2>/dev/null | grep -q "inet "; then break; fi
    sleep 3; waited=$((waited + 3))
done
if [ "$waited" -ge "$MAX_WAIT" ]; then
    echo "$CELL_IFACE has no IPv4 after ${MAX_WAIT}s"; exit 1
fi

# 2) 优先用运营商下发的 DNS；取不到就用公共 DNS 兜底
DNS_LIST=$(ip -4 route show dev "$CELL_IFACE" 2>/dev/null | grep -oP 'via \K[0-9.]+' | head -1)
OPER_DNS=""
if [ -f /etc/resolv.conf ]; then
    OPER_DNS=$(grep -oP '^nameserver \K[0-9.]+' /etc/resolv.conf 2>/dev/null \
               | grep -vE '^127\.' | tr '\n' ' ')
fi
# 公共 DNS 兜底（阿里 + 114）
[ -z "$OPER_DNS" ] && OPER_DNS="223.5.5.5 114.114.114.114"

if command -v resolvectl &>/dev/null; then
    resolvectl dns "$CELL_IFACE" $OPER_DNS
    resolvectl domain "$CELL_IFACE" '~.'
    resolvectl default-route "$CELL_IFACE" yes
    echo "registered DNS for $CELL_IFACE: $OPER_DNS"
else
    # 没有 resolvectl 时退回直接写 resolv.conf（去掉 NM 的托管标记）
    for d in $OPER_DNS; do
        grep -q "nameserver $d" /etc/resolv.conf 2>/dev/null || echo "nameserver $d" >> /etc/resolv.conf
    done
    echo "appended DNS to /etc/resolv.conf: $OPER_DNS"
fi
DNS_EOF
    chmod 0755 /usr/local/sbin/x1-cellular-dns.sh

    cat > /etc/systemd/system/x1-cellular-dns.service <<'DNSUNIT_EOF'
[Unit]
Description=X1 cellular DNS registration (rmnet_data0)
After=network-online.target x1-cellular-rf.service
Wants=network-online.target
# 射频服务负责开射频，本服务等它之后再看接口有没有起来
After=x1-cellular-rf.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/x1-cellular-dns.sh
# DNS 注册失败不阻塞开机
SuccessExitStatus=0 1

[Install]
WantedBy=multi-user.target
DNSUNIT_EOF

    systemctl daemon-reload
    systemctl enable x1-cellular-dns.service &>/dev/null

    cat > "/etc/systemd/system/${RF_SERVICE_NAME}" <<EOF
[Unit]
Description=X1 cellular RF on (AT+CFUN=1)
# 必须等 port_bridge 提供 AT 通道；用 wants 而非 requires，避免拖垮启动
After=port_bridge.service network.target
Wants=port_bridge.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/x1-cellular-rf-on.sh
# 射频没开起来不阻塞开机（5G 只是兜底出口）
SuccessExitStatus=0 1

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable "$RF_SERVICE_NAME" &>/dev/null
    log_info "  已启用（开机执行 AT+CFUN=1，已启用则跳过）"
    log_info "  已启用 x1-cellular-dns.service（5G 出口 DNS 注册，WiFi 掉线时可解析域名）"
}

# --------------------------- 6. VNC（移远板自身桌面） ---------------------------

ensure_apt_mirror() {
    # 出厂镜像默认用官方源 ports.ubuntu.com（国外），实测板子经 WiFi 访问不了，
    # 会导致 xfce4 等依赖装不上。这里探测一次，不通就切到国内镜像（阿里云）。
    local f=/etc/apt/sources.list
    [[ -f "$f" ]] || return 0

    local cur_host
    # 只看生效的 deb 行（排除 # 注释，否则会误取到 help.ubuntu.com 这类说明里的主机）
    cur_host=$(grep -E '^[[:space:]]*deb[[:space:]]' "$f" 2>/dev/null \
                 | grep -m1 -oE 'https?://[^ /]+' | sed 's|https\?://||' || true)
    [[ -n "$cur_host" ]] || return 0

    # 已镜像不可达？（用 HTTP 80 端口探测，避免依赖 curl）
    if timeout 6 bash -c "cat < /dev/null > /dev/tcp/${cur_host}/80" 2>/dev/null; then
        log_info "  apt 源 ${cur_host} 可达"
        return 0
    fi

    log_warn "  apt 源 ${cur_host} 不可达，切换到 mirrors.aliyun.com"
    if ! timeout 6 bash -c "cat < /dev/null > /dev/tcp/mirrors.aliyun.com/80" 2>/dev/null; then
        log_warn "  国内镜像也不可达，保持原源"
        return 0
    fi

    cp "$f" "${f}.bak.$(date +%Y%m%d%H%M%S)"
    # 官方源 host 替换为阿里云（保留路径 /ubuntu-ports 等不变）
    sed -i -E "s|https?://${cur_host//./\\.}|http://mirrors.aliyun.com|g" "$f"
    log_info "  apt 源已切换（备份: ${f}.bak.*）"
}

install_vnc_deps() {
    # 交付板出厂镜像常缺 TigerVNC / xfce4，这里自动补齐（换板一键到底）
    log_info "检查 VNC 依赖（TigerVNC + xfce4）..."
    local need=()
    command -v vncserver  &>/dev/null || need+=("tigervnc-standalone-server")
    command -v vncpasswd  &>/dev/null || need+=("tigervnc-standalone-server")
    command -v startxfce4 &>/dev/null || need+=("xfce4" "xfce4-terminal")
    command -v dbus-launch &>/dev/null || need+=("dbus-x11")   # xstartup 依赖 dbus-launch

    if [[ ${#need[@]} -eq 0 ]]; then
        log_info "  VNC 依赖已齐全"
        return 0
    fi

    # 去重（vncserver/vncpasswd 同属一个包）
    local pkgs=() p q dup
    for p in "${need[@]}"; do
        dup=0
        for q in "${pkgs[@]}"; do [[ "$q" == "$p" ]] && dup=1; done
        [[ $dup -eq 0 ]] && pkgs+=("$p")
    done

    log_warn "  缺少依赖，准备安装: ${pkgs[*]}"
    if ! command -v apt-get &>/dev/null; then
        log_error "本机无 apt-get，请手动安装: ${pkgs[*]}"
        exit 1
    fi

    # apt 需要外网：此时 wlan0 应已联网（脚本开头配置过）
    if ! ping -c1 -W3 mirrors.aliyun.com &>/dev/null && ! ping -c1 -W3 8.8.8.8 &>/dev/null; then
        log_error "外网不可达，无法 apt 安装依赖。请先确认 WiFi（wlan0）已联网后重跑本脚本"
        exit 1
    fi

    export DEBIAN_FRONTEND=noninteractive
    ensure_apt_mirror
    apt-get update -qq || log_warn "  apt-get update 失败，尝试直接安装..."
    if ! apt-get install -y "${pkgs[@]}"; then
        log_error "依赖安装失败，请手动执行: apt-get install -y ${pkgs[*]}"
        exit 1
    fi

    # 复核关键命令
    local still=()
    command -v vncserver  &>/dev/null || still+=("vncserver")
    command -v vncpasswd  &>/dev/null || still+=("vncpasswd")
    command -v startxfce4 &>/dev/null || still+=("startxfce4")
    command -v dbus-launch &>/dev/null || still+=("dbus-launch")
    if [[ ${#still[@]} -ne 0 ]]; then
        log_error "安装后仍缺少命令: ${still[*]}（请检查 apt 源）"
        exit 1
    fi
    log_info "  VNC 依赖安装完成"
}

configure_vnc() {
    [[ "$VNC_ENABLE" != "yes" ]] && { log_warn "跳过 VNC 配置（--no-vnc）"; return; }

    log_info "配置 VNC（移远板自身桌面，端口 ${VNC_PORT}，display ${VNC_DISPLAY}）"

    install_vnc_deps

    # VNC 密码固定 leju_x1（写死，与登录密码一致；VNC 协议只认前 8 字符）
    VNC_PASSWORD="${VNC_PASSWORD:0:8}"

    local home
    home=$(getent passwd "$VNC_USER" | cut -d: -f6)
    mkdir -p "$home/.vnc"
    chown "$VNC_USER":"$VNC_USER" "$home/.vnc"

    # xstartup：xfce4 桌面
    cat > "$home/.vnc/xstartup" << 'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec dbus-launch --exit-with-session startxfce4
EOF
    chmod +x "$home/.vnc/xstartup"
    chown "$VNC_USER":"$VNC_USER" "$home/.vnc/xstartup"

    # 密码（VncAuth，passwd 600 权限）
    if command -v x11vnc &>/dev/null; then
        x11vnc -storepasswd "$VNC_PASSWORD" "$home/.vnc/passwd" 2>/dev/null || \
            echo "$VNC_PASSWORD" | vncpasswd -f > "$home/.vnc/passwd"
    else
        echo "$VNC_PASSWORD" | vncpasswd -f > "$home/.vnc/passwd"
    fi
    chmod 600 "$home/.vnc/passwd"
    chown "$VNC_USER":"$VNC_USER" "$home/.vnc/passwd"

    # systemd 服务模板（vncserver@.service）
    # 用未加引号的 EOF 以展开 $VNC_USER；%i 是 systemd 实例占位符，脚本层不展开
    cat > /etc/systemd/system/vncserver@.service << EOF
[Unit]
Description=VNC Server (per-display)
After=network.target

[Service]
Type=forking
User=${VNC_USER}
Group=${VNC_USER}
ExecStart=/usr/bin/vncserver %i -geometry 1280x720 -rfbport 5901 -localhost no --I-KNOW-THIS-IS-INSECURE -SecurityTypes VncAuth
ExecStop=/usr/bin/vncserver -kill %i

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    # 实例名带冒号，如 vncserver@:2.service（对应 display :2，rfbport 5901）
    systemctl enable "vncserver@${VNC_DISPLAY}.service" 2>/dev/null || \
        systemctl enable "vncserver@${VNC_DISPLAY#:}" 2>/dev/null || true
    # 若已在运行则重启以应用新密码/配置
    systemctl restart "vncserver@${VNC_DISPLAY}" 2>/dev/null || true
    systemctl start "vncserver@${VNC_DISPLAY}" 2>/dev/null || \
        log_warn "  VNC 服务启动失败，检查: journalctl -u vncserver@${VNC_DISPLAY} -n 20"

    # polkit-agent-helper-1 权限修复（VNC 桌面报 incorrect permissions）
    if [[ -f /usr/lib/policykit-1/polkit-agent-helper-1 ]]; then
        chmod 4755 /usr/lib/policykit-1/polkit-agent-helper-1
        log_info "  polkit-agent-helper-1 权限已修复（4755）"
    fi

    sleep 1
    if check_port_listen "$VNC_PORT"; then
        log_info "  VNC 已监听 :${VNC_PORT}"
    else
        log_warn "  VNC 未监听 :${VNC_PORT}"
    fi
}

# --------------------------- 验证 ---------------------------

verify() {
    echo ""
    log_info "========== 验证 =========="
    echo "  ${LAN_IFACE} IP:   $(ip -o -4 addr show dev "$LAN_IFACE" 2>/dev/null | awk '{print $4}')"
    echo "  ${DDS_IFACE} IP:   $(ip -o -4 addr show dev "$DDS_IFACE" 2>/dev/null | awk '{print $4}')  （28 网段 DDS）"
    echo "  默认路由:   $(ip route show default | head -1)"
    echo "  ip_forward: $(cat /proc/sys/net/ipv4/ip_forward)"
    echo ""
    echo "  --- NAT ---"
    iptables -t nat -S | grep -E "MASQUERADE|DNAT" | sed 's/^/    /'
    echo "  --- FORWARD ---"
    iptables -S FORWARD | head -12 | sed 's/^/    /'
    echo ""
    if check_port_listen "$VNC_PORT"; then
        echo "  VNC ${VNC_PORT}: 监听中 ✓"
    fi
    echo ""
    # 28 网段连通性自检（DDS 链路）
    if ip -o -4 addr show dev "$DDS_IFACE" 2>/dev/null | grep -q "$DDS_IP"; then
        echo "  28 网段 ${DDS_IFACE} = ${DDS_IP}: 已配置 ✓"
    else
        log_warn "  28 网段 ${DDS_IFACE} 未配置（${DDS_IFACE} 可能未接网线）"
    fi
    echo ""
    # 外网自检（wlan0）
    if ping -c1 -W3 www.baidu.com &>/dev/null; then
        log_info "  移远板外网/DNS 正常 ✓（出口: $(detect_wan_iface)）"
    else
        log_warn "  外网暂不可达（检查 WiFi 出口）"
    fi
}

# --------------------------- 主流程 ---------------------------

main() {
    # 参数解析
    for arg in "$@"; do
        case "$arg" in
            --no-vnc) VNC_ENABLE="no" ;;
            --no-rf)  RF_ENABLE="no" ;;
            -h|--help)
                sed -n '1,40p' "$0" | grep '^#' | sed 's/^# \{0,1\}//'
                exit 0 ;;
        esac
    done

    check_root
    check_platform
    log_info "开始恢复 Kuavo X1 移远板网络配置..."
    echo ""

    configure_user
    configure_networkmanager
    configure_dhcpcd
    configure_dhcpd
    enable_ip_forward
    configure_dds_28
    configure_iptables
    configure_cellular_rf
    configure_vnc
    verify

    log_info "完成！配置已恢复并持久化，重启后依然生效。"
    echo ""
    log_info "外部访问（<移远板IP> 为板子 WiFi 出口 IP）："
    for rule in "${DNAT_RULES[@]}"; do
        IFS=':' read -r ext_port dest_ip dest_port <<< "$rule"
        printf "    %s:%-5s → %s:%s\n" "<移远板IP>" "$ext_port" "$dest_ip" "$dest_port"
    done
    echo ""
    log_info "移远板自身 VNC: vncviewer <移远板IP>:${VNC_PORT}"
}

trap 'log_error "脚本在第 $LINENO 行失败，退出码: $?"' ERR
main "$@"
