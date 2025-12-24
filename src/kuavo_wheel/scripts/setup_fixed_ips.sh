#!/bin/bash

# Kuavo Fixed IP Setup Script
# 设置固定IP地址：一次配置一台机器（上位机或下位机）
# 注意：下位机是kuavo_master，上位机是kuavo_slave

set -e

SCRIPT_NAME="$(basename "$0")"

# 默认配置
DEFAULT_UPPER_IP="169.254.128.136"    # 上位机 (kuavo_slave)
DEFAULT_LOWER_IP="169.254.128.130"    # 下位机 (kuavo_master)
DEFAULT_WHEEL_IP="169.254.128.2"      # 轮式底盘
DEFAULT_NETMASK="255.255.255.0"
DEFAULT_NETWORK="169.254.128.0"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $SCRIPT_NAME [OPTIONS]"
    echo ""
    echo "机器角色说明:"
    echo "  上位机 (kuavo_slave):  高级控制逻辑，连接到下位机的ROS Master"
    echo "                         默认网关将设置为下位机IP ($DEFAULT_LOWER_IP)"
    echo "  下位机 (kuavo_master): 运行ROS Master，控制机器人本体"
    echo ""
    echo "Options:"
    echo "  -h, --help                显示帮助信息"
    echo "  --upper                   配置上位机 (kuavo_slave, IP: $DEFAULT_UPPER_IP)"
    echo "  --lower                   配置下位机 (kuavo_master, IP: $DEFAULT_LOWER_IP)"
    echo "  --netmask MASK            设置子网掩码 (默认: $DEFAULT_NETMASK)"
    echo ""
    echo "示例:"
    echo "  $SCRIPT_NAME --upper                     # 配置上位机"
    echo "  $SCRIPT_NAME --lower                     # 配置下位机"
    echo ""
    echo "默认IP分配:"
    echo "  上位机 (kuavo_slave):  $DEFAULT_UPPER_IP"
    echo "  下位机 (kuavo_master): $DEFAULT_LOWER_IP"
    echo "  轮式底盘:              $DEFAULT_WHEEL_IP"
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

get_network_interfaces() {
    # 获取所有网络接口信息，优先使用ifconfig，如果不存在则使用ip命令
    local interfaces=()
    if command -v ifconfig >/dev/null 2>&1; then
        # 使用ifconfig获取接口列表
        interfaces=($(ifconfig -a | grep -E "^[a-zA-Z0-9]+" | grep -v "lo:" | awk '{print $1}' | sed 's/:$//' | sort))
    elif command -v ip >/dev/null 2>&1; then
        # 使用ip命令获取接口列表
        interfaces=($(ip link show 2>/dev/null | grep -E "^[0-9]+:" | grep -v "lo:" | awk -F': ' '{print $2}' | cut -d'@' -f1))
    else
        # 从/sys/class/net获取接口列表
        interfaces=($(ls /sys/class/net 2>/dev/null | grep -v lo | sort))
    fi
    
    echo "${interfaces[@]}"
}

display_network_interfaces() {
    log_info "检测可用网络接口..."
    
    # 获取网络接口列表
    local interfaces_array=($(get_network_interfaces))
    
    if [ ${#interfaces_array[@]} -eq 0 ]; then
        log_error "未找到可用的网络接口"
        return 1
    fi
    
    echo ""
    echo "可用网络接口:"
    echo "============================================"
    
    local i=1
    for iface in "${interfaces_array[@]}"; do
        local status="UNKNOWN"
        local ip_info=""
        
        # 获取接口状态
        if command -v ifconfig >/dev/null 2>&1; then
            if ifconfig "$iface" 2>/dev/null | grep -q "UP"; then
                status="UP"
            else
                status="DOWN"
            fi
            # 获取IP信息
            ip_info=$(ifconfig "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | head -1)
        elif command -v ip >/dev/null 2>&1; then
            status=$(ip link show "$iface" 2>/dev/null | grep -o "state [A-Z]*" | awk '{print $2}')
            ip_info=$(ip addr show "$iface" 2>/dev/null | grep "inet " | awk '{print $2}' | head -1)
        fi
        
        printf "%2d) %-15s [%s]" "$i" "$iface" "$status"
        if [ -n "$ip_info" ]; then
            printf " - 当前IP: %s" "$ip_info"
        fi
        echo ""
        
        ((i++))
    done
    
    echo "============================================"
    echo ""
}

select_network_interface() {
    # 获取网络接口列表
    local interfaces_array=($(get_network_interfaces))
    
    if [ ${#interfaces_array[@]} -eq 0 ]; then
        return 1
    fi
    
    # 让用户选择
    while true; do
        read -p "请选择网络接口 (1-${#interfaces_array[@]}): " choice
        
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#interfaces_array[@]} ]; then
            selected_interface="${interfaces_array[$((choice-1))]}"
            echo "$selected_interface"
            return 0
        else
            log_warn "无效选择，请输入 1-${#interfaces_array[@]} 之间的数字"
        fi
    done
}

check_ip_conflict() {
    local ip="$1"
    log_info "检查IP冲突: $ip"
    
    if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
        log_warn "IP地址 $ip 可能已被其他设备使用"
        return 1
    fi
    return 0
}

configure_static_ip() {
    local interface="$1"
    local ip="$2"
    local netmask="$3"
    local machine_type="$4"
    
    log_info "配置静态IP: $interface -> $ip/$netmask"
    
    # 清除现有IP配置
    sudo ip addr flush dev "$interface" 2>/dev/null || true
    
    # 设置新IP
    sudo ip addr add "$ip/$netmask" dev "$interface"
    sudo ip link set "$interface" up
    
    # 如果是上位机，设置默认网关为下位机
    if [[ "$machine_type" == "upper" ]]; then
        log_info "设置默认网关: $DEFAULT_LOWER_IP"
        # 删除现有默认路由（如果存在）
        sudo ip route del default 2>/dev/null || true
        # 添加新的默认路由
        sudo ip route add default via "$DEFAULT_LOWER_IP" dev "$interface"
        log_success "默认网关已设置为: $DEFAULT_LOWER_IP"
    fi
    
    # 验证配置
    if ip addr show "$interface" | grep -q "$ip"; then
        log_success "IP配置成功: $interface = $ip"
    else
        log_error "IP配置失败"
        return 1
    fi
}

create_netplan_config() {
    local interface="$1"
    local ip="$2"
    local netmask="$3"
    local machine_type="$4"
    
    local netplan_file="/etc/netplan/99-kuavo-fixed-ip.yaml"
    
    log_info "创建Netplan配置: $netplan_file"
    
    # 计算CIDR前缀长度
    local cidr_prefix
    case "$netmask" in
        "255.255.255.0") cidr_prefix="24" ;;
        "255.255.0.0") cidr_prefix="16" ;;
        "255.0.0.0") cidr_prefix="8" ;;
        *) 
            log_warn "未识别的子网掩码: $netmask，使用默认 /24"
            cidr_prefix="24"
            ;;
    esac
    
    local config_content="network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $interface:
      dhcp4: false
      addresses:
        - $ip/$cidr_prefix
      optional: true"
    
    # 如果是上位机，添加网关配置
    if [[ "$machine_type" == "upper" ]]; then
        config_content="$config_content
      gateway4: $DEFAULT_LOWER_IP"
    fi
    
    config_content="$config_content
"
    
    # 创建netplan配置
    echo "$config_content" | sudo tee "$netplan_file" > /dev/null
    
    # 应用配置
    if sudo netplan apply 2>/dev/null; then
        log_success "Netplan配置已创建并应用"
    else
        log_error "Netplan配置应用失败"
        echo ""
        echo "可能的原因："
        echo "1) 配置文件语法错误"
        echo "2) 网络接口不存在或不可用"
        echo "3) 权限不足"
        echo ""
        echo "建议操作："
        echo "1) 检查配置文件: sudo netplan --debug generate"
        echo "2) 手动验证配置: sudo netplan try"
        echo "3) 查看详细错误: sudo netplan apply"
        echo ""
        return 1
    fi
}

print_env_vars() {
    local machine_type="$1"
    local ip="$2"
    
    echo ""
    log_info "建议的环境变量配置:"
    echo "在 ~/.bashrc 中添加以下内容:"
    echo "=========================================="
    
    if [[ "$machine_type" == "upper" ]]; then
        echo "# 上位机 (kuavo_slave) ROS 环境变量"
        echo "export ROS_MASTER_URI=http://$DEFAULT_LOWER_IP:11311"
        echo "export ROS_IP=$ip"
        echo "export KUAVO_MASTER_URI=http://$DEFAULT_LOWER_IP:11311"
        echo "export KUAVO_SLAVE_IP=$ip"
    else
        echo "# 下位机 (kuavo_master) ROS 环境变量"
        echo "export ROS_MASTER_URI=http://$ip:11311"
        echo "export ROS_IP=$ip"
        echo "export KUAVO_MASTER_URI=http://$ip:11311"
        echo "export KUAVO_SLAVE_IP=$DEFAULT_UPPER_IP"
    fi
    
    echo "=========================================="
    echo ""
    echo "重新加载配置: source ~/.bashrc"
}

# 解析命令行参数
MACHINE_TYPE=""
NETMASK="$DEFAULT_NETMASK"

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            print_usage
            exit 0
            ;;
        --upper)
            MACHINE_TYPE="upper"
            shift
            ;;
        --lower)
            MACHINE_TYPE="lower"
            shift
            ;;
        --netmask)
            NETMASK="$2"
            shift 2
            ;;
        *)
            log_error "未知参数: $1"
            print_usage
            exit 1
            ;;
    esac
done

# 主程序
main() {
    echo "======================================"
    echo "    Kuavo 固定IP设置脚本"
    echo "======================================"
    echo ""
    
    # 检查必需参数
    if [[ -z "$MACHINE_TYPE" ]]; then
        log_error "请指定机器类型: --upper 或 --lower"
        print_usage
        exit 1
    fi
    
    # 检查netplan工具
    if ! command -v netplan >/dev/null 2>&1; then
        log_info "检测到系统中未安装netplan工具，稍后将自动安装"
        echo ""
    fi
    

    
    # 确定目标IP
    local target_ip
    if [[ "$MACHINE_TYPE" == "upper" ]]; then
        target_ip="$DEFAULT_UPPER_IP"
    else
        target_ip="$DEFAULT_LOWER_IP"
    fi
    
    # 显示配置信息
    local machine_desc
    if [[ "$MACHINE_TYPE" == "upper" ]]; then
        machine_desc="上位机 (kuavo_slave)"
    else
        machine_desc="下位机 (kuavo_master)"
    fi
    
    log_info "配置参数:"
    echo "  机器类型:        $machine_desc"
    echo "  目标IP:          $target_ip"
    echo "  子网掩码:        $NETMASK"
    echo ""
    
    # 显示网络接口列表
    display_network_interfaces
    if [[ $? -ne 0 ]]; then
        log_error "无法获取网络接口"
        exit 1
    fi
    
    # 选择网络接口
    local interface
    interface=$(select_network_interface)
    if [[ $? -ne 0 ]]; then
        log_error "无法选择网络接口"
        exit 1
    fi
    
    echo ""
    log_info "使用网络接口: $interface"
    
    # 检查IP冲突
    check_ip_conflict "$target_ip" || log_warn "继续配置..."
    
    # 配置静态IP
    configure_static_ip "$interface" "$target_ip" "$NETMASK" "$MACHINE_TYPE"
    
    # 创建持久化配置
    if command -v netplan >/dev/null 2>&1; then
        create_netplan_config "$interface" "$target_ip" "$NETMASK" "$MACHINE_TYPE"
    else
        log_warn "未找到netplan工具，正在自动安装..."
        
        log_info "更新软件包列表..."
        if sudo apt update >/dev/null 2>&1; then
            log_info "正在安装netplan.io..."
            if sudo apt install -y netplan.io >/dev/null 2>&1; then
                log_success "netplan安装成功"
                create_netplan_config "$interface" "$target_ip" "$NETMASK" "$MACHINE_TYPE"
            else
                log_error "netplan安装失败"
                echo ""
                echo "请手动安装netplan："
                echo "  sudo apt update"
                echo "  sudo apt install netplan.io"
                echo ""
                echo "然后重新运行此脚本，或者手动创建网络配置。"
                echo ""
                log_warn "当前IP配置将在系统重启后丢失"
            fi
        else
            log_error "无法更新软件包列表"
            echo ""
            echo "请检查网络连接或手动安装netplan："
            echo "  sudo apt update"
            echo "  sudo apt install netplan.io"
            echo ""
            log_warn "当前IP配置将在系统重启后丢失"
        fi
    fi
    
    echo ""
    log_success "配置完成！"
    echo ""
    log_info "验证配置:"
    ip addr show "$interface" | grep inet || true
    echo ""
    log_info "测试连接:"
    echo "  ping $target_ip"
    
    # 如果是上位机，显示网关信息
    if [[ "$MACHINE_TYPE" == "upper" ]]; then
        echo ""
        log_info "网关配置:"
        ip route show default 2>/dev/null || echo "  未找到默认路由"
        echo "  测试网关连接: ping $DEFAULT_LOWER_IP"
    fi
    
    # 显示环境变量建议
    print_env_vars "$MACHINE_TYPE" "$target_ip"
}

# 检查是否以root权限运行
if [[ $EUID -ne 0 ]]; then
    log_error "请使用sudo运行此脚本"
    exit 1
fi

main "$@" 