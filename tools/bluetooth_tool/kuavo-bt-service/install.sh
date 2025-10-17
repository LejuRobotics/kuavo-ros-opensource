#!/bin/bash

# 蓝牙自动连接服务安装脚本

set -e

# 配置变量
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/opt/lejurobot/kuavo-bt-service"
SERVICE_NAME="kuavo-bt-service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
CONFIG_FILE="${INSTALL_DIR}/config.conf"
PYTHON_SCRIPT="${SCRIPT_DIR}/bluetooth_device_scanner.py"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查root权限
check_root() {
    if [[ $EUID -ne 0 ]]; then
        error "此脚本需要root权限运行"
        error "请使用: sudo $0"
        exit 1
    fi
}

# 创建安装目录
create_install_dir() {
    log "创建安装目录: $INSTALL_DIR"
    mkdir -p "$INSTALL_DIR"
    mkdir -p "$(dirname "$CONFIG_FILE")"
}

# 复制文件
copy_files() {
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    log "复制Python服务脚本"
    cp "$script_dir/kuavo-bt-service.py" "$INSTALL_DIR/"
    chmod +x "$INSTALL_DIR/kuavo-bt-service.py"
    
    log "复制设备检查脚本"
    cp "$script_dir/bluetooth_device_scanner.py" "$INSTALL_DIR/"
    chmod +x "$INSTALL_DIR/bluetooth_device_scanner.py"
    
    log "复制卸载脚本"
    cp "$script_dir/uninstall.sh" "$INSTALL_DIR/uninstall.sh"
    chmod +x "$INSTALL_DIR/uninstall.sh"
    
    log "复制服务文件"
    cp "$script_dir/kuavo-bt-service.service" "$SERVICE_FILE"
}

# 检查蓝牙控制器
check_bluetooth_controllers() {
    log "检查蓝牙控制器"
    
    # 等待蓝牙服务启动
    local timeout=10
    local count=0
    while [[ $count -lt $timeout ]]; do
        if systemctl is-active --quiet bluetooth; then
            break
        fi
        sleep 1
        ((count++))
    done
    
    echo ""
    echo -e "${GREEN}=== 蓝牙控制器信息 ===${NC}"
    echo "--------------------------------"
    hciconfig -a
    echo ""
    
    echo -e "${GREEN}=== 当前默认控制器 ===${NC}"
    echo "--------------------------------"
    bluetoothctl list
    echo ""
    
    echo ""
    echo -e "${YELLOW}💡 提示: 如果您想切换默认控制器，请输入 'n' 并选择其他控制器${NC}"
    echo -e "${YELLOW}💡 提示: 默认控制器将用于所有蓝牙设备连接操作${NC}"
    echo -e "${YELLOW}💡 提示: 通常建议使用外置USB蓝牙适配器而非内置控制器${NC}"
    echo ""
    echo -ne "${YELLOW}默认控制器是否正确? [Y/n]: ${NC}"
    read -n 1 -r
    echo
    if [[ $REPLY =~ ^[Nn]$ ]]; then
        # 获取所有控制器
        local controllers=()
        while IFS= read -r line; do
            if [[ $line =~ ^Controller\ ([0-9A-F:]+)\ (.+)$ ]]; then
                local mac="${BASH_REMATCH[1]}"
                local name="${BASH_REMATCH[2]}"
                controllers+=("$mac|$name")
            fi
        done < <(bluetoothctl list 2>/dev/null)
        
        if [[ ${#controllers[@]} -le 1 ]]; then
            echo "只有一个控制器可用，无法切换"
            return
        fi
        
        echo ""
        echo -e "${YELLOW}=== 请选择要作为默认的控制器 ===${NC}"
        echo -e "${YELLOW}💡 建议选择信号强、稳定性好的控制器${NC}"
        echo -e "${YELLOW}💡 外置USB蓝牙适配器通常比内置控制器更稳定${NC}"
        echo ""
        for i in "${!controllers[@]}"; do
            local controller="${controllers[$i]}"
            local mac="${controller%%|*}"
            local name="${controller##*|}"
            echo -e "  ${GREEN}$((i+1)).${NC} $name (${CYAN}$mac${NC})"
        done
        
        while true; do
            echo -ne "${YELLOW}请输入控制器编号 (1-${#controllers[@]}): ${NC}"
            read -n 1 -r
            echo
            if [[ $REPLY =~ ^[0-9]+$ ]] && [[ $REPLY -ge 1 ]] && [[ $REPLY -le ${#controllers[@]} ]]; then
                local selected_index=$((REPLY - 1))
                local selected_controller="${controllers[$selected_index]}"
                local selected_mac="${selected_controller%%|*}"
                
                echo -e "${GREEN}已选择控制器: ${CYAN}$selected_mac${NC}"
                echo -e "${YELLOW}💡 此控制器将被用于所有蓝牙设备连接操作${NC}"
                echo -e "${YELLOW}💡 服务启动时会自动设置此控制器为默认控制器${NC}"
                SELECTED_CONTROLLER="$selected_mac"
                break
            else
                echo -e "${RED}无效输入，请输入 1-${#controllers[@]} 之间的数字${NC}"
            fi
        done
    else
        echo -e "${GREEN}✓ 使用当前默认控制器${NC}"
        echo -e "${YELLOW}💡 此控制器将被用于所有蓝牙设备连接操作${NC}"
        echo -e "${YELLOW}💡 服务启动时会自动设置此控制器为默认控制器${NC}"
        # 获取当前默认控制器MAC地址
        while IFS= read -r line; do
            if [[ $line =~ ^Controller\ ([0-9A-F:]+)\ (.+)\ \[default\]$ ]]; then
                SELECTED_CONTROLLER="${BASH_REMATCH[1]}"
                break
            fi
        done < <(bluetoothctl list 2>/dev/null)
    fi
}

# 检查已连接的蓝牙设备
check_connected_devices() {
    log "检查已连接的蓝牙设备"
    
    # 使用Python脚本获取已连接设备
    local connected_devices=()
    
    # 检查Python脚本是否存在
    if [[ -f "$PYTHON_SCRIPT" ]]; then
        local script_output=$(python3 "$PYTHON_SCRIPT" --simple 2>/dev/null)
        
        if [[ -n "$script_output" ]]; then
            while IFS= read -r line; do
                if [[ $line =~ ^([0-9A-F:]+)\ (.+)$ ]]; then
                    local mac="${BASH_REMATCH[1]}"
                    local name="${BASH_REMATCH[2]}"
                    
                    # 检查设备信任和配对状态
                    local trusted_status="no"
                    local paired_status="no"
                    
                    # 使用bluetoothctl检查设备详细信息
                    local device_info=$(bluetoothctl info "$mac" 2>/dev/null)
                    if [[ -n "$device_info" ]]; then
                        if [[ "$device_info" =~ "Trusted: yes" ]]; then
                            trusted_status="yes"
                        fi
                        if [[ "$device_info" =~ "Paired: yes" ]]; then
                            paired_status="yes"
                        fi
                    fi
                    
                    connected_devices+=("$mac|$name|$trusted_status|$paired_status")
                fi
            done <<< "$script_output"
        else
            log "Python脚本执行成功，但没有输出设备信息"
        fi
    else
        error "Python脚本不存在: $PYTHON_SCRIPT"
        error "请确保 bluetooth_device_scanner.py 文件存在"
        exit 1
    fi
    
    # 显示找到的已连接设备
    if [[ ${#connected_devices[@]} -gt 0 ]]; then
        echo ""
        echo -e "${GREEN}=== 找到已连接的设备 ===${NC}"
        for device_info in "${connected_devices[@]}"; do
            local mac=$(echo "$device_info" | cut -d'|' -f1)
            local name=$(echo "$device_info" | cut -d'|' -f2)
            local trusted=$(echo "$device_info" | cut -d'|' -f3)
            local paired=$(echo "$device_info" | cut -d'|' -f4)
            
            echo -e "  ${GREEN}•${NC} $name (${CYAN}$mac${NC})"
            echo -e "    ${CYAN}状态:${NC} 配对=$paired, 信任=$trusted, 连接=yes"
        done
        echo ""
        echo -e "${YELLOW}注意: 服务将自动连接这些设备${NC}"
        echo ""
        
        echo -ne "${YELLOW}是否继续安装? [Y/n]: ${NC}"
        read -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            echo -e "${YELLOW}⚠️  安装已取消${NC}"
            exit 0
        fi
    else
        echo ""
        error "❌ 未找到已连接的蓝牙设备"
        echo ""
        echo -e "${YELLOW}=== 请先连接蓝牙设备，然后重新运行安装脚本 ===${NC}"
        echo ""
        exit 1
    fi
    
    # 保存连接设备列表供配置文件使用（只保存MAC和名称）
    CONNECTED_DEVICES_LIST=()
    for device_info in "${connected_devices[@]}"; do
        local mac=$(echo "$device_info" | cut -d'|' -f1)
        local name=$(echo "$device_info" | cut -d'|' -f2)
        CONNECTED_DEVICES_LIST+=("$mac|$name")
    done
}

# 创建配置文件
create_config() {
    log "创建/更新配置文件"
    # 从已连接设备列表中提取MAC地址
    local target_devices=""
    log "已连接设备数量: ${#CONNECTED_DEVICES_LIST[@]}"
    if [[ ${#CONNECTED_DEVICES_LIST[@]} -gt 0 ]]; then
        for device in "${CONNECTED_DEVICES_LIST[@]}"; do
            local mac="${device%%|*}"
            log "处理设备: $device, MAC: $mac"
            if [[ -z "$target_devices" ]]; then
                target_devices="$mac"
            else
                target_devices="$target_devices,$mac"
            fi
        done
    fi
    log "最终目标设备: $target_devices"
    
    cat > "$CONFIG_FILE" << EOF
# 蓝牙自动连接服务配置文件

# 目标设备列表 (MAC地址或设备名称，用逗号分隔)
TARGET_DEVICES="$target_devices"

# 默认蓝牙控制器MAC地址
DEFAULT_CONTROLLER="$SELECTED_CONTROLLER"
EOF
}

# 设置用户蓝牙权限
setup_bluetooth_permissions() {
    log "设置用户蓝牙权限"
    
    # 获取当前用户（调用sudo的用户）
    if [[ -n "$SUDO_USER" ]]; then
        CURRENT_USER="$SUDO_USER"
        log "当前用户: $CURRENT_USER"
        
        # 将当前用户添加到bluetooth组
        if ! groups "$CURRENT_USER" | grep -q bluetooth; then
            usermod -aG bluetooth "$CURRENT_USER"
            log "已将用户 $CURRENT_USER 添加到bluetooth组"
        else
            log "用户 $CURRENT_USER 已经在bluetooth组中"
        fi
    else
        log "警告: 无法获取当前用户信息"
    fi
    
    # 检查lab用户是否存在
    if id "lab" &>/dev/null; then
        log "lab用户存在，添加到bluetooth组"
        if ! groups "lab" | grep -q bluetooth; then
            usermod -aG bluetooth "lab"
            log "已将用户 lab 添加到bluetooth组"
        else
            log "用户 lab 已经在bluetooth组中"
        fi
    else
        log "lab用户不存在，跳过"
    fi
    
    }

# 设置权限
set_permissions() {
    log "设置文件权限"
    chown -R root:root "$INSTALL_DIR"
    chmod 755 "$INSTALL_DIR"
    chmod 644 "$SERVICE_FILE"
    chmod 644 "$CONFIG_FILE"
}

# 重新加载systemd
reload_systemd() {
    log "重新加载systemd配置"
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
}

# 启动服务
start_service() {
    log "启动蓝牙自动连接服务"
    systemctl start "$SERVICE_NAME"
    
    # 检查服务状态
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        log "服务启动成功"
    else
        error "服务启动失败"
        systemctl status "$SERVICE_NAME"
        exit 1
    fi
}

# 显示使用说明
show_instructions() {
    echo ""
    echo -e "${GREEN}安装完成！${NC}"
    echo ""
    echo -e "${YELLOW}🔧 服务管理:${NC}"
    echo "  启动服务:    systemctl start $SERVICE_NAME"
    echo "  停止服务:    systemctl stop $SERVICE_NAME"
    echo "  重启服务:    systemctl restart $SERVICE_NAME"
    echo "  查看状态:    systemctl status $SERVICE_NAME"
    echo "  开机自启:    systemctl enable $SERVICE_NAME"
    echo "  禁用自启:    systemctl disable $SERVICE_NAME"
    echo ""
    echo -e "${YELLOW}⚙️ 配置文件:${NC}"
    echo "  位置: $CONFIG_FILE"
    echo "  编辑配置后重启服务生效"
    echo ""
    echo -e "${YELLOW}📊 日志查看:${NC}"
    echo "  服务日志:    journalctl -u $SERVICE_NAME -f"
    echo ""
    echo -e "${YELLOW}💡 控制器持久化:${NC}"
    echo "  1. 服务启动时自动设置默认控制器"
    echo "  2. 服务运行时每10秒检查一次控制器状态"
    echo "  3. 如果控制器被重置，会自动重新设置"
    echo "  4. 控制器配置保存在服务配置文件中"
    echo ""
    echo -e "${YELLOW}⚠️ 注意事项:${NC}"
    echo "  1. 确保蓝牙服务已启动: systemctl status bluetooth"
    echo "  2. 当前用户已自动添加到bluetooth组中"
    echo "  3. 如果lab用户存在，也已添加到bluetooth组中"
    echo "  4. 首次使用前请先配对并信任蓝牙设备"
    echo "  5. 控制器设置需要root权限"
    echo ""
}

# 停止现有服务
stop_existing_service() {
    if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
        log "停止现有服务"
        systemctl stop "$SERVICE_NAME" 2>/dev/null || true
    fi
    
    if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        log "禁用现有服务"
        systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    fi
}

# 清理旧文件
cleanup_old_files() {
    log "清理旧文件"
    
    # 删除旧的服务文件
    if [[ -f "$SERVICE_FILE" ]]; then
        rm -f "$SERVICE_FILE"
        systemctl daemon-reload 2>/dev/null || true
    fi
    
        
        
    # 删除旧的安装目录
    if [[ -d "$INSTALL_DIR" ]]; then
        # 备份配置文件
        if [[ -f "$CONFIG_FILE" ]]; then
            log "备份配置文件"
            cp "$CONFIG_FILE" "/tmp/kuavo-bt-config.conf.bak"
        fi
        
        # 删除安装目录
        rm -rf "$INSTALL_DIR"
    fi
}

# 主函数
main() {
    check_root
    log "开始安装蓝牙自动连接服务"
    
    # 停止现有服务并清理旧文件
    stop_existing_service
    cleanup_old_files
    
    check_bluetooth_controllers
    check_connected_devices
    create_install_dir
    copy_files
    create_config
    set_permissions
    setup_bluetooth_permissions
    reload_systemd
    start_service
    show_instructions
}

# 运行主函数
main "$@"