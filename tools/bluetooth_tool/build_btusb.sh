#!/bin/bash

set -e

# 内核源码目录配置
KERNEL_DIR="/home/lab/kenel/linux-5.15.158"
BUILD_DIR="/tmp/build-kernel"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log() {
    local color=""
    case $1 in
        "INFO") color=$BLUE ;;
        "SUCCESS") color=$GREEN ;;
        "WARNING") color=$YELLOW ;;
        "ERROR") color=$RED ;;
    esac
    echo -e "${color}[$1]${NC} $2"
}

# 检查是否为root用户
check_root() {
    if [[ $EUID -ne 0 ]]; then
        log "ERROR" "此脚本需要root权限运行"
        log "INFO" "请使用: sudo $0"
        exit 1
    fi
}

# 检查内核源码目录
check_kernel_source() {
    if [[ ! -d "$KERNEL_DIR" ]]; then
        log "ERROR" "内核源码目录不存在: $KERNEL_DIR"
        log "INFO" "请确保内核源码路径正确"
        echo "提示：可以从其他机器人上获取或者找乐聚技术支持人员获取"
        exit 1
    fi
    
    echo "$KERNEL_DIR"
}

# 步骤1: 准备构建环境
prepare_build_env() {
    local kernel_dir=$1
    
    log "INFO" "步骤1: 准备构建环境"
    
    # 复制内核源码到构建目录
    log "INFO" "复制内核源码到构建目录"
    if [[ -d "$BUILD_DIR" ]]; then
        log "INFO" "清理已存在的构建目录"
        rm -rf "$BUILD_DIR"
    fi
    
    log "INFO" "复制内核源码从 $kernel_dir 到 $BUILD_DIR"
    cp -r "$kernel_dir" "$BUILD_DIR"
    
    cd "$BUILD_DIR"
    
    # 复制当前系统内核配置
    log "INFO" "复制当前系统内核配置"
    local boot_config="/boot/config-$(uname -r)"
    
    if [[ -f "$boot_config" ]]; then
        log "INFO" "从 /boot/config-$(uname -r) 复制配置"
        cp "$boot_config" .config
    fi
    
    # 确保蓝牙相关配置都启用（基于当前工作配置）
    log "INFO" "启用蓝牙相关配置"
    scripts/config --module CONFIG_BT
    scripts/config --enable CONFIG_BT_BREDR
    scripts/config --enable CONFIG_BT_LE
    scripts/config --enable CONFIG_BT_DEBUGFS
    scripts/config --module CONFIG_BT_INTEL
    scripts/config --module CONFIG_BT_BCM
    scripts/config --module CONFIG_BT_RTL
    scripts/config --module CONFIG_BT_HCIBTUSB
    scripts/config --enable CONFIG_BT_HCIBTUSB_BCM
    scripts/config --enable CONFIG_BT_HCIBTUSB_RTL
    scripts/config --module CONFIG_BT_HCIBTSDIO
    scripts/config --module CONFIG_BT_HCIUART
    scripts/config --enable CONFIG_BT_HCIUART_SERDEV
    scripts/config --module CONFIG_BT_HCIRSI
    scripts/config --enable CONFIG_BTT
    
    # 蓝牙协议支持
    scripts/config --module CONFIG_BT_RFCOMM
    scripts/config --enable CONFIG_BT_RFCOMM_TTY
    
    # 准备构建环境
    log "INFO" "准备构建环境"
    make olddefconfig
    make modules_prepare
    
    log "SUCCESS" "构建环境准备完成"
}

# 步骤2: 编译蓝牙核心模块
compile_bt_modules() {
    log "INFO" "步骤2: 编译蓝牙核心模块"
    
    # 编译HCI核心模块
    echo "编译HCI核心模块..."
    make M=net/bluetooth modules
    
    # 验证蓝牙核心模块
    if [[ -f "net/bluetooth/bluetooth.ko" ]]; then
        log "SUCCESS" "蓝牙核心模块编译成功"
    else
        log "ERROR" "蓝牙核心模块编译失败"
        exit 1
    fi
    
    
    # 编译蓝牙驱动模块
    echo "编译蓝牙驱动模块..."
    make -j$(nproc) M=drivers/bluetooth KBUILD_EXTRA_SYMBOLS=$(pwd)/net/bluetooth/Module.symvers
    
    # 验证关键模块
    if [[ -f "drivers/bluetooth/btusb.ko" ]]; then
        log "SUCCESS" "蓝牙USB驱动模块编译成功"
    else
        log "ERROR" "蓝牙USB驱动模块编译失败"
        exit 1
    fi
    
    # 显示生成的蓝牙模块
    log "INFO" "已生成的蓝牙模块:"
    find . -name "*.ko" -path "*/bluetooth/*" | head -10
}

# 步骤3: 安装模块到系统目录
install_modules() {
    log "INFO" "步骤3: 安装模块到系统目录"
    
    local kernel_version=$(uname -r)
    local net_bt_dir="/lib/modules/$kernel_version/kernel/net/bluetooth"
    local drivers_bt_dir="/lib/modules/$kernel_version/kernel/drivers/bluetooth"
    
    # 创建必要的目录结构
    log "INFO" "创建目录结构"
    mkdir -p "$net_bt_dir"
    mkdir -p "$drivers_bt_dir"
    
    # 只复制蓝牙相关的模块
    log "INFO" "安装蓝牙核心模块"
    cp ./net/bluetooth/*.ko "$net_bt_dir/"
    
    log "INFO" "安装蓝牙协议模块"
    cp -r ./net/bluetooth/rfcomm/*.ko "$net_bt_dir/" 2>/dev/null || true
    cp -r ./net/bluetooth/bnep/*.ko "$net_bt_dir/" 2>/dev/null || true
    cp -r ./net/bluetooth/hidp/*.ko "$net_bt_dir/" 2>/dev/null || true
    
    log "INFO" "安装蓝牙驱动模块"
    cp ./drivers/bluetooth/*.ko "$drivers_bt_dir/"
    
    # 显示已安装的模块
    log "INFO" "已安装的蓝牙模块:"
    ls -la "$net_bt_dir/" "$drivers_bt_dir/"
    
    log "SUCCESS" "蓝牙模块安装完成"
}

# 步骤4: 更新模块依赖关系
update_module_deps() {
    log "INFO" "步骤4: 更新模块依赖关系"
    depmod -a
    log "SUCCESS" "模块依赖关系更新完成"
}

# 步骤5: 加载蓝牙核心模块
load_bt_modules() {
    log "INFO" "步骤5: 加载蓝牙核心模块"
    
    # 加载蓝牙核心模块
    log "INFO" "加载蓝牙核心模块"
    modprobe bluetooth
    
    # 加载蓝牙协议模块
    log "INFO" "加载RFCOMM协议模块"
    modprobe rfcomm
    
    # 加载蓝牙USB驱动
    log "INFO" "加载蓝牙USB驱动"
    modprobe btusb
    
    # 验证模块加载
    log "INFO" "验证模块加载"
    if lsmod | grep -q -E 'bluetooth|btusb|rfcomm'; then
        log "SUCCESS" "蓝牙模块加载成功"
        lsmod | grep -E 'bluetooth|btusb|rfcomm'
    else
        log "ERROR" "蓝牙模块加载失败"
        exit 1
    fi
}

# 步骤6: 配置开机自动加载
setup_autoload() {
    log "INFO" "步骤6: 配置开机自动加载"
    
    # 创建配置文件确保每次启动自动加载模块
    echo "bluetooth" > /etc/modules-load.d/bluetooth.conf
    echo "rfcomm" >> /etc/modules-load.d/bluetooth.conf
    echo "btusb" >> /etc/modules-load.d/bluetooth.conf
    echo "snd-usb-audio" >> /etc/modules-load.d/bluetooth.conf
    
    log "SUCCESS" "开机自动加载配置完成"
}

# 步骤7: 重启蓝牙服务
restart_bt_service() {
    log "INFO" "步骤7: 重启蓝牙服务"
    
    # 重启蓝牙服务
    if systemctl is-active --quiet bluetooth; then
        systemctl restart bluetooth
        log "SUCCESS" "蓝牙服务重启完成"
    else
        systemctl start bluetooth
        log "SUCCESS" "蓝牙服务启动完成"
    fi
    
    # 开启蓝牙
    log "INFO" "开启蓝牙"
    if command -v rfkill &> /dev/null; then
        rfkill unblock bluetooth
        log "SUCCESS" "蓝牙已开启"
    else
        log "WARNING" "rfkill 命令不可用，跳过蓝牙开启"
    fi
}

# 步骤8: 设置用户蓝牙权限
setup_bluetooth_permissions() {
    log "INFO" "步骤8: 设置用户蓝牙权限"
    
    # 获取当前用户（调用sudo的用户）
    if [[ -n "$SUDO_USER" ]]; then
        CURRENT_USER="$SUDO_USER"
        log "INFO" "当前用户: $CURRENT_USER"
        
        # 将当前用户添加到bluetooth组
        if ! groups "$CURRENT_USER" | grep -q bluetooth; then
            usermod -aG bluetooth "$CURRENT_USER"
            log "SUCCESS" "已将用户 $CURRENT_USER 添加到bluetooth组"
        else
            log "INFO" "用户 $CURRENT_USER 已经在bluetooth组中"
        fi
    else
        log "WARNING" "无法获取当前用户信息"
    fi
    
    # 检查lab用户是否存在
    if id "lab" &>/dev/null; then
        log "INFO" "lab用户存在，添加到bluetooth组"
        if ! groups "lab" | grep -q bluetooth; then
            usermod -aG bluetooth "lab"
            log "SUCCESS" "已将用户 lab 添加到bluetooth组"
        else
            log "INFO" "用户 lab 已经在bluetooth组中"
        fi
    else
        log "INFO" "lab用户不存在，跳过"
    fi
    
        
    log "SUCCESS" "用户蓝牙权限设置完成"
}

# 步骤9: 检查蓝牙控制器
check_bt_controller() {
    log "INFO" "步骤9: 检查蓝牙控制器"
    
    # 检查蓝牙控制器
    log "INFO" "检查蓝牙控制器状态"
    if command -v hciconfig &> /dev/null; then
        hciconfig -a
    else
        log "WARNING" "hciconfig 命令不可用"
    fi
    
    # 检查蓝牙服务状态
    log "INFO" "检查蓝牙服务状态"
    systemctl status bluetooth --no-pager -l
    
    log "SUCCESS" "蓝牙控制器检查完成"
}

# 步骤9: 列出蓝牙适配器并创建别名
list_and_alias_bt_adapter() {
    log "INFO" "步骤9: 列出蓝牙适配器并创建别名"
    
    # 等待蓝牙服务完全启动
    sleep 3
    
    if command -v bluetoothctl &> /dev/null; then
        # 列出本地蓝牙适配器
        log "INFO" "列出本地蓝牙适配器:"
        bluetoothctl list
        
        # 获取所有适配器
        local adapters=()
        while IFS= read -r line; do
            local adapter=$(echo "$line" | awk '{print $2}' | tr -d ' ')
            if [[ -n "$adapter" ]]; then
                adapters+=("$adapter")
            fi
        done < <(bluetoothctl list)
        
        if [[ ${#adapters[@]} -eq 0 ]]; then
            log "WARNING" "未发现蓝牙适配器"
            return
        fi
        
        log "INFO" "发现 ${#adapters[@]} 个蓝牙适配器"
        
        # 显示所有适配器信息
        for adapter in "${adapters[@]}"; do
            log "INFO" "适配器 $adapter 信息:"
            bluetoothctl show "$adapter"
            echo
        done
        
        # 询问是否要创建别名
        echo
        read -p "是否要为蓝牙适配器创建别名? (y/n): " create_alias
        if [[ "$create_alias" == "y" || "$create_alias" == "Y" ]]; then
            if [[ ${#adapters[@]} -eq 1 ]]; then
                # 单个适配器，直接设置
                local adapter="${adapters[0]}"
                read -p "请输入适配器别名: " adapter_alias
                if [[ -n "$adapter_alias" ]]; then
                    log "INFO" "设置适配器 $adapter 别名: $adapter_alias"
                    bluetoothctl -- select "$adapter" && bluetoothctl -- system-alias "$adapter_alias"
                    log "SUCCESS" "适配器 $adapter 别名设置为 $adapter_alias"
                fi
            else
                # 多个适配器，让用户选择
                echo "请选择要设置别名的适配器:"
                for i in "${!adapters[@]}"; do
                    echo "$((i+1)). ${adapters[i]}"
                done
                
                while true; do
                    read -p "请输入适配器编号 (1-${#adapters[@]}) 或输入q退出: " choice
                    if [[ "$choice" == "q" ]]; then
                        break
                    fi
                    
                    if [[ "$choice" =~ ^[0-9]+$ ]] && [[ "$choice" -ge 1 ]] && [[ "$choice" -le ${#adapters[@]} ]]; then
                        local adapter="${adapters[$((choice-1))]}"
                        read -p "请输入适配器别名: " adapter_alias
                        if [[ -n "$adapter_alias" ]]; then
                            log "INFO" "设置适配器 $adapter 别名: $adapter_alias"
                            bluetoothctl -- select "$adapter" && bluetoothctl -- system-alias "$adapter_alias"
                            log "SUCCESS" "适配器 $adapter 别名设置为 $adapter_alias"
                        fi
                        break
                    else
                        log "ERROR" "无效的选择，请输入 1-${#adapters[@]} 之间的数字"
                    fi
                done
            fi
        fi
        
        # 最终显示所有适配器信息
        log "INFO" "最终蓝牙适配器信息:"
        for adapter in "${adapters[@]}"; do
            bluetoothctl show "$adapter"
            echo
        done
        
    else
        log "WARNING" "bluetoothctl 命令不可用，无法管理蓝牙适配器"
    fi
    
    log "SUCCESS" "蓝牙适配器别名设置完成"
}

# 显示帮助信息
show_help() {
    cat << EOF
蓝牙模块构建脚本

用法: sudo $0

此脚本将自动执行以下步骤：
    1  准备构建环境
    2  编译蓝牙核心模块
    3  安装模块到系统目录
    4  更新模块依赖关系
    5  加载蓝牙核心模块
    6  配置开机自动加载
    7  重启蓝牙服务
    8  检查蓝牙控制器

EOF
}

# 主函数
main() {
    # 如果提供了参数，显示帮助信息
    if [[ $# -gt 0 ]]; then
        show_help
        exit 0
    fi
    
    # 检查root权限
    check_root
    
    # 检查内核源码目录
    local kernel_dir=$(check_kernel_source)
    
    log "INFO" "开始构建蓝牙模块"
    log "INFO" "内核源码目录: $kernel_dir"
    
    # 执行所有步骤
    prepare_build_env "$kernel_dir"
    compile_bt_modules
    install_modules
    update_module_deps
    load_bt_modules
    setup_autoload
    restart_bt_service
    setup_bluetooth_permissions
    check_bt_controller
    
    log "SUCCESS" "蓝牙模块构建完成！"
}

# 执行主函数
main "$@"