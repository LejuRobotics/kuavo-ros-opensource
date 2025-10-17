#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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
if [[ $EUID -ne 0 ]]; then
    error "此脚本需要root权限运行"
    error "请使用: sudo $0"
    exit 1
fi

log "配置蓝牙用户权限"

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
    warn "无法获取当前用户信息"
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


cat << CAT_START > /etc/polkit-1/localauthority/50-local.d/org.freedesktop.Bluetooth.pkla

[bluetooth]
Identity=unix-group:bluetooth
Action=org.freedesktop.Bluetooth.*
ResultAny=yes
ResultInactive=no
ResultActive=yes

CAT_START

cat << CAT_START > /etc/polkit-1/localauthority/50-local.d/org.bluez.pkla

[bluez]
Identity=unix-group:bluetooth
Action=org.bluez.*
ResultAny=yes
ResultInactive=no
ResultActive=yes

CAT_START

systemctl restart bluetooth
systemctl restart polkit