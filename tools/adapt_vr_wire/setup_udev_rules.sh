#!/bin/bash

set -e

# 日志函数
log() {
    echo "[`date '+%F %T'`] $1"
}

# 检查 root 权限
if [[ $EUID -ne 0 ]]; then
    log "错误：此脚本必须以 root 权限运行！"
    log "请使用 sudo 执行: sudo $0"
    exit 1
fi

# 定义udev规则文件路径
ASIX_RULE_SOURCE="/home/lab/kuavo-ros-opensource/tools/adapt_vr_wire/99-usb-net-asix.rules"
REALTEK_RULE_SOURCE="/home/lab/kuavo-ros-opensource/tools/adapt_vr_wire/99-usb-net-realtek.rules"
ASIX_RULE_DEST="/etc/udev/rules.d/99-usb-net-asix.rules"
REALTEK_RULE_DEST="/etc/udev/rules.d/99-usb-net-realtek.rules"

# 检查源规则文件是否存在
if [ ! -f "$ASIX_RULE_SOURCE" ]; then
    log "错误：ASIX udev规则文件 $ASIX_RULE_SOURCE 不存在！"
    exit 1
fi

if [ ! -f "$REALTEK_RULE_SOURCE" ]; then
    log "错误：Realtek udev规则文件 $REALTEK_RULE_SOURCE 不存在！"
    exit 1
fi

# 复制udev规则文件
log "复制 ASIX 网卡 udev 规则文件..."
sudo cp $ASIX_RULE_SOURCE $ASIX_RULE_DEST

log "复制 Realtek 网卡 udev 规则文件..."
sudo cp $REALTEK_RULE_SOURCE $REALTEK_RULE_DEST

# 重新加载udev规则并重启服务使其生效
log "重新加载 udev 规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger


log "=== udev规则配置完成 ==="
log "之后可运行 configure_dhcp.sh 完成DHCP配置"
log "请您重启机器人！！！"