#!/bin/bash

# 日志函数
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# --- 设备信息 ---
vendor_id="4c4a"
product_id="4155"


AUDIO_MODULE_RULES_FILE="/etc/udev/rules.d/99-custom-jieli-audio.rules"

log "使用固定的 USB 麦克风设备 ID: VendorID=$vendor_id ProductID=$product_id"

rule_line="ACTION==\"add\", SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vendor_id\", ATTR{idProduct}==\"$product_id\", RUN+=\"/sbin/modprobe snd-usb-audio\""

# --- 检查并写入规则 ---
if [ ! -f "$AUDIO_MODULE_RULES_FILE" ]; then
    echo "$rule_line" | sudo tee "$AUDIO_MODULE_RULES_FILE" > /dev/null
    log "已创建新的规则文件并添加规则: $AUDIO_MODULE_RULES_FILE"
else
    log "规则已存在于文件: VendorID=$vendor_id ProductID=$product_id"
fi

# --- 重新加载 udev 规则并触发事件 ---
log "重新加载 udev 规则并触发事件..."

# 使用 --reload-rules 重新加载规则文件
if sudo udevadm control --reload-rules; then
    log "udev 规则已重新加载。"
else
    log "错误：udev 规则重新加载失败。"
    exit 1
fi

# 触发 udev 事件，使新规则对已连接设备生效
if sudo udevadm trigger; then
    log "udev 事件已触发，规则应该已生效。"
else
    log "错误：udev 事件触发失败。"
    exit 1
fi

log "脚本执行完毕。"