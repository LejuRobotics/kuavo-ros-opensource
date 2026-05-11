#!/bin/bash

set -e

# 日志函数
log() {
    echo "[$(date '+%F %T')] $1"
}

# 检查 root 权限
if [[ $EUID -ne 0 ]]; then
    log "错误：此脚本必须以 root 权限运行！"
    log "请使用 sudo 执行: sudo $0"
    exit 1
fi

# ============================================================
# USB 声卡 (C-Media CM108 Audio Controller) 配置
# 设备 ID: 0d8c:013c
#
# 原理：
# 1. udev 规则：USB 声卡插入时加载 snd-usb-audio 驱动
# 2. blacklist：禁止 Intel HDA 声卡模块加载
# 结果：系统只剩 USB 声卡（card 0），所有程序默认输出指向它
# ============================================================

VENDOR_ID="0d8c"
PRODUCT_ID="013c"

log "===== 开始配置 USB 声卡 (VendorID=$VENDOR_ID ProductID=$PRODUCT_ID) ====="

# 1. 配置 udev 规则，确保 USB 声卡插入时加载驱动
UDEV_RULES_FILE="/etc/udev/rules.d/90-usb-audio.rules"
log "正在配置 udev 规则 $UDEV_RULES_FILE ..."
cat > "$UDEV_RULES_FILE" <<EOF
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="$VENDOR_ID", ATTR{idProduct}=="$PRODUCT_ID", RUN+="/sbin/modprobe snd-usb-audio"
EOF
udevadm control --reload
log "udev 规则配置完成。"

# 2. blacklist Intel HDA 声卡，使 USB 声卡成为唯一声卡
MODPROBE_CONF="/etc/modprobe.d/alsa-card-order.conf"
log "正在配置 $MODPROBE_CONF ..."
cat > "$MODPROBE_CONF" <<EOF
# 完全禁止 Intel HDA 声卡模块自动加载
blacklist snd_hda_intel
blacklist snd_hda_codec_realtek
blacklist snd_hda_codec_generic
blacklist snd_hda_codec
blacklist snd_hda_core
blacklist snd_intel_dspcfg
blacklist snd_intel_sdw_acpi
EOF
log "$MODPROBE_CONF 配置完成。"

# 3. 删除旧的 .asoundrc（只剩一个声卡，无需手动指定默认设备）
for asoundrc in /root/.asoundrc /home/lab/.asoundrc; do
    if [ -f "$asoundrc" ]; then
        rm -f "$asoundrc"
        log "已删除旧的 $asoundrc（不再需要）"
    fi
done

# 4. 允许 root 用户使用 PulseAudio
PULSE_SERVICE_FILE="/lib/systemd/user/pulseaudio.service"
if [ -f "$PULSE_SERVICE_FILE" ]; then
    if grep -q "^ConditionUser=!root" "$PULSE_SERVICE_FILE"; then
        sed -i 's/^ConditionUser=!root/#ConditionUser=!root/' "$PULSE_SERVICE_FILE"
        log "已注释 $PULSE_SERVICE_FILE 中的 ConditionUser=!root"
    else
        log "$PULSE_SERVICE_FILE 中无需修改"
    fi
fi

# 5. 将用户加入 audio 组
for user in root lab; do
    if id "$user" &>/dev/null; then
        usermod -aG audio "$user"
        log "$user 用户已加入 audio 组"
    fi
done

# 6. 安装依赖
log "检查并安装依赖..."
if ! command -v alsa &>/dev/null; then
    log "安装 alsa-utils..."
    apt-get install -y alsa-utils
fi

if ! command -v sox &>/dev/null; then
    log "安装 sox..."
    apt-get install -y sox
fi

# 7. 创建音乐目录并拷贝文件
MUSIC_DIR="/home/lab/.config/lejuconfig/music"
mkdir -p "$MUSIC_DIR"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_MUSIC_DIR="$SCRIPT_DIR/music"
if [ -d "$SRC_MUSIC_DIR" ]; then
    cp -rf "$SRC_MUSIC_DIR/"* "$MUSIC_DIR"/
    log "已将 $SRC_MUSIC_DIR 下的文件拷贝到 $MUSIC_DIR"
else
    log "未找到 $SRC_MUSIC_DIR 目录，跳过文件拷贝"
fi

# 8. 更新 initramfs 并重载 ALSA
log "更新 initramfs..."
update-initramfs -u

log "执行 alsa force-reload..."
alsa force-reload || log "alsa force-reload 失败，重启后生效"

log "===== 声卡配置完成 ====="
log "重启后 aplay -l 应只显示 USB 声卡 (card 0)。"
