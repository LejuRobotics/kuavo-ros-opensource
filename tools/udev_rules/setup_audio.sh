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

# 固定的 VendorID 和 ProductID
vendor_id="1b3f"
product_id="2008"
AUDIO_MODULE_RULES_FILE="/etc/udev/rules.d/90-usb-audio.rules"
log "使用固定的 USB 音频设备 ID: VendorID=$vendor_id ProductID=$product_id"

# 生成规则条目
rule_line="ACTION==\"add\", SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vendor_id\", ATTR{idProduct}==\"$product_id\", RUN+=\"/sbin/modprobe snd-usb-audio\""

# 检查是否已存在相同规则
if ! grep -q "$rule_line" "$AUDIO_MODULE_RULES_FILE" 2>/dev/null; then
    echo "$rule_line" | tee -a "$AUDIO_MODULE_RULES_FILE"
    log "已创建规则: $AUDIO_MODULE_RULES_FILE"
else
    log "已存在规则: VendorID=$vendor_id ProductID=$product_id"
fi

# 复制其他 udev 规则文件
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_RULES_DIR="/etc/udev/rules.d"

# 确保目标目录存在
if [ ! -d "$UDEV_RULES_DIR" ]; then
    log "错误：目标目录 $UDEV_RULES_DIR 不存在！"
    exit 1
fi

# 复制 90-roban-audio.rules
ROBAN_AUDIO_RULES_SRC="$SCRIPT_DIR/90-roban-audio.rules"
ROBAN_AUDIO_RULES_DST="$UDEV_RULES_DIR/90-roban-audio.rules"
if [ -f "$ROBAN_AUDIO_RULES_SRC" ]; then
    if cp -f "$ROBAN_AUDIO_RULES_SRC" "$ROBAN_AUDIO_RULES_DST"; then
        log "已复制规则文件: $ROBAN_AUDIO_RULES_DST"
    else
        log "错误：复制规则文件失败: $ROBAN_AUDIO_RULES_DST"
        exit 1
    fi
else
    log "警告：未找到源文件: $ROBAN_AUDIO_RULES_SRC，跳过复制"
fi

# 复制 99-usb-net.rules
USB_NET_RULES_SRC="$SCRIPT_DIR/99-usb-net.rules"
USB_NET_RULES_DST="$UDEV_RULES_DIR/99-usb-net.rules"
if [ -f "$USB_NET_RULES_SRC" ]; then
    if cp -f "$USB_NET_RULES_SRC" "$USB_NET_RULES_DST"; then
        log "已复制规则文件: $USB_NET_RULES_DST"
    else
        log "错误：复制规则文件失败: $USB_NET_RULES_DST"
        exit 1
    fi
else
    log "警告：未找到源文件: $USB_NET_RULES_SRC，跳过复制"
fi

# 重新加载 udev 规则
log "重新加载 udev 规则..."
if udevadm control --reload; then
    log "udev 规则已成功重新加载"
else
    log "udev 规则重新加载失败"
    exit 1
fi

if udevadm trigger; then
    log "udev 触发成功"
else
    log "udev 触发失败"
    exit 1
fi

log "操作完成！请重新插入 USB 音频设备以测试规则。"

log "声卡配置完成，进行root用户权限配置。"
# 注释 /lib/systemd/user/pulseaudio.service 文件中的 ConditionUser=!root 一行
PULSE_SERVICE_FILE="/lib/systemd/user/pulseaudio.service"
if [ -f "$PULSE_SERVICE_FILE" ]; then
    if grep -q "^ConditionUser=!root" "$PULSE_SERVICE_FILE"; then
        sed -i 's/^ConditionUser=!root/#ConditionUser=!root/' "$PULSE_SERVICE_FILE"
        log "已注释 $PULSE_SERVICE_FILE 中的 ConditionUser=!root"
    else
        log "$PULSE_SERVICE_FILE 中未找到 ConditionUser=!root，无需修改"
    fi
else
    log "未找到 $PULSE_SERVICE_FILE 文件，跳过注释操作"
fi
log "正在创建 /root/.asoundrc 配置文件..."
cat >/root/.asoundrc <<EOF
pcm.!default {
  type hw
  card 1
}

ctl.!default {
  type hw
  card 1
}
EOF
log "/root/.asoundrc 配置文件已创建。"

log "执行 alsa force-reload..."
if ! command -v alsa &> /dev/null; then
    log "alsa 未安装，正在尝试安装 alsa-utils..."
    if apt-get install -y alsa-utils; then
        log "alsa-utils 安装成功。"
    else
        log "alsa-utils 安装失败，请手动安装。"
        exit 1
    fi
fi

log "安装sox...."
if apt-get install -y sox; then
    log "sox 安装成功。"
else
    log "sox 安装失败，请手动安装。"
    exit 1
fi  

if sudo alsa force-reload; then
    log "alsa force-reload 执行成功。"
else
    log "alsa force-reload 执行失败，请检查 alsa 是否已正确安装。"
fi

# 将 change_dhcp_host 目录下的 music 文件夹中的所有文件拷贝到 /home/lab/.config/lejuconfig/music 目录下
SRC_MUSIC_DIR="$(cd "$SCRIPT_DIR/../change_dhcp_host" && pwd)/music"
MUSIC_DIR="/home/lab/.config/lejuconfig/music"

# 在 /home/lab/.config/lejuconfig/ 目录下创建 music 文件夹
if [ ! -d "$MUSIC_DIR" ]; then
    mkdir -p "$MUSIC_DIR"
    log "已创建目录 $MUSIC_DIR"
else
    log "目录 $MUSIC_DIR 已存在，无需创建"
fi

if [ -d "$SRC_MUSIC_DIR" ]; then
    # 检查目标目录是否存在
    if [ ! -d "$MUSIC_DIR" ]; then
        mkdir -p "$MUSIC_DIR"
        log "已创建目标目录: $MUSIC_DIR"
    fi
    # 检查源目录是否为空
    if [ -n "$(ls -A "$SRC_MUSIC_DIR" 2>/dev/null)" ]; then
        cp -rf "$SRC_MUSIC_DIR/"* "$MUSIC_DIR"/
        log "已将 $SRC_MUSIC_DIR 下的文件拷贝到 $MUSIC_DIR"
    else
        log "$SRC_MUSIC_DIR 目录为空，跳过文件拷贝"
    fi
else
    log "未找到 $SRC_MUSIC_DIR 目录，跳过文件拷贝"
fi

log "将 root 用户加入 audio 组..."
if usermod -aG audio root; then
    log "root 用户已加入 audio 组。"
else
    log "root 用户加入 audio 组失败。"
    exit 1
fi

if sudo usermod -aG audio lab; then
    log "lab 用户已加入 audio 组。"
else
    log "lab 用户加入 audio 组失败。"
    exit 1
fi

