#!/bin/bash
set -e

# H12 接收机的 FT232R 序列号在 usb_remote.rules 中固定，不能再根据宽泛的
# product 名称动态生成规则，否则会匹配到其他 USB 串口设备。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SOURCE="$SCRIPT_DIR/usb_remote.rules"
RULE_TARGET="/etc/udev/rules.d/usb_remote.rules"

if [ ! -f "$RULE_SOURCE" ]; then
    echo "未找到规则文件: $RULE_SOURCE" >&2
    exit 1
fi

install -m 0644 "$RULE_SOURCE" "$RULE_TARGET"
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty
udevadm settle

echo "已覆盖安装 H12 udev 规则: $RULE_TARGET"
if [ -e /dev/usb_remote ]; then
    echo "/dev/usb_remote -> $(readlink -f /dev/usb_remote)"
else
    echo "未检测到 H12 接收机；规则已安装，插入接收机后会自动生成 /dev/usb_remote。"
fi
