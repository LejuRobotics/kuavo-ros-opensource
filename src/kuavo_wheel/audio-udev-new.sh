#!/bin/bash
# 一键安装 udev 规则，使 C-Media CM108 声卡插入时自动加载音频驱动

# 检查是否为 root 用户
if [ "$EUID" -ne 0 ]; then
  echo "❌ 必须使用 root 用户运行此脚本！"
  echo "👉 请使用命令：sudo $0"
  exit 1
fi

RULE_FILE="/etc/udev/rules.d/90-audio-new.rules"

echo "[1/2] 创建 udev 规则文件：$RULE_FILE"
sudo bash -c "cat > $RULE_FILE" <<'EOF'
# C-Media CM108 USB 声卡自动加载驱动
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0d8c", ATTR{idProduct}=="013c", RUN+="/sbin/modprobe snd-usb-audio"
EOF

echo "[2/2] 重新加载 udev 规则..."
sudo udevadm control --reload

echo "🎉 规则安装完成。重启验证是否自动加载驱动。"
