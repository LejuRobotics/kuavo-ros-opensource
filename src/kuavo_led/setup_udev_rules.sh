#!/bin/bash

# 获取脚本所在目录的父目录（即 kuavo-ros-control 仓库根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# 复制 udev rules 文件
sudo cp "$SCRIPT_DIR/rules/99-led.rules" /etc/udev/rules.d/

# 重新加载 udev rules
sudo udevadm control --reload-rules

# 触发 udev 事件
sudo udevadm trigger

echo "LED udev rules 已成功安装并重新加载！"
echo ""
echo "=========================================="
echo "注意：请重启设备以使 udev rules 生效！"
echo "=========================================="
