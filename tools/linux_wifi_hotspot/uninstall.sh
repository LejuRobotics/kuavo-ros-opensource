#!/bin/bash

echo -e "\033[32m\n🚀🚀🚀 开始卸载...\n\033[0m"

# 先停掉当前运行的热点，再取消开机自启（避免卸载后热点仍在运行）
if systemctl is-active --quiet create_ap; then
  sudo systemctl stop create_ap
fi

if systemctl list-unit-files | grep -Fq 'create_ap.service'; then
  sudo systemctl disable create_ap
fi

if dpkg -l | grep -q linux-wifi-hotspot; then
  sudo apt remove -y linux-wifi-hotspot
fi

# 重新加载 systemd，彻底移除已删除的单元定义
sudo systemctl daemon-reload

echo -e "\033[32m\n🚀🚀🚀 卸载成功...\n\033[0m"
