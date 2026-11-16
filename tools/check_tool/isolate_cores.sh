#!/usr/bin/env bash
# isolate_cores.sh
# 用途：隔离 CPU 核心；aarch64 Orin 仅核心 2，x86 核心 2/3/7，需 root 权限运行，重启后生效

set -e

GRUB_CONF="/etc/default/grub"
BACKUP_CONF="/etc/default/grub.bak.$(date +%Y%m%d%H%M%S)"

# 确保以 root 身份运行
if [[ $EUID -ne 0 ]]; then
  echo "请以 root 身份运行：sudo $0"
  exit 1
fi

# 备份原始 grub 配置
echo "备份 $GRUB_CONF → $BACKUP_CONF"
cp "$GRUB_CONF" "$BACKUP_CONF"

# 按架构选择隔离核心：aarch64 Orin 仅核心 2；x86 核心 2、3、7（7 供 Ruiwo 电机）
ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" ]]; then
  ISOLCPUS="2"
else
  ISOLCPUS="2,3,7"
fi

# 在 GRUB_CMDLINE_LINUX 中添加 isolcpus
if grep -q "isolcpus[=]" "$GRUB_CONF"; then
  # 如果已有 isolcpus，则直接替换数字
  sed -i -r "s/isolcpus=[0-9,-]*/isolcpus=${ISOLCPUS}/g" "$GRUB_CONF"
else
  # 否则在末尾追加
  sed -i -r "s/^(GRUB_CMDLINE_LINUX=\")/\1isolcpus=${ISOLCPUS} /" "$GRUB_CONF"
fi

echo "更新完毕，新的 GRUB 配置："
grep "^GRUB_CMDLINE_LINUX" "$GRUB_CONF"

# 生成新的 grub 配置
echo "正在执行 update-grub ……"
update-grub

echo -e "\n完成！请重启系统以应用隔离：\n  sudo reboot\n"
