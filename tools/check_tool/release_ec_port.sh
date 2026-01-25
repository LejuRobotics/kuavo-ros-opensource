#!/bin/bash

# ============================================================================
# 脚本功能：释放EC网口并配置固定IP
# 1. 将 03:00.0 网口从 atemsys 释放，绑定到 igc 驱动
# 2. 配置 enp3s0 网口为固定IP: 192.168.26.1/24
# ============================================================================

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}开始执行EC网口释放和IP配置...${NC}"

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}错误: 此脚本需要root权限，请使用sudo运行${NC}"
    exit 1
fi

# ============================================================================
# 步骤1: 修改 udev rules 文件
# ============================================================================
UDEV_RULES_FILE="/etc/udev/rules.d/99-unbind-igc.rules"

if [ ! -f "$UDEV_RULES_FILE" ]; then
    echo -e "${RED}错误: 文件 $UDEV_RULES_FILE 不存在${NC}"
    exit 1
fi

echo -e "${YELLOW}步骤1: 修改 udev rules 配置...${NC}"

# 备份原文件
BACKUP_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
cp "$UDEV_RULES_FILE" "${UDEV_RULES_FILE}.backup.${BACKUP_TIMESTAMP}"
echo -e "${GREEN}已备份原文件到 ${UDEV_RULES_FILE}.backup.${BACKUP_TIMESTAMP}${NC}"

# 注释掉配置选项2（两个网卡都绑定到 ATEMSYS）
sed -i '31,32s/^/# /' "$UDEV_RULES_FILE"
echo -e "${GREEN}已注释配置选项2（两个网卡都绑定到 ATEMSYS）${NC}"

# 取消注释配置选项4（02:00.0 绑定到 ATEMSYS, 03:00.0 绑定到 IGC）
sed -i '45,46s/^# //' "$UDEV_RULES_FILE"
echo -e "${GREEN}已启用配置选项4（02:00.0 绑定到 ATEMSYS, 03:00.0 绑定到 IGC）${NC}"

# ============================================================================
# 步骤2: 重新加载 udev 规则
# ============================================================================
echo -e "${YELLOW}步骤2: 重新加载 udev 规则...${NC}"
udevadm control --reload
udevadm trigger
echo -e "${GREEN}udev 规则已重新加载${NC}"

# ============================================================================
# 步骤3: 配置 netplan 网络配置
# ============================================================================
NETPLAN_FILE="/etc/netplan/01-network-manager-all.yaml"

if [ ! -f "$NETPLAN_FILE" ]; then
    echo -e "${RED}错误: 文件 $NETPLAN_FILE 不存在${NC}"
    exit 1
fi

echo -e "${YELLOW}步骤3: 配置 enp3s0 网口固定IP...${NC}"

# 备份原文件
NETPLAN_BACKUP_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
cp "$NETPLAN_FILE" "${NETPLAN_FILE}.backup.${NETPLAN_BACKUP_TIMESTAMP}"
echo -e "${GREEN}已备份原文件到 ${NETPLAN_FILE}.backup.${NETPLAN_BACKUP_TIMESTAMP}${NC}"

# 创建新的netplan配置
cat > "$NETPLAN_FILE" << 'EOF'
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp3s0:
      addresses:
        - 192.168.26.1/24
      dhcp4: false
      dhcp6: false
EOF

echo -e "${GREEN}已配置 enp3s0 网口为固定IP: 192.168.26.1/24${NC}"

# ============================================================================
# 步骤4: 应用 netplan 配置
# ============================================================================
echo -e "${YELLOW}步骤4: 应用 netplan 配置...${NC}"
netplan apply
echo -e "${GREEN}netplan 配置已应用${NC}"

# ============================================================================
# 完成
# ============================================================================
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}配置完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${YELLOW}配置摘要:${NC}"
echo -e "  - 03:00.0 网口已从 atemsys 释放，绑定到 igc 驱动"
echo -e "  - enp3s0 网口已配置为固定IP: 192.168.26.1/24"
echo -e ""
echo -e "${YELLOW}注意:${NC}"
echo -e "  - 如果网口名称不是 enp3s0，请手动检查并修改 netplan 配置"
echo -e "  - 可以使用 'ip addr' 命令查看网口名称"
echo -e "  - 如果配置后网络连接有问题，可以使用备份文件恢复"
echo -e ""
echo -e "${YELLOW}【请重启系统，执行：sudo reboot】${NC}"
