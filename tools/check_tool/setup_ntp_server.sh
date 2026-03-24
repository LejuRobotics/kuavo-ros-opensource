#!/bin/bash
# NTP 服务器配置脚本（服务器端）
# 用途：将本机配置为 NTP 服务器，供内网其他设备同步时间

ALLOW_NET="192.168.26.0/24"
CONFIG_FILE="/etc/chrony/chrony.conf"

echo "=== NTP 服务器配置脚本 ==="

# 1. 安装 chrony
echo "[1/3] 安装 chrony..."
sudo apt update
sudo apt install -y chrony

# 2. 编辑配置文件
echo "[2/3] 配置 chrony..."

# 检查是否已存在 allow 配置
if grep -q "^allow " "$CONFIG_FILE"; then
    echo "已存在 allow 配置，正在更新..."
    sudo sed -i "s/^allow .*/allow $ALLOW_NET/" "$CONFIG_FILE"
else
    echo "添加 allow 配置（位于 keyfile 上方）..."
    sudo sed -i "/^keyfile/i allow $ALLOW_NET" "$CONFIG_FILE"
fi

echo "配置文件已更新：$CONFIG_FILE"

# 3. 启动服务
echo "[3/3] 启动 chrony 服务..."
sudo systemctl daemon-reload
sudo systemctl restart chrony
sudo systemctl enable chrony

echo ""
echo "✓ NTP 服务器配置完成！"
echo ""
echo "=== 请手动验证 ==="
echo "执行以下命令检查 UTC 时间是否正确："
echo "  sudo chronyc tracking"
echo ""
echo "如果 Reference ID 和 Ref time (UTC) 显示有效值，表示已同步到上游 NTP 服务器"
echo ""
echo "=== 客户端配置 ==="
echo "在客户端 (192.168.26.22) 上执行："
echo "  sudo systemctl restart systemd-timesyncd"
echo "  sleep 5"
echo "  timedatectl status"