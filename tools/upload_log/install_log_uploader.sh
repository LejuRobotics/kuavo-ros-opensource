#!/bin/bash
CURRENT_DIR=$(cd $(dirname $0); pwd)
echo "当前目录: $CURRENT_DIR"

# 根据 ROBOT_VERSION 环境变量选择下载链接
if [[ -n "$ROBOT_VERSION" ]] && [[ "$ROBOT_VERSION" -gt 50 ]] && [[ "$ROBOT_VERSION" -lt 60 ]]; then
    echo "检测到版本 $ROBOT_VERSION (51-59)，使用新版链接"
    UNINSTALL_URL="https://download.coscene.cn/cosbinary/script/latest/uninstall.sh"
    INSTALL_URL="https://download.coscene.cn/cosbinary/script/latest/install.sh"
else
    echo "使用默认链接 (ROBOT_VERSION=${ROBOT_VERSION:-未设置})"
    UNINSTALL_URL="https://download.coscene.cn/coscout/uninstall.sh"
    INSTALL_URL="https://download.coscene.cn/coscout/v2/install.sh"
fi

# 自动检测无线网卡（排除 ap0）
NETWORK_INTERFACE=""
for iface in /sys/class/net/*; do
    iface_name=$(basename "$iface")
    if [[ -d "$iface/wireless" ]] && [[ "$iface_name" != "ap0" ]]; then
        NETWORK_INTERFACE="$iface_name"
        break
    fi
done

if [[ -z "$NETWORK_INTERFACE" ]]; then
    echo "错误: 未找到无线网卡"
    exit 1
fi
echo "检测到无线网卡: $NETWORK_INTERFACE"

# 获取网卡的 MAC 地址并去掉冒号，排除广播地址
ROBOT_NUMBER=$(ip link show "$NETWORK_INTERFACE" | grep -o -E '([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}' | head -n1 | tr -d ':')

echo "当前设备编号: $ROBOT_NUMBER"

# 卸载 colink
curl -fsSL "$UNINSTALL_URL" -o $CURRENT_DIR/uninstall.sh
chmod +x $CURRENT_DIR/uninstall.sh
$CURRENT_DIR/uninstall.sh
rm $CURRENT_DIR/uninstall.sh

curl -fsSL "$INSTALL_URL" -o $CURRENT_DIR/upload_log_install.sh
sudo chmod +x $CURRENT_DIR/upload_log_install.sh
sudo $CURRENT_DIR/upload_log_install.sh --mod="default" --org_slug="lejurobot" --project_slug="kfjqrrz" --server_url="https://openapi.coscene.cn" --serial_num=$ROBOT_NUMBER --remove_config

rm $CURRENT_DIR/upload_log_install.sh
