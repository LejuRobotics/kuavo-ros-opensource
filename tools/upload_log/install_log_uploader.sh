#!/bin/bash
CURRENT_DIR=$(cd $(dirname $0); pwd)
echo "当前目录: $CURRENT_DIR"

# 检查 wlo1 网卡是否存在
if ! ip link show wlo1 >/dev/null 2>&1; then
    echo "错误: wlo1 网卡不存在"
    exit 1
fi

# 获取 wlo1 网卡的 MAC 地址并去掉冒号，排除广播地址
ROBOT_NUMBER=$(ip link show wlo1 | grep -o -E '([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}' | head -n1 | tr -d ':')

echo "当前设备编号: $ROBOT_NUMBER"

# 卸载 colink
curl -fsSL https://download.coscene.cn/coscout/uninstall.sh -o $CURRENT_DIR/uninstall.sh
chmod +x $CURRENT_DIR/uninstall.sh
$CURRENT_DIR/uninstall.sh
rm $CURRENT_DIR/uninstall.sh

curl -fsSL https://download.coscene.cn/coscout/v2/install.sh -o $CURRENT_DIR/upload_log_install.sh
sudo chmod +x $CURRENT_DIR/upload_log_install.sh
sudo $CURRENT_DIR/upload_log_install.sh --mod="default" --org_slug="lejurobot" --project_slug="kfjqrrz" --server_url="https://openapi.coscene.cn" --serial_num=$ROBOT_NUMBER --remove_config

rm $CURRENT_DIR/upload_log_install.sh
