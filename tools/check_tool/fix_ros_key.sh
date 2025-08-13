#!/bin/bash

TIMEOUT=30
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

echo ">>> 开始修复 ROS GPG 密钥问题..."

# 检查sudo权限
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用sudo运行此脚本"
    exit 1
fi

# 网络检查
ping -c 3 mirrors.tuna.tsinghua.edu.cn > /dev/null 2>&1 || {
    echo "警告: 无法连接到清华镜像站，尝试继续..."
}

# 密钥下载函数
download_ros_key() {
    local url="https://kuavo.lejurobot.com/statics/ros.key"
    local backup_url="http://repo.ros2.org/ros.key"
    
    if ! curl -fsSL --connect-timeout $TIMEOUT "$url" | gpg --dearmor | sudo tee "$KEYRING" > /dev/null; then
        echo "主源下载失败，尝试备用源..."
        curl -fsSL --connect-timeout $TIMEOUT "$backup_url" | gpg --dearmor | sudo tee "$KEYRING" > /dev/null || {
            echo "错误: 无法下载ROS密钥"
            return 1
        }
    fi
    sudo chmod 644 "$KEYRING"
    sudo chown root:root "$KEYRING"
    return 0
}

# 安装ros-archive-keyring
echo ">>> 处理ros-archive-keyring..."
if ! dpkg -s ros-archive-keyring &> /dev/null; then
    sudo apt update -y || echo "警告: apt更新失败"
    sudo apt install -y ros-archive-keyring || {
        echo "尝试手动安装密钥..."
        KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
        download_ros_key
    }
fi

# 验证密钥
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
if [ -f "$KEYRING" ]; then
    if gpg --list-keys --keyring "$KEYRING" | grep -i expired; then
        echo "发现过期密钥，更新..."
        download_ros_key
    fi
else
    download_ros_key
fi

# 清理旧源
echo ">>> 清理旧ROS源..."
sudo rm -f /etc/apt/sources.list.d/ros*.list

# 添加清华源（仅ROS1）
echo "deb [arch=amd64 signed-by=$KEYRING] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/ros1.list > /dev/null

# 强制更新APT
echo ">>> 强制更新APT..."
sudo apt-key del F42ED6FBAB17C654 2> /dev/null
sudo apt update -o Acquire::AllowInsecureRepositories=true
sudo apt update --fix-missing

echo ">>> 修复完成。"
