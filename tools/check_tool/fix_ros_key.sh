#!/bin/bash

# 设置超时时间（秒）
TIMEOUT=30
# 获取当前日期时间
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

echo ">>> 开始修复 ROS GPG 密钥问题..."

# 检查是否有sudo权限
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用sudo运行此脚本"
    exit 1
fi

# 检查网络连接
echo ">>> 检查网络连接..."
ping -c 3 raw.githubusercontent.com > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "警告: 无法连接到raw.githubusercontent.com，可能影响后续操作"
fi

# 检查并安装ros-archive-keyring包
echo ">>> 检查并安装 ros-archive-keyring 包..."
if ! dpkg -s ros-archive-keyring &> /dev/null; then
    echo "未找到 ros-archive-keyring 包，尝试安装..."
    sudo apt update -y || {
        echo "警告: apt更新失败，尝试继续安装..."
    }
    sudo apt install -y ros-archive-keyring
else
    echo "ros-archive-keyring 包已安装，尝试更新..."
    sudo apt update -y || {
        echo "警告: apt更新失败，尝试继续更新包..."
    }
    sudo apt install --only-upgrade -y ros-archive-keyring
fi

# 验证密钥状态
echo ">>> 验证 ROS GPG 密钥状态..."
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
if [ -f "$KEYRING" ]; then
    # 导出密钥信息并检查过期状态
    KEY_INFO=$(gpg --no-default-keyring --keyring "$KEYRING" --list-keys)
    echo "密钥信息："
    echo "$KEY_INFO"
    
    # 检查是否有过期密钥
    if echo "$KEY_INFO" | grep -i "expired" &> /dev/null; then
        echo "警告: 发现过期密钥，尝试手动更新..."
        # 尝试手动更新密钥
        curl -fsSL --connect-timeout $TIMEOUT https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
          gpg --dearmor | sudo tee "$KEYRING" > /dev/null
    else
        echo "密钥状态正常，未发现过期密钥"
    fi
else
    echo "错误: 未找到密钥文件 $KEYRING，尝试重新创建..."
    curl -fsSL --connect-timeout $TIMEOUT https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
      gpg --dearmor | sudo tee "$KEYRING" > /dev/null
fi

# 备份现有源列表（添加时间戳）
echo ">>> 备份现有源列表..."
mkdir -p ~/ros_source_backup
BACKUP_DIR="~/ros_source_backup/backup_${TIMESTAMP}"
mkdir -p $BACKUP_DIR
cp -f /etc/apt/sources.list.d/ros*.list $BACKUP_DIR/ 2> /dev/null
echo "已备份到: $BACKUP_DIR"

# 写入 ROS1 清华源（带 signed-by）
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/ros1.list > /dev/null

# 写入 ROS2 清华源（带 signed-by）
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 检查apt锁
echo ">>> 检查软件包管理器状态..."
if [ -f /var/lib/dpkg/lock-frontend ]; then
    echo "警告: 检测到dpkg锁，尝试释放..."
    sudo rm -f /var/lib/dpkg/lock-frontend
    sudo rm -f /var/lib/dpkg/lock
    sudo dpkg --configure -a
fi

# 更新 apt
echo ">>> 更新软件源..."
sudo apt update -y || {
    echo "警告: apt更新部分失败，尝试恢复..."
    sudo apt update --fix-missing -y
}

echo ">>> 修复完成。"
