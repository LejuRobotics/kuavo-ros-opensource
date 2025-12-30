#!/bin/bash

SERVICE_NAME="websocket_start"

# 检查服务是否active,如果active则停止服务并禁用服务
if systemctl is-active --quiet ${SERVICE_NAME}.service; then
    echo "${SERVICE_NAME}.service 正在运行，正在停止..."
    sudo systemctl stop ${SERVICE_NAME}.service
    sudo systemctl disable ${SERVICE_NAME}.service
    echo "${SERVICE_NAME}.service 已停止"
else
    echo "${SERVICE_NAME}.service 未在运行"
fi

# 更新服务脚本路径

ROBOT_VERSION=$ROBOT_VERSION

# 获取当前脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 拼接 websocket_start.service 的绝对路径
SERVICE_FILE="$SCRIPT_DIR/websocket_start.service"
START_SCRIPT_PATH="$SCRIPT_DIR/websocket_start_script.sh"

# 替换 ExecStart 路径
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $SERVICE_FILE
sudo sed -i "s|^ExecStart=.*|ExecStart=/bin/bash $START_SCRIPT_PATH|" $SERVICE_FILE
sudo cp $SERVICE_FILE /etc/systemd/system/

cd "$SCRIPT_DIR"

# 使用 git 命令获取仓库根目录
REPO_ROOT=$(git rev-parse --show-toplevel)
if [[ -z "$REPO_ROOT" ]]; then
    echo "错误：无法找到 git 仓库根目录"
    exit 1
fi
echo "仓库根目录: $REPO_ROOT"
cd "$REPO_ROOT"

# 重新设置配置文件权限，保证编译能够正常进行
CONFIG_FILE="/home/lab/.config/lejuconfig/ImuType.ini"

if [ -f "$CONFIG_FILE" ]; then
    OWNER_GROUP=$(stat -c "%U:%G" "$CONFIG_FILE")
    if [ "$OWNER_GROUP" != "lab:lab" ]; then
        echo "$CONFIG_FILE 存在，但不属于 lab 用户及 lab 用户组，正在切换权限..."
        sudo chown lab:lab "$CONFIG_FILE"
    else
        echo "$CONFIG_FILE 已经属于 lab 用户及 lab 用户组"
    fi
else
    echo "$CONFIG_FILE 文件不存在"
fi

# 重新执行编译操作
sudo catkin clean -y
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
source installed/setup.bash     # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers
catkin build humanoid_plan_arm_trajectory
catkin build planarmwebsocketservice

# 安装 sshpass，确保远程拷贝音乐文件正常工作
sudo apt-get update
sudo apt-get install sshpass

# 重新启动服务
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}.service
sudo systemctl start ${SERVICE_NAME}.service

echo "服务已重新启动并启用"
