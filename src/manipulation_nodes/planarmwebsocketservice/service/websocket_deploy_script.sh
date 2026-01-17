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
REPO_ROOT=$(dirname $(dirname $(dirname $(dirname $SCRIPT_DIR))))

# 拼接 websocket_start.service 的绝对路径
SERVICE_FILE="$SCRIPT_DIR/websocket_start.service"
START_SCRIPT_PATH="$SCRIPT_DIR/websocket_start_script.sh"

if [ -z "$ROBOT_NAME" ]; then
    ROBOT_NAME="KUAVO"
    echo "ROBOT_NAME is empty, using default: $ROBOT_NAME"
fi
 
echo "Current ROBOT_NAME: $ROBOT_NAME"
echo "Current robot version: $ROBOT_VERSION"

if [ -z "$ROS_MASTER_URI" ]; then
    ROS_MASTER_URI="http://localhost:11311"
    echo "ROS_MASTER_URI is empty, using default: $ROS_MASTER_URI"
fi

if [ -z "$ROS_IP" ]; then
    ROS_IP="127.0.0.1"
    echo "ROS_IP is empty, using default: $ROS_IP"
fi

if [ -z "$ROS_HOSTNAME" ]; then
    if [ "$ROS_MASTER_URI" == "http://kuavo_master:11311" ]; then
        ROS_HOSTNAME=kuavo_master  
        echo "ROS_MASTER_URI is http://kuavo_master:11311, using ROS_HOSTNAME: $ROS_HOSTNAME"
    fi
fi

echo "Current ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Current ROS_IP: $ROS_IP"
echo "Current ROS_HOSTNAME:$ROS_HOSTNAME"

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $SERVICE_FILE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $SERVICE_FILE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $SERVICE_FILE

sudo sed -i "s|^Environment=ROBOT_NAME=.*|Environment=ROBOT_NAME=$ROBOT_NAME|" $SERVICE_FILE

# 询问相机型号
    echo
    echo "请选择相机型号："
    echo "1. realsense"
    echo "2. orbbec"
    echo "3. 不启用相机"
    read -p "请输入数字 (1 或 2 或 3): " camera_choice

    case $camera_choice in
        1)
            CAMERA_MODEL="realsense"
            ;;
        2)
            CAMERA_MODEL="orbbec"
            ;;
        3)
            CAMERA_MODEL="none"
            ;;
        *)
            echo "无效的输入，不启用相机"
            CAMERA_MODEL="none"
            ;;
    esac

echo "已选择相机型号: $CAMERA_MODEL"

sed -i "s|^Environment=CAMERA_TYPE=.*|Environment=CAMERA_TYPE=$CAMERA_MODEL|" $SERVICE_FILE




# 替换 ExecStart 路径
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $SERVICE_FILE
sudo sed -i "s|^ExecStart=.*|ExecStart=/bin/bash $START_SCRIPT_PATH|" $SERVICE_FILE
sudo sed -i "s|^Environment=REPO_ROOT=.*|Environment=REPO_ROOT=$REPO_ROOT|" $SERVICE_FILE
sudo cp $SERVICE_FILE /etc/systemd/system/

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

# 安装 yolo 环境
pip install -r "$REPO_ROOT/src/manipulation_nodes/planarmwebsocketservice/requirements.txt"

# 安装 wesocket_sdk 环境
sudo bash "$REPO_ROOT/src/kuavo_humanoid_sdk/install.sh"

# 重新执行编译操作
sudo catkin clean -y
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
source installed/setup.bash     # 加载一些已经安装的ROS包依赖环境，包括硬件包等
catkin build humanoid_controllers
catkin build humanoid_plan_arm_trajectory
catkin build planarmwebsocketservice
catkin build kuavo_mapping
catkin build voice_control_node # 编译语音控制节点
catkin build taiji_trigger_node # 太极监听节点

# 安装 sshpass，确保远程拷贝音乐文件正常工作
sudo apt-get update
sudo apt-get install sshpass

# 重新启动服务
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}.service
sudo systemctl start ${SERVICE_NAME}.service

echo "服务已重新启动并启用"
