#!/bin/bash
set -e


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SERVICE_DIR=$(dirname $SCRIPT_DIR)/service
KUAVO_WHEEL_BRIDGE_SERVICE=$SERVICE_DIR/kuavo_wheel_bridge.service
START_WHEEL_BRIDGE=$SCRIPT_DIR/start_wheel_bridge.sh
KUAVO_ROS_CONTROL_WS_PATH=$(dirname $(dirname $(dirname $SCRIPT_DIR)))

echo "KUAVO_ROS_CONTROL_WS_PATH: $KUAVO_ROS_CONTROL_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "START_WHEEL_BRIDGE: $START_WHEEL_BRIDGE"

# 检查必要文件是否存在
if [[ ! -f "$KUAVO_WHEEL_BRIDGE_SERVICE" ]]; then
    echo "Error: Service file not found at $KUAVO_WHEEL_BRIDGE_SERVICE"
    exit 1
fi

if [[ ! -f "$START_WHEEL_BRIDGE" ]]; then
    echo "Error: Startup script not found at $START_WHEEL_BRIDGE"
    exit 1
fi

if [[ ! -f "$SCRIPT_DIR/wheel_bridge.py" ]]; then
    echo "Error: Python script not found at $SCRIPT_DIR/wheel_bridge.py"
    exit 1
fi

# 设置ROS环境变量
if [ -z "$ROS_MASTER_URI" ]; then
    ROS_MASTER_URI="http://localhost:11311"
    echo "ROS_MASTER_URI is empty, using default: $ROS_MASTER_URI"
fi

if [ -z "$ROS_IP" ]; then
    ROS_IP="127.0.0.1"
    echo "ROS_IP is empty, using default: $ROS_IP"
fi

echo "Current ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Current ROS_IP: $ROS_IP"

# 更新服务文件中的路径和配置
sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $KUAVO_WHEEL_BRIDGE_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $KUAVO_WHEEL_BRIDGE_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $KUAVO_WHEEL_BRIDGE_SERVICE
sed -i "s|^ExecStart=.*|ExecStart=$START_WHEEL_BRIDGE|" $KUAVO_WHEEL_BRIDGE_SERVICE

# 复制服务文件到系统目录
sudo cp $KUAVO_WHEEL_BRIDGE_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable kuavo_wheel_bridge.service
sudo systemctl start kuavo_wheel_bridge.service