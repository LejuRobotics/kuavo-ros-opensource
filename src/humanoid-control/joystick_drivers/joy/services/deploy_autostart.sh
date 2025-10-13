#! /bin/bash
set -e

if systemctl is-active --quiet roban_joy_monitor.service; then
    echo "服务 roban_joy_monitor.service 已开启，正在停止..."
    sudo systemctl stop roban_joy_monitor.service
    sudo systemctl disable roban_joy_monitor.service
    echo "服务 roban_joy_monitor.service 已停止。"
else
    echo "服务 roban_joy_monitor.service 未开启。"
fi


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
JOY_NODE_DIR=$(dirname $SCRIPT_DIR)
SERVICE_DIR=$(dirname $SCRIPT_DIR)/services
ROBAN_JOY_MONITOR_SERVICE=$SERVICE_DIR/roban_joy_monitor.service
START_ROBAN_JOY_NODE=$SCRIPT_DIR/start_roban_joy_node.sh
MONITOR_ROBAN_JOY=$SCRIPT_DIR/monitor_roban_joy.sh
KUAVO_ROS_CONTROL_WS_PATH=$(dirname $(dirname $(dirname $(dirname $(dirname $SCRIPT_DIR)))))
NOITOM_HI5_HAND_UDP_PYTHON=$KUAVO_ROS_CONTROL_WS_PATH/src/manipulation_nodes/noitom_hi5_hand_udp_python
ROBOT_VERSION=$ROBOT_VERSION
INSTALLED_DIR=$KUAVO_ROS_CONTROL_WS_PATH/installed
cd $JOY_NODE_DIR
pip3 install -r $SCRIPT_DIR/requirements.txt
pip3 install -r $NOITOM_HI5_HAND_UDP_PYTHON/requirements.txt

echo "KUAVO_ROS_CONTROL_WS_PATH: $KUAVO_ROS_CONTROL_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "MONITOR_ROBAN_JOY: $MONITOR_ROBAN_JOY"

cd $KUAVO_ROS_CONTROL_WS_PATH
if [ -d "$INSTALLED_DIR" ] && [ -f "$INSTALLED_DIR/setup.bash" ]; then
    echo "Sourcing existing installation..."
    source $INSTALLED_DIR/setup.bash
fi
catkin build humanoid_controllers
catkin build humanoid_plan_arm_trajectory


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

# 询问用户选择楼梯建图相机类型
echo "请问机器人楼梯建图的相机类型为："
echo "1. 奥比中光"
echo "2. D435"
echo -n "请选择 (默认为奥比中光，直接回车选择默认): "
read -r camera_choice

case $camera_choice in
    2)
        STAIR_DETECTION_CAMERA="d435"
        echo "已选择: D435"
        ;;
    1|""|*)
        STAIR_DETECTION_CAMERA="orbbec"
        echo "已选择: 奥比中光"
        ;;
esac

echo "楼梯建图相机类型: $STAIR_DETECTION_CAMERA"

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=NODE_SCRIPT=.*|Environment=NODE_SCRIPT=$START_ROBAN_JOY_NODE|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=STAIR_DETECTION_CAMERA=.*|Environment=STAIR_DETECTION_CAMERA=$STAIR_DETECTION_CAMERA|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^ExecStart=.*|ExecStart=$MONITOR_ROBAN_JOY|" $ROBAN_JOY_MONITOR_SERVICE

sudo cp $ROBAN_JOY_MONITOR_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload

sudo apt-get install tmux

if [ ! -f ~/.tmux.conf ]; then
    touch ~/.tmux.conf
fi

if ! grep -q "set-option -g default-shell /bin/bash" ~/.tmux.conf; then
    echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf
    echo "set-option -g mouse on" >> ~/.tmux.conf
fi


sudo systemctl start roban_joy_monitor.service
sudo systemctl enable roban_joy_monitor.service

echo "joy monitor service deploy successfully"
