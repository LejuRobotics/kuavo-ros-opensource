#! /bin/bash
set -e

if systemctl is-active --quiet ocs2_h12pro_monitor.service; then
    echo "服务 ocs2_h12pro_monitor.service 已开启，正在停止..."
    sudo systemctl stop ocs2_h12pro_monitor.service
    sudo systemctl disable ocs2_h12pro_monitor.service
    echo "服务 ocs2_h12pro_monitor.service 已停止。"
else
    echo "服务 ocs2_h12pro_monitor.service 未开启。"
fi

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
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
if [ -d "$INSTALLED_DIR" ] && [ -f "$INSTALLED_DIR/setup.bash" ]; then
    echo "Sourcing existing installation..."
    source $INSTALLED_DIR/setup.bash
fi
catkin build humanoid_controllers
catkin build humanoid_plan_arm_trajectory
catkin build voice_control_node


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

# TODO 确定是否有其他可以确定机器人类型的方法，这里暂时先使用ROBOT_VERSION环境变量
# 如果是ROBAN环境，才询问是否启动语音控制节点（1开头的版本号视为roban环境）
if [[ "$ROBOT_VERSION" =~ ^1[0-9]$ ]]; then
    # 询问是否启动语音控制节点，并设置用于启动的launch文件
    echo "是否在机器人启动时启动语音控制节点？(y/n, 默认为y)"
    read -r enable_voice_control
    case $enable_voice_control in
        n|N)
            LAUNCH_VOICE_CONTROL_REAL_CMD="roslaunch humanoid_controllers load_kuavo_real.launch start_way:=auto"
            echo "使用launch命令 $LAUNCH_VOICE_CONTROL_REAL_CMD: 不启动语音控制节点"
            ;;
        *)
            LAUNCH_VOICE_CONTROL_REAL_CMD="roslaunch voice_control_node voice_control.launch start_way:=auto"
            echo "使用launch命令 $LAUNCH_VOICE_CONTROL_REAL_CMD: 启动语音控制节点"
            ;;
    esac
fi


sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=NODE_SCRIPT=.*|Environment=NODE_SCRIPT=$START_ROBAN_JOY_NODE|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=STAIR_DETECTION_CAMERA=.*|Environment=STAIR_DETECTION_CAMERA=$STAIR_DETECTION_CAMERA|" $ROBAN_JOY_MONITOR_SERVICE
sed -i "s|^Environment=LAUNCH_VOICE_CONTROL_REAL_CMD=.*|Environment=\"LAUNCH_VOICE_CONTROL_REAL_CMD=$LAUNCH_VOICE_CONTROL_REAL_CMD\"|" $ROBAN_JOY_MONITOR_SERVICE
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

echo "stop all ros node"
sudo pkill ros -f
echo "stop all ros node successfully"