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

while true; do
    echo "请选择控制方案 (1: ocs2, 2: rl)。若为 rl，请先修改 ROBOT_VERSION=46，并将正确的仓库路径修改在脚本中，再运行该脚本:"
    read -r user_input
    if [ "$user_input" = "1" ]; then
        KUAVO_CONTROL_SCHEME=ocs2
        echo "已选择: ocs2"
        break
    elif [ "$user_input" = "2" ]; then
        KUAVO_CONTROL_SCHEME=rl
        echo "已选择: rl"
        break
    else
        echo "输入无效，请输入1或2。"
    fi
done

KUAVO_RL_WS_PATH="/home/lab/kuavo-RL/kuavo-robot-deploy" # 在没合并到 kuavo-ros-control 的之前，先固定路径或手动修改
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
H12PRO_CONTROLLER_NODE_DIR=$(dirname $SCRIPT_DIR)
SERVICE_DIR=$(dirname $SCRIPT_DIR)/services
OCS2_H12PRO_MONITOR_SERVICE=$SERVICE_DIR/ocs2_h12pro_monitor.service
START_OCS2_H12PRO_NODE=$SCRIPT_DIR/start_ocs2_h12pro_node.sh
MONITOR_OCS2_H12PRO=$SCRIPT_DIR/monitor_ocs2_h12pro.sh
KUAVO_ROS_CONTROL_WS_PATH=$(dirname $(dirname $(dirname $(dirname $SCRIPT_DIR))))
NOITOM_HI5_HAND_UDP_PYTHON=$KUAVO_ROS_CONTROL_WS_PATH/src/manipulation_nodes/noitom_hi5_hand_udp_python
KUAVO_REMOTE_PATH=$(dirname $SCRIPT_DIR)/lib/kuavo_remote
ROBOT_VERSION=$ROBOT_VERSION
INSTALLED_DIR=$KUAVO_ROS_CONTROL_WS_PATH/installed
cd $H12PRO_CONTROLLER_NODE_DIR
pip3 install -r requirements.txt
pip3 install -r $NOITOM_HI5_HAND_UDP_PYTHON/requirements.txt

echo "KUAVO_ROS_CONTROL_WS_PATH: $KUAVO_ROS_CONTROL_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "MONITOR_OCS2_H12PRO: $MONITOR_OCS2_H12PRO"
echo "KUAVO_REMOTE_PATH: $KUAVO_REMOTE_PATH"

if [ "$KUAVO_CONTROL_SCHEME" = "rl" ]; then
    cd $KUAVO_RL_WS_PATH
    catkin build humanoid_controllers
fi

cd $KUAVO_ROS_CONTROL_WS_PATH
if [ -d "$INSTALLED_DIR" ] && [ -f "$INSTALLED_DIR/setup.bash" ]; then
    echo "Sourcing existing installation..."
    source $INSTALLED_DIR/setup.bash
fi
catkin build humanoid_controllers
catkin build h12pro_controller_node
catkin build humanoid_plan_arm_trajectory
catkin build kuavo_ros_interfaces

if ls /dev | grep usb_remote; then
  echo "Device file exists."
else
  echo "Device file does not exist."
  cd $KUAVO_REMOTE_PATH
  sudo chmod +x creat_remote_udev_rule.sh
  sudo ./creat_remote_udev_rule.sh
fi

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

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_CONTROL_SCHEME=.*|Environment=KUAVO_CONTROL_SCHEME=$KUAVO_CONTROL_SCHEME|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_RL_WS_PATH=.*|Environment=KUAVO_RL_WS_PATH=$KUAVO_RL_WS_PATH|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=NODE_SCRIPT=.*|Environment=NODE_SCRIPT=$START_OCS2_H12PRO_NODE|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=STAIR_DETECTION_CAMERA=.*|Environment=STAIR_DETECTION_CAMERA=$STAIR_DETECTION_CAMERA|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^ExecStart=.*|ExecStart=$MONITOR_OCS2_H12PRO|" $OCS2_H12PRO_MONITOR_SERVICE

sudo cp $OCS2_H12PRO_MONITOR_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload

sudo apt-get install tmux

if [ ! -f ~/.tmux.conf ]; then
    touch ~/.tmux.conf
fi

if ! grep -q "set-option -g default-shell /bin/bash" ~/.tmux.conf; then
    echo "set-option -g default-shell /bin/bash" >> ~/.tmux.conf
    echo "set-option -g mouse on" >> ~/.tmux.conf
fi


sudo systemctl start ocs2_h12pro_monitor.service
sudo systemctl enable ocs2_h12pro_monitor.service

echo "h12pro monitor service deploy successfully"
