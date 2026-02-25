#! /bin/bash
set -e

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

cd $KUAVO_ROS_CONTROL_WS_PATH
if [ -d "$INSTALLED_DIR" ] && [ -f "$INSTALLED_DIR/devel/setup.bash" ]; then
    echo "Sourcing existing installation..."
    source $INSTALLED_DIR/devel/setup.bash
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
sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROS_HOSTNAME=.*|Environment=ROS_HOSTNAME=$ROS_HOSTNAME|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=KUAVO_ROS_CONTROL_WS_PATH=.*|Environment=KUAVO_ROS_CONTROL_WS_PATH=$KUAVO_ROS_CONTROL_WS_PATH|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=ROBOT_VERSION=.*|Environment=ROBOT_VERSION=$ROBOT_VERSION|" $OCS2_H12PRO_MONITOR_SERVICE
sed -i "s|^Environment=NODE_SCRIPT=.*|Environment=NODE_SCRIPT=$START_OCS2_H12PRO_NODE|" $OCS2_H12PRO_MONITOR_SERVICE
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
