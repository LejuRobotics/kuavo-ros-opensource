#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic

source $ROS_WS_PATH/setup.bash
source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash
H12PRO_CONTROLLER_NODE_DIR=$(rospack find h12pro_controller_node)
export PYTHONPATH=$PYTHONPATH:$H12PRO_CONTROLLER_NODE_DIR
cd $H12PRO_CONTROLLER_NODE_DIR

echo "current robot version: $ROBOT_VERSION"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

LAUNCH_PATTERN="roslaunch .*h12pro_controller_node .*h12pro_autostart.launch"

# Avoid touching ros master before roslaunch brings it up.
if ! pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
    roslaunch h12pro_controller_node h12pro_autostart.launch
else
    echo "Launch tree h12pro_autostart.launch is already running"
    echo "Please check running processes with 'pgrep -af \"$LAUNCH_PATTERN\"'"
fi
