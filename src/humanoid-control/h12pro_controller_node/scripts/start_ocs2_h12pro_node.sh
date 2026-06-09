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

# Check if safe to start launch
# Condition: rosmaster not running OR both key nodes are absent
can_start=true

if pgrep rosmaster > /dev/null; then
    # rosmaster exists, check if either key node is already running
    if rosnode list 2>/dev/null | grep -q "/h12pro_channel_publisher" || \
       rosnode list 2>/dev/null | grep -q "/joy_node"; then
        can_start=false
    fi
fi

if $can_start; then
    roslaunch h12pro_controller_node h12pro_autostart.launch
else
    echo "Node /h12pro_channel_publisher or /joy_node is already running"
    echo "Please check running nodes with 'rosnode list'"
fi