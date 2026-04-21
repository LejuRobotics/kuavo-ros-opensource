#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic

source $ROS_WS_PATH/setup.bash
source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash
JOY_NODE_DIR=$(rospack find joy)
export PYTHONPATH=$PYTHONPATH:$JOY_NODE_DIR
cd $JOY_NODE_DIR

echo "current robot version: $ROBOT_VERSION"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

# Check if both nodes are not running
if ! rosnode list | grep -q "/joy_node"; then
    roslaunch humanoid_controllers joy_control_bt.launch real:=true start_way:=auto
else
    echo "Node /joy_node is already running"
    echo "Please check running nodes with 'rosnode list'"
fi
