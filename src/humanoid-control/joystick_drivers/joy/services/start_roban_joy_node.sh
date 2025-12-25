#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic

# 确保环境变量被导出，以便传递给子进程
if [ -z "$KUAVO_ROS_CONTROL_WS_PATH" ]; then
    echo "Warning: KUAVO_ROS_CONTROL_WS_PATH is not set, using default"
    export KUAVO_ROS_CONTROL_WS_PATH=/home/lab/kuavo-ros-opensource
fi

source $ROS_WS_PATH/setup.bash
source $KUAVO_ROS_CONTROL_WS_PATH/devel/setup.bash
JOY_NODE_DIR=$(rospack find joy)
export PYTHONPATH=$PYTHONPATH:$JOY_NODE_DIR
cd $JOY_NODE_DIR

echo "current robot version: $ROBOT_VERSION"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"
echo "KUAVO_ROS_CONTROL_WS_PATH: $KUAVO_ROS_CONTROL_WS_PATH"

# Check if both nodes are not running
if ! rosnode list | grep -q "/joy_node"; then
    roslaunch humanoid_controllers joy_control_bt.launch real:=true start_way:=auto respawn:=false
else
    echo "Node /joy_node is already running"
    echo "Please check running nodes with 'rosnode list'"
fi
