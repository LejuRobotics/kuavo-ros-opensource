#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic
export CATKIN_WS_PATH=/home/lab/kuavo_ros1_workspace
export H12PRO_CONTROLLER_NODE_PATH=$CATKIN_WS_PATH/src/h12pro_controller_node
export PYTHONPATH=$PYTHONPATH:$H12PRO_CONTROLLER_NODE_PATH

source $ROS_WS_PATH/setup.bash
source $CATKIN_WS_PATH/devel/setup.bash
cd $H12PRO_CONTROLLER_NODE_PATH

pip3 install -r requirements.txt
python3.8 src/main.py --real