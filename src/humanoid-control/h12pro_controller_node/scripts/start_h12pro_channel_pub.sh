#!/bin/bash
export ROS_WS_PATH=/opt/ros/noetic
export CATKIN_WS_PATH=/home/lab/kuavo_ros1_workspace

whoami
source $ROS_WS_PATH/setup.bash
source $CATKIN_WS_PATH/devel/setup.bash

rosrun h12pro_controller_node h12pro_channel_publisher
