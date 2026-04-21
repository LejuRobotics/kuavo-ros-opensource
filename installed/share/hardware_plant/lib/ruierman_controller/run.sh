#!/bin/bash
export LD_LIBRARY_PATH=/home/lab/kuavo_ros2_ws/src/ruierman_controller/ruierman_controller/python-can-3.3.4/can/interfaces/bmcan:$LD_LIBRARY_PATH
export PYTHONPATH=/home/lab/kuavo_ros2_ws/src/ruierman_controller/ruierman_controller/python-can-3.3.4:$PYTHONPATH
# source ~/kuavo_ros2_ws/install/setup.bash
. ~/ros2_humble/install/local_setup.bash
cd ~/kuavo_ros2_ws
ros2 run ruierman_controller ruierman_joint_control_node
