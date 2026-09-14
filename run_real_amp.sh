#!/bin/bash
source devel/setup.bash
export ROBOT_VERSION=52
roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2