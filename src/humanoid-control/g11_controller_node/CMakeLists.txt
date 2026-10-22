cmake_minimum_required(VERSION 3.0.2)
project(g11_controller_node)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)
###########
## Install
###########
