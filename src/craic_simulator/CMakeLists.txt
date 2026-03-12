cmake_minimum_required(VERSION 3.0.2)
project(craic_simulator)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

## Install Python scripts (utils as executables)
catkin_install_python(PROGRAMS
  utils/topic_repeater.py
  utils/lidar_mid360_node.py
  scripts/patch_tibvh.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########
include_directories(
)

## Mark launch and config for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
## Uncomment if config folder exists:
## install(DIRECTORY config/
##   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
##   FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml")
