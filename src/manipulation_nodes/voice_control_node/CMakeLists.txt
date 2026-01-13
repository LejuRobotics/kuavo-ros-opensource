cmake_minimum_required(VERSION 3.0.2)
project(voice_control_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  kuavo_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voice_control_node
  CATKIN_DEPENDS rospy kuavo_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python modules
install(DIRECTORY
  scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

## Install configuration files
# install(FILES
#   scripts/key_words.json
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

## Install launch files
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

