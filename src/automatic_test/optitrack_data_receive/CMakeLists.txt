cmake_minimum_required(VERSION 3.0.2)
project(optitrack_data_receive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES optitrack_ros
  CATKIN_DEPENDS rospy geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/OptiTrack_Data_Receive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)
