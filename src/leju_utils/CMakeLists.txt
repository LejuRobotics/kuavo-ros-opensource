cmake_minimum_required(VERSION 3.0.2)
project(leju_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(kuavo_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES leju_utils
  CATKIN_DEPENDS kuavo_msgs geometry_msgs std_msgs sensor_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${kuavo_msgs_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)