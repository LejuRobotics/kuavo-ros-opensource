cmake_minimum_required(VERSION 3.0.2)
project(kuavo_monitor)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# add_compile_options(-std=c++11)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ocs2_core
  ocs2_msgs
)
find_package(Boost 1.65 REQUIRED COMPONENTS thread system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)
# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(mpc_policy_publisher
  include/${PROJECT_NAME}/mpc_policy_publisher.h
  src/mpc_policy_publisher.cpp)

target_link_libraries(mpc_policy_publisher PUBLIC
  Boost::thread
  Boost::system
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node 
  include/kuavo_monitor/data_analyzer.hpp
  src/kuavo_monitor.cpp
  src/kuavo_monitor_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_BINARY_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_node
  mpc_policy_publisher
  yaml-cpp
  ${catkin_LIBRARIES})
#############
## Install ##
#############

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )