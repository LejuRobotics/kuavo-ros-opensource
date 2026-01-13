cmake_minimum_required(VERSION 3.0.2)
project(ruiwo_controller_example)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

## Set dependencies directory
set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../installed)

#############
## Catkin Specific Configuration ##
#############

catkin_package()

#############
## Build ##
#############

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${DEPS_DIR}/include/
  ${DEPS_DIR}/include/hardware_plant
)

## Link directories
link_directories(${DEPS_DIR}/lib)

## Declare a C++ executable
add_executable(motor_control_node src/motor_control_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(motor_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(motor_control_node
  ${catkin_LIBRARIES}
  ruiwo_actuatorCXXLib
  bmapi64
  yaml-cpp
  pthread
)

###########
## Install ##
###########

## Mark executables for installation
install(TARGETS motor_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation
install(FILES
  launch/motor_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)