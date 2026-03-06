cmake_minimum_required(VERSION 3.0.2)
project(humanoid_wheel_interface_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  tf
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  geometry_msgs
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_sqp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  humanoid_wheel_interface
  kuavo_common
  kuavo_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# 首先创建 MobileManipulatorDummyVisualization 库
add_library(${PROJECT_NAME}
  src/MobileManipulatorDummyVisualization.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

# MPC node
add_executable(mobile_manipulator_mpc_node
  src/MobileManipulatorMpcNode.cpp
)
add_dependencies(mobile_manipulator_mpc_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_mpc_node
  ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})

# SQP MPC node
add_executable(mobile_manipulator_sqp_mpc_node
  src/MobileManipulatorSqpMpcNode.cpp
)
add_dependencies(mobile_manipulator_sqp_mpc_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_sqp_mpc_node
  ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_sqp_mpc_node PUBLIC ${FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
  src/MobileManipulatorDummyMRT.cpp
  # src/MobileManipulatorDummyVisualization.cpp
)
add_dependencies(mobile_manipulator_dummy_mrt_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_dummy_mrt_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})

# Target node
add_executable(mobile_manipulator_target
  src/MobileManipulatorTarget.cpp
)
add_dependencies(mobile_manipulator_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_target
  ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})

# Mobile manipulator joy command node
add_executable(mobile_manipulator_joy_command_node
  src/MobileManipulatorJoyCommandNode.cpp
)
add_dependencies(mobile_manipulator_joy_command_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mobile_manipulator_joy_command_node
  ${catkin_LIBRARIES}
)
target_compile_options(mobile_manipulator_joy_command_node PUBLIC ${FLAGS})

# Chassis direct joy control node (底盘直接控制节点)
add_executable(chassis_direct_joy_control_node
  src/ChassisDirectJoyControl.cpp
)
add_dependencies(chassis_direct_joy_control_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(chassis_direct_joy_control_node
  ${catkin_LIBRARIES}
)
target_compile_options(chassis_direct_joy_control_node PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS mobile_manipulator_mpc_node mobile_manipulator_sqp_mpc_node mobile_manipulator_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  TARGETS 
    mobile_manipulator_mpc_node 
    mobile_manipulator_sqp_mpc_node
    mobile_manipulator_dummy_mrt_node 
    mobile_manipulator_target
    mobile_manipulator_joy_command_node
    chassis_direct_joy_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/chassis_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_features(mobile_manipulator_mpc_node PUBLIC cxx_std_17)
target_compile_features(mobile_manipulator_sqp_mpc_node PUBLIC cxx_std_17)
target_compile_features(mobile_manipulator_joy_command_node PUBLIC cxx_std_17)
target_compile_features(chassis_direct_joy_control_node PUBLIC cxx_std_17)
