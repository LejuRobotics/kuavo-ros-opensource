cmake_minimum_required(VERSION 3.0.2)
project(humanoid_interface_ros)

set(CMAKE_BUILD_TYPE=RelWithDebInfo)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_msgs
  ocs2_self_collision
  kuavo_common
  kuavo_msgs
  humanoid_interface
  ocs2_ros_interfaces
  humanoid_interface_drake
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(drake CONFIG REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

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
    message_runtime
  DEPENDS
    Boost
    pinocchio
)

###########
## Build ##
###########

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/HumanoidVisualizer.cpp
  src/gait/GaitSwitchByNamePublisher.cpp
  src/visualization/HumanoidRbdPublisher.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  drake::drake
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# DDP-MPC node for biped robot
add_executable(humanoid_ddp_mpc
  src/HumanoidDdpMpcNode.cpp
)
add_dependencies(humanoid_ddp_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_ddp_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(humanoid_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for biped robot
add_executable(humanoid_sqp_mpc
  src/HumanoidSqpMpcNode.cpp
)
add_dependencies(humanoid_sqp_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_sqp_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(humanoid_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for biped robot
# add_executable(humanoid_ipm_mpc
#   src/HumanoidIpmMpcNode.cpp
# )
# add_dependencies(humanoid_ipm_mpc
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(humanoid_ipm_mpc
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# target_compile_options(humanoid_ipm_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(humanoid_dummy
  src/HumanoidDummyNode.cpp
)
add_dependencies(humanoid_dummy
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Hand command node for mpc
add_executable(humanoid_hand_control
  src/HumanoidHandCommandNode.cpp
)
add_dependencies(humanoid_hand_control
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_hand_control
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake
)
target_compile_options(humanoid_hand_control PRIVATE ${OCS2_CXX_FLAGS})

# VR Hand command node for mpc
add_executable(humanoid_VR_hand_control
  src/HumanoidVRHandCommandNode.cpp
)
add_dependencies(humanoid_VR_hand_control
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_VR_hand_control
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake
)
target_compile_options(humanoid_VR_hand_control PRIVATE ${OCS2_CXX_FLAGS})

# Arm target (with time traj) command node for mpc
add_executable(humanoid_Arm_time_target_control
  src/HumanoidArmTargetPosesNode.cpp
)
add_dependencies(humanoid_Arm_time_target_control
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_Arm_time_target_control
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake
)
target_compile_options(humanoid_Arm_time_target_control PRIVATE ${OCS2_CXX_FLAGS})   


# Joystick command node for mpc
# add_executable(humanoid_joy_control
#   src/HumanoidJoyCommandNode.cpp
# )
# add_dependencies(humanoid_joy_control
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(humanoid_joy_control
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   drake::drake 
# )
# target_compile_options(humanoid_joy_control PRIVATE ${OCS2_CXX_FLAGS})

# # Joystick command node for mpc with arm
# add_executable(humanoid_joy_control_with_arm
#   src/HumanoidJoyCommandNodeWithArm.cpp
# )
# add_dependencies(humanoid_joy_control_with_arm
# ${PROJECT_NAME}
# ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS}
# )
# # Joystick command node for mpc with vel
# add_executable(humanoid_joy_control_with_vel
#   src/newTargetPublisher/HumanoidJoyCommandNodeWithVel.cpp
# )
# add_dependencies(humanoid_joy_control_with_vel
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(humanoid_joy_control_with_arm
# ${PROJECT_NAME}
# ${catkin_LIBRARIES}
# drake::drake 
# )
# target_link_libraries(humanoid_joy_control_with_vel
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   drake::drake 
# )
# target_compile_options(humanoid_joy_control_with_arm PRIVATE ${OCS2_CXX_FLAGS})
# target_compile_options(humanoid_joy_control_with_vel PRIVATE ${OCS2_CXX_FLAGS})

# # Joystick command node for mpc(auto gait)
# add_executable(humanoid_joy_control_auto_gait
#   src/HumanoidAutoGaitJoyCommandNode.cpp
# )
# add_dependencies(humanoid_joy_control_auto_gait
#   ${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(humanoid_joy_control_auto_gait
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   drake::drake 
# )
# target_compile_options(humanoid_joy_control_auto_gait PRIVATE ${OCS2_CXX_FLAGS})

# Joystick command node for mpc(auto gait) with vel
add_executable(humanoid_joy_control_auto_gait_with_vel
  src/newTargetPublisher/HumanoidAutoGaitJoyCommandNodeWithVel.cpp
)
add_dependencies(humanoid_joy_control_auto_gait_with_vel
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_joy_control_auto_gait_with_vel
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_joy_control_auto_gait_with_vel PRIVATE ${OCS2_CXX_FLAGS})


# Keyboard command node for target trajectories
add_executable(humanoid_target
  src/HumanoidPoseCommandNode.cpp
)
add_dependencies(humanoid_target
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories with arm
add_executable(humanoid_target_with_arm
  src/HumanoidPoseCommandNodeWithArm.cpp
)
add_dependencies(humanoid_target_with_arm
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_target_with_arm
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_target_with_arm PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(humanoid_gait_command
  src/HumanoidGaitCommandNode.cpp
)
add_dependencies(humanoid_gait_command
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_gait_command PRIVATE ${OCS2_CXX_FLAGS})

# Quest joystick command node for mpc with arm
add_executable(humanoid_quest_control_with_arm
  src/QuestControlFSMNode.cpp
)
add_dependencies(humanoid_quest_control_with_arm
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_quest_control_with_arm
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_quest_control_with_arm PRIVATE ${OCS2_CXX_FLAGS})


# Gait switch by name node for switching between different gaits
add_executable(humanoid_gait_switch_by_name
  src/HumanoidGaitSwitchByNameNode.cpp
)
add_dependencies(humanoid_gait_switch_by_name
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(humanoid_gait_switch_by_name
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  drake::drake 
)
target_compile_options(humanoid_gait_switch_by_name PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_humanoid_ros")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
       humanoid_ddp_mpc
       humanoid_sqp_mpc
       humanoid_ipm_mpc
       humanoid_dummy
       humanoid_target
       humanoid_gait_command
       humanoid_quest_control_with_arm
       humanoid_joy_control_auto_gait_with_vel
       humanoid_VR_hand_control
       humanoid_hand_control
       humanoid_target_with_arm
       humanoid_Arm_time_target_control
       humanoid_gait_switch_by_name
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  TARGETS
    humanoid_ddp_mpc
    humanoid_sqp_mpc
    # humanoid_ipm_mpc
    humanoid_dummy
    humanoid_target
    humanoid_quest_control_with_arm
    humanoid_gait_command
    humanoid_joy_control_auto_gait_with_vel
    humanoid_VR_hand_control
    humanoid_hand_control
    humanoid_target_with_arm
    humanoid_Arm_time_target_control
    humanoid_gait_switch_by_name
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                   GROUP_EXECUTE GROUP_READ
                   WORLD_EXECUTE WORLD_READ
)


#############
## Testing ##
#############

# 删除服务消息生成配置
# add_service_files(
#   FILES
#   SetJoyTopic.srv
# )
# 
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
