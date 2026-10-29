cmake_minimum_required(VERSION 3.16)
project(wheel_arm_abs_vel LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy roslib std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs visualization_msgs nodelet pluginlib kuavo_msgs kuavo_assets kuavo_common leju_utils humanoid_controllers humanoid_interface humanoid_interface_drake humanoid_interface_ros humanoid_wheel_interface humanoid_wheel_interface_ros humanoid_wheel_wbc motion_capture_ik noitom_hi5_hand_udp_python leju_mobile_base_msgs ocs2_core ocs2_msgs)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(drake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
add_message_files(FILES
  AbsoluteIkCommitDebug.msg QuestTcpMappingDebug.msg TeleopTcpTarget.msg
  VelocityIkFeasibilityDebug.msg VelocityIkInputDebug.msg VelocityIkMapperStatus.msg
  VelocityIkShadow.msg WholeBodyIkReference.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(INCLUDE_DIRS include
  LIBRARIES wheel_absolute_controllers wheel_absolute_dynamics
  CATKIN_DEPENDS message_runtime roscpp rospy roslib std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs visualization_msgs nodelet pluginlib kuavo_msgs kuavo_assets kuavo_common leju_utils humanoid_controllers humanoid_interface humanoid_interface_drake humanoid_interface_ros humanoid_wheel_interface humanoid_wheel_interface_ros humanoid_wheel_wbc motion_capture_ik noitom_hi5_hand_udp_python leju_mobile_base_msgs ocs2_core ocs2_msgs)
include_directories(include ${catkin_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
# Pinocchio exports MPL arity 30; Boost's preprocessed headers assume 20.
# Set this before all ROS/OCS2 headers, independent of include order.
add_compile_definitions(KUAVO_HAS_DRAKE HUMANOID_CONTROLLERS_HAS_DRAKE
  BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
set(VELOCITY_IK_CATKIN_BUILD ON)
set(VELOCITY_IK_MESSAGE_INCLUDE "${CATKIN_DEVEL_PREFIX}/include")
set(VELOCITY_IK_MESSAGE_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_subdirectory(velocity_ik)
add_library(wheel_absolute_dynamics
  src/absolute/AbsoluteWbcBase.cpp src/absolute/AbsoluteWeightedWbc.cpp
  src/absolute/AbsoluteContactForceWbc.cpp src/RneaBackend.cpp)
target_link_libraries(wheel_absolute_dynamics ${catkin_LIBRARIES} pinocchio::pinocchio)
add_dependencies(wheel_absolute_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(wheel_absolute_controllers
  src/absolute/humanoidController_wheel_absolute.cpp
  src/absolute/ControlDataManager.cpp src/absolute/ArmTrajReceiver.cpp
  src/absolute/DesiredForceManager.cpp)
target_link_libraries(wheel_absolute_controllers wheel_absolute_dynamics
  ${catkin_LIBRARIES} pinocchio::pinocchio yaml-cpp drake::drake ${JSONCPP_LIBRARIES})
add_dependencies(wheel_absolute_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_library(nodelet_wheel_absolute_controller src/absolute/controllerNodelet.cpp)
target_link_libraries(nodelet_wheel_absolute_controller wheel_absolute_controllers ${catkin_LIBRARIES})
add_dependencies(nodelet_wheel_absolute_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Fail at link time when a private controller callback has no implementation.
foreach(target wheel_absolute_dynamics wheel_absolute_controllers nodelet_wheel_absolute_controller)
  target_link_options(${target} PRIVATE "-Wl,--no-undefined")
endforeach()
add_executable(fixed_ik_reference_services_node src/FixedIkReferenceServicesNode.cpp)
target_link_libraries(fixed_ik_reference_services_node ${catkin_LIBRARIES} pinocchio::pinocchio)
add_dependencies(fixed_ik_reference_services_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(humanoid_quest_absolute_control src/QuestAbsoluteControlFSMNode.cpp)
# Reuse the existing inline utility without copying or editing its implementation.
target_include_directories(humanoid_quest_absolute_control PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../humanoid-control/humanoid_interface_ros/src)
target_link_libraries(humanoid_quest_absolute_control ${catkin_LIBRARIES} drake::drake)
add_dependencies(humanoid_quest_absolute_control ${catkin_EXPORTED_TARGETS})
add_executable(sg100_mapper_bridge src/mapper_bridge.cpp)
target_link_libraries(sg100_mapper_bridge ${catkin_LIBRARIES})
add_dependencies(sg100_mapper_bridge ${catkin_EXPORTED_TARGETS})
install(TARGETS wheel_absolute_dynamics wheel_absolute_controllers nodelet_wheel_absolute_controller
  fixed_ik_reference_services_node humanoid_quest_absolute_control sg100_mapper_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch plugins docs scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS PATTERN "__pycache__" EXCLUDE)
file(GLOB runtime_scripts scripts/*.py scripts/*.sh)
install(PROGRAMS ${runtime_scripts} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_test(NAME mapper_scheduling_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mapper_scheduling_test.py)
  add_test(NAME mapper_math_optimization_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mapper_math_optimization_test.py)
  add_test(NAME optical_follow_entry_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/optical_follow_entry_test.py)
  add_test(NAME tail_filter_frequency_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/tail_filter_frequency_test.py)
  enable_testing()
  find_package(PythonInterp 3 REQUIRED)
  foreach(name shared_model_resolution arm_hold_base pico_receive_isolation fixed_pipeline_hold startup_arm_action pico_head_follow arm_torso_hold torso_controls absolute_control_mechanism fixed_follow_operation absolute_sim_source pico_discovery pico_input_parity terminal_initial_start runtime_partition runtime_topology x86_topology model_platform_matrix absolute_robot_version absolute_end_effector wheel_base_command velocity_ik_base_command)
    add_test(NAME ${name}_test COMMAND ${PYTHON_EXECUTABLE}
      -B ${CMAKE_CURRENT_SOURCE_DIR}/test/${name}_test.py)
  endforeach()
  foreach(name clocked_reference_freshness mobile_base_lock execution_handshake_timing)
    add_executable(${name}_test test/${name}_test.cpp)
    target_compile_options(${name}_test PRIVATE -UNDEBUG)
    add_test(NAME ${name}_test COMMAND ${name}_test)
  endforeach()
  add_executable(execution_reference_regression test/execution_reference_regression.cpp)
  target_compile_options(execution_reference_regression PRIVATE -UNDEBUG)
  add_test(NAME execution_reference_regression COMMAND execution_reference_regression)
  add_executable(rnea_backend_test test/rnea_backend_test.cpp)
  target_link_libraries(rnea_backend_test wheel_absolute_dynamics velocity_ik_model)
  target_compile_options(rnea_backend_test PRIVATE -UNDEBUG)
  add_test(NAME rnea_backend_test COMMAND rnea_backend_test ${CMAKE_CURRENT_SOURCE_DIR}/../..)
  add_test(NAME rnea_backend_s63_test COMMAND rnea_backend_test ${CMAKE_CURRENT_SOURCE_DIR}/../.. 63)
  add_test(NAME rnea_backend_400062_test COMMAND rnea_backend_test ${CMAKE_CURRENT_SOURCE_DIR}/../.. 400062)
  add_executable(full_model_contract_test test/full_model_contract_test.cpp)
  add_test(NAME full_model_contract_test COMMAND full_model_contract_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../kuavo_assets)
  add_test(NAME quest_startup_config_test COMMAND ${PYTHON_EXECUTABLE}
    -B ${CMAKE_CURRENT_SOURCE_DIR}/test/quest_startup_config_test.py
    $<TARGET_FILE:humanoid_quest_absolute_control>)
  set_tests_properties(quest_startup_config_test PROPERTIES TIMEOUT 120)
  add_test(NAME launch_model_contract_test COMMAND ${PYTHON_EXECUTABLE}
    -B ${CMAKE_CURRENT_SOURCE_DIR}/test/launch_model_contract_test.py)
  add_test(NAME s63_model_contract_test COMMAND ${PYTHON_EXECUTABLE}
    -B ${CMAKE_CURRENT_SOURCE_DIR}/test/s63_model_contract_test.py)
  foreach(name whole_body_motion_policy whole_body_dof_layout whole_body_recruitment)
    catkin_add_gtest(${name}_test test/${name}_test.cpp)
    if(TARGET ${name}_test)
      target_link_libraries(${name}_test gtest_main)
    endif()
  endforeach()
endif()

include(cmake/absolute_simulation.cmake)
