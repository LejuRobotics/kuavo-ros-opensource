cmake_minimum_required(VERSION 3.16)
if(NOT VELOCITY_IK_CATKIN_BUILD)
  project(pinocchio_velocity_ik LANGUAGES CXX)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(pinocchio REQUIRED)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party_libs/proxsuite
  ${CMAKE_CURRENT_BINARY_DIR}/proxsuite)
if(NOT VELOCITY_IK_CATKIN_BUILD)
  find_package(catkin REQUIRED COMPONENTS roscpp roslib nav_msgs std_msgs std_srvs kuavo_msgs)
endif()
if(NOT VELOCITY_IK_CATKIN_BUILD)
  set(VELOCITY_IK_MESSAGE_INCLUDE "" CACHE PATH "Generated D message headers")
endif()
if(NOT VELOCITY_IK_CATKIN_BUILD AND NOT EXISTS "${VELOCITY_IK_MESSAGE_INCLUDE}/wheel_arm_abs_vel/TeleopTcpTarget.h")
  message(FATAL_ERROR "Generate the matching motion_capture_ik messages first")
endif()
add_library(velocity_ik_model STATIC model_pinocchio.cpp)
target_link_libraries(velocity_ik_model PUBLIC pinocchio::pinocchio)
target_include_directories(velocity_ik_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_library(velocity_ik_qp STATIC weighted_qp.cpp)
target_link_libraries(velocity_ik_qp PUBLIC kuavo_proxsuite)
target_include_directories(velocity_ik_qp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(pinocchio_velocity_ik_node ../src/pinocchio_velocity_ik_node.cpp)
target_include_directories(pinocchio_velocity_ik_node PRIVATE ../include ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
target_link_libraries(pinocchio_velocity_ik_node PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
if(VELOCITY_IK_CATKIN_BUILD)
  add_dependencies(pinocchio_velocity_ik_node ${VELOCITY_IK_MESSAGE_TARGETS})
  install(TARGETS pinocchio_velocity_ik_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

find_package(Threads REQUIRED)
target_link_libraries(velocity_ik_qp PUBLIC Threads::Threads)

if(CATKIN_ENABLE_TESTING)
add_executable(velocity_ik_frozen_qp_test frozen_qp_test.cpp)
target_link_libraries(velocity_ik_frozen_qp_test PRIVATE velocity_ik_qp)
add_executable(velocity_ik_local_regression_test local_regression_test.cpp)
target_link_libraries(velocity_ik_local_regression_test PRIVATE velocity_ik_qp)
# Isolated transaction logic: no ROS publisher or control runtime.
add_executable(velocity_ik_fixed_prepare_transaction_test fixed_prepare_transaction_test.cpp)
add_executable(velocity_ik_sample_freshness_test sample_freshness_test.cpp)

# Certified zero-endpoint seeding for the normal stopping-domain inversion.
add_executable(velocity_ik_exact_stop_test exact_stop_test.cpp)
target_link_libraries(velocity_ik_exact_stop_test PRIVATE velocity_ik_model)

# Keep test assertions active in Release; production targets retain their flags.
foreach(test_target velocity_ik_sample_freshness_test
    velocity_ik_fixed_prepare_transaction_test velocity_ik_exact_stop_test
    velocity_ik_local_regression_test)
  target_compile_options(${test_target} PRIVATE -UNDEBUG)
endforeach()

enable_testing()
add_executable(velocity_ik_startup_readiness_test startup_readiness_test.cpp)
target_compile_options(velocity_ik_startup_readiness_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_startup_readiness_test COMMAND velocity_ik_startup_readiness_test)
add_executable(velocity_ik_innovation_reference_test innovation_reference_test.cpp)
target_link_libraries(velocity_ik_innovation_reference_test PRIVATE velocity_ik_model)
target_compile_options(velocity_ik_innovation_reference_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_innovation_reference_test COMMAND velocity_ik_innovation_reference_test)
add_executable(velocity_ik_reference_dynamics_test reference_dynamics_test.cpp)
target_link_libraries(velocity_ik_reference_dynamics_test PRIVATE velocity_ik_model)
target_compile_options(velocity_ik_reference_dynamics_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_reference_dynamics_test COMMAND velocity_ik_reference_dynamics_test)
add_executable(velocity_ik_continuous_reference_test continuous_reference_test.cpp)
target_link_libraries(velocity_ik_continuous_reference_test PRIVATE velocity_ik_model)
target_compile_options(velocity_ik_continuous_reference_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_continuous_reference_test COMMAND velocity_ik_continuous_reference_test)
add_executable(velocity_ik_adaptive_reference_test adaptive_reference_test.cpp)
target_link_libraries(velocity_ik_adaptive_reference_test PRIVATE velocity_ik_model)
target_compile_options(velocity_ik_adaptive_reference_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_adaptive_reference_test COMMAND velocity_ik_adaptive_reference_test)
find_package(PythonInterp 3 REQUIRED)
add_test(NAME velocity_ik_pose_velocity_kalman_test
  COMMAND ${CMAKE_COMMAND} -E env
    ADAPTIVE_REFERENCE_RUNNER=$<TARGET_FILE:velocity_ik_adaptive_reference_test>
    OPENBLAS_NUM_THREADS=1
    ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/pose_velocity_kalman_test.py)
add_test(NAME velocity_ik_fixed_follow_operation_test
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/fixed_follow_operation_test.py)
add_executable(velocity_ik_teleop_continuity_test teleop_continuity_test.cpp)
target_compile_options(velocity_ik_teleop_continuity_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_teleop_continuity_test COMMAND velocity_ik_teleop_continuity_test)
option(VELOCITY_IK_BUILD_OPERATING_REPLAY "Build isolated actual-IK offline replay (no publishers)" OFF)
if(VELOCITY_IK_BUILD_OPERATING_REPLAY)
  add_executable(velocity_ik_xa_prepare_handoff_test xa_prepare_handoff_test.cpp)
  target_include_directories(velocity_ik_xa_prepare_handoff_test PRIVATE ../include ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
  target_link_libraries(velocity_ik_xa_prepare_handoff_test PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
  add_executable(velocity_ik_continuity_integration_test continuity_integration_test.cpp)
  target_include_directories(velocity_ik_continuity_integration_test PRIVATE ../include ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
  target_link_libraries(velocity_ik_continuity_integration_test PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
endif()
add_executable(velocity_ik_operating_temporal_test operating_temporal_test.cpp)
add_executable(velocity_ik_execution_governor_test execution_governor_test.cpp)
target_link_libraries(velocity_ik_execution_governor_test PRIVATE velocity_ik_qp)
target_compile_options(velocity_ik_execution_governor_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_execution_governor_test COMMAND velocity_ik_execution_governor_test)
target_link_libraries(velocity_ik_operating_temporal_test PRIVATE velocity_ik_qp)
target_compile_options(velocity_ik_operating_temporal_test PRIVATE -UNDEBUG)
foreach(test_target velocity_ik_operating_temporal_test velocity_ik_sample_freshness_test
    velocity_ik_fixed_prepare_transaction_test velocity_ik_exact_stop_test velocity_ik_local_regression_test)
  add_test(NAME ${test_target} COMMAND ${test_target})
endforeach()

# Saturation/stop regressions for the separated orientation bandwidth path.
add_executable(velocity_ik_orientation_bandwidth_test orientation_bandwidth_test.cpp)
target_link_libraries(velocity_ik_orientation_bandwidth_test PRIVATE velocity_ik_model)
target_compile_options(velocity_ik_orientation_bandwidth_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_orientation_bandwidth_test COMMAND velocity_ik_orientation_bandwidth_test)

add_executable(velocity_ik_fixed_bandwidth_test fixed_bandwidth_reference_test.cpp)
target_compile_options(velocity_ik_fixed_bandwidth_test PRIVATE -UNDEBUG)
target_link_libraries(velocity_ik_fixed_bandwidth_test Eigen3::Eigen)
add_test(NAME velocity_ik_fixed_bandwidth_test COMMAND velocity_ik_fixed_bandwidth_test)

add_executable(velocity_ik_joint_jerk_test joint_jerk_test.cpp)
target_compile_options(velocity_ik_joint_jerk_test PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_joint_jerk_test COMMAND velocity_ik_joint_jerk_test)
if(VELOCITY_IK_BUILD_OPERATING_REPLAY)
  foreach(name joint_jerk_integration_test fixed_bandwidth_integration_test)
    add_executable(velocity_ik_${name} ${name}.cpp)
    target_include_directories(velocity_ik_${name} PRIVATE ../include ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
    target_link_libraries(velocity_ik_${name} PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
    target_compile_options(velocity_ik_${name} PRIVATE -UNDEBUG)
  endforeach()
endif()

# Canonical S62 reduction, named joint mapping, full-URDF FK and finite-difference Jacobians.
add_executable(velocity_ik_model_alignment_test model_alignment_test.cpp)
target_link_libraries(velocity_ik_model_alignment_test PRIVATE velocity_ik_model)
add_test(NAME velocity_ik_model_alignment_test COMMAND velocity_ik_model_alignment_test
  ${CMAKE_CURRENT_SOURCE_DIR}/../../kuavo_assets ${CMAKE_CURRENT_SOURCE_DIR}/../config/model_s62.json)
add_test(NAME velocity_ik_model_alignment_400062_test COMMAND velocity_ik_model_alignment_test
  ${CMAKE_CURRENT_SOURCE_DIR}/../../kuavo_assets ${CMAKE_CURRENT_SOURCE_DIR}/../config/model_s400062.json 400062)
add_test(NAME velocity_ik_neutral_model_geometry_test
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/neutral_model_geometry_test.py)

add_executable(velocity_ik_runtime_window_test runtime_window_test.cpp)
target_compile_options(velocity_ik_runtime_window_test PRIVATE -UNDEBUG)
target_link_libraries(velocity_ik_runtime_window_test pthread)
add_test(NAME velocity_ik_runtime_window_test COMMAND velocity_ik_runtime_window_test)

add_executable(velocity_ik_j3_window_replay EXCLUDE_FROM_ALL j3_window_replay.cpp)
target_include_directories(velocity_ik_j3_window_replay PRIVATE ../include ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
target_link_libraries(velocity_ik_j3_window_replay PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
target_compile_options(velocity_ik_j3_window_replay PRIVATE -UNDEBUG)
add_test(NAME velocity_ik_frozen_qp_test COMMAND velocity_ik_frozen_qp_test
  ${CMAKE_CURRENT_SOURCE_DIR}/frozen_qp_regression.json)
endif()  # CATKIN_ENABLE_TESTING
