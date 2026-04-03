cmake_minimum_required(VERSION 3.0.2)
project(humanoid_wheel_interface)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_sqp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  humanoid_interface
  kuavo_msgs
  ruckig_catkin
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

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# mobile maniulator interface library
add_library(${PROJECT_NAME}
  src/FactoryFunctions.cpp
  src/MobileManipulatorPreComputation.cpp
  src/MobileManipulatorPinocchioMapping.cpp
  src/constraint/EndEffectorConstraint.cpp
  src/constraint/EndEffectorMultiPointConstraint.cpp
  src/constraint/EndEffectorLocalMultiPointConstraint.cpp
  src/constraint/EndEffectorLocalConstraint.cpp
  src/constraint/EndEffectorTorsoConstraint.cpp
  src/constraint/TorsoTrackingConstraint.cpp
  src/dynamics/WheelBasedMobileManipulatorDynamics.cpp
  src/dynamics/WheelWorldBasedMobileManipulatorDynamics.cpp
  src/dynamics/DefaultManipulatorDynamics.cpp
  src/dynamics/FloatingArmManipulatorDynamics.cpp
  src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp
  src/HumanoidWheelInterface.cpp
  src/reference_manager/MobileManipulatorReferenceManager.cpp
  src/motion_planner/VelocityLimiter.cpp
  src/motion_planner/cmdPosePlannerWithRuckig.cpp
  src/motion_planner/cmdVelPlannerWithRuckig.cpp
  src/motion_planner/posePlannerTimedScheduler.cpp
  src/filters/jointCmdLimiter.cpp
  src/filters/KinemicLimitFilter.cpp
  src/filters/HybridRuckigExtrapolator.cpp
  src/cost/LinearStateInequalitySoftConstraint.cpp
  src/cost/TorsoTrackingBoxSoftCost.cpp
  src/cost/EndEffectorBoxSoftCost.cpp
  src/cost/EndEffectorMultiPointBoxSoftCost.cpp
  src/cost/EndEffectorLocalBoxSoftCost.cpp
  src/cost/selfDistanceBoxSoftCost.cpp
  src/cost/EndEffectorLocalMultiPointBoxSoftCost.cpp
  src/cost/EndEffectorJointBias.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)