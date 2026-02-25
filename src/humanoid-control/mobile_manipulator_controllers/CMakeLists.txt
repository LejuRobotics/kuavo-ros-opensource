cmake_minimum_required(VERSION 3.10)
project(mobile_manipulator_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") # 添加调试信息

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  ocs2_mobile_manipulator
  ocs2_mobile_manipulator_ros
  kuavo_msgs
  std_srvs
)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    ocs2_mobile_manipulator
    ocs2_mobile_manipulator_ros
    kuavo_msgs
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/include
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

## Declare IK Target library
add_library(${PROJECT_NAME}_ik_target_lib
  src/mobileManipulatorIkTarget.cpp
)

target_link_libraries(${PROJECT_NAME}_ik_target_lib
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  yaml-cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}_base
  src/mobileManipulatorControllerBase.cpp
  src/mobileManipulatorVisualization.cpp
  src/TopicLogger.cpp
)

target_link_libraries(${PROJECT_NAME}_base
  ${PROJECT_NAME}_ik_target_lib
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  yaml-cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/mobileManipulatorController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  yaml-cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}_general
  src/mobileManipulatorControllerGeneral.cpp
)

target_link_libraries(${PROJECT_NAME}_general
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  yaml-cpp
)

# node
add_executable(${PROJECT_NAME}_node
  src/controllerNode.cpp
  )
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_compile_options(${PROJECT_NAME}_node PRIVATE ${OCS2_CXX_FLAGS})

# node
add_executable(${PROJECT_NAME}_node_general
  src/controllerNodeGeneral.cpp
  )
add_dependencies(${PROJECT_NAME}_node_general
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_node_general
  ${PROJECT_NAME}_general
  ${catkin_LIBRARIES}
  )
target_compile_options(${PROJECT_NAME}_node_general PRIVATE ${OCS2_CXX_FLAGS})

add_subdirectory(test)

#### install ####
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)