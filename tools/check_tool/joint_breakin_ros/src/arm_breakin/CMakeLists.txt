cmake_minimum_required(VERSION 3.0.2)
project(arm_breakin)

## Compile as C++17 (required by motorevo_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  breakin_msgs
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Find yaml-cpp for YAML configuration parsing
find_package(yaml-cpp REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   breakin_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS breakin_msgs roscpp std_msgs
)

###########
## Build ##
###########

## 查找hardware_plant目录，然后添加canbus_sdk和motorevo_controller作为子目录
# 尝试多个可能的路径
get_filename_component(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)

# 可能的hardware_plant路径列表
set(POSSIBLE_HARDWARE_PLANT_PATHS
    "${WORKSPACE_ROOT}/../../src/kuavo-ros-control-lejulib/hardware_plant"
    "${WORKSPACE_ROOT}/../../../src/kuavo-ros-control-lejulib/hardware_plant"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src/kuavo-ros-control-lejulib/hardware_plant"
)

# 查找hardware_plant目录
set(HARDWARE_PLANT_DIR "")
foreach(PATH ${POSSIBLE_HARDWARE_PLANT_PATHS})
    get_filename_component(ABS_PATH "${PATH}" ABSOLUTE)
    if(EXISTS "${ABS_PATH}/CMakeLists.txt")
        set(HARDWARE_PLANT_DIR "${ABS_PATH}")
        message(STATUS "找到hardware_plant: ${HARDWARE_PLANT_DIR}")
        break()
    endif()
endforeach()

if(HARDWARE_PLANT_DIR STREQUAL "")
    message(FATAL_ERROR "找不到hardware_plant目录。请确保路径正确")
endif()

# 设置库目录路径
set(CANBUS_SDK_DIR "${HARDWARE_PLANT_DIR}/lib/canbus_sdk")
set(MOTOREVO_CONTROLLER_DIR "${HARDWARE_PLANT_DIR}/lib/motorevo_controller")

# 检查目录是否存在
if(NOT EXISTS "${CANBUS_SDK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "找不到canbus_sdk目录: ${CANBUS_SDK_DIR}")
endif()

if(NOT EXISTS "${MOTOREVO_CONTROLLER_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "找不到motorevo_controller目录: ${MOTOREVO_CONTROLLER_DIR}")
endif()

message(STATUS "CANBUS_SDK_DIR: ${CANBUS_SDK_DIR}")
message(STATUS "MOTOREVO_CONTROLLER_DIR: ${MOTOREVO_CONTROLLER_DIR}")

# 添加canbus_sdk和motorevo_controller作为子目录，让它们在同一个构建系统中被构建
# 注意：需要先添加canbus_sdk，因为motorevo_controller依赖它
add_subdirectory(${CANBUS_SDK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/canbus_sdk EXCLUDE_FROM_ALL)
add_subdirectory(${MOTOREVO_CONTROLLER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/motorevo_controller EXCLUDE_FROM_ALL)

## Specify additional locations of header files（参考motorevo_controller/CMakeLists.txt）
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${MOTOREVO_CONTROLLER_DIR}/
    ${MOTOREVO_CONTROLLER_DIR}/src/
    ${MOTOREVO_CONTROLLER_DIR}/include
    ${MOTOREVO_CONTROLLER_DIR}/../../include
    ${CANBUS_SDK_DIR}/include
    ${yaml-cpp_INCLUDE_DIRS}
)

## Declare C++ executables
# roban_v14_dual 机型节点
add_executable(arm_breakin_node src/arm_breakin_node.cpp)

# roban_v17_dual 机型节点
add_executable(arm_breakin_roban_v17_dual_node src/arm_breakin_roban_v17_dual_node.cpp)

# kuavo_v52_dual 机型节点
add_executable(arm_breakin_kuavo_v52_dual_node src/arm_breakin_kuavo_v52_dual_node.cpp)

# kuavo_v53_dual 机型节点
add_executable(arm_breakin_kuavo_v53_dual_node src/arm_breakin_kuavo_v53_dual_node.cpp)

## Add cmake target dependencies
# 确保在构建arm_breakin_node之前先构建motorevo_controller和canbus_sdk
add_dependencies(arm_breakin_node 
    ${catkin_EXPORTED_TARGETS}
    motorevo_controller
    canbus_sdk
)

# 确保在构建arm_breakin_roban_v17_dual_node之前先构建motorevo_controller和canbus_sdk
add_dependencies(arm_breakin_roban_v17_dual_node 
    ${catkin_EXPORTED_TARGETS}
    motorevo_controller
    canbus_sdk
)

# 确保在构建arm_breakin_kuavo_v52_dual_node之前先构建motorevo_controller和canbus_sdk
add_dependencies(arm_breakin_kuavo_v52_dual_node 
    ${catkin_EXPORTED_TARGETS}
    motorevo_controller
    canbus_sdk
)

# 确保在构建arm_breakin_kuavo_v53_dual_node之前先构建motorevo_controller和canbus_sdk
add_dependencies(arm_breakin_kuavo_v53_dual_node 
    ${catkin_EXPORTED_TARGETS}
    motorevo_controller
    canbus_sdk
)

## Specify libraries to link
# 直接链接到通过add_subdirectory添加的库目标
# roban_v14_dual 节点
target_link_libraries(arm_breakin_node
    ${catkin_LIBRARIES}
    motorevo_controller
    canbus_sdk
    yaml-cpp
    pthread
    m
    stdc++
)

# roban_v17_dual 节点
target_link_libraries(arm_breakin_roban_v17_dual_node
    ${catkin_LIBRARIES}
    motorevo_controller
    canbus_sdk
    yaml-cpp
    pthread
    m
    stdc++
)

# kuavo_v52_dual 节点
target_link_libraries(arm_breakin_kuavo_v52_dual_node
    ${catkin_LIBRARIES}
    motorevo_controller
    canbus_sdk
    yaml-cpp
    pthread
    m
    stdc++
)

# kuavo_v53_dual 节点
target_link_libraries(arm_breakin_kuavo_v53_dual_node
    ${catkin_LIBRARIES}
    motorevo_controller
    canbus_sdk
    yaml-cpp
    pthread
    m
    stdc++
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
install(TARGETS arm_breakin_node arm_breakin_roban_v17_dual_node arm_breakin_kuavo_v52_dual_node arm_breakin_kuavo_v53_dual_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS arm_breakin_node arm_breakin_kuavo_v53_dual_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arm_breakin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
