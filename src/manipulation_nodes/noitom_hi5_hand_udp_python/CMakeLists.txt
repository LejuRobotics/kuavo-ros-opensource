cmake_minimum_required(VERSION 3.0.2)
project(noitom_hi5_hand_udp_python)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../kuavo_common/cmake/kuavo_arch.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  tf
  tf2_msgs
  kuavo_ros_interfaces
  kuavo_msgs
  roslib
)

# Quest3 C++ monitor uses protobuf; wheel-arm / Jammy often only needs ROS msgs (PoseInfoList, etc.).
option(BUILD_MONITOR_QUEST3_CPP "Build monitor_quest3_cpp (protobuf)" ON)
if(KUAVO_BUILD_FOR_AARCH64)
  set(BUILD_MONITOR_QUEST3_CPP OFF CACHE BOOL "Build monitor_quest3_cpp (protobuf)" FORCE)
  # Jammy aarch64: skip roseus/eus (geneus paths break when CMAKE_PREFIX_PATH is overlaid).
  set(CATKIN_MESSAGE_GENERATORS "gencpp;genlisp;gennodejs;genpy")
endif()

add_message_files(
  FILES handRotation.msg
  QuaternionArray.msg
  handRotationEular.msg
  PoseInfo.msg
  PoseInfoList.msg
  JoySticks.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs sensor_msgs tf tf2_msgs kuavo_ros_interfaces kuavo_msgs message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#   std_msgs
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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuavo_ctrl_package
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(BUILD_MONITOR_QUEST3_CPP)
  find_package(Protobuf REQUIRED)
  if(KUAVO_BUILD_FOR_AARCH64)
    set(_proto_dir "${CMAKE_CURRENT_SOURCE_DIR}/protos")
    set(_proto_out "${CMAKE_CURRENT_BINARY_DIR}/protos_c")
    file(MAKE_DIRECTORY "${_proto_out}")
    set(PROTO_SRCS "")
    foreach(_proto hand_pose robot_info robot_state)
      set(_src "${_proto_out}/${_proto}.pb.cc")
      set(_hdr "${_proto_out}/${_proto}.pb.h")
      list(APPEND PROTO_SRCS "${_src}")
      add_custom_command(
        OUTPUT "${_src}" "${_hdr}"
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
          --cpp_out=${_proto_out}
          -I ${_proto_dir}
          ${_proto_dir}/${_proto}.proto
        DEPENDS ${_proto_dir}/${_proto}.proto
        COMMENT "Generating ${_proto}.pb from proto"
      )
    endforeach()
    include_directories(${Protobuf_INCLUDE_DIRS} ${_proto_out})
  else()
    # x86: use pre-generated protos (protoc 3.6.1), same as eee7f6b8 — avoids optional-field proto3 errors
    include_directories(${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/protos_c)
    set(PROTO_SRCS
      ${CMAKE_CURRENT_SOURCE_DIR}/protos_c/hand_pose.pb.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/protos_c/robot_info.pb.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/protos_c/robot_state.pb.cc
    )
  endif()
  add_executable(monitor_quest3_cpp src/monitor_quest3_node.cpp ${PROTO_SRCS})
  add_dependencies(monitor_quest3_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(monitor_quest3_cpp ${catkin_LIBRARIES} ${Protobuf_LIBRARIES})
endif()

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
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
if(BUILD_MONITOR_QUEST3_CPP)
  install(TARGETS monitor_quest3_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

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
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kuavo_ctrl_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
install(PROGRAMS
  scripts/monitor_quest3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
