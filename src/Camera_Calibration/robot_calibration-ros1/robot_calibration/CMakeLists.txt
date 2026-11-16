cmake_minimum_required(VERSION 3.0.2)
project(robot_calibration)

# Preserve catkin's environment-provided workspace prefixes before adding paths.
if (NOT DEFINED CMAKE_PREFIX_PATH)
  string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
endif ()

# Include the standard Noetic installation even when it was not sourced.
if (EXISTS "/opt/ros/noetic")
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
  if (EXISTS "/opt/ros/noetic/share")
    if ("$ENV{ROS_PACKAGE_PATH}" STREQUAL "")
      set(ENV{ROS_PACKAGE_PATH} "/opt/ros/noetic/share")
    else ()
      set(ENV{ROS_PACKAGE_PATH} "$ENV{ROS_PACKAGE_PATH}:/opt/ros/noetic/share")
    endif ()
  endif ()
endif ()

# Some Ubuntu 24.04 Noetic installations keep additional ROS packages here.
set(CALIBRATION_EXTRA_PREFIX "/home/lab/ros-noetic-extra/devel" CACHE PATH
    "Optional prefix containing additional robot_calibration dependencies")
if (EXISTS "${CALIBRATION_EXTRA_PREFIX}/.catkin")
  list(APPEND CMAKE_PREFIX_PATH "${CALIBRATION_EXTRA_PREFIX}")
  file(READ "${CALIBRATION_EXTRA_PREFIX}/.catkin" CALIBRATION_EXTRA_SOURCES)
  foreach(CALIBRATION_EXTRA_SOURCE ${CALIBRATION_EXTRA_SOURCES})
    if (IS_DIRECTORY "${CALIBRATION_EXTRA_SOURCE}")
      set(ENV{ROS_PACKAGE_PATH} "$ENV{ROS_PACKAGE_PATH}:${CALIBRATION_EXTRA_SOURCE}")
    endif ()
  endforeach()
endif ()

find_package(Boost REQUIRED system thread)

find_library(tinyxml_library tinyxml)
if (tinyxml_library)
  message (STATUS "Looking for libtinyxml - found")
  set(tinyxml_LIBRARIES ${tinyxml_library})
endif ()
find_path(tinyxml_include_dirs NAMES tinyxml.h PATH_SUFFIXES tinyxml)
if (NOT tinyxml_include_dirs)
   message (STATUS "Looking for tinyxml/tinyxml.hpp or tinyxml/tinyxml.h - not found.")
endif ()

find_package(orocos_kdl)
function(install_calibration_packages)
  message(STATUS "Installing missing calibration dependencies: ${ARGN}")
  find_program(APT_GET_EXECUTABLE apt-get)
  if (NOT APT_GET_EXECUTABLE)
    message(FATAL_ERROR "apt-get is unavailable. Install these dependencies manually: ${ARGN}")
  endif ()
  execute_process(COMMAND id -u OUTPUT_VARIABLE CURRENT_UID OUTPUT_STRIP_TRAILING_WHITESPACE)
  if (CURRENT_UID STREQUAL "0")
    set(INSTALL_PREFIX "")
  else ()
    find_program(SUDO_EXECUTABLE sudo)
    if (NOT SUDO_EXECUTABLE)
      message(FATAL_ERROR "sudo is unavailable. Install these dependencies manually: ${ARGN}")
    endif ()
    set(INSTALL_PREFIX "${SUDO_EXECUTABLE}" -n)
  endif ()
  execute_process(COMMAND ${INSTALL_PREFIX} "${APT_GET_EXECUTABLE}" update RESULT_VARIABLE APT_UPDATE_RESULT)
  if (NOT APT_UPDATE_RESULT STREQUAL "0")
    message(FATAL_ERROR "Could not update apt indexes. Check the configured apt repositories and network.")
  endif ()
  execute_process(COMMAND ${INSTALL_PREFIX} "${APT_GET_EXECUTABLE}" install -y ${ARGN} RESULT_VARIABLE APT_INSTALL_RESULT)
  if (NOT APT_INSTALL_RESULT STREQUAL "0")
    message(FATAL_ERROR "Could not install dependencies. Install them manually and retry: ${ARGN}")
  endif ()
endfunction()

set(MISSING_CALIBRATION_PACKAGES "")
find_package(Ceres QUIET)
if (NOT Ceres_FOUND)
  list(APPEND MISSING_CALIBRATION_PACKAGES libceres-dev)
endif ()

find_program(ROSPACK_EXECUTABLE rospack HINTS /opt/ros/noetic/bin)
if (NOT ROSPACK_EXECUTABLE)
  message(FATAL_ERROR "rospack not found. Source the ROS Noetic environment before building.")
endif ()
foreach(ROS_PACKAGE geometric_shapes moveit_msgs shape_msgs object_recognition_msgs octomap_msgs eigen_stl_containers random_numbers)
  execute_process(COMMAND "${ROSPACK_EXECUTABLE}" find "${ROS_PACKAGE}"
    RESULT_VARIABLE ROS_PACKAGE_RESULT OUTPUT_QUIET ERROR_QUIET)
  if (NOT ROS_PACKAGE_RESULT STREQUAL "0")
    string(REPLACE "_" "-" APT_PACKAGE "${ROS_PACKAGE}")
    list(APPEND MISSING_CALIBRATION_PACKAGES "ros-noetic-${APT_PACKAGE}")
  endif ()
endforeach()

if (MISSING_CALIBRATION_PACKAGES)
  install_calibration_packages(${MISSING_CALIBRATION_PACKAGES})
  execute_process(COMMAND "${ROSPACK_EXECUTABLE}" profile OUTPUT_QUIET ERROR_QUIET)
endif ()
if (NOT Ceres_FOUND)
  find_package(Ceres REQUIRED)
endif ()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    geometric_shapes
    kdl_parser
    moveit_msgs
    nav_msgs
    pluginlib
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    geometric_shapes
    kdl_parser
    moveit_msgs
    nav_msgs
    pluginlib
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    Boost
    orocos_kdl
  LIBRARIES
    robot_calibration
    robot_calibration_feature_finders
)

if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${CERES_INCLUDES}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

add_library(robot_calibration
  src/base_calibration.cpp
  src/calibration_export.cpp
  src/calibration_offset_parser.cpp
  src/calibration_csv_io.cpp
  src/fk_debug_utils.cpp
  src/capture_manager.cpp
  src/capture_poses.cpp
  src/chain_manager.cpp
  src/mesh_loader.cpp
  src/models.cpp
  src/optimization_params.cpp
  src/optimizer.cpp
)
target_link_libraries(robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
add_dependencies(robot_calibration
  robot_calibration_msgs_generate_messages_cpp
)

add_library(robot_calibration_feature_finders
  src/finders/checkerboard_finder.cpp
  src/finders/led_finder.cpp
  src/finders/plane_finder.cpp
  src/finders/robot_finder.cpp
  src/finders/scan_finder.cpp
)
target_link_libraries(robot_calibration_feature_finders
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(robot_calibration_feature_finders
  robot_calibration_msgs_generate_messages_cpp
)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(calibrate robot_calibration)

add_executable(capture_to_csv src/capture_to_csv.cpp)
target_link_libraries(capture_to_csv
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(capture_to_csv robot_calibration)

add_executable(optimize_from_csv src/optimize_from_csv.cpp)
target_link_libraries(optimize_from_csv
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(optimize_from_csv robot_calibration)

# Mocap joint-zero calibration: reads capture_*.json, builds CalibrationData,
# runs Ceres with chain3d_to_mocap error block.
add_executable(optimize_mocap src/optimize_mocap.cpp)
target_link_libraries(optimize_mocap
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  jsoncpp
)
add_dependencies(optimize_mocap robot_calibration)

add_executable(base_calibration_node src/base_calibration_node.cpp)
target_link_libraries(base_calibration_node
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(magnetometer_calibration src/magnetometer_calibration.cpp)
target_link_libraries(magnetometer_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(to_rpy src/tools/to_rpy.cpp)
target_link_libraries(to_rpy
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
add_dependencies(to_rpy robot_calibration)

add_executable(viz src/tools/viz.cpp)
target_link_libraries(viz
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
add_dependencies(viz robot_calibration)

add_executable(viz_mesh src/tools/viz_mesh.cpp)
target_link_libraries(viz_mesh
  robot_calibration
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
add_dependencies(viz_mesh robot_calibration)

add_subdirectory(test)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  base_calibration_node
  calibrate
  capture_to_csv
  optimize_from_csv
  optimize_mocap
  magnetometer_calibration
  robot_calibration
  robot_calibration_feature_finders
  to_rpy
  viz
  viz_mesh
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES robot_calibration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
