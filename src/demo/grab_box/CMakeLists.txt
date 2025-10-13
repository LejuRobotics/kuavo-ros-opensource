cmake_minimum_required(VERSION 3.5)

project(grab_box)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    rospy
    std_msgs
    ocs2_mpc
    ocs2_msgs
    ocs2_ros_interfaces
    ocs2_robotic_tools
    kuavo_msgs
    sensor_msgs
    motion_capture_ik
    humanoid_interface
    humanoid_interface_ros
    apriltag_ros
    behaviortree_cpp_v3)

catkin_package(
  INCLUDE_DIRS 
    include
    ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(GrabBoxLib INTERFACE)
target_include_directories(GrabBoxLib INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${drake_INCLUDE_DIRS}
  # /opt/drake/include
)
target_link_libraries(GrabBoxLib INTERFACE drake::drake)

# grab_box
add_executable(${PROJECT_NAME}_demo src/grab_box_demo.cpp src/poseTransformer.cpp)
target_link_libraries(${PROJECT_NAME}_demo 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GrabBoxLib}
  yaml-cpp
  drake::drake
  )
target_include_directories(${PROJECT_NAME}_demo PUBLIC
  ${PROJECT_BINARY_DIR}/include
)

add_executable(tag_tracker_node src/tagTracker.cpp src/poseTransformer.cpp)
target_link_libraries(tag_tracker_node ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GrabBoxLib}
  yaml-cpp
  drake::drake
  )
target_include_directories(tag_tracker_node PUBLIC
  ${PROJECT_BINARY_DIR}/include
)

# example
add_executable(btcpp_sample test/test1.cpp)
target_link_libraries(btcpp_sample  ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(btcpp_sample PUBLIC
  ${PROJECT_BINARY_DIR}/include
)
# single step
add_executable(single_step_test test/singleStepControlTest.cpp)
target_link_libraries(single_step_test  ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GrabBoxLib})
target_include_directories(single_step_test PUBLIC
  ${PROJECT_BINARY_DIR}/include
)
# hand trajectory
add_executable(handTrajectoryTest test/handTrajectoryTest.cpp)
target_link_libraries(handTrajectoryTest  ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GrabBoxLib})
target_include_directories(handTrajectoryTest PUBLIC
  ${PROJECT_BINARY_DIR}/include
)
#### install ####
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)