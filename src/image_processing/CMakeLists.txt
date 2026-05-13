cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_package(
  LIBRARIES image_processing
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    image_transport
)

# Build C++ version of depth_inpainter
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(depth_inpainter_cpp src/depth_inpainter_cpp.cpp)
# target_link_libraries(depth_inpainter_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# install(TARGETS depth_inpainter_cpp
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(
  PROGRAMS
  scripts/depth_inpainter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
