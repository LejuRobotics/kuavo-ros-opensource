cmake_minimum_required(VERSION 3.0.2)
project(kuavo_xsense_gmr)

## 找到 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  kuavo_msgs
)

## Python 设置
catkin_python_setup()

## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuavo_xsense_udp_core
  CATKIN_DEPENDS 
    roscpp
    rospy 
    std_msgs
    geometry_msgs 
    tf
    kuavo_msgs
)

###########
## Build ##
###########

## 指定 Python 包的位置
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kuavo_xsense_udp_core
  src/xsens_udp/parser.cpp
  src/xsens_udp/fragment_assembler.cpp
  src/xsens_udp/udp_receiver.cpp
)
target_link_libraries(kuavo_xsense_udp_core
  ${catkin_LIBRARIES}
)
add_dependencies(kuavo_xsense_udp_core
  ${catkin_EXPORTED_TARGETS}
)

add_executable(xsense_udp_receiver_node
  src/xsense_udp_receiver_node.cpp
)
target_link_libraries(xsense_udp_receiver_node
  kuavo_xsense_udp_core
  ${catkin_LIBRARIES}
)
add_dependencies(xsense_udp_receiver_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(xsense_pico_joy_receiver_node
  src/xsense_pico_joy_receiver_node.cpp
)
target_link_libraries(xsense_pico_joy_receiver_node
  ${catkin_LIBRARIES}
)
add_dependencies(xsense_pico_joy_receiver_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(xsense_pico_control_node
  src/xsense_pico_control_node.cpp
)
target_link_libraries(xsense_pico_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(xsense_pico_control_node
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/xsense_streaming_node.py
  scripts/xsens_bag_to_xsense_topics.py
  scripts/xsense_bvh_desc_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS kuavo_xsense_udp_core xsense_udp_receiver_node xsense_pico_joy_receiver_node xsense_pico_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Wrapper Shell 脚本
install(PROGRAMS
  scripts/run_with_venv.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# 安装 assets 文件
install(DIRECTORY assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/assets
)

# 安装 Python 模块
install(DIRECTORY kuavo_gmr/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/kuavo_gmr
  PATTERN ".svn" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)
