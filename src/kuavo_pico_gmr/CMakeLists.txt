cmake_minimum_required(VERSION 3.0.2)
project(kuavo_pico_gmr)

## 找到 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf
  kuavo_msgs
)

## Python 设置
catkin_python_setup()

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    geometry_msgs 
    tf
    kuavo_msgs
)

###########
## Build ##
###########

## 指定 Python 包的位置
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/pico_realtime_to_robot.py
  scripts/pico_dexhand_control_node.py
  scripts/extract_pico_data.py
  scripts/gmr_ros_node_dev.py
  scripts/vis_robot_motion.py
  scripts/vis_robot_motion_dataset.py
  scripts/vis_robot_urdf.py
  scripts/visualize_kuavo.py
  scripts/visualize_pico_bones_matplotlib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 Wrapper Shell 脚本
install(PROGRAMS
  scripts/pico_comm_minimal_wrapper.sh
  scripts/pico_realtime_to_robot_wrapper.sh
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

