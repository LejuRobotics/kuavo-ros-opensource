cmake_minimum_required(VERSION 3.0.2)
project(breakin_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  breakin_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs breakin_msgs
)

# 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/breakin_controller_simple.py
  scripts/arm_breakin_standalone.py
  scripts/leg_breakin_standalone.py
  scripts/breakin_main_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件（如果launch目录存在）
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )