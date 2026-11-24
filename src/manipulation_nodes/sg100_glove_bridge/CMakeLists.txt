cmake_minimum_required(VERSION 3.0.2)
project(sg100_glove_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  kuavo_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/calibrate_glove_gestures.py
  scripts/glove_gesture_node.py
  scripts/sg100_glove_node.py
  scripts/sg100_windows_udp_node.py
  scripts/sg100_mujoco_viewer_float.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/gesture_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# sg100_runtime/ 为内置 Linux SG100 SDK (Python 3.12 独立环境运行,
# 详见 README), 整目录随包安装。
install(DIRECTORY launch sg100_runtime docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".venv_sg100_native" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install the directory guide, not operator recordings or active profiles.
install(FILES config/glove_gesture_calibration/README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/glove_gesture_calibration
)
