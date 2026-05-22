cmake_minimum_required(VERSION 3.0.2)
project(kuavo_camera_calibration)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  demos/kuavo_head_demo/head_table_publisher.py
  demos/kuavo_right_wrist/right_wrist_table_publisher.py
  demos/kuavo_left_wrist/left_wrist_table_publisher.py
  demos/kuavo_both_arms/both_arms_table_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY demos/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/demos
  FILES_MATCHING PATTERN "*.yaml"
)

install(FILES biped_v3_arm.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
