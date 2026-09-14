cmake_minimum_required(VERSION 3.0.2)
project(kuavo_mocap_gmr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  kuavo_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs kuavo_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/gmr_streaming_node.py
  scripts/gmr_sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install shell scripts (not as Python)
install(PROGRAMS
  scripts/run_with_venv.sh
  scripts/setup_env.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
