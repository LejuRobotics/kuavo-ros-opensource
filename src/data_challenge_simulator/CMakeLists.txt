cmake_minimum_required(VERSION 3.0.2)
project(data_challenge_simulator)

find_package(catkin REQUIRED COMPONENTS geometry_msgs kuavo_msgs rospy sensor_msgs std_msgs std_srvs tf tf2_ros)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/model_entry.py
  scripts/rosbag_compat_publisher.py
  scripts/source_bin_latch_v2.py
  utils/topic_repeater.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
