cmake_minimum_required(VERSION 3.0.2)
project(craic_task_template)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/scene1_patrol.py
  scripts/scene2_sorting.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
