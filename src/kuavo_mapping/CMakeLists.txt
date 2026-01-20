cmake_minimum_required(VERSION 3.0.2)
project(kuavo_mapping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nav_msgs
)

# 添加消息文件
add_message_files(
  DIRECTORY msg
  FILES
  TaskPoint.msg
)

# 添加服务文件
add_service_files(
  DIRECTORY srv
  FILES
  GetAllMaps.srv
  TaskPointOperation.srv
  NavigateToTaskPoint.srv
  SetInitialPose.srv
  GetCurrentMap.srv
  LoadMap.srv
  SaveMap.srv
  StopMapping.srv
  StartMapping.srv
  RenameMap.srv
  EditMap.srv
  InitialPoseWithTaskPoint.srv
  DeleteMap.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs nav_msgs
)