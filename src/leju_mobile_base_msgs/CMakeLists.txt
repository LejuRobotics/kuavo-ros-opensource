cmake_minimum_required(VERSION 3.0.2)
project(leju_mobile_base_msgs)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  map_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Battery.msg
  BMS.msg
  CostmapUpdates.msg
  Information.msg
  LED.msg
  LiftControl3Feedback.msg
  LiftControl3Goal.msg
  LiftControl3Result.msg
  LiftStatus.msg
  MoveBaseActionGoal.msg
  NavigationModeGoal.msg
  Parameter.msg
  RobotStatus.msg
  StepControlGoal.msg
  TaskDetailStatus.msg
  MoveToTargetOptions.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ControlLaserScan.srv
  ExecTask.srv
  GetGlobalOdometer.srv
  GetLaserScanList.srv
  ResetTemporaryOdometer.srv
  SetMapServer.srv
  StartLocalization.srv
  SyncCostmap.srv
  MoveToTarget.srv
  RelativeBaseMove.srv
  CheckArrived.srv
)

## Generate added messages and services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  map_msgs
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    map_msgs
)
