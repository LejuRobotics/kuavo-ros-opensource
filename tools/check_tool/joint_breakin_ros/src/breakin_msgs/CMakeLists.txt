cmake_minimum_required(VERSION 3.0.2)
project(breakin_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# 如果需要自定义消息，取消下面的注释
# add_message_files(
#   FILES
#   BreakinStatus.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)