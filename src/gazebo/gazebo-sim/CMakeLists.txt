cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  humanoid_controllers  
  roscpp
  gazebo_ros
  gazebo_ros_control
  controller_manager
  # joint_state_controller
  # robot_state_publisher
)

find_package(gazebo REQUIRED)

# 使用pkg-config查找uuid库
find_package(PkgConfig REQUIRED)
pkg_check_modules(UUID REQUIRED uuid)

# 创建一个导入的目标来满足 UUID::UUID 的依赖
add_library(UUID::UUID INTERFACE IMPORTED)
set_target_properties(UUID::UUID PROPERTIES
    INTERFACE_LINK_LIBRARIES "${UUID_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${UUID_INCLUDE_DIRS}"
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    ${PROJECT_NAME}_shm_interface

  CATKIN_DEPENDS
    humanoid_controllers
    roscpp
    gazebo_ros
    gazebo_ros_control
    controller_manager
    # joint_state_controller
    # robot_state_publisher
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# 添加库搜索路径
link_directories(
  ${GAZEBO_LIBRARY_DIRS}

)

# 添加Gazebo插件
add_library(${PROJECT_NAME}_shm_interface
  src/gazebo_shm_interface.cpp
)


target_link_libraries(${PROJECT_NAME}_shm_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  UUID::UUID
)

# add_dependencies(${PROJECT_NAME}_shm_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装Python脚本
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

# 安装模型文件
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config"
)

# 添加测试程序
add_executable(shm_test src/shm_test.cpp)

# 确保在链接之前已经构建了依赖库
add_dependencies(shm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(shm_test
  rt
  ${catkin_LIBRARIES}
)

