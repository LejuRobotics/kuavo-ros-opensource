cmake_minimum_required(VERSION 3.0.2)
project(motion_capture_ik)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
find_package(drake CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# catkin_package(DEPENDS Eigen3) expects Eigen3_INCLUDE_DIRS/Eigen3_LIBRARIES.
# FindEigen3.cmake typically defines EIGEN3_INCLUDE_DIRS instead (note the case).
if(DEFINED EIGEN3_INCLUDE_DIRS AND NOT DEFINED Eigen3_INCLUDE_DIRS)
  set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIRS}")
endif()
if(TARGET Eigen3::Eigen AND NOT DEFINED Eigen3_LIBRARIES)
  set(Eigen3_LIBRARIES Eigen3::Eigen)
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  rosbag
  rosbag_storage
  rospy
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  kuavo_msgs
  noitom_hi5_hand_udp_python
  leju_utils
  humanoid_wheel_interface
  ocs2_core
)
find_package(BZip2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

# use kuavo_msgs instead

# add_message_files(
  # FILES
  # robotArmQVVD.msg
  # ikSolveError.msg
  # handPose.msg
  # armHandPose.msg
  # twoArmHandPose.msg
  # headBodyPose.msg
  # ikSolveParam.msg
  # twoArmHandPoseCmd.msg
  # robotHandPosition.msg
  # headBodyPose.msg
# )
# add_service_files(
#   DIRECTORY srv
#   FILES

  # changeArmCtrlMode.srv
  # twoArmHandPoseCmdSrv.srv
  # changeArmCtrlModeKuavo.srv
  # fkSrv.srv
# )

# generate_messages already handled by kuavo_msgs

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES motion_capture_ik_lib
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs kuavo_msgs noitom_hi5_hand_udp_python message_runtime leju_utils ocs2_core humanoid_wheel_interface
  DEPENDS drake Eigen3
)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# 定义变量
set(PLANT_IK_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/plantIK.cc")

# 检查 src/plantIK.cc 是否存在
if(EXISTS "${PLANT_IK_FILE}")
  message(STATUS "Found plantIK.cc")
    add_library(plantIK SHARED src/plantIK.cc include/plantIK.h)
    target_link_libraries(plantIK drake::drake)

    # 指定输出目录
    set_target_properties(plantIK PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
    )
else()
    # 当 src/plantIK.cc 不存在时，加载动态库
    find_library(PLANTIK_LIBRARY NAMES plantIK PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
    if(NOT PLANTIK_LIBRARY)
        message(FATAL_ERROR "Could not find the plantIK library in lib/")
    endif()
    message(STATUS "Using existing library: ${PLANTIK_LIBRARY}")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${humanoid_wheel_interface_INCLUDE_DIRS}
)

add_subdirectory(test)

add_executable(arms_ik_node src/arms_ik_node.cpp)
if(EXISTS "${PLANT_IK_FILE}")
  target_link_libraries(arms_ik_node plantIK ${catkin_LIBRARIES} drake::drake)
else()
  target_link_libraries(arms_ik_node ${PLANTIK_LIBRARY} ${catkin_LIBRARIES} drake::drake)
endif()
target_include_directories(arms_ik_node PUBLIC
  ${PROJECT_BINARY_DIR}/include
)

#############
## Install ##
#############
if(EXISTS "${PLANT_IK_FILE}")
install(TARGETS plantIK
    LIBRARY DESTINATION lib
)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# topic_logger
add_executable(topic_logger src/topic_logger.cpp)
target_include_directories(topic_logger PRIVATE
  ${PROJECT_BINARY_DIR}/include
)
target_link_libraries(topic_logger
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(topic_logger PRIVATE ${FLAGS})

# ik_ros_uni_cpp_node
add_executable(ik_ros_uni_cpp_node 
  src/ik_ros_uni_cpp_node.cpp 
  src/ArmControlBaseROS.cpp
  src/Quest3IkIncrementalROS.cpp 
  src/IncrementalControlModule.cpp
  src/KeyFramesVisualizer.cpp
  src/BaseIKSolver.cpp
  src/TwoStageTorsoIKRec.cpp
  src/OneStageIKEndEffector.cpp
  src/Quest3ArmInfoTransformer.cpp
  src/ArmLengthMeasurement.cpp
  src/JoyStickHandler.cpp
  src/Quest3Debugger.cpp
  src/HandSmoother.cpp
)
target_include_directories(ik_ros_uni_cpp_node PRIVATE
  ${PROJECT_BINARY_DIR}/include
)
target_link_libraries(ik_ros_uni_cpp_node ${catkin_LIBRARIES} drake::drake)

# wheel_ik_ros_uni_cpp_node
add_executable(wheel_ik_ros_uni_cpp_node 
  src/wheel_ik_ros_uni_cpp_node.cpp 
  src/WheelArmControlBaseROS.cpp
  src/WheelQuest3IkIncrementalROS.cpp 
  src/WheelQuest3IkIncrementalTools.cpp
  src/WheelIncrementalControlModule.cpp
  src/WheelKeyFramesVisualizer.cpp
  src/BaseIKSolver.cpp
  src/WheelOneStageIKEndEffector.cpp
  src/Quest3ArmInfoTransformer.cpp
  src/ArmLengthMeasurement.cpp
  src/WheelJoyStickHandler.cpp
  src/WheelHandSmoother.cpp
)
add_dependencies(wheel_ik_ros_uni_cpp_node ${catkin_EXPORTED_TARGETS})
target_include_directories(wheel_ik_ros_uni_cpp_node PRIVATE
  ${PROJECT_BINARY_DIR}/include
  ${humanoid_wheel_interface_INCLUDE_DIRS}
)
target_link_libraries(wheel_ik_ros_uni_cpp_node ${catkin_LIBRARIES} ${humanoid_wheel_interface_LIBRARIES} drake::drake)
target_compile_features(wheel_ik_ros_uni_cpp_node PUBLIC cxx_std_17)