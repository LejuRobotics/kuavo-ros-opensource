# ==============================================================================
# Tests Configuration
# ==============================================================================
# 测试程序的CMakeLists.txt

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 查找ROS包
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    kuavo_msgs
)

# 电机测试可执行文件
add_executable(hardware_plant_motor_test 
    ${CMAKE_CURRENT_SOURCE_DIR}/motor_test.cpp
)

# 链接库
target_link_libraries(hardware_plant_motor_test 
    PUBLIC 
        hardware_plant
        ${catkin_LIBRARIES}
)

# 包含目录
target_include_directories(hardware_plant_motor_test 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${catkin_INCLUDE_DIRS}
)

# 编译定义
target_compile_definitions(hardware_plant_motor_test
    PRIVATE
        PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

# 安装目标
install(TARGETS 
    hardware_plant_motor_test
    RUNTIME DESTINATION bin
) 


# Find required packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    kuavo_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..  # hardware_plant src directory
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/hipnuc_imu/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/xsens_ros_mti_driver/src
)

# Create the executable
add_executable(kuavo_imu_test
    kuavo_imu_test.cpp
)

# Link libraries
target_link_libraries(kuavo_imu_test
    ${catkin_LIBRARIES}
    hardware_plant
    hipnuc_imu_receiver
    xsens_mti_driver
    pthread
)

# Add compile definitions
target_compile_definitions(kuavo_imu_test
    PRIVATE
        PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../"
)

# Hardware plant wheel arm test executable 
add_executable(hardware_plant_wheel_arm_test
    wheel_arm_ec_test.cpp
)

# Add dependencies to ensure message headers are generated
add_dependencies(kuavo_imu_test hardware_plant_wheel_arm_test
    kuavo_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

# Link libraries
target_link_libraries(hardware_plant_wheel_arm_test
    ${catkin_LIBRARIES}
    hardware_plant
)

# Add compile definitions
target_compile_definitions(hardware_plant_wheel_arm_test
    PRIVATE
        PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../.."
)

# Install
install(TARGETS kuavo_imu_test hardware_plant_wheel_arm_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 