cmake_minimum_required(VERSION 3.10)
project(motor_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set dependencies directory
set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../installed)

# Include directories
include_directories(
    ${DEPS_DIR}/include/ruiwo_controller_cxx/include
    ${DEPS_DIR}/include/hardware_plant
)

# Link directories
link_directories(${DEPS_DIR}/lib)

# Set library list
set(LIB_LIST
    ruiwo_actuatorCXXLib
    bmapi64
    yaml-cpp
    pthread
)

# Create executable
add_executable(${PROJECT_NAME} motor_control.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${LIB_LIST})

