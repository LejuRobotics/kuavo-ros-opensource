cmake_minimum_required(VERSION 3.12)
project(ruiwoTest)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Python 库
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(PythonInterp REQUIRED 3.8)
if(Python3_FOUND)
    message(STATUS "Found Python: ${Python3_EXECUTABLE}")
    message(STATUS "Python version: ${Python3_VERSION}")
    
else()
    message(FATAL_ERROR "Python not found")
endif()

find_package(yaml-cpp REQUIRED)

add_library(ruiwo_actuatorLib SHARED ruiwo_actuator.cpp)
target_include_directories(ruiwo_actuatorLib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/ruiwo_controller>
  $<INSTALL_INTERFACE:${Python3_INCLUDE_DIRS}>
)
target_link_libraries(ruiwo_actuatorLib PUBLIC Python3::Python stdc++fs -pthread yaml-cpp)

# 添加可执行文件
add_executable(ruiwo_test ruiwo_test.cpp)
target_include_directories(ruiwo_test PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(ruiwo_test PUBLIC ruiwo_actuatorLib)

install(TARGETS ruiwo_actuatorLib
    # EXPORT hardware_plantTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

include(GNUInstallDirs)

get_directory_property(PARENT_DIR PARENT_DIRECTORY)
get_filename_component(PARENT_PROJECT_NAME ${PARENT_DIR} NAME)

install(DIRECTORY ./
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PARENT_PROJECT_NAME}/lib/ruiwo_controller
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                   GROUP_EXECUTE GROUP_READ
                   WORLD_EXECUTE WORLD_READ
)

file(GLOB RUIWO_CONTROLLER_HEADERS
    "${PROJECT_SOURCE_DIR}/*.h"
)

install(FILES ${RUIWO_CONTROLLER_HEADERS}
        DESTINATION include/ruiwo_controller)