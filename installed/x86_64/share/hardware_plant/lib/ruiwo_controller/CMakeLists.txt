cmake_minimum_required(VERSION 3.12)
project(ruiwoTest)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ruiwo C++ 内嵌 CPython；Focal 4PRO 优先 3.8，避免 CMake 选中 3.9 且缺配套 dev/numpy。
# NumPy 仅 ruiwo_actuator.py 运行时需要，编译 C++ 不包含 numpy 头。
if(NOT Python3_EXECUTABLE AND EXISTS "/usr/bin/python3.8")
  set(Python3_EXECUTABLE "/usr/bin/python3.8" CACHE FILEPATH "Ruiwo embedded Python" FORCE)
endif()
find_package(Python3 3.8 COMPONENTS Interpreter Development REQUIRED)
if(NOT Python3_FOUND)
  message(FATAL_ERROR "Python3 >= 3.8 not found for ruiwo_controller")
endif()
if(NOT Python3_Development_FOUND)
  message(FATAL_ERROR
    "Python3 Development (headers/libpython) not found for: ${Python3_EXECUTABLE}\n"
    "Ubuntu 20.04 (4PRO): sudo apt install python3.8-dev python3-numpy\n"
    "Other: install python3-dev matching that interpreter, or pass -DPython3_EXECUTABLE=...")
endif()
message(STATUS "[ruiwo_controller] Python: ${Python3_EXECUTABLE} (${Python3_VERSION})")

find_package(yaml-cpp REQUIRED)

add_library(ruiwo_actuatorLib SHARED ruiwo_actuator.cpp)
target_include_directories(ruiwo_actuatorLib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
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