cmake_minimum_required(VERSION 3.5.1)
project(message_logger)

if("$ENV{ROS_VERSION}" STREQUAL "1")

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Catkin
find_package(catkin REQUIRED)

# Import options and set dependencies accordingly.
include(cmake/message_logger.cmake)
if(NOT MELO_USE_COUT)
  set(CATKIN_PACKAGE_HEADER_DEPENDENCIES ${CATKIN_PACKAGE_HEADER_DEPENDENCIES} roscpp)
endif()
if(MELO_USE_SENTRY)
  set(CATKIN_PACKAGE_HEADER_DEPENDENCIES ${CATKIN_PACKAGE_HEADER_DEPENDENCIES} anymal_sentry_native sentry_native)
endif()

# Configure header file before catkin_package
catkin_destinations()
configure_file(
include/message_logger/log/log_messages_backend_config.hpp.in
${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/log/log_messages_backend_config.hpp @ONLY
)

# Find Catkin
find_package(catkin REQUIRED
COMPONENTS
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

catkin_package(
INCLUDE_DIRS
  include
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
LIBRARIES
  ${PROJECT_NAME}
CATKIN_DEPENDS
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
CFG_EXTRAS
  message_logger.cmake
)

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
src/log/log_sentry.cpp
src/time/Time.cpp
src/time/TimeStd.cpp
)
add_dependencies(${PROJECT_NAME}
${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
include
${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(demo
src/demo.cpp
)
add_dependencies(demo
${PROJECT_NAME}
${catkin_EXPORTED_TARGETS}
)
target_include_directories(demo PRIVATE
include
${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_include_directories(demo SYSTEM PUBLIC
${catkin_INCLUDE_DIRS}
)
target_link_libraries(demo
${catkin_LIBRARIES}
${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN "*.in" EXCLUDE
)
install(TARGETS
  ${PROJECT_NAME}
  demo
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
DIRECTORY
  launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
  test/EmptyTests.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
  add_dependencies(test_${PROJECT_NAME}
  ${PROJECT_NAME}
)
  target_include_directories(test_${PROJECT_NAME} PRIVATE
  include
)
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
  target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
  TARGETS
    ${PROJECT_NAME}
    demo
)
endif(cmake_clang_tools_FOUND)

else() # ROS version 2

###############
## Configure ##
###############


set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Import options and set dependencies accordingly.
include(cmake/message_logger.cmake)

# Ament packages
set(AMENT_PACKAGE_DEPENDENCIES
)

if(NOT MELO_USE_COUT)
  find_package(rclcpp REQUIRED)
  list(APPEND AMENT_PACKAGE_DEPENDENCIES rclcpp)
endif()
if(MELO_USE_SENTRY)
  message(WARNING "Sentry is not supported for ROS2. Setting MELO_USE_SENTRY to false.")
  set(MELO_USE_SENTRY OFF)
  list(APPEND AMENT_PACKAGE_DEPENDENCIES anymal_sentry_native sentry_native)
endif()

# Find dependencies for building this package.
find_package(ament_cmake REQUIRED)
foreach(PKG IN LISTS AMENT_PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

# Configure header file
configure_file(include/message_logger/log/log_messages_backend_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/message_logger/log/log_messages_backend_config.hpp @ONLY)

add_definitions(-fPIC)


###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/log/log_sentry.cpp
  src/time/Time.cpp
  src/time/TimeStd.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)

add_executable(demo
  src/demo.cpp
)
target_include_directories(demo PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(demo
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(demo
  ${PROJECT_NAME}
)


#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.in"
  EXCLUDE
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/include/message_logger/log/log_messages_backend_config.hpp
  DESTINATION include/${PROJECT_NAME}/message_logger/log
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

# Export old-style CMake variables
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/EmptyTests.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

ament_package()

endif()
