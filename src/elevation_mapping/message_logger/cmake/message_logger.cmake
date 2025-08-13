# Ensure compatibility with CMake 3.28+ by setting policy CMP0127 to NEW
if(POLICY CMP0127)
  cmake_policy(SET CMP0127 NEW)
endif()

# Option to use std::cout instead of ros.
include(CMakeDependentOption)
cmake_dependent_option(MELO_USE_COUT "Use std::cout" OFF "DEFINED ENV{ROS_DISTRO}" ON)
if(MELO_USE_COUT)
  message(STATUS "Building message_logger using std::cout.")
else()
  message(STATUS "Building message_logger using ros.")
endif()

# Option to use sentry for automatic error reporting.
# Suggest the usage of sentry if its dependencies are found.
find_package(anymal_sentry_native QUIET)
find_package(sentry_native QUIET)
if(anymal_sentry_native_FOUND AND sentry_native_FOUND)
  set(MELO_USE_SENTRY_DEFAULT ON)
else()
  set(MELO_USE_SENTRY_DEFAULT OFF)
endif()
option(MELO_USE_SENTRY "Enable Sentry for automatic error reporting" ${MELO_USE_SENTRY_DEFAULT})
if(MELO_USE_SENTRY)
  message(STATUS "Building message_logger with Sentry.")
  add_compile_definitions(MELO_USE_SENTRY)
else()
  message(STATUS "Building message_logger without Sentry.")
endif()

# Option to enable additional function prints.
if(DEFINED MELO_FUNCTION_PRINTS)
  add_definitions(-DMELO_FUNCTION_PRINTS)
endif()
