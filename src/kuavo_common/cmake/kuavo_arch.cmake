# kuavo_arch.cmake - architecture detection for Kuavo ROS Control (x86_64 / aarch64)
# Include from package CMakeLists.txt:
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../../kuavo_common/cmake/kuavo_arch.cmake)
# or use find_path / catkin path when available.

if(DEFINED KUAVO_ARCH_INCLUDED)
  return()
endif()
set(KUAVO_ARCH_INCLUDED TRUE)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(KUAVO_ARCH "aarch64")
  set(KUAVO_BUILD_FOR_AARCH64 TRUE)
  set(KUAVO_LIB_DIR_TRIPLET "aarch64-linux-gnu")
  set(KUAVO_UNITREE_SDK_ARCH_SUBDIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
  set(KUAVO_ARCH "armv7")
  set(KUAVO_BUILD_FOR_AARCH64 FALSE)
  set(KUAVO_LIB_DIR_TRIPLET "arm-linux-gnueabihf")
  set(KUAVO_UNITREE_SDK_ARCH_SUBDIR "aarch64")
else()
  set(KUAVO_ARCH "x86_64")
  set(KUAVO_BUILD_FOR_AARCH64 FALSE)
  set(KUAVO_LIB_DIR_TRIPLET "x86_64-linux-gnu")
  set(KUAVO_UNITREE_SDK_ARCH_SUBDIR "x86_64")
endif()

# Drake / OpenVINO are optional on all platforms; callers set KUAVO_HAS_DRAKE after find_package.
if(NOT DEFINED KUAVO_HAS_DRAKE)
  set(KUAVO_HAS_DRAKE FALSE)
endif()
if(NOT DEFINED KUAVO_HAS_OPENVINO)
  set(KUAVO_HAS_OPENVINO FALSE)
endif()

message(STATUS "[kuavo_arch] KUAVO_ARCH=${KUAVO_ARCH} KUAVO_BUILD_FOR_AARCH64=${KUAVO_BUILD_FOR_AARCH64}")
