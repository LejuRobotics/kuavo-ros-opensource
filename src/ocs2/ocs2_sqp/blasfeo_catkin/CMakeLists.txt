cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)

find_package(catkin REQUIRED)

include(FetchContent)

# Define directories
set(BLASFEO_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE STRING "Blasfeo install path")
set(BLASFEO_INCLUDE_DIR ${BLASFEO_DEVEL_PREFIX}/include)
set(BLASFEO_LIB_DIR ${BLASFEO_DEVEL_PREFIX}/lib)
set(BLASFEO_DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}/download)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})
file(MAKE_DIRECTORY ${BLASFEO_LIB_DIR})
file(MAKE_DIRECTORY ${BLASFEO_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${BLASFEO_BUILD_DIR})

# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
# Select BLASFEO TARGET per host architecture.
# X64_AUTOMATIC selects best ISA at runtime on x86_64 only; on aarch64 we must
# pick an ARMV8A target (or GENERIC) explicitly.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # GENERIC: full symbol set on AGX Orin (Cortex-A78); tuned A57 kernels miss dgetrf_pivot_8_*).
    set(TARGET GENERIC CACHE STRING "Target architecture" FORCE)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
    set(TARGET ARMV7A_ARM_CORTEX_A15 CACHE STRING "Target architecture" FORCE)
else()
    set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
endif()
message(STATUS "[blasfeo_catkin] CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR} -> TARGET=${TARGET}")
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

# Download & build source
# FetchContent_Declare(blasfeoDownload
# 	GIT_REPOSITORY https://github.com/giaf/blasfeo
# 	GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
# 	UPDATE_COMMAND ""
# 	SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
# 	BINARY_DIR ${BLASFEO_BUILD_DIR}
# 	BUILD_COMMAND $(MAKE)
# 	INSTALL_COMMAND "$(MAKE) install"
# )
# FetchContent_MakeAvailable(blasfeoDownload)
FetchContent_Declare(blasfeoDownload
	URL ""
	SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
	BINARY_DIR ${BLASFEO_BUILD_DIR}
	BUILD_COMMAND $(MAKE)
	INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(blasfeoDownload)
# Copy header to where catkin expects them
file(GLOB_RECURSE HEADERS "${BLASFEO_DOWNLOAD_DIR}/include/*.h")
foreach(HEADER_FILE ${HEADERS})
	message(STATUS "FOUND HEADER: " ${HEADER_FILE})
	file(COPY ${HEADER_FILE} DESTINATION ${BLASFEO_INCLUDE_DIR})
endforeach()

# Install the library where catkin expects them
set_target_properties(blasfeo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BLASFEO_LIB_DIR})

# Propagate dependencies
catkin_package(
	INCLUDE_DIRS ${BLASFEO_INCLUDE_DIR}
	LIBRARIES blasfeo
	CFG_EXTRAS blasfeo-extras.cmake
)
