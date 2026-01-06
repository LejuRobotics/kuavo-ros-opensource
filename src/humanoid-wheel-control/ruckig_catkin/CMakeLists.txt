cmake_minimum_required(VERSION 3.15)
project(ruckig_catkin)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

include(FetchContent)

# Define directories
set(RUCKIG_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE STRING "RUCKIG install path")
set(RUCKIG_INCLUDE_DIR ${RUCKIG_DEVEL_PREFIX}/include)
set(RUCKIG_LIB_DIR ${RUCKIG_DEVEL_PREFIX}/lib)
set(RUCKIG_DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}/download)
set(RUCKIG_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${RUCKIG_INCLUDE_DIR})
file(MAKE_DIRECTORY ${RUCKIG_LIB_DIR})
file(MAKE_DIRECTORY ${RUCKIG_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${RUCKIG_BUILD_DIR})

# HPIPM Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(RUCKIG_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# Download & build source
# FetchContent_Declare(qpoasesDownload
#         GIT_REPOSITORY https://github.com/coin-or/qpOASES
#         GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f
#         UPDATE_COMMAND ""
#         SOURCE_DIR ${QPOASES_DOWNLOAD_DIR}
#         BINARY_DIR ${QPOASES_BUILD_DIR}
#         BUILD_COMMAND $(MAKE)
#         INSTALL_COMMAND "$(MAKE) install"
#         )
# FetchContent_MakeAvailable(qpoasesDownload)
set(RUCKIG_LOCAL_SOURCE_DIR ${RUCKIG_DOWNLOAD_DIR})

# 使用本地源代码目录进行构建
FetchContent_Declare(ruckigDownload
        URL ""
        SOURCE_DIR ${RUCKIG_LOCAL_SOURCE_DIR}  # 指定本地源代码目录路径
        BINARY_DIR ${RUCKIG_BUILD_DIR}
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND "$(MAKE) install"
        )
FetchContent_MakeAvailable(ruckigDownload)

file(GLOB_RECURSE HEADERS "${RUCKIG_DOWNLOAD_DIR}/include/ruckig/*")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${RUCKIG_INCLUDE_DIR}/ruckig)
endforeach ()

set_target_properties(ruckig PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${RUCKIG_LIB_DIR})

catkin_package(
        INCLUDE_DIRS ${RUCKIG_INCLUDE_DIR}
        LIBRARIES ruckig
)
