# ==============================================================================
# Project Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.10)
project(kuavo_common)

# ==============================================================================
# Catkin Configuration (if building with catkin)
# ==============================================================================
if(CATKIN_DEVEL_PREFIX)
    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        DEPENDS EIGEN3
    )
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
# Find Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 is required but not found!")
endif()

# ==============================================================================
# Build Settings
# ==============================================================================
# Enforce C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add debug information
add_compile_options(-g)

# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Include Directories
# ==============================================================================
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

if(CATKIN_DEVEL_PREFIX)
    include_directories(${catkin_INCLUDE_DIRS})
endif()

# ==============================================================================
# Source Files
# ==============================================================================
set(SRC_OBJECTS
    src/common/common.cpp
    src/kuavo_common.cpp
    src/common/kuavo_settings.cpp
    src/common/utils.cpp
    include/${PROJECT_NAME}/common/json.hpp
    include/${PROJECT_NAME}/common/json_config_reader.hpp
)

# ==============================================================================
# Library Target
# ==============================================================================
# Create shared library
add_library(${PROJECT_NAME} SHARED ${SRC_OBJECTS})

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
    ${EIGEN3_LIBRARIES}
    stdc++fs
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)

if(CATKIN_DEVEL_PREFIX)
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

# ==============================================================================
# Testing
# ==============================================================================
enable_testing()

# Add test executable
add_executable(${PROJECT_NAME}_test tests/kuavo_common_test.cpp)
target_link_libraries(${PROJECT_NAME}_test
    PRIVATE
        ${PROJECT_NAME}
        gtest
        gtest_main
        pthread
)

# Register test
add_test(
    NAME KuavoCommonTest
    COMMAND ${PROJECT_NAME}_test
)

# ==============================================================================
# Installation
# ==============================================================================
if(CATKIN_DEVEL_PREFIX)
    # Catkin installation
    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
else()
    # Standard CMake installation
    include(GNUInstallDirs)

    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
    )

    # Install export targets
    install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    # Generate and install CMake config files
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
    )

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif()