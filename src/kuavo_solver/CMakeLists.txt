cmake_minimum_required(VERSION 3.5)
project(kuavo_solver)

# ==============================================================================
# Catkin Configuration (if building with catkin)
# ==============================================================================
if(CATKIN_DEVEL_PREFIX OR DEFINED catkin_EXTENSIONS)
    find_package(catkin REQUIRED)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================
# 查找 Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 is required but not found!")
endif()

# ==============================================================================
# Build Settings
# ==============================================================================
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add debug information
add_compile_options(-g)

# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CATKIN_DEVEL_PREFIX OR DEFINED catkin_EXTENSIONS)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
    )
endif()

# ==============================================================================
# Include Directories
# ==============================================================================
if(CATKIN_DEVEL_PREFIX OR DEFINED catkin_EXTENSIONS)
	include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)


# ==============================================================================
# Library Target
# ==============================================================================
# 创建动态链接库
add_library(kuavo_solver SHARED 
  src/ankle_solver.cpp
  src/roban_ankle_solver.cpp
)

# 链接依赖库
target_link_libraries(kuavo_solver
  ${EIGEN3_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)

if(CATKIN_DEVEL_PREFIX OR DEFINED catkin_EXTENSIONS)
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

# 编译完成后设置库文件权限
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND chmod 755 $<TARGET_FILE:${PROJECT_NAME}>
)

# ==============================================================================
# Installation
# ==============================================================================
if(CATKIN_DEVEL_PREFIX OR DEFINED catkin_EXTENSIONS)
    # Catkin installation
    install(TARGETS kuavo_solver
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    )

    install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      PATTERN "*" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    )
else()
    # Standard CMake installation
    include(GNUInstallDirs)

    install(TARGETS kuavo_solver
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