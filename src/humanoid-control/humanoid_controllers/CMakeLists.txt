cmake_minimum_required(VERSION 3.10)
project(humanoid_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_interface
        humanoid_common
        humanoid_interface
        humanoid_interface_ros
        humanoid_wbc
        humanoid_estimation
        ocs2_self_collision_visualization
        # ocs2_biped_robot_ros
        angles
        kuavo_msgs
        humanoid_interface_drake
        kuavo_common
        )
find_package(drake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(kuavo_estimation)
if(kuavo_estimation_FOUND)
    add_definitions(-DKUAVO_CONTROL_LIB_FOUND)
    message(STATUS "Found kuavo_estimation")
else()
    message(WARNING "Could not find kuavo_estimation, disabling kuavo_control library, if you need it, just update submodules")
endif()


catkin_package(
        INCLUDE_DIRS
                include
        LIBRARIES
                ${PROJECT_NAME}
                humanoid_interface_shm

        CATKIN_DEPENDS
                roscpp
                controller_interface
                humanoid_common
                humanoid_interface
                humanoid_interface_ros
                # mujoco_sim
                humanoid_wbc
                humanoid_estimation
                kuavo_estimation
                ocs2_self_collision_visualization
                
                # ocs2_biped_robot_ros
                angles
                kuavo_common
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)
if (kuavo_estimation_FOUND)
message(STATUS "add kuavo_estimation include")
include_directories(${kuavo_estimation_INCLUDE_DIRS})
endif()

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# 添加共享内存管理库
add_library(humanoid_interface_shm SHARED
  src/shm_manager.cpp
)

target_link_libraries(humanoid_interface_shm PUBLIC
  rt  # 添加rt库用于POSIX共享内存
)

add_dependencies(humanoid_interface_shm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 确保其他包能找到这个库
target_include_directories(humanoid_interface_shm PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


# 设置输出目录
set_target_properties(humanoid_interface_shm
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib
)

# 安装规则
install(TARGETS humanoid_interface_shm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/humanoidController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        humanoid_interface_shm
        drake::drake 
        )
if (kuavo_estimation_FOUND)
message(STATUS "add kuavo_estimation link")
target_link_libraries(${PROJECT_NAME}
        ${kuavo_estimation_LIBRARIES}
        )
endif()

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
add_executable(humanoid_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
add_dependencies(humanoid_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(humanoid_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(humanoid_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

# Cheat Controller Node
add_executable(cheat_controller_node
        src/CheatControllerNode.cpp
        )
add_dependencies(cheat_controller_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cheat_controller_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(cheat_controller_node PRIVATE ${OCS2_CXX_FLAGS})

# Controller Nodelet

add_library(nodelet_controller
  src/controllerNodelet.cpp
)
set_target_properties(nodelet_controller PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)

add_dependencies(nodelet_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_controller
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

install(TARGETS nodelet_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# playback nodelet
add_library(nodelet_player
  src/playBackNodelet.cpp
)
set_target_properties(nodelet_player PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)

add_dependencies(nodelet_player ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_player
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        drake::drake 
)
# if (kuavo_estimation_FOUND)
# message(STATUS "add kuavo_estimation link to nodelet_player")
# target_link_libraries(nodelet_player
#         ${kuavo_estimation_LIBRARIES}
#         )
# endif()
install(TARGETS nodelet_player
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Normal Controller Node
add_executable(normal_controller_node
        src/NormalControllerNode.cpp
        )
add_dependencies(normal_controller_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(normal_controller_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(normal_controller_node PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} humanoid_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS humanoid_target_trajectories_publisher cheat_controller_node normal_controller_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES humanoid_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
