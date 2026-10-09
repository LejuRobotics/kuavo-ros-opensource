cmake_minimum_required(VERSION 3.0.2)
project(kuavo_assets)

find_package(catkin REQUIRED COMPONENTS
roscpp
roslib
rospy
std_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED src/package_path.cc) 
target_include_directories(${PROJECT_NAME} PUBLIC include  ${catkin_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_custom_target(update_mass_config ALL
    COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/scripts/update_mass.sh
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/update_mass.sh ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_ecmaster_type.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Updating mass configuration"
)

# Install deps
add_custom_target(install_deps ALL
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/install_deps_packages.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Install deps"
)

###########
## Build ##
###########

# # Resolve for the package path at compile time.
# configure_file (
#   "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
#   "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
# )

# #############
# ## Install ##
# #############

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY resources
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
