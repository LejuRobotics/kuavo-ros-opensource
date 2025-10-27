cmake_minimum_required(VERSION 3.10.2)
project(async_web_server_cpp VERSION 1.0.3 LANGUAGES CXX)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem regex thread)
find_package(OpenSSL REQUIRED)

if(CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS roslib rospy)
    find_package(rostest REQUIRED)
endif()

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Boost
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
    src/http_connection.cpp
    src/http_reply.cpp
    src/http_request.cpp
    src/http_request_handler.cpp
    src/http_request_parser.cpp
    src/http_server.cpp
    src/websocket_connection.cpp
    src/websocket_message.cpp
    src/websocket_request_handler.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Boost::boost Boost::filesystem Boost::thread
    PRIVATE
        Boost::regex
        OpenSSL::Crypto
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        $<$<BOOL:${WIN32}>:_WIN32_WINNT=0x0A00>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

###########
# Testing #
###########

if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
