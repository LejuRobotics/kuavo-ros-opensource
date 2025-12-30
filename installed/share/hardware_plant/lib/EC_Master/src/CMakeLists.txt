cmake_minimum_required(VERSION 3.0.0)
project(EcDemo VERSION 0.1.0)

set(EcDemo_INCLUDE_DIRS
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Common>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/SDK/INC>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/SDK/INC/Linux>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Sharelib/Common>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Sharelib/EcMasterDemo>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Sharelib/Linux>
  $<INSTALL_INTERFACE:include/EC_Master/Common>
  $<INSTALL_INTERFACE:include/EC_Master/SDK/INC>
  $<INSTALL_INTERFACE:include/EC_Master/SDK/INC/Linux>
  $<INSTALL_INTERFACE:include/EC_Master/Sharelib/Common>
  $<INSTALL_INTERFACE:include/EC_Master/Sharelib/EcMasterDemo>
  $<INSTALL_INTERFACE:include/EC_Master/Sharelib/Linux>
)

set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/EcThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/EcTimer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcDemoParms.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcLogging.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcNotification.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSdoServices.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSelectLinkLayer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSlaveInfo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp
)

set(LINK_LIB
${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/
${CMAKE_CURRENT_SOURCE_DIR}/../lib
)


## EcDemoMain
add_executable(${PROJECT_NAME} ${SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/Examples/Linux/EcDemoMain.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${EcDemo_INCLUDE_DIRS})  
target_link_libraries(${PROJECT_NAME} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libAtemRasSrv.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libEcMaster.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/libemllI8254x.so>
  $<INSTALL_INTERFACE:libAtemRasSrv.a>
  $<INSTALL_INTERFACE:libEcMaster.a>
  $<INSTALL_INTERFACE:libemllI8254x.so>
  -pthread
  -ldl
  # lcm
  # lcm-types-cpp
)

## ec_master_main
add_library(ec_master_main STATIC ${SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Linux/EcDemoMain.cpp)
target_include_directories(ec_master_main PUBLIC ${EcDemo_INCLUDE_DIRS})  
target_link_libraries(ec_master_main
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libAtemRasSrv.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libEcMaster.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/libemllI8254x.so>
  $<INSTALL_INTERFACE:libAtemRasSrv.a>
  $<INSTALL_INTERFACE:libEcMaster.a>
  $<INSTALL_INTERFACE:libemllI8254x.so>
  -pthread
  -ldl
  # lcm
  # lcm-types-cpp
)

## ec_app
add_library(ec_app STATIC ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp )
target_include_directories(ec_app PUBLIC ${EcDemo_INCLUDE_DIRS})  
target_link_libraries(ec_app 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libAtemRasSrv.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libEcMaster.a>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/libemllI8254x.so>
  $<INSTALL_INTERFACE:libAtemRasSrv.a>
  $<INSTALL_INTERFACE:libEcMaster.a>
  $<INSTALL_INTERFACE:libemllI8254x.so>
  -pthread
  -ldl
  # lcm
  # lcm-types-cpp
)

# Install libraries
install(TARGETS ec_master_main ec_app
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install dependent library
install(FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/libemllI8254x.so
  ${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libAtemRasSrv.a
  ${CMAKE_CURRENT_SOURCE_DIR}/SDK/LIB/Linux/x64/libEcMaster.a
  DESTINATION lib
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/Common
  DESTINATION include/EC_Master
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/SDK/INC
  ${PROJECT_SOURCE_DIR}/SDK/INC/Linux
  DESTINATION include/EC_Master/SDK
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/Sharelib/Common
  ${PROJECT_SOURCE_DIR}/Sharelib/EcMasterDemo
  ${PROJECT_SOURCE_DIR}/Sharelib/Linux
  DESTINATION include/EC_Master/Sharelib
)

# set_target_properties(ec_master_main PROPERTIES IMPORTED_LOCATION ./EC_Master/src/libec_master_main.so)
# set_target_properties(ec_app PROPERTIES IMPORTED_LOCATION ./EC_Master/src/libec_app.so)

## 宏开关
# add_definitions(-DERROR_FIX) ## 快速报错修复开关