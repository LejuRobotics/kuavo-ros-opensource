cmake_minimum_required(VERSION 3.15)
project(EcDemo VERSION 0.1.0)

set(Python_ADDITIONAL_VERSIONS 3.8)
find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
message(STATUS "Python3_VERSION: ${Python3_VERSION}")

# 头文件路径
set(EcTool_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/EcSdk/INC 
    ${CMAKE_CURRENT_SOURCE_DIR}/EcSdk/INC/Linux
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/ 
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/EcMasterDemo/
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Linux/
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/
    CACHE PATH "My include directories")

# 源文件
set(EcTool_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/EcThread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/EcTimer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcDemoParms.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcLogging.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcNotification.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSdoServices.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSelectLinkLayer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Common/EcSlaveInfo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/EcMotor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/EcSdo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/util.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/EcLogFile.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/UserAppWorkpd.cpp
)

# 第三方库
set(THIRDPARTY_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/EcSdk/LIB/Linux/x64/libAtemRasSrv.a
    ${CMAKE_CURRENT_SOURCE_DIR}/EcSdk/LIB/Linux/x64/libEcMaster.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/check_tool/roban2_joint_breakin/roban2_leg_breakin/lib/libemllI8254x.so  # 动态库，运行时需 RPATH
    pthread
    dl
)

# ec_master_main 静态库
add_library(ec_master_main STATIC ${EcTool_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/Linux/EcDemoMain.cpp)
set_target_properties(ec_master_main PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_main PUBLIC ${EcTool_INCLUDE_DIRS})  
target_link_libraries(ec_master_main ${THIRDPARTY_LIBS})

# ec_master_tools 静态库
add_library(ec_master_tools STATIC ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp )
set_target_properties(ec_master_tools PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_tools PUBLIC ${EcTool_INCLUDE_DIRS})

# leg_breakin Python 模块
add_library(leg_breakin MODULE 
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/leg_breakin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Sharelib/User/EcSdo.cpp
)
target_include_directories(leg_breakin PUBLIC 
    ${EcTool_INCLUDE_DIRS} 
    ${Python3_INCLUDE_DIRS} 
    ${pybind11_INCLUDE_DIRS}
)
target_link_libraries(leg_breakin PRIVATE 
    ec_master_main 
    ec_master_tools 
    ${Python3_LIBRARIES}
    ${THIRDPARTY_LIBS}
    pybind11::module
)

# ✅ 设置 RPATH 以便 leg_breakin.so 在运行时能找到 ../lib 中的 .so 文件
set_target_properties(leg_breakin PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    BUILD_RPATH "$ORIGIN/../../../../tools/check_tool/roban2_joint_breakin/roban2_leg_breakin/lib"
    INSTALL_RPATH "$ORIGIN/../../../../tools/check_tool/roban2_joint_breakin/roban2_leg_breakin/lib"
)
