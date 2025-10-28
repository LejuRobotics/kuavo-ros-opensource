cmake_minimum_required(VERSION 3.15)
project(ec_master_tools VERSION 0.1.0)

# 设置第三方库路径
set(EC_TOOLS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party_libs/ec-tools")

# 查找 Python 和 pybind11
set(Python_ADDITIONAL_VERSIONS 3.8)
find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# 头文件路径 - 指向third_party_libs
set(EcTool_INCLUDE_DIRS
    ${EC_TOOLS_DIR}/Common
    ${EC_TOOLS_DIR}/EcSdk/INC 
    ${EC_TOOLS_DIR}/EcSdk/INC/Linux
    ${EC_TOOLS_DIR}/Sharelib/Common/ 
    ${EC_TOOLS_DIR}/Sharelib/EcMasterDemo/
    ${EC_TOOLS_DIR}/Sharelib/Linux/
    ${EC_TOOLS_DIR}/Sharelib/User/
    CACHE PATH "EC tools include directories")

# 源文件 - 指向third_party_libs
set(EcTool_SOURCES
    ${EC_TOOLS_DIR}/Common/EcThread.cpp
    ${EC_TOOLS_DIR}/Common/EcTimer.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcDemoParms.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcLogging.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcNotification.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcSdoServices.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcSelectLinkLayer.cpp
    ${EC_TOOLS_DIR}/Sharelib/Common/EcSlaveInfo.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/EcMotor.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/EcSdo.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/util.cpp
    ${EC_TOOLS_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/EcLogFile.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/UserAppWorkpd.cpp
)

# 第三方库 - 指向third_party_libs
set(THIRDPARTY_LIBS
    ${EC_TOOLS_DIR}/EcSdk/LIB/Linux/x64/libAtemRasSrv.a
    ${EC_TOOLS_DIR}/EcSdk/LIB/Linux/x64/libEcMaster.a
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libemllI8254x.so  # 动态库，运行时需 RPATH
    pthread
    dl
)

# ec_master_main 静态库
add_library(ec_master_main STATIC ${EcTool_SOURCES} ${EC_TOOLS_DIR}/Sharelib/Linux/EcDemoMain.cpp)
set_target_properties(ec_master_main PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_main PUBLIC ${EcTool_INCLUDE_DIRS})  
target_link_libraries(ec_master_main ${THIRDPARTY_LIBS})

# ec_master_tools 静态库
add_library(ec_master_tools STATIC ${EC_TOOLS_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp )
set_target_properties(ec_master_tools PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_tools PUBLIC ${EcTool_INCLUDE_DIRS})

# leg_breakin Python 模块
add_library(leg_breakin MODULE 
    ${EC_TOOLS_DIR}/Sharelib/User/leg_breakin.cpp
    ${EC_TOOLS_DIR}/Sharelib/User/EcSdo.cpp
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

# 设置 RPATH 以便 leg_breakin.so 在运行时能找到 lib 中的 .so 文件
set_target_properties(leg_breakin PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    BUILD_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH "$ORIGIN/../lib"
)

## EC Demo的原配置
set(CMAKE_BUILD_TYPE "Release")

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
