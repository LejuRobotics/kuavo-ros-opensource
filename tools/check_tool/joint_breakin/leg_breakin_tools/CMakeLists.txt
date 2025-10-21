cmake_minimum_required(VERSION 3.15)
project(leg_breakin_tools VERSION 0.1.0)


## EC Demo的原配置
set(CMAKE_BUILD_TYPE "Release")

include(CTest)
enable_testing()

# 设置第三方库源码路径
set(THIRD_PARTY_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party_libs/ec-tools)

# 头文件路径
set(EcTool_INCLUDE_DIRS
    ${THIRD_PARTY_SRC_DIR}/Common
    ${THIRD_PARTY_SRC_DIR}/EcSdk/INC 
    ${THIRD_PARTY_SRC_DIR}/EcSdk/INC/Linux
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/ 
    ${THIRD_PARTY_SRC_DIR}/Sharelib/EcMasterDemo/
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Linux/
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/
    CACHE PATH "My include directories")

# 源文件
set(EcTool_SOURCES
    ${THIRD_PARTY_SRC_DIR}/Common/EcThread.cpp
    ${THIRD_PARTY_SRC_DIR}/Common/EcTimer.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcDemoParms.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcLogging.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcNotification.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcSdoServices.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcSelectLinkLayer.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/Common/EcSlaveInfo.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/EcMotor.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/EcSdo.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/util.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/EcLogFile.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/UserAppWorkpd.cpp
)

# 第三方库
set(THIRDPARTY_LIBS
    ${THIRD_PARTY_SRC_DIR}/EcSdk/LIB/Linux/x64/libAtemRasSrv.a
    ${THIRD_PARTY_SRC_DIR}/EcSdk/LIB/Linux/x64/libEcMaster.a
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libemllI8254x.so  # 动态库，运行时需 RPATH
    pthread
    dl
)

# ec_master_main 静态库
add_library(ec_master_main STATIC ${EcTool_SOURCES} ${THIRD_PARTY_SRC_DIR}/Sharelib/Linux/EcDemoMain.cpp)
set_target_properties(ec_master_main PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_main PUBLIC ${EcTool_INCLUDE_DIRS})  
target_link_libraries(ec_master_main ${THIRDPARTY_LIBS})

# ec_master_tools 静态库
add_library(ec_master_tools STATIC ${THIRD_PARTY_SRC_DIR}/Sharelib/EcMasterDemo/EcDemoApp.cpp )
set_target_properties(ec_master_tools PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ec_master_tools PUBLIC ${EcTool_INCLUDE_DIRS})

# ec_master_wrap Python 模块
add_library(ec_master_wrap MODULE 
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/ec_master_wrap.cpp
    ${THIRD_PARTY_SRC_DIR}/Sharelib/User/EcSdo.cpp
)
target_include_directories(ec_master_wrap PUBLIC 
    ${EcTool_INCLUDE_DIRS} 
    ${Python3_INCLUDE_DIRS} 
    ${pybind11_INCLUDE_DIRS}
)
target_link_libraries(ec_master_wrap PRIVATE 
    ec_master_main 
    ec_master_tools 
    ${Python3_LIBRARIES}
    ${THIRDPARTY_LIBS}
    pybind11::module
)

# 设置 RPATH 以便 ec_master_wrap.so 在运行时能找到 ../lib 中的 .so 文件
set_target_properties(ec_master_wrap PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    BUILD_RPATH "$ORIGIN/../../lib"
    INSTALL_RPATH "$ORIGIN/../../lib"
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# install(DIRECTORY config lib LinkOsLayer src
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib/EC_Master
#   PATTERN "*.c" EXCLUDE
#   PATTERN "*.cpp" EXCLUDE
# )
##

# 查找 Python 和 pybind11
set(Python_ADDITIONAL_VERSIONS 3.8)
find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# # 设置隐藏所有符号，只有显式标记的导出
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

# # 编译工具库（可以是静态或动态库）
# add_library(ec_master_tools SHARED
#     ec_tool_lib/libec_master_main.a
#     ec_tool_lib/libec_master_tools.a
#        # 或者 .cpp 文件
#     # 其他源文件...
# )


##

# 对于 Python 扩展模块，一般不希望生成类似 "libec_master_tools_module.so" 的前缀，所以去掉前缀
# set_target_properties(ec_master_wrap PROPERTIES PREFIX "")

# set_target_properties(ec_master_wrap PROPERTIES SUFFIX ".so")