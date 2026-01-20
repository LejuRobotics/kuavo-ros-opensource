cmake_minimum_required(VERSION 3.15)
project(ec_master_tools VERSION 0.1.0)


## EC Demo的原配置
set(CMAKE_BUILD_TYPE "Release")

include(CTest)
enable_testing()

add_subdirectory(src)

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