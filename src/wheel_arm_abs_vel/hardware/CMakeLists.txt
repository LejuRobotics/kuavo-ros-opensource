# Private hardware implementation: never replace a shared hardware library.
find_package(catkin REQUIRED COMPONENTS roscpp roslib nodelet kuavo_common kuavo_solver kuavo_msgs humanoid_interface humanoid_interface_drake leju_utils kuavo_assets urdf)
set(HW_BASE ${CMAKE_CURRENT_SOURCE_DIR}/../../kuavo-ros-control-lejulib)
set(PLANT ${HW_BASE}/hardware_plant)
set(SDK ${PLANT}/lib)
set(PRIVATE_SDK ${CMAKE_CURRENT_SOURCE_DIR}/hardware_plant/lib)
set(MOTION ${CMAKE_CURRENT_SOURCE_DIR}/../../manipulation_nodes/motion_capture_ik)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(EC ${SDK}/EC_Master_arm64/src)
  set(BM_NAME bmapi64.arm)
  set(SG_PLATFORM linux-arm64)
  set(STARK ${SDK}/dexhand_arm64_sdk)
  add_definitions(-DARCH_AARCH64)
else()
  set(EC ${SDK}/EC_Master/src)
  set(BM_NAME bmapi64)
  set(SG_PLATFORM linux-x86_64)
  set(STARK ${SDK}/dexhand_sdk/modbus_sdk/dist)
  add_definitions(-DARCH_X86_64)
endif()
set(ABS_PRIVATE_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/hardware_node/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hardware_plant/src
  ${CMAKE_CURRENT_SOURCE_DIR}/hardware_node/include/hardware_node
  ${CMAKE_CURRENT_SOURCE_DIR}/sg100/include
  ${PRIVATE_SDK}/motorevo_controller/include
  ${PRIVATE_SDK}/leju_claw_driver/include
  ${HW_BASE}/hardware_node/include ${HW_BASE}/hardware_node/include/hardware_node
  ${PLANT}/src ${PLANT}/include ${SDK}/dexhand_sdk ${STARK}/include
  ${SDK}/ruiwo_controller_cxx/include ${SDK}/leju_claw_driver/include
  ${SDK}/Math ${SDK}/xsens_ros_mti_driver/src ${SDK}/xsens_ros_mti_driver/lib/xspublic ${SDK}/hipnuc_imu/src
  ${SDK}/canbus_sdk/include ${SDK}/canbus_sdk/src ${SDK}/BMAPI/include ${SDK}/LEJU_CAN_API
  ${SDK}/SG100_SDK_Publish/include
  ${EC}/Common ${EC}/SDK/INC ${EC}/SDK/INC/Linux
  ${EC}/Sharelib/Common ${EC}/Sharelib/EcMasterDemo ${EC}/Sharelib/Linux
  ${catkin_INCLUDE_DIRS} ${MOTION}/include)
get_property(ABS_PARENT_INCLUDES DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
set_property(DIRECTORY PROPERTY INCLUDE_DIRECTORIES ${ABS_PRIVATE_INCLUDES} ${ABS_PARENT_INCLUDES})
file(GLOB CAN_SOURCES ${SDK}/canbus_sdk/src/*.cpp)
list(FILTER CAN_SOURCES EXCLUDE REGEX "canbus_ctrl\\.cpp$")
file(GLOB MOTOR_SOURCES ${PRIVATE_SDK}/motorevo_controller/src/*.cpp)
file(GLOB CLAW_SOURCES ${SDK}/leju_claw_driver/src/*.cpp ${SDK}/leju_claw_driver/src/*.cc ${SDK}/leju_claw_driver/src/*.c)
list(FILTER CLAW_SOURCES EXCLUDE REGEX "lejuclaw_can_customed\\.cpp$")
file(GLOB DEX_SOURCES ${SDK}/dexhand_sdk/src/*.cpp)
list(FILTER DEX_SOURCES EXCLUDE REGEX "protobuf_dexhand\\.cpp$")
set(PRIVATE_SOURCES ${CAN_SOURCES} ${MOTOR_SOURCES} ${CLAW_SOURCES} ${DEX_SOURCES}
  ${SDK}/LEJU_CAN_API/CANFD_leju.cpp
  ${PRIVATE_SDK}/canbus_sdk/src/canbus_ctrl.cpp
  ${PRIVATE_SDK}/leju_claw_driver/src/lejuclaw_can_customed.cpp
  hardware_node/src/hardware_node.cc hardware_node/src/sg100_hand_driver.cpp
  ${HW_BASE}/hardware_node/include/hardware_node/touch_dexhand_ros1.cpp
  ${HW_BASE}/hardware_node/src/revo2_dexhand_ros.cpp
  sg100/src/SG100GestureLibrary.cpp sg100/src/SG100HandBridge.cpp ${MOTION}/src/SG100JointLimits.cpp
  hardware_plant/src/hardware_plant.cc hardware_plant/src/lejuclaw_controller.cc)
foreach(name actuators_interface.cpp Kalman.cpp kalman_estimate.cc utils.cpp motor_status_manager.cpp touch_hand_controller.cpp linker_hand_mapping.cpp revo2_hand_controller.cpp ec_firmware_param_check.cpp)
  list(APPEND PRIVATE_SOURCES ${PLANT}/src/${name})
endforeach()
# Hidden symbols and local binding prevent ABI collisions with the beta libraries
# loaded by other nodelets in the same manager; only this plugin owns these fixes.
add_library(absolute_hardware_impl STATIC ${PRIVATE_SOURCES})
target_compile_options(absolute_hardware_impl PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
target_compile_definitions(absolute_hardware_impl PRIVATE ROBOT_VERSION_INT=62)
set(ABS_HARDWARE_VENDOR_ROOT "${SDK}" CACHE PATH "Unmodified hardware vendor dependency tree")
set(VENDOR_LIBS)
foreach(name xsens_mti_driver ec_master_main ec_app mathtools_hw hipnuc_imu_receiver ruiwo_actuatorCXXLib)
  find_library(ABS_${name} NAMES ${name} PATHS ${CATKIN_DEVEL_PREFIX}/lib)
  if(NOT ABS_${name})
    message(FATAL_ERROR "Missing unchanged vendor dependency ${name}; build hardware_node first")
  endif()
  list(APPEND VENDOR_LIBS ${ABS_${name}})
endforeach()
find_library(ABS_BM NAMES ${BM_NAME} PATHS ${SDK}/ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan REQUIRED)
find_library(ABS_STARK NAMES bc_stark_sdk PATHS ${STARK}/shared/linux REQUIRED)
# Static vendor libraries need their transitive SDK dependencies in standalone
# package builds. A normal workspace may provide these from the hardware build.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ABS_EC_VENDOR ${ABS_HARDWARE_VENDOR_ROOT}/EC_Master_arm64/src/SDK/LIB/Linux/aarch64)
  set(ABS_EC_RAS ${ABS_EC_VENDOR}/libEcMasterRasServer.a)
else()
  set(ABS_EC_VENDOR ${ABS_HARDWARE_VENDOR_ROOT}/EC_Master/src/SDK/LIB/Linux/x64)
  set(ABS_EC_RAS ${ABS_EC_VENDOR}/libAtemRasSrv.a)
endif()
foreach(name xscontroller xscommon xstypes)
  list(APPEND VENDOR_LIBS ${ABS_HARDWARE_VENDOR_ROOT}/xsens_ros_mti_driver/lib/xspublic/${name}/lib${name}.a)
endforeach()
list(APPEND VENDOR_LIBS ${ABS_EC_RAS} ${ABS_EC_VENDOR}/libEcMaster.a)
target_link_libraries(absolute_hardware_impl PUBLIC ${catkin_LIBRARIES} ${VENDOR_LIBS}
  ${ABS_BM} ${ABS_STARK} ${SDK}/SG100_SDK_Publish/lib/${SG_PLATFORM}/libsg100_sdk.so
  lcm udev yaml-cpp ${JSONCPP_LIBRARIES} drake::drake usb-1.0 stdc++fs pthread dl rt)
add_dependencies(absolute_hardware_impl ${catkin_EXPORTED_TARGETS})
add_library(nodelet_absolute_hardware hardware_node/src/hardware_nodelet.cc)
target_link_libraries(nodelet_absolute_hardware PRIVATE absolute_hardware_impl)
target_compile_options(nodelet_absolute_hardware PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
target_link_options(nodelet_absolute_hardware PRIVATE -Wl,-Bsymbolic -Wl,--no-undefined -Wl,--exclude-libs,ALL)
install(TARGETS nodelet_absolute_hardware LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  foreach(name sg100_control_policy sg100_initialization_retry sg100_command_parameters can_feedback_regression rt_send_limiter)
    add_executable(absolute_${name}_test ../test/${name}_test.cpp)
    target_compile_options(absolute_${name}_test PRIVATE -UNDEBUG)
    target_link_libraries(absolute_${name}_test PRIVATE absolute_hardware_impl gtest_main gtest)
    add_test(NAME absolute_${name}_test COMMAND absolute_${name}_test)
  endforeach()
endif()

add_executable(absolute_sg100_hand_node hardware_node/src/sg100_hand_main.cpp)
target_link_libraries(absolute_sg100_hand_node PRIVATE absolute_hardware_impl)
install(TARGETS absolute_sg100_hand_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_test(NAME absolute_sg100_driver_initialization_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/sg100_driver_initialization_test.py $<TARGET_FILE:absolute_sg100_hand_node>)
  set_tests_properties(absolute_sg100_driver_initialization_test PROPERTIES TIMEOUT 35)
  foreach(name sg100_curve_validation sg100_action_list sg100_gesture_library end_effector_contract sg100_parameter_bridge)
    add_executable(absolute_${name}_test ../test/${name}_test.cpp)
    target_compile_options(absolute_${name}_test PRIVATE -UNDEBUG)
    target_link_libraries(absolute_${name}_test PRIVATE absolute_hardware_impl gtest_main gtest)
    if(name STREQUAL "sg100_parameter_bridge")
      add_test(NAME absolute_${name}_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/run_sg100_parameter_bridge_test.py $<TARGET_FILE:absolute_${name}_test>)
    elseif(name STREQUAL "sg100_action_list")
      add_test(NAME absolute_${name}_test COMMAND absolute_${name}_test ${CMAKE_CURRENT_SOURCE_DIR}/../test/data)
    else()
      add_test(NAME absolute_${name}_test COMMAND absolute_${name}_test)
    endif()
  endforeach()
endif()
if(CATKIN_ENABLE_TESTING)
  add_executable(private_hardware_plugin_test ../test/private_hardware_plugin_test.cpp)
  target_link_libraries(private_hardware_plugin_test PRIVATE ${catkin_LIBRARIES})
  add_dependencies(private_hardware_plugin_test nodelet_absolute_hardware)
  add_test(NAME private_hardware_plugin_test COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../test/run_sg100_parameter_bridge_test.py $<TARGET_FILE:private_hardware_plugin_test>)
  set_tests_properties(private_hardware_plugin_test PROPERTIES TIMEOUT 40)
endif()
