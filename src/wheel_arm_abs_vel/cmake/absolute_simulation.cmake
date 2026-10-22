# Private ROS-only simulator. Shared mujoco_cpp files and binaries stay untouched.
set(abs_mujoco_root "${CMAKE_CURRENT_SOURCE_DIR}/../mujoco")
set(abs_sg_root "${CMAKE_CURRENT_SOURCE_DIR}/../manipulation_nodes/motion_capture_ik")
find_package(PythonInterp 3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../kuavo_common/cmake/kuavo_arch.cmake)
if(KUAVO_BUILD_FOR_AARCH64)
  set(abs_mujoco_lib "${abs_mujoco_root}/lib/aarch64/libmujoco.so")
else()
  set(abs_mujoco_lib "${abs_mujoco_root}/lib/x86_64/libmujoco.so")
endif()
if(NOT EXISTS "${abs_mujoco_lib}")
  message(FATAL_ERROR "Missing bundled MuJoCo: ${abs_mujoco_lib}")
endif()
set(abs_sim_source "${CMAKE_CURRENT_BINARY_DIR}/absolute_mujoco_node.cc")
add_custom_command(OUTPUT "${abs_sim_source}"
  COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_absolute_sim.py"
          "${abs_mujoco_root}/src/mujoco_node.cc" "${abs_sim_source}"
  DEPENDS "${abs_mujoco_root}/src/mujoco_node.cc"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_absolute_sim.py"
  VERBATIM)
file(GLOB abs_sim_helpers "${abs_mujoco_root}/src/simulate/*.cc"
  "${abs_mujoco_root}/src/simulate/*.cpp"
  "${abs_mujoco_root}/src/dexhand/*.cpp"
  "${abs_mujoco_root}/utils/cpp_ray_caster/*.cpp")
add_executable(absolute_mujoco_sim src/simulation/main.cpp src/simulation/x11_private_ipc.cpp "${abs_sim_source}"
  ${abs_sim_helpers} "${abs_mujoco_root}/src/dexhand_mujoco_node.cpp"
  "${abs_sg_root}/src/SG100HandBridge.cpp"
  "${abs_sg_root}/src/SG100GestureLibrary.cpp"
  "${abs_sg_root}/src/SG100JointLimits.cpp")
target_include_directories(absolute_mujoco_sim PRIVATE
  "${abs_mujoco_root}/include" "${abs_mujoco_root}/include/simulate"
  "${CMAKE_CURRENT_SOURCE_DIR}/../lcm_msg/include"
  "${abs_mujoco_root}/src" "${abs_mujoco_root}/utils/cpp_ray_caster"
  "${abs_sg_root}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../kuavo-ros-control-lejulib/hardware_node/include"
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(absolute_mujoco_sim ${catkin_LIBRARIES} "${abs_mujoco_lib}"
  ${CMAKE_DL_LIBS} Threads::Threads glfw Eigen3::Eigen ${OpenCV_LIBRARIES} yaml-cpp)
add_dependencies(absolute_mujoco_sim ${catkin_EXPORTED_TARGETS})
install(TARGETS absolute_mujoco_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_options(absolute_mujoco_sim PRIVATE "-Wl,--export-dynamic")

if(CATKIN_ENABLE_TESTING)
  add_executable(simulation_servo_test test/simulation_servo_test.cpp)
  target_include_directories(simulation_servo_test PRIVATE "${abs_mujoco_root}/include")
  target_link_libraries(simulation_servo_test ${catkin_LIBRARIES} "${abs_mujoco_lib}")
  target_compile_options(simulation_servo_test PRIVATE -UNDEBUG)
  add_test(NAME simulation_servo_test COMMAND simulation_servo_test)
endif()
