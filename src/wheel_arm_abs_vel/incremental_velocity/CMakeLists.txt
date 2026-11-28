add_executable(pinocchio_incremental_velocity_ik_node src/pinocchio_velocity_ik_node.cpp src/dagger_candidate.cpp)
target_include_directories(pinocchio_incremental_velocity_ik_node PRIVATE
  ../include ../velocity_ik
  ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
target_link_libraries(pinocchio_incremental_velocity_ik_node PRIVATE velocity_ik_model velocity_ik_qp nlohmann_json::nlohmann_json ${catkin_LIBRARIES})
add_dependencies(pinocchio_incremental_velocity_ik_node ${VELOCITY_IK_MESSAGE_TARGETS})
install(TARGETS pinocchio_incremental_velocity_ik_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/run_incremental_velocity.py scripts/run_incremental_dual_quest.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_executable(incremental_velocity_kernel_test velocity_ik/incremental_kernel_test.cpp)
  target_include_directories(incremental_velocity_kernel_test PRIVATE ../include ../velocity_ik
    ${VELOCITY_IK_MESSAGE_INCLUDE} ${catkin_INCLUDE_DIRS})
  target_link_libraries(incremental_velocity_kernel_test PRIVATE velocity_ik_model velocity_ik_qp ${catkin_LIBRARIES})
  target_compile_options(incremental_velocity_kernel_test PRIVATE -UNDEBUG)
  add_dependencies(incremental_velocity_kernel_test ${VELOCITY_IK_MESSAGE_TARGETS})
endif()

pybind11_add_module(_incremental_mapper_numeric scripts/tools/native/mapper_numeric.cpp)
target_link_libraries(_incremental_mapper_numeric PRIVATE Eigen3::Eigen)
target_compile_options(_incremental_mapper_numeric PRIVATE -ffp-contract=off -fno-fast-math)
target_compile_definitions(_incremental_mapper_numeric PRIVATE EIGEN_DONT_PARALLELIZE)
set_target_properties(_incremental_mapper_numeric PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}")
install(TARGETS _incremental_mapper_numeric LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  foreach(name incremental_handoff incremental_compat incremental_clutch incremental_no_auto_hold incremental_variant_matrix incremental_launch incremental_runtime)
    add_test(NAME test_port_${name} COMMAND ${CMAKE_COMMAND} -E env KUAVO_MAPPER_NUMERIC_BACKEND=python ${PYTHON_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/${name}_test.py)
  endforeach()
endif()

install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/incremental_velocity/scripts
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/incremental_velocity/config)
