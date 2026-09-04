add_executable(plant_ik_test plant_ik_test.cpp)
if(EXISTS "${PLANT_IK_FILE}")
  target_link_libraries(plant_ik_test plantIK ${catkin_LIBRARIES} drake::drake)
else()
  target_link_libraries(plant_ik_test ${PLANTIK_LIBRARY} ${catkin_LIBRARIES} drake::drake)
endif()

add_executable(wheel_natural_elbow_guide_test wheel_natural_elbow_guide_test.cpp)
target_compile_features(wheel_natural_elbow_guide_test PUBLIC cxx_std_17)

add_executable(
  wheel_incremental_orientation_frame_test
  wheel_incremental_orientation_frame_test.cpp
  ../src/WheelIncrementalControlModule.cpp
)
target_link_libraries(wheel_incremental_orientation_frame_test ${catkin_LIBRARIES})
target_compile_features(wheel_incremental_orientation_frame_test PUBLIC cxx_std_17)

add_executable(wheel_chest_reference_mapping_test wheel_chest_reference_mapping_test.cpp)
target_compile_features(wheel_chest_reference_mapping_test PUBLIC cxx_std_17)
add_test(
  NAME wheel_chest_reference_mapping_test
  COMMAND $<TARGET_FILE:wheel_chest_reference_mapping_test>
)

add_executable(wheel_one_stage_state_resync_test wheel_one_stage_state_resync_test.cpp)
target_link_libraries(wheel_one_stage_state_resync_test ${catkin_LIBRARIES} drake::drake)
target_compile_features(wheel_one_stage_state_resync_test PUBLIC cxx_std_17)
add_test(
  NAME wheel_one_stage_state_resync_test
  COMMAND $<TARGET_FILE:wheel_one_stage_state_resync_test>
)

add_executable(
  wheel_three_point_grip_ik_test
  wheel_three_point_grip_ik_test.cpp
  ../src/BaseIKSolver.cpp
  ../src/WheelOneStageIKEndEffector.cpp
)
target_link_libraries(wheel_three_point_grip_ik_test ${catkin_LIBRARIES} drake::drake)
target_compile_features(wheel_three_point_grip_ik_test PUBLIC cxx_std_17)
add_test(
  NAME wheel_three_point_grip_ik_test
  COMMAND $<TARGET_FILE:wheel_three_point_grip_ik_test>
)

add_executable(
  wheel_kinemic_limit_filter_range_reset_test
  wheel_kinemic_limit_filter_range_reset_test.cpp
)
target_link_libraries(
  wheel_kinemic_limit_filter_range_reset_test
  ${catkin_LIBRARIES}
  ${humanoid_wheel_interface_LIBRARIES}
)
target_compile_features(wheel_kinemic_limit_filter_range_reset_test PUBLIC cxx_std_17)
add_test(
  NAME wheel_kinemic_limit_filter_range_reset_test
  COMMAND $<TARGET_FILE:wheel_kinemic_limit_filter_range_reset_test>
)

add_test(
  NAME wheel_incremental_orientation_frame_test
  COMMAND $<TARGET_FILE:wheel_incremental_orientation_frame_test>
)
