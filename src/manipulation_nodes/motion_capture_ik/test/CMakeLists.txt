add_executable(plant_ik_test plant_ik_test.cpp)
if(EXISTS "${PLANT_IK_FILE}")
  target_link_libraries(plant_ik_test plantIK ${catkin_LIBRARIES} drake::drake)
else()
  target_link_libraries(plant_ik_test ${PLANTIK_LIBRARY} ${catkin_LIBRARIES} drake::drake)
endif()

add_executable(wheel_natural_elbow_guide_test wheel_natural_elbow_guide_test.cpp)
target_compile_features(wheel_natural_elbow_guide_test PUBLIC cxx_std_17)

add_executable(grip_quaternion_recovery_test grip_quaternion_recovery_test.cpp)
target_compile_features(grip_quaternion_recovery_test PUBLIC cxx_std_17)
add_test(NAME grip_quaternion_recovery_test COMMAND $<TARGET_FILE:grip_quaternion_recovery_test>)

add_executable(bounded_hand_quaternion_test bounded_hand_quaternion_test.cpp)
target_compile_features(bounded_hand_quaternion_test PUBLIC cxx_std_17)
add_test(NAME bounded_hand_quaternion_test COMMAND $<TARGET_FILE:bounded_hand_quaternion_test>)

add_executable(chest_point_opt_reanchor_test chest_point_opt_reanchor_test.cpp)
target_compile_features(chest_point_opt_reanchor_test PUBLIC cxx_std_17)
target_link_libraries(chest_point_opt_reanchor_test drake::drake)
add_test(NAME chest_point_opt_reanchor_test COMMAND $<TARGET_FILE:chest_point_opt_reanchor_test>)

add_executable(wheel_resume_lower_body_test
  wheel_resume_lower_body_test.cpp
  ../src/WheelOneStageIKEndEffector.cpp
  ../src/BaseIKSolver.cpp)
target_compile_features(wheel_resume_lower_body_test PUBLIC cxx_std_17)
target_compile_definitions(wheel_resume_lower_body_test PRIVATE
  KUAVO_ASSETS_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../kuavo_assets")
target_link_libraries(wheel_resume_lower_body_test ${catkin_LIBRARIES} drake::drake)
add_test(NAME wheel_resume_lower_body_test COMMAND $<TARGET_FILE:wheel_resume_lower_body_test>)

## SG100 heiman 手:手势库与关节限位单测(纯逻辑,无需 ROS master / 硬件)
add_executable(
  sg100_joint_limits_test
  sg100_joint_limits_test.cpp
  ../src/SG100JointLimits.cpp
)
target_link_libraries(sg100_joint_limits_test ${catkin_LIBRARIES})
target_compile_features(sg100_joint_limits_test PUBLIC cxx_std_17)
add_test(
  NAME sg100_joint_limits_test
  COMMAND $<TARGET_FILE:sg100_joint_limits_test>
)

add_executable(
  sg100_gesture_library_test
  sg100_gesture_library_test.cpp
  ../src/SG100GestureLibrary.cpp
)
target_link_libraries(sg100_gesture_library_test ${catkin_LIBRARIES} yaml-cpp)
target_compile_features(sg100_gesture_library_test PUBLIC cxx_std_17)
add_test(
  NAME sg100_gesture_library_test
  COMMAND $<TARGET_FILE:sg100_gesture_library_test>
)
