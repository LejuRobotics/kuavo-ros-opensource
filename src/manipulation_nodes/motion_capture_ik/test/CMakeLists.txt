add_executable(plant_ik_test plant_ik_test.cpp)
if(EXISTS "${PLANT_IK_FILE}")
  target_link_libraries(plant_ik_test plantIK ${catkin_LIBRARIES} drake::drake)
else()
  target_link_libraries(plant_ik_test ${PLANTIK_LIBRARY} ${catkin_LIBRARIES} drake::drake)
endif()

add_executable(wheel_natural_elbow_guide_test wheel_natural_elbow_guide_test.cpp)
target_compile_features(wheel_natural_elbow_guide_test PUBLIC cxx_std_17)

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
