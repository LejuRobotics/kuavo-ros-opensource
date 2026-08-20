add_executable(plant_ik_test plant_ik_test.cpp)
if(EXISTS "${PLANT_IK_FILE}")
  target_link_libraries(plant_ik_test plantIK ${catkin_LIBRARIES} drake::drake)
else()
  target_link_libraries(plant_ik_test ${PLANTIK_LIBRARY} ${catkin_LIBRARIES} drake::drake)
endif()

add_executable(wheel_natural_elbow_guide_test wheel_natural_elbow_guide_test.cpp)
target_compile_features(wheel_natural_elbow_guide_test PUBLIC cxx_std_17)
