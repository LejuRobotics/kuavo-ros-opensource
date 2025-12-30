
add_executable(kinematicMpcTest
  kinematicMpcTest.cpp
  )
add_dependencies(kinematicMpcTest
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(kinematicMpcTest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_compile_options(kinematicMpcTest PRIVATE ${OCS2_CXX_FLAGS})
