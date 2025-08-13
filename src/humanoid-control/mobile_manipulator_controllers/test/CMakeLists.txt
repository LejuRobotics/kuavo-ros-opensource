
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

add_executable(mobileManipulatorControllerBaseTest
  mobileManipulatorControllerBaseTest.cpp
  )
add_dependencies(mobileManipulatorControllerBaseTest
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(mobileManipulatorControllerBaseTest
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
  )
target_compile_options(mobileManipulatorControllerBaseTest PRIVATE ${OCS2_CXX_FLAGS})

add_executable(mobileManipulatorControllerBaseTrackingTest
  mobileManipulatorControllerBaseTrackingTest.cpp
  )
add_dependencies(mobileManipulatorControllerBaseTrackingTest
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(mobileManipulatorControllerBaseTrackingTest
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
  )
target_compile_options(mobileManipulatorControllerBaseTrackingTest PRIVATE ${OCS2_CXX_FLAGS})

# 新增的Google Test单元测试
find_package(GTest REQUIRED)
if(GTEST_FOUND)
  add_executable(mobileManipulatorControllerBaseUnitTest
    mobileManipulatorControllerBaseUnitTest.cpp
    )
  add_dependencies(mobileManipulatorControllerBaseUnitTest
    ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )
  target_link_libraries(mobileManipulatorControllerBaseUnitTest
    ${PROJECT_NAME}_base
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    gmock
    gmock_main
    pthread
    )
  target_compile_options(mobileManipulatorControllerBaseUnitTest PRIVATE ${OCS2_CXX_FLAGS} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_ERROR)
  target_include_directories(mobileManipulatorControllerBaseUnitTest PRIVATE ${GTEST_INCLUDE_DIRS})
  
  # 添加测试到CTest
  add_test(NAME MobileManipulatorControllerBaseUnitTest 
           COMMAND mobileManipulatorControllerBaseUnitTest)
  
  # IK Target库测试
  add_executable(test_ik_target_lib
    test_ik_target_lib.cpp
    )
  add_dependencies(test_ik_target_lib
    ${PROJECT_NAME}_ik_target_lib
    ${catkin_EXPORTED_TARGETS}
    )
  target_link_libraries(test_ik_target_lib
    ${PROJECT_NAME}_ik_target_lib
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    gmock
    gmock_main
    pthread
    )
  target_compile_options(test_ik_target_lib PRIVATE ${OCS2_CXX_FLAGS} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_ERROR)
  target_include_directories(test_ik_target_lib PRIVATE ${GTEST_INCLUDE_DIRS})
  
  # 添加IK Target测试到CTest
  add_test(NAME TestIkTargetLib 
           COMMAND test_ik_target_lib)
  
  # 死锁检测测试
  add_executable(deadlockDetectionTest
    deadlockDetectionTest.cpp
    )
  add_dependencies(deadlockDetectionTest
    ${PROJECT_NAME}_base
    ${catkin_EXPORTED_TARGETS}
    )
  target_link_libraries(deadlockDetectionTest
    ${PROJECT_NAME}_base
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    gmock
    gmock_main
    pthread
    )
  target_compile_options(deadlockDetectionTest PRIVATE ${OCS2_CXX_FLAGS} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_ERROR)
  target_include_directories(deadlockDetectionTest PRIVATE ${GTEST_INCLUDE_DIRS})
  
  # 添加死锁检测测试到CTest
  add_test(NAME DeadlockDetectionTest 
           COMMAND deadlockDetectionTest)
endif()