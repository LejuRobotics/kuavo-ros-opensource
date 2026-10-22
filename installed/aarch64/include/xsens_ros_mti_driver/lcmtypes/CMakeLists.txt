
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM lcm REQUIRED)

set(_lcmtypes_dir ${CMAKE_CURRENT_BINARY_DIR})
set(_lcm_files lcm_std_msgs.lcm lcm_sensor_msgs.lcm)

foreach(_lcm_file ${_lcm_files})
  get_filename_component(_stem ${_lcm_file} NAME_WE)
  add_custom_command(
    OUTPUT ${_lcmtypes_dir}/${_stem}.cpp
    COMMAND lcm-gen -x --cpp-hpath ${_lcmtypes_dir} --cpp-include ${_lcmtypes_dir}
            ${CMAKE_CURRENT_SOURCE_DIR}/${_lcm_file}
    COMMAND touch ${_lcmtypes_dir}/${_stem}.cpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_lcm_file}
    COMMENT "Generating LCM C++ types from ${_lcm_file}"
    VERBATIM
  )
  list(APPEND _cpp_sources ${_lcmtypes_dir}/${_stem}.cpp)
endforeach()

add_custom_target(xsens_lcmtypes_gen DEPENDS ${_cpp_sources})

add_library(xsens-lcm-types-lib STATIC ${_cpp_sources})
add_dependencies(xsens-lcm-types-lib xsens_lcmtypes_gen)
target_include_directories(xsens-lcm-types-lib PUBLIC
  $<BUILD_INTERFACE:${_lcmtypes_dir}>
  $<INSTALL_INTERFACE:include/xsens_ros_mti_driver/lcmtypes>
  ${LCM_INCLUDE_DIRS}
)
target_link_libraries(xsens-lcm-types-lib PUBLIC ${LCM_LIBRARIES})

install(TARGETS xsens-lcm-types-lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
