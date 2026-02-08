#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hardware_plant::hardware_plant" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::hardware_plant APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::hardware_plant PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libhardware_plant.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::hardware_plant )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::hardware_plant "${_IMPORT_PREFIX}/lib/libhardware_plant.a" )

# Import target "hardware_plant::hardware_plant_cxx" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::hardware_plant_cxx APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::hardware_plant_cxx PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libhardware_plant_cxx.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::hardware_plant_cxx )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::hardware_plant_cxx "${_IMPORT_PREFIX}/lib/libhardware_plant_cxx.a" )

# Import target "hardware_plant::xsens_mti_driver" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::xsens_mti_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::xsens_mti_driver PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libxsens_mti_driver.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::xsens_mti_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::xsens_mti_driver "${_IMPORT_PREFIX}/lib/libxsens_mti_driver.a" )

# Import target "hardware_plant::ec_master_main" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::ec_master_main APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::ec_master_main PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libec_master_main.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::ec_master_main )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::ec_master_main "${_IMPORT_PREFIX}/lib/libec_master_main.a" )

# Import target "hardware_plant::ec_app" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::ec_app APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::ec_app PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libec_app.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::ec_app )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::ec_app "${_IMPORT_PREFIX}/lib/libec_app.a" )

# Import target "hardware_plant::mathtools_hw" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::mathtools_hw APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::mathtools_hw PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libmathtools_hw.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libmathtools_hw.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::mathtools_hw )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::mathtools_hw "${_IMPORT_PREFIX}/lib/libmathtools_hw.so" )

# Import target "hardware_plant::lejuclaw" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::lejuclaw APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::lejuclaw PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "C;CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/liblejuclaw.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::lejuclaw )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::lejuclaw "${_IMPORT_PREFIX}/lib/liblejuclaw.a" )

# Import target "hardware_plant::hipnuc_imu_receiver" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::hipnuc_imu_receiver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::hipnuc_imu_receiver PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libhipnuc_imu_receiver.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libhipnuc_imu_receiver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::hipnuc_imu_receiver )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::hipnuc_imu_receiver "${_IMPORT_PREFIX}/lib/libhipnuc_imu_receiver.so" )

# Import target "hardware_plant::dexhand_sdk" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::dexhand_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::dexhand_sdk PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libdexhand_sdk.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::dexhand_sdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::dexhand_sdk "${_IMPORT_PREFIX}/lib/libdexhand_sdk.a" )

# Import target "hardware_plant::canbus_sdk" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::canbus_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::canbus_sdk PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libcanbus_sdk.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libcanbus_sdk.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::canbus_sdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::canbus_sdk "${_IMPORT_PREFIX}/lib/libcanbus_sdk.so" )

# Import target "hardware_plant::motorevo_controller" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::motorevo_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::motorevo_controller PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libmotorevo_controller.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libmotorevo_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::motorevo_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::motorevo_controller "${_IMPORT_PREFIX}/lib/libmotorevo_controller.so" )

# Import target "hardware_plant::ruiwo_actuatorLib" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::ruiwo_actuatorLib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::ruiwo_actuatorLib PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libruiwo_actuatorLib.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libruiwo_actuatorLib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::ruiwo_actuatorLib )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::ruiwo_actuatorLib "${_IMPORT_PREFIX}/lib/libruiwo_actuatorLib.so" )

# Import target "hardware_plant::ruiwo_actuatorCXXLib" for configuration "RelWithDebInfo"
set_property(TARGET hardware_plant::ruiwo_actuatorCXXLib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(hardware_plant::ruiwo_actuatorCXXLib PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libruiwo_actuatorCXXLib.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libruiwo_actuatorCXXLib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hardware_plant::ruiwo_actuatorCXXLib )
list(APPEND _IMPORT_CHECK_FILES_FOR_hardware_plant::ruiwo_actuatorCXXLib "${_IMPORT_PREFIX}/lib/libruiwo_actuatorCXXLib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
