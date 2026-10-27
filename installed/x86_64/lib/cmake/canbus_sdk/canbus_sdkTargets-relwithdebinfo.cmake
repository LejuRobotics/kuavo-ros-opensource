#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "canbus_sdk::canbus_sdk" for configuration "RelWithDebInfo"
set_property(TARGET canbus_sdk::canbus_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(canbus_sdk::canbus_sdk PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libcanbus_sdk.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libcanbus_sdk.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS canbus_sdk::canbus_sdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_canbus_sdk::canbus_sdk "${_IMPORT_PREFIX}/lib/libcanbus_sdk.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
