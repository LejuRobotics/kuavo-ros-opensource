# Import the pinned OpenVINO Runtime SDK shipped with the local simulation.
# This is intentionally Runtime-only; it is not a replacement for the full SDK.
if(NOT KUAVO_DEPENDENCIES)
  message(FATAL_ERROR "Set KUAVO_DEPENDENCIES to the pinned simulation SDK directory")
endif()
if(OpenVINO_FIND_COMPONENTS AND NOT OpenVINO_FIND_COMPONENTS STREQUAL "Runtime")
  message(FATAL_ERROR "The pinned SDK only exports the OpenVINO Runtime component")
endif()
set(_ov_library "${KUAVO_DEPENDENCIES}/bound_lib/libopenvino.so.2025.2.0")
set(_ov_include "${KUAVO_DEPENDENCIES}/build_headers")
if(NOT EXISTS "${_ov_library}" OR NOT EXISTS "${_ov_include}/openvino/openvino.hpp")
  message(FATAL_ERROR "Incomplete pinned OpenVINO 2025.2 SDK in ${KUAVO_DEPENDENCIES}")
endif()
if(NOT TARGET openvino::runtime)
  add_library(openvino::runtime SHARED IMPORTED)
  set_target_properties(openvino::runtime PROPERTIES
    IMPORTED_LOCATION "${_ov_library}"
    INTERFACE_INCLUDE_DIRECTORIES "${_ov_include}"
    INTERFACE_COMPILE_DEFINITIONS "_GLIBCXX_USE_CXX11_ABI=1;OV_THREAD=OV_THREAD_TBB"
    INTERFACE_COMPILE_FEATURES "cxx_std_17")
endif()
set(OpenVINO_FOUND TRUE)
set(OpenVINO_Runtime_FOUND TRUE)
set(OpenVINO_VERSION "2025.2.0")
unset(_ov_library)
unset(_ov_include)
