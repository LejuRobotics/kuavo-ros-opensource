
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was hardware_plantConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)

set(hardware_plant_INCLUDE_DIRS "$<BUILD_INTERFACE:/media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control/src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller>;$<INSTALL_INTERFACE:include/ruiwo_controller>")
set(hardware_plant_LIBRARIES "hardware_plant")

set(hardware_plant_CXX_INCLUDE_DIRS "$<BUILD_INTERFACE:/media/data/gitlab-runner/builds/8rHoW4Dt/0/highlydynamic/kuavo-ros-control/src/kuavo-ros-control-lejulib/hardware_plant/lib/ruiwo_controller_cxx>;$<INSTALL_INTERFACE:include/ruiwo_controller_cxx>")
set(hardware_plant_CXX_LIBRARIES "hardware_plant_cxx")

find_dependency(Boost REQUIRED COMPONENTS filesystem system)
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
find_dependency(lcm REQUIRED)
find_dependency(drake REQUIRED)
find_dependency(kuavo_common REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/hardware_plantTargets.cmake")
