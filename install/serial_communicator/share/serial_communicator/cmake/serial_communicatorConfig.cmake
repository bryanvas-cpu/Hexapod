# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_serial_communicator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED serial_communicator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(serial_communicator_FOUND FALSE)
  elseif(NOT serial_communicator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(serial_communicator_FOUND FALSE)
  endif()
  return()
endif()
set(_serial_communicator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT serial_communicator_FIND_QUIETLY)
  message(STATUS "Found serial_communicator: 0.0.0 (${serial_communicator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'serial_communicator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${serial_communicator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(serial_communicator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${serial_communicator_DIR}/${_extra}")
endforeach()
