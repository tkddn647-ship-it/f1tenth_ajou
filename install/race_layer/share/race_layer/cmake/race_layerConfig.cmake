# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_race_layer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED race_layer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(race_layer_FOUND FALSE)
  elseif(NOT race_layer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(race_layer_FOUND FALSE)
  endif()
  return()
endif()
set(_race_layer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT race_layer_FIND_QUIETLY)
  message(STATUS "Found race_layer: 0.1.0 (${race_layer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'race_layer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${race_layer_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(race_layer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${race_layer_DIR}/${_extra}")
endforeach()
