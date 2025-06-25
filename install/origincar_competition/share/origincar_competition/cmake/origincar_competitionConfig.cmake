# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_origincar_competition_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED origincar_competition_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(origincar_competition_FOUND FALSE)
  elseif(NOT origincar_competition_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(origincar_competition_FOUND FALSE)
  endif()
  return()
endif()
set(_origincar_competition_CONFIG_INCLUDED TRUE)

# output package information
if(NOT origincar_competition_FIND_QUIETLY)
  message(STATUS "Found origincar_competition: 0.0.0 (${origincar_competition_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'origincar_competition' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${origincar_competition_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(origincar_competition_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${origincar_competition_DIR}/${_extra}")
endforeach()
