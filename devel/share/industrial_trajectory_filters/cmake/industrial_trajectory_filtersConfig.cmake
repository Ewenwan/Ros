# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(industrial_trajectory_filters_CONFIG_INCLUDED)
  return()
endif()
set(industrial_trajectory_filters_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(industrial_trajectory_filters_SOURCE_PREFIX /home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_trajectory_filters)
  set(industrial_trajectory_filters_DEVEL_PREFIX /home/ewenwan/ewenwan/catkin_ws/devel)
  set(industrial_trajectory_filters_INSTALL_PREFIX "")
  set(industrial_trajectory_filters_PREFIX ${industrial_trajectory_filters_DEVEL_PREFIX})
else()
  set(industrial_trajectory_filters_SOURCE_PREFIX "")
  set(industrial_trajectory_filters_DEVEL_PREFIX "")
  set(industrial_trajectory_filters_INSTALL_PREFIX /home/ewenwan/ewenwan/catkin_ws/install)
  set(industrial_trajectory_filters_PREFIX ${industrial_trajectory_filters_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'industrial_trajectory_filters' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(industrial_trajectory_filters_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_trajectory_filters/include " STREQUAL " ")
  set(industrial_trajectory_filters_INCLUDE_DIRS "")
  set(_include_dirs "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_trajectory_filters/include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${industrial_trajectory_filters_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'industrial_trajectory_filters' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Shaun Edwards <sedwards@swri.org>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'industrial_trajectory_filters' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_trajectory_filters/${idir}'.  Ask the maintainer 'Shaun Edwards <sedwards@swri.org>' to fix it.")
    endif()
    _list_append_unique(industrial_trajectory_filters_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "industrial_trajectory_filters")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND industrial_trajectory_filters_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND industrial_trajectory_filters_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND industrial_trajectory_filters_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/ewenwan/ewenwan/catkin_ws/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(industrial_trajectory_filters_LIBRARY_DIRS ${lib_path})
      list(APPEND industrial_trajectory_filters_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'industrial_trajectory_filters'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND industrial_trajectory_filters_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(industrial_trajectory_filters_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${industrial_trajectory_filters_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "moveit_ros_planning;trajectory_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 industrial_trajectory_filters_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${industrial_trajectory_filters_dep}_FOUND)
      find_package(${industrial_trajectory_filters_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${industrial_trajectory_filters_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(industrial_trajectory_filters_INCLUDE_DIRS ${${industrial_trajectory_filters_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(industrial_trajectory_filters_LIBRARIES ${industrial_trajectory_filters_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${industrial_trajectory_filters_dep}_LIBRARIES})
  _list_append_deduplicate(industrial_trajectory_filters_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(industrial_trajectory_filters_LIBRARIES ${industrial_trajectory_filters_LIBRARIES})

  _list_append_unique(industrial_trajectory_filters_LIBRARY_DIRS ${${industrial_trajectory_filters_dep}_LIBRARY_DIRS})
  list(APPEND industrial_trajectory_filters_EXPORTED_TARGETS ${${industrial_trajectory_filters_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${industrial_trajectory_filters_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
