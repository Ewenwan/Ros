cmake_minimum_required(VERSION 2.8.3)

#
# Generate a rosserial_client ros_lib folder using the make_libraries
# script supplied by a particular package. The default is to fall back
# on the generic script in rosserial_client itself.
#
# :param PACKAGE: name of package to look in for lib generating script.
# :type PACKAGE: string
# :param SCRIPT: name of script, for example `make_libraries.py`.
# :type SCRIPT: string
#
# @public
#
function(rosserial_generate_ros_lib)
  cmake_parse_arguments(make_libraries "" "PACKAGE;SCRIPT" "" ${ARGN}) 
  if(NOT make_libraries_PACKAGE)
    set(make_libraries_PACKAGE rosserial_client)
  endif()
  if(NOT make_libraries_SCRIPT)
    set(make_libraries_SCRIPT make_libraries)
  endif()

  message(STATUS "Using ${make_libraries_PACKAGE}/${make_libraries_SCRIPT} to make rosserial client library.")

  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/ros_lib
    COMMAND ${CATKIN_ENV} rosrun ${make_libraries_PACKAGE} ${make_libraries_SCRIPT} ${PROJECT_BINARY_DIR}
  )
  add_custom_target(${PROJECT_NAME}_ros_lib DEPENDS ${PROJECT_BINARY_DIR}/ros_lib)
  add_dependencies(${PROJECT_NAME}_ros_lib rosserial_msgs_genpy std_msgs_genpy)
  set(${PROJECT_NAME}_ROS_LIB_DIR "${PROJECT_BINARY_DIR}/ros_lib" PARENT_SCOPE)
endfunction()

#
# Configure a CMake project located in a subfolder of a catkin project,
# optionally specifying a CMake toolchain to use in the build of the
# subproject.
#
# :param DIRECTORY: subdirectory of current package to configure.
# :type DIRECTORY: string
# :param TOOLCHAIN_FILE: full path to toolchain file.
# :type TOOLCHAIN_FILE: string
#
# @public
#
function(rosserial_configure_client)
  cmake_parse_arguments(client "" "DIRECTORY;TOOLCHAIN_FILE" "CMAKE_ARGUMENTS" ${ARGN})
  if(NOT client_DIRECTORY)
    message(SEND_ERROR "rosserial_client_add_client called without DIRECTORY argument.")
  endif()

  if(client_TOOLCHAIN_FILE)
    set(DTOOLCHAIN_FILE -DCMAKE_TOOLCHAIN_FILE=${client_TOOLCHAIN_FILE})
  endif()

  # Create a build tree directory for configuring the client's CMake project.
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${client_DIRECTORY})
  add_custom_target(${PROJECT_NAME}_${client_DIRECTORY}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${client_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/${client_DIRECTORY}
      -DROS_LIB_DIR=${${PROJECT_NAME}_ROS_LIB_DIR}
      -DEXECUTABLE_OUTPUT_PATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
      ${client_CMAKE_ARGUMENTS}
      ${DTOOLCHAIN_FILE}
  )
  add_dependencies(${PROJECT_NAME}_${client_DIRECTORY} ${PROJECT_NAME}_ros_lib)
endfunction()

#
# Create a catkin target which builds a target in the subproject.
#
# :param client_directory: subdirectory of current package with subproject.
# :type client_directory: string
# :param client_target: name of target in subproject to build.
# :type client_target: string
# :param ARGN: additional arguments for target (eg, ALL).
# :type ARGN: list of strings
#
# @public
#
function(rosserial_add_client_target client_directory client_target)
  add_custom_target(${PROJECT_NAME}_${client_directory}_${client_target} ${ARGN}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${client_directory}
    COMMAND ${CMAKE_COMMAND} --build ${PROJECT_BINARY_DIR}/${client_directory} -- ${client_target}
  )
  add_dependencies(${PROJECT_NAME}_${client_directory}_${client_target}
                   ${PROJECT_NAME}_${client_directory})
endfunction()
