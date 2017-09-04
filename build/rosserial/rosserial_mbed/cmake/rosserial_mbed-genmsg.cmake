# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rosserial_mbed: 1 messages, 1 services")

set(MSG_I_FLAGS "-Irosserial_mbed:/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rosserial_mbed_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv" NAME_WE)
add_custom_target(_rosserial_mbed_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosserial_mbed" "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg" NAME_WE)
add_custom_target(_rosserial_mbed_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rosserial_mbed" "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_mbed
)

### Generating Services
_generate_srv_cpp(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_mbed
)

### Generating Module File
_generate_module_cpp(rosserial_mbed
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_mbed
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rosserial_mbed_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rosserial_mbed_generate_messages rosserial_mbed_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_cpp _rosserial_mbed_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_cpp _rosserial_mbed_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_mbed_gencpp)
add_dependencies(rosserial_mbed_gencpp rosserial_mbed_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_mbed_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_mbed
)

### Generating Services
_generate_srv_lisp(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_mbed
)

### Generating Module File
_generate_module_lisp(rosserial_mbed
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_mbed
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rosserial_mbed_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rosserial_mbed_generate_messages rosserial_mbed_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_lisp _rosserial_mbed_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_lisp _rosserial_mbed_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_mbed_genlisp)
add_dependencies(rosserial_mbed_genlisp rosserial_mbed_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_mbed_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed
)

### Generating Services
_generate_srv_py(rosserial_mbed
  "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed
)

### Generating Module File
_generate_module_py(rosserial_mbed
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rosserial_mbed_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rosserial_mbed_generate_messages rosserial_mbed_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/srv/Test.srv" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_py _rosserial_mbed_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/rosserial/rosserial_mbed/msg/Adc.msg" NAME_WE)
add_dependencies(rosserial_mbed_generate_messages_py _rosserial_mbed_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rosserial_mbed_genpy)
add_dependencies(rosserial_mbed_genpy rosserial_mbed_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rosserial_mbed_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_mbed)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rosserial_mbed
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_mbed)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rosserial_mbed
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rosserial_mbed
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
