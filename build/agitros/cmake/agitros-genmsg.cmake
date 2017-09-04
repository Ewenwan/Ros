# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "agitros: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iagitros:/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(agitros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg" NAME_WE)
add_custom_target(_agitros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agitros" "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_agitros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "agitros" "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agitros
)

### Generating Services
_generate_srv_cpp(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agitros
)

### Generating Module File
_generate_module_cpp(agitros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agitros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(agitros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(agitros_generate_messages agitros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg" NAME_WE)
add_dependencies(agitros_generate_messages_cpp _agitros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(agitros_generate_messages_cpp _agitros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agitros_gencpp)
add_dependencies(agitros_gencpp agitros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agitros_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agitros
)

### Generating Services
_generate_srv_lisp(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agitros
)

### Generating Module File
_generate_module_lisp(agitros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agitros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(agitros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(agitros_generate_messages agitros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg" NAME_WE)
add_dependencies(agitros_generate_messages_lisp _agitros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(agitros_generate_messages_lisp _agitros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agitros_genlisp)
add_dependencies(agitros_genlisp agitros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agitros_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros
)

### Generating Services
_generate_srv_py(agitros
  "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros
)

### Generating Module File
_generate_module_py(agitros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(agitros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(agitros_generate_messages agitros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/msg/Num.msg" NAME_WE)
add_dependencies(agitros_generate_messages_py _agitros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/agitros/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(agitros_generate_messages_py _agitros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(agitros_genpy)
add_dependencies(agitros_genpy agitros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS agitros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agitros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/agitros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(agitros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agitros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/agitros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(agitros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/agitros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(agitros_generate_messages_py std_msgs_generate_messages_py)
endif()
