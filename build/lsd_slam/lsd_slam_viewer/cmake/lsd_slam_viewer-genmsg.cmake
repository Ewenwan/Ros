# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lsd_slam_viewer: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilsd_slam_viewer:/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lsd_slam_viewer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg" NAME_WE)
add_custom_target(_lsd_slam_viewer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lsd_slam_viewer" "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg" NAME_WE)
add_custom_target(_lsd_slam_viewer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lsd_slam_viewer" "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lsd_slam_viewer
)
_generate_msg_cpp(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lsd_slam_viewer
)

### Generating Services

### Generating Module File
_generate_module_cpp(lsd_slam_viewer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lsd_slam_viewer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lsd_slam_viewer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lsd_slam_viewer_generate_messages lsd_slam_viewer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_cpp _lsd_slam_viewer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_cpp _lsd_slam_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lsd_slam_viewer_gencpp)
add_dependencies(lsd_slam_viewer_gencpp lsd_slam_viewer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lsd_slam_viewer_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lsd_slam_viewer
)
_generate_msg_lisp(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lsd_slam_viewer
)

### Generating Services

### Generating Module File
_generate_module_lisp(lsd_slam_viewer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lsd_slam_viewer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lsd_slam_viewer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lsd_slam_viewer_generate_messages lsd_slam_viewer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_lisp _lsd_slam_viewer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_lisp _lsd_slam_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lsd_slam_viewer_genlisp)
add_dependencies(lsd_slam_viewer_genlisp lsd_slam_viewer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lsd_slam_viewer_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer
)
_generate_msg_py(lsd_slam_viewer
  "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer
)

### Generating Services

### Generating Module File
_generate_module_py(lsd_slam_viewer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lsd_slam_viewer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lsd_slam_viewer_generate_messages lsd_slam_viewer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_py _lsd_slam_viewer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/lsd_slam/lsd_slam_viewer/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(lsd_slam_viewer_generate_messages_py _lsd_slam_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lsd_slam_viewer_genpy)
add_dependencies(lsd_slam_viewer_genpy lsd_slam_viewer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lsd_slam_viewer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lsd_slam_viewer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lsd_slam_viewer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lsd_slam_viewer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lsd_slam_viewer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lsd_slam_viewer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
