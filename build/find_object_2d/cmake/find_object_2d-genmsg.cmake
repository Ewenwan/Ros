# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "find_object_2d: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifind_object_2d:/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(find_object_2d_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg" NAME_WE)
add_custom_target(_find_object_2d_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "find_object_2d" "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg" "std_msgs/Float32MultiArray:std_msgs/MultiArrayDimension:std_msgs/Header:std_msgs/MultiArrayLayout"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(find_object_2d
  "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_object_2d
)

### Generating Services

### Generating Module File
_generate_module_cpp(find_object_2d
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_object_2d
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(find_object_2d_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(find_object_2d_generate_messages find_object_2d_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg" NAME_WE)
add_dependencies(find_object_2d_generate_messages_cpp _find_object_2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_object_2d_gencpp)
add_dependencies(find_object_2d_gencpp find_object_2d_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_object_2d_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(find_object_2d
  "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_object_2d
)

### Generating Services

### Generating Module File
_generate_module_lisp(find_object_2d
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_object_2d
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(find_object_2d_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(find_object_2d_generate_messages find_object_2d_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg" NAME_WE)
add_dependencies(find_object_2d_generate_messages_lisp _find_object_2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_object_2d_genlisp)
add_dependencies(find_object_2d_genlisp find_object_2d_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_object_2d_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(find_object_2d
  "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_object_2d
)

### Generating Services

### Generating Module File
_generate_module_py(find_object_2d
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_object_2d
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(find_object_2d_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(find_object_2d_generate_messages find_object_2d_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/msg/ObjectsStamped.msg" NAME_WE)
add_dependencies(find_object_2d_generate_messages_py _find_object_2d_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_object_2d_genpy)
add_dependencies(find_object_2d_genpy find_object_2d_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_object_2d_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_object_2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_object_2d
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(find_object_2d_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(find_object_2d_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_object_2d)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_object_2d
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(find_object_2d_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(find_object_2d_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_object_2d)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_object_2d\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_object_2d
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(find_object_2d_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(find_object_2d_generate_messages_py sensor_msgs_generate_messages_py)
endif()
