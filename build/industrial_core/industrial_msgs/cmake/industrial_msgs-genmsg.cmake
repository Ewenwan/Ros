# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "industrial_msgs: 6 messages, 6 services")

set(MSG_I_FLAGS "-Iindustrial_msgs:/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg;-Itrajectory_msgs:/opt/ros/indigo/share/trajectory_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(industrial_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv" "trajectory_msgs/JointTrajectoryPoint:industrial_msgs/ServiceReturnCode:std_msgs/Header:trajectory_msgs/JointTrajectory"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg" "industrial_msgs/TriState:std_msgs/Header:industrial_msgs/RobotMode"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv" "industrial_msgs/ServiceReturnCode"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv" "industrial_msgs/ServiceReturnCode:industrial_msgs/DebugLevel"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv" "industrial_msgs/ServiceReturnCode"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv" "industrial_msgs/DeviceInfo:industrial_msgs/ServiceReturnCode"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv" "industrial_msgs/ServiceReturnCode"
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg" ""
)

get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg" NAME_WE)
add_custom_target(_industrial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_msgs" "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)

### Generating Services
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_cpp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
)

### Generating Module File
_generate_module_cpp(industrial_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(industrial_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(industrial_msgs_generate_messages industrial_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_cpp _industrial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_msgs_gencpp)
add_dependencies(industrial_msgs_gencpp industrial_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_msg_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)

### Generating Services
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)
_generate_srv_lisp(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
)

### Generating Module File
_generate_module_lisp(industrial_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(industrial_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(industrial_msgs_generate_messages industrial_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_lisp _industrial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_msgs_genlisp)
add_dependencies(industrial_msgs_genlisp industrial_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_msg_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)

### Generating Services
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)
_generate_srv_py(industrial_msgs
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv"
  "${MSG_I_FLAGS}"
  "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg;/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
)

### Generating Module File
_generate_module_py(industrial_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(industrial_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(industrial_msgs_generate_messages industrial_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg" NAME_WE)
add_dependencies(industrial_msgs_generate_messages_py _industrial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_msgs_genpy)
add_dependencies(industrial_msgs_genpy industrial_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(industrial_msgs_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(industrial_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(industrial_msgs_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(industrial_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(industrial_msgs_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(industrial_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
