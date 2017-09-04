# Install script for directory: /home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/ewenwan/ewenwan/catkin_ws/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs/msg" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DebugLevel.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/DeviceInfo.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotMode.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/RobotStatus.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/ServiceReturnCode.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/msg/TriState.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs/srv" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/CmdJointTrajectory.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/GetRobotInfo.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetDrivePower.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/SetRemoteLoggerLevel.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StartMotion.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/srv/StopMotion.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/industrial_core/industrial_msgs/catkin_generated/installspace/industrial_msgs-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/include/industrial_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/share/common-lisp/ros/industrial_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/industrial_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/industrial_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/industrial_core/industrial_msgs/catkin_generated/installspace/industrial_msgs.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/industrial_core/industrial_msgs/catkin_generated/installspace/industrial_msgs-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs/cmake" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/build/industrial_core/industrial_msgs/catkin_generated/installspace/industrial_msgsConfig.cmake"
    "/home/ewenwan/ewenwan/catkin_ws/build/industrial_core/industrial_msgs/catkin_generated/installspace/industrial_msgsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/industrial_msgs" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/industrial_msgs/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

