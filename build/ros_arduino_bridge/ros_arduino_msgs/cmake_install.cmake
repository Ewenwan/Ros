# Install script for directory: /home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs/msg" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/msg/AnalogFloat.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/msg/Analog.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/msg/ArduinoConstants.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/msg/Digital.msg"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/msg/SensorState.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs/srv" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/DigitalSetDirection.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/DigitalWrite.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/DigitalRead.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/ServoRead.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/ServoWrite.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/AnalogWrite.srv"
    "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/srv/AnalogRead.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/ros_arduino_bridge/ros_arduino_msgs/catkin_generated/installspace/ros_arduino_msgs-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/include/ros_arduino_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/share/common-lisp/ros/ros_arduino_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/ros_arduino_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/ros_arduino_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/ros_arduino_bridge/ros_arduino_msgs/catkin_generated/installspace/ros_arduino_msgs.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/ros_arduino_bridge/ros_arduino_msgs/catkin_generated/installspace/ros_arduino_msgs-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs/cmake" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/build/ros_arduino_bridge/ros_arduino_msgs/catkin_generated/installspace/ros_arduino_msgsConfig.cmake"
    "/home/ewenwan/ewenwan/catkin_ws/build/ros_arduino_bridge/ros_arduino_msgs/catkin_generated/installspace/ros_arduino_msgsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_arduino_msgs" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/ros_arduino_msgs/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

