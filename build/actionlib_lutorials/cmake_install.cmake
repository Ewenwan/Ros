# Install script for directory: /home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/msg" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/msg/Velocity.msg")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/action" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/action/Fibonacci.action"
    "/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/action/Averaging.action"
    "/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/action/Shape.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/msg" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciAction.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciActionGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciActionResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciActionFeedback.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/FibonacciFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/msg" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingAction.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingActionGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingActionResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingActionFeedback.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/AveragingFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/msg" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeAction.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeActionGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeActionResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeActionFeedback.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeGoal.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeResult.msg"
    "/home/ewenwan/ewenwan/catkin_ws/devel/share/actionlib_lutorials/msg/ShapeFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/actionlib_lutorials/catkin_generated/installspace/actionlib_lutorials-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/include/actionlib_lutorials")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/share/common-lisp/ros/actionlib_lutorials")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/actionlib_lutorials")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ewenwan/ewenwan/catkin_ws/devel/lib/python2.7/dist-packages/actionlib_lutorials")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/actionlib_lutorials/catkin_generated/installspace/actionlib_lutorials.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/cmake" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/build/actionlib_lutorials/catkin_generated/installspace/actionlib_lutorials-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials/cmake" TYPE FILE FILES
    "/home/ewenwan/ewenwan/catkin_ws/build/actionlib_lutorials/catkin_generated/installspace/actionlib_lutorialsConfig.cmake"
    "/home/ewenwan/ewenwan/catkin_ws/build/actionlib_lutorials/catkin_generated/installspace/actionlib_lutorialsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_lutorials" TYPE FILE FILES "/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

