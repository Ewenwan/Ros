cmake_minimum_required(VERSION 2.8.12)

project(industrial_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs trajectory_msgs genmsg message_generation)

add_message_files(
  FILES
  DebugLevel.msg
  DeviceInfo.msg
  RobotMode.msg
  RobotStatus.msg
  ServiceReturnCode.msg
  TriState.msg)

add_service_files(
  FILES
  CmdJointTrajectory.srv
  GetRobotInfo.srv
  SetDrivePower.srv
  SetRemoteLoggerLevel.srv
  StartMotion.srv
  StopMotion.srv)

generate_messages(
  DEPENDENCIES trajectory_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs genmsg
)
