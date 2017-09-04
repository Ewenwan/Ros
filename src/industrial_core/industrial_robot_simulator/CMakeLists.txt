cmake_minimum_required(VERSION 2.8.12)

project(industrial_robot_simulator)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs
  control_msgs trajectory_msgs)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs control_msgs trajectory_msgs
)

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch
    DEPENDENCIES
      joint_trajectory_action)
endif()

#############
## Install ##
#############


install(PROGRAMS industrial_robot_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
