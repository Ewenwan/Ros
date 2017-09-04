cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_firmware)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
