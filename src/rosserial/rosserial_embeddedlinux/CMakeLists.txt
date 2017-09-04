cmake_minimum_required(VERSION 2.8.3)
project(rosserial_embeddedlinux)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(DIRECTORY src/examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(PROGRAMS src/rosserial_embeddedlinux/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
