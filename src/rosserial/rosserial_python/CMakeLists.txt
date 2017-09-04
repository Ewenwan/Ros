cmake_minimum_required(VERSION 2.8.3)
project(rosserial_python)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(PROGRAMS nodes/serial_node.py nodes/message_info_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
