cmake_minimum_required(VERSION 2.8.3)
project(rosserial_client)

find_package(catkin REQUIRED)
catkin_package(
  CFG_EXTRAS rosserial_client-extras.cmake
)

catkin_python_setup()

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
install(PROGRAMS scripts/make_libraries
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rosserial_msgs REQUIRED)
  include_directories(src ${rosserial_msgs_INCLUDE_DIRS})
  catkin_add_gtest(float64_test test/float64_test.cpp)
  catkin_add_gtest(subscriber_test test/subscriber_test.cpp)
endif()
