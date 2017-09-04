cmake_minimum_required(VERSION 2.8.3)
project(rosserial_test)

find_package(catkin REQUIRED COMPONENTS roscpp rosserial_client rosserial_msgs rosserial_python rosserial_server rostest std_msgs)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  # Generate a client library for the test harnesses to use.
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/include/rosserial
    COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh rosrun rosserial_test generate_client_ros_lib ${PROJECT_BINARY_DIR}/include
  )
  add_custom_target(${PROJECT_NAME}_rosserial_lib DEPENDS ${PROJECT_BINARY_DIR}/include/rosserial)
  add_dependencies(${PROJECT_NAME}_rosserial_lib ${catkin_EXPORTED_TARGETS})

  include_directories(
    include ${PROJECT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS}
  )

  # Helper for building and linking test executables.
  function(add_rosserial_test_executable test_source)
    set(target ${PROJECT_NAME}_${test_source})
    add_executable(${target} EXCLUDE_FROM_ALL src/${test_source})
    add_dependencies(${target} ${PROJECT_NAME}_rosserial_lib)
    target_link_libraries(${target} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
    add_dependencies(tests ${target})
  endfunction()

  add_rosserial_test_executable(publish_subscribe)

  add_rostest(test/rosserial_server_socket.test)
  add_rostest(test/rosserial_server_serial.test)
  add_rostest(test/rosserial_python_socket.test)
  # Disabled due to reconnect logic in rosserial_python not being robust enough.
  # add_rostest(test/rosserial_python_serial.test)
endif()
