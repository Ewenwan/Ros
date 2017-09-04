cmake_minimum_required(VERSION 2.8.12)

project(simple_message)

find_package(catkin REQUIRED COMPONENTS roscpp industrial_msgs)

# Build static libs, to reduce dependency-chaining for industrial_robot_client
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)

# The simple_message library is designed to cross compile on Ubuntu
# and various robot controllers.  This requires conditionally compiling
# certain functions and headers.  The definition below enables compiling
# for a ROS node.
add_definitions(-DROS=1)           #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication

set(SRC_FILES src/byte_array.cpp
	src/simple_message.cpp
	src/smpl_msg_connection.cpp

	src/socket/simple_socket.cpp
	src/socket/udp_socket.cpp
	src/socket/udp_client.cpp
	src/socket/udp_server.cpp
	src/socket/tcp_socket.cpp
	src/socket/tcp_client.cpp
	src/socket/tcp_server.cpp

	src/message_handler.cpp
	src/message_manager.cpp
	src/ping_handler.cpp
	src/ping_message.cpp
	src/joint_data.cpp
	src/joint_feedback.cpp
	src/joint_traj_pt.cpp
	src/joint_traj_pt_full.cpp
	src/joint_traj.cpp
	src/robot_status.cpp

	src/messages/joint_message.cpp
	src/messages/joint_feedback_message.cpp
	src/messages/joint_traj_pt_message.cpp
	src/messages/joint_traj_pt_full_message.cpp
	src/messages/robot_status_message.cpp

	src/simple_comms_fault_handler.cpp)
					
set(UTEST_SRC_FILES test/utest.cpp test/utest_message.cpp)

# The simple message make file builds two libraries: simple_message and
# simple_message_byte_swapping.
#
# simple_message - is the default library.  This library should be used
# when the target for the simple message is the same endian (i.e. both
# big-endian or little-endian).  Intel based machines are little endian
#
# simple_message_byte_swapping - is an alternative library that can be used
# when the target for simple message is a DIFFERENT endian AND when the target
# target cannot perform byte swapping (as is the case for some industrial
# controllers).  This library performs byte swapping at the lowest load/unload
# levels.

# NOTE: The libraries generated this package are not included in the catkin_package
# macro because libraries must be explicitly linked in projects that depend on this
# package.  If this is not done (and these libraries were exported), then multiple
# library definitions (normal - simple_message and byteswapped - simple_message_bswap)
# are both included (this is bad).

catkin_package(
    CATKIN_DEPENDS roscpp industrial_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_dummy
    CFG_EXTRAS issue46_workaround.cmake
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)


# generate dummy library (we export it in catkin_package(..)), to force catkin
# to set up LIBRARY_DIRS properly.
# TODO: find out if LIBRARY_DIRS can be exported without dummy library target
add_custom_command(
  OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)
add_library(${PROJECT_NAME}_dummy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_dummy.cpp)
# unfortunately this will have to be installed, but the linker will remove it
# from the library dependencies of dependent targets, as it contains no symbols
# that can be imported from it.
install(TARGETS ${PROJECT_NAME}_dummy DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


# NOTE: All test files require TEST_PORT_BASE to be defined.  Defining different
# ports for each test executable allows them to run in parallel.

# DEFAULT LIBRARY (SAME ENDIAN)
add_library(simple_message ${SRC_FILES})
target_link_libraries(simple_message ${catkin_LIBRARIES})
add_dependencies(simple_message ${industrial_msgs_EXPORTED_TARGETS})

catkin_add_gtest(utest ${UTEST_SRC_FILES})
set_target_properties(utest PROPERTIES COMPILE_DEFINITIONS "TEST_PORT_BASE=51000")
target_link_libraries(utest simple_message)

# ALTERNATIVE LIBRARY (DIFFERENT ENDIAN)
add_library(simple_message_bswap ${SRC_FILES})
set_target_properties(simple_message_bswap PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
target_link_libraries(simple_message_bswap ${catkin_LIBRARIES})
add_dependencies(simple_message_bswap ${industrial_msgs_EXPORTED_TARGETS})

catkin_add_gtest(utest_byte_swapping ${UTEST_SRC_FILES})
set_target_properties(utest_byte_swapping PROPERTIES COMPILE_DEFINITIONS "TEST_PORT_BASE=52000")
target_link_libraries(utest_byte_swapping simple_message_bswap)

# ALTERNATIVE LIBRARY (64-bit floats)
add_library(simple_message_float64 ${SRC_FILES})
set_target_properties(simple_message_float64 PROPERTIES COMPILE_DEFINITIONS "FLOAT64")
target_link_libraries(simple_message_float64 ${catkin_LIBRARIES})
add_dependencies(simple_message_float64 ${industrial_msgs_EXPORTED_TARGETS})

catkin_add_gtest(utest_float64 ${UTEST_SRC_FILES})
set_target_properties(utest_float64 PROPERTIES COMPILE_DEFINITIONS "TEST_PORT_BASE=53000;FLOAT64")
target_link_libraries(utest_float64 simple_message_float64)

catkin_add_gtest(utest_udp ${UTEST_SRC_FILES})
set_target_properties(utest_udp PROPERTIES COMPILE_DEFINITIONS "TEST_PORT_BASE=55000;UDP_TEST")
target_link_libraries(utest_udp simple_message)

install(
    TARGETS simple_message simple_message_bswap simple_message_float64 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

