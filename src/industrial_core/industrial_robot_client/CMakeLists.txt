cmake_minimum_required(VERSION 2.8.12)

project(industrial_robot_client)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
  control_msgs trajectory_msgs simple_message actionlib_msgs actionlib 
  urdf industrial_msgs industrial_utils)

find_package(Boost REQUIRED COMPONENTS system thread)

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)           #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries

set(SRC_FILES src/joint_relay_handler.cpp
              src/robot_status_relay_handler.cpp
              src/joint_trajectory_downloader.cpp
              src/joint_trajectory_streamer.cpp
              src/joint_trajectory_interface.cpp
              src/robot_state_interface.cpp
              src/utils.cpp)

# NOTE: The libraries generated this package are not included in the catkin_package
# macro because libraries must be explicitly linked in projects that depend on this
# package.  If this is not done (and these libraries were exported), then multiple
# library definitions (normal - industrial_robot_client and byteswapped.
# industrial_robot_client_bswap) are both included (this is bad).
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs
      simple_message actionlib_msgs actionlib urdf industrial_msgs
      industrial_utils
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


# This library depends on the simple_message library, which is available
# in two formats to support endian differences between robot & PC.
#
# As in simple_message, two libraries are created: one that does direct
# passthrough of binary numeric data to the robot controller, and one that
# uses byte-swapping to account for endian mis-matches.
#
# Due to catkin dependency limitations, higher-level code must
# explicitly link to the desired industrial_robot_client library:
#    target_link_libraries(my_node industrial_robot_client)
#      or
#    target_link_libraries(my_node industrial_robot_client_bswap)
#

add_library(industrial_robot_client ${SRC_FILES})
target_link_libraries(industrial_robot_client simple_message)

add_library(industrial_robot_client_bswap ${SRC_FILES})
target_link_libraries(industrial_robot_client_bswap simple_message_bswap)

# The following executables(nodes) are for applications where the robot
# controller and pc have the same byte order (i.e. byte swapping NOT
# required)

add_executable(robot_state
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state 
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})

add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface 
  industrial_robot_client 
  simple_message
  ${catkin_LIBRARIES})

add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface 
  industrial_robot_client 
  simple_message
  ${catkin_LIBRARIES})

# The following executables(nodes) are for applications where the robot
# controller and pc have different same byte order (i.e. byte swapping IS
# required)

add_executable(robot_state_bswap
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state_bswap 
  industrial_robot_client_bswap 
  simple_message_bswap
  ${catkin_LIBRARIES})

add_executable(motion_streaming_interface_bswap
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface_bswap 
  industrial_robot_client_bswap  
  simple_message_bswap
  ${catkin_LIBRARIES})

add_executable(motion_download_interface_bswap
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface_bswap 
  industrial_robot_client_bswap  
  simple_message_bswap
  ${catkin_LIBRARIES})

# The following executables(nodes) interface with the robot controller
# at a higher level so there is no need to create two versions (one with
# byte swapping, one without)

add_executable(joint_trajectory_action 
  src/generic_joint_trajectory_action_node.cpp
  src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action 
  industrial_robot_client ${catkin_LIBRARIES})

##########
## Test ##
##########
# Testing - Only performed on normal (non byte swapped library)
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest_robot_client test/utest.cpp)
  target_link_libraries(utest_robot_client
    industrial_robot_client
    ${catkin_LIBRARIES})
endif()

# ROS launch testing
## ROS launch test should be enabled when launch parameters are supported,
## see details below:
## robot_interface_streaming.launch: 'robot_ip'
## robot_state_visualize.launch]: 'urdf_path'
## robot_interface_download.launch]: 'robot_ip'
##find_package(roslaunch)
##roslaunch_add_file_check(launch)

install(
    TARGETS industrial_robot_client industrial_robot_client_bswap
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS 
        robot_state 
        robot_state_bswap 
        motion_streaming_interface 
        motion_download_interface 
        motion_streaming_interface_bswap 
        motion_download_interface_bswap 
        joint_trajectory_action 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

