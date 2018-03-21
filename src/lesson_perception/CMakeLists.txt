cmake_minimum_required(VERSION 2.8.3)
project(lesson_perception)
#add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf_conversions
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

find_package(PCL REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
CATKIN_DEPENDS 
  roscpp
  sensor_msgs
  tf
  tf_conversions
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(perception_node src/perception_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lesson_perception_node lesson_perception_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(perception_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


