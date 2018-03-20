cmake_minimum_required(VERSION 2.8.3)
project(plan_and_run)
add_definitions(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_planner
  descartes_moveit
  descartes_utilities
  moveit_msgs
  moveit_ros_planning_interface
  ur5_demo_descartes
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_and_run
#  CATKIN_DEPENDS descartes_core descartes_planner trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(${PROJECT_NAME}
  src/demo_application.cpp
  src/tasks/init_ros.cpp
  src/tasks/init_descartes.cpp
  src/tasks/move_home.cpp
  src/tasks/generate_trajectory.cpp
  src/tasks/plan_path.cpp
  src/tasks/run_path.cpp
  src/tasks/load_parameters.cpp  
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(plan_and_run_node src/plan_and_run_node.cpp)
target_link_libraries(plan_and_run_node ${PROJECT_NAME} ${catkin_LIBRARIES})



