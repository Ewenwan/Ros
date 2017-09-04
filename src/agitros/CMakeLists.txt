cmake_minimum_required(VERSION 2.8.3)
project(agitros)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msg)
## is used, also find other catkin packages
##                         cpp文件依赖 发布小乌龟消息依赖库  订阅turtlesim pose消息依赖库  消息文件 .msg 运行依赖
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs turtlesim message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
## if add msg file , add generate_messages() too
 add_message_files(
    FILES
#   Message1.msg
#   Message2.msg
    Num.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
    FILES
#   Service1.srv
#   Service2.srv
    AddTwoInts.srv 
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agitros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/agitros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/agitros_node.cpp)
# add executable file  add_executable(executable-name source-files)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


# locate the build executable cpp file and assigned the project_node_name 
 add_executable(helloROS src/HelloWorld.cpp)                #节点类型名字
 target_link_libraries(helloROS ${catkin_LIBRARIES})

#  a node which can publish the turtle veloity  information to the topic
 add_executable(pubVel src/PubVel.cpp)
 target_link_libraries(pubVel ${catkin_LIBRARIES})

# a node which can subscribe the topic published with the turtle pose inormtion 
  add_executable(subPose src/SubPose.cpp)
 target_link_libraries(subPose ${catkin_LIBRARIES})

# log 
  add_executable(countlog src/CountAndLog.cpp)
  target_link_libraries(countlog ${catkin_LIBRARIES})

# a anony node which has an anonymous name that can run a lot copys at a same time
  add_executable(anony_dy src/Anony.cpp)
  target_link_libraries(anony_dy  ${catkin_LIBRARIES})

# a reverse cmd vel node 
  add_executable(reverseCmdVel src/ReverseCmdVel.cpp)
  target_link_libraries(reverseCmdVel  ${catkin_LIBRARIES})
  
# set background color  
  add_executable(setBgColor src/set_bg_color.cpp)
  target_link_libraries(setBgColor  ${catkin_LIBRARIES})
  
  # a reverse cmd vel node 
  add_executable(pubVelWMaxVel src/PubVel_with_maxVel.cpp)
  target_link_libraries(pubVelWMaxVel  ${catkin_LIBRARIES})
  
  # a service client spawn turtle node 
  add_executable(spawnTurtle src/spawnTurtle.cpp)
  target_link_libraries(spawnTurtle ${catkin_LIBRARIES})
  
  # a service server change turtle forward and rotate node 
  add_executable(changeForwardRotate src/changeForwardRotate.cpp)
  target_link_libraries(changeForwardRotate ${catkin_LIBRARIES})
  
  
  
  
##beginner_tutorials 
  
 # a publisher node which pub hello ros
  add_executable(pub_Hello_ros src/talker.cpp)
  target_link_libraries(pub_Hello_ros ${catkin_LIBRARIES})
 
 # a subscribe node which sub hello ros topic
  add_executable(sub_Hello_ros src/listener.cpp)
  target_link_libraries(sub_Hello_ros ${catkin_LIBRARIES})
 
 
  # a server node which can  calculate the sum of two numbers
  add_executable(addTwoIntsServ src/add_two_ints_server.cpp)
  target_link_libraries(addTwoIntsServ ${catkin_LIBRARIES})
  
    # a server node which can  calculate the sum of two numbers
  add_executable(addTwoIntsClient src/add_two_ints_client.cpp)
  target_link_libraries(addTwoIntsClient ${catkin_LIBRARIES})
 
  

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agitros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
