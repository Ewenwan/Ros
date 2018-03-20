cmake_minimum_required(VERSION 2.8.3)
project(myworkcell_core)

add_compile_options(-std=c++11) # 支持c++ 11 新标准


# 包编译依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  fake_ar_publisher#依赖其他的包
  message_generation
  geometry_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

# 信息生成头文件
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

#添加服务文件
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LocalizePart.srv
#   Service1.srv
#   Service2.srv
 )

# 行动文件
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#信息生成 依赖
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

#包运行依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myworkcell_core
  CATKIN_DEPENDS 
    roscpp
    fake_ar_publisher#运行依赖 其他的包
    message_runtime   
    geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/myworkcell_core.cpp
# )

# 可视化marker 
add_executable(vision_node src/vision_node.cpp)
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_node ${catkin_LIBRARIES})

# 提供获取目标位置 服务的节点 订阅话题监控 服务
add_executable(ARserver_node src/ARserver.cpp) # 可执行文件
#依赖
add_dependencies(ARserver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#连接
target_link_libraries(ARserver_node ${catkin_LIBRARIES})

# 客户端 请求获取目标位置的服务
add_executable(ARclient_node src/ARclient.cpp) # 可执行文件
#依赖
add_dependencies(ARclient_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#连接
target_link_libraries(ARclient_node ${catkin_LIBRARIES})

# adv_descartes　高级笛卡尔轨迹规划 提供从轨迹点csv文件生成轨迹点的服务　 
add_executable(adv_descartes_node src/adv_descartes_node.cpp)
add_dependencies(adv_descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(adv_descartes_node ${catkin_LIBRARIES})

# 　高级笛卡尔应用节点
add_executable(adv_myworkcell_node src/adv_myworkcell_node.cpp)
add_dependencies(adv_myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(adv_myworkcell_node ${catkin_LIBRARIES})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_myworkcell_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
