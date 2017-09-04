cmake_minimum_required(VERSION 2.8.3)
project(actionlib_lutorials)

## 编译包依赖
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  # 与小乌龟相关的
  angles
  rosconsole
  turtlesim
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg FILES Velocity.msg )


## Generate actions in the 'action' folder
 add_action_files( DIRECTORY action FILES Fibonacci.action Averaging.action Shape.action)

## Generate added messages and services with any dependencies listed here
 generate_messages( DEPENDENCIES actionlib_msgs std_msgs )

catkin_package( CATKIN_DEPENDS actionlib_msg )

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# 斐波纳契 序列 运动效果生成 序列  服务器
 add_executable(fibonacciServer src/fibonacciServer.cpp)
 target_link_libraries(fibonacciServer  ${catkin_LIBRARIES} )
 add_dependencies(fibonacciServer ${actionlib_lutorials_EXPORTED_TARGETS})
 
# 斐波纳契 序列 运动效果生成 客户端 发送 目标序列
#                  编译生存的可执行文件名   源文件名
 add_executable(fibonacciClient src/fibonacciClient.cpp)
 # 链接库
 target_link_libraries(fibonacciClient  ${catkin_LIBRARIES} )
 # 依赖库
 add_dependencies(fibonacciClient ${actionlib_lutorials_EXPORTED_TARGETS})

# 求均值 标准差  服务
 add_executable(averagingServer src/averagingServer.cpp)
 # 链接库
 target_link_libraries(averagingServer  ${catkin_LIBRARIES} )
 # 依赖库
 add_dependencies(averagingServer ${actionlib_lutorials_EXPORTED_TARGETS})
 
 
 # 求均值 标准差  客户端
 add_executable(averagingClient src/averagingClient.cpp)
 # 链接库
 target_link_libraries(averagingClient  ${catkin_LIBRARIES} )
 # 依赖库
 add_dependencies(averagingClient ${actionlib_lutorials_EXPORTED_TARGETS})


##  小乌龟形状服务器 画正五边形状（可定义几边形状和半径大小） 可执行文件
add_executable(turte_shape_server src/shapeServer.cpp)
target_link_libraries(turte_shape_server ${catkin_LIBRARIES})
add_dependencies(turte_shape_server ${actionlib_lutorials_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 小乌龟形状客户端 可执行文件
add_executable(turte_shape_client src/shapeClient.cpp)
target_link_libraries(turte_shape_client ${catkin_LIBRARIES})
add_dependencies(turte_shape_client ${actionlib_lutorials_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


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

  ## Mark executables and/or libraries for installation
#   install(TARGETS shape_server shape_client
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_actionlib_lutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
