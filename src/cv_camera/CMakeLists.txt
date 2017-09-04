cmake_minimum_required(VERSION 2.8.3)
project(cv_camera)

find_package(catkin REQUIRED COMPONENTS
  image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager rostest roslint)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cv_camera
  CATKIN_DEPENDS image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager
  DEPENDS opencv2
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(cv_camera src/capture.cpp src/driver.cpp)
add_library(cv_camera_nodelet src/cv_camera_nodelet.cpp)
  

## Declare a cpp executable
add_executable(cv_camera_node src/cv_camera_node.cpp)
add_dependencies(cv_camera_node cv_camera)
add_dependencies(cv_camera_nodelet cv_camera)

## Specify libraries to link a library or executable target against
target_link_libraries(cv_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

target_link_libraries(cv_camera_nodelet
  ${catkin_LIBRARIES}
  cv_camera
  )

target_link_libraries(cv_camera_node
  ${catkin_LIBRARIES}
  cv_camera
  )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS cv_camera cv_camera_nodelet cv_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# add xml file
install(FILES cv_camera_nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


#############
## Testing ##
#############

roslint_cpp(
  src/capture.cpp src/cv_camera_node.cpp src/cv_camera_nodelet.cpp src/driver.cpp
  )

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(test_cv_camera test/cv_camera.test
    test/test_cv_camera.cpp)
  target_link_libraries(test_cv_camera ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(test_cv_camera_no_yaml test/no_yaml.test
    test/test_cv_camera_no_yaml.cpp)
  target_link_libraries(test_cv_camera_no_yaml ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
