cmake_minimum_required(VERSION 2.8.3)
project(nodelet_tutorial_math)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES nodelet_math
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

## Create the nodelet tutorial library
add_library(nodelet_math src/plus.cpp)
target_link_libraries(nodelet_math ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_math ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS nodelet_math
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_math.xml plus.launch plus_default.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
