cmake_minimum_required(VERSION 2.8.12)

project(industrial_utils)

find_package(catkin REQUIRED COMPONENTS roscpp urdf)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    CATKIN_DEPENDS roscpp urdf
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/utils.cpp src/param_utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})



if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest_inds_utils test/utest.cpp)
    target_link_libraries(utest_inds_utils ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()


install(
    TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
