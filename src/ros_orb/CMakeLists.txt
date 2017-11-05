cmake_minimum_required(VERSION 2.8.3)

project(ros_orb)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
## 自己 oRB_SLAM2/Examples/ROS/ORB_SLAM2 工程地址
set(CODE_SOURCE_DIR /home/ewenwan/ewenwan/learn/vSLAM/test/vSLAM/ch9project/oRB_SLAM2/Examples/ROS/ORB_SLAM2)
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH 
# eigen3
${CODE_SOURCE_DIR}/../../../cmake_modules
# 添加cmake模块以使用g2o ceres库
${PROJECT_SOURCE_DIR}/cmake_modules 
)

#find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenCV 3.0 QUIET) #support opencv3
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

find_package( PCL 1.7 REQUIRED )

catkin_package()

include_directories(
${CODE_SOURCE_DIR}
${CODE_SOURCE_DIR}/../../../
${CODE_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(LIBS
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${Pangolin_LIBRARIES}
${CODE_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
#g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
${CODE_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${CODE_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera 单目相机
add_executable(mono
src/ros_mono.cc
)
target_link_libraries(mono
${LIBS}
)
# 单目相机 Augmented Reality 增强现实
#add_executable(monoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)
#target_link_libraries(mono
#${LIBS}
#)

# Node for RGB-D camera 深度相机
add_executable(rgbd
src/ros_rgbd.cc
)
target_link_libraries(rgbd
${LIBS}
)

# Node for stereo camera 双目立体相机
add_executable(stereo
src/ros_stereo.cc
)
target_link_libraries(stereo
${LIBS}
)


