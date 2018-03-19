cmake_minimum_required(VERSION 2.8.3)
project(collision_avoidance_pick_and_place)
#add_definitions (-std=c++11)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  moveit_msgs
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
  robot_io
  object_manipulation_msgs
  actionlib
  industrial_robot_client
  industrial_robot_simulator
  std_msgs
  shape_msgs
  pcl_ros
  pcl_msgs
  tf_conversions
  message_generation
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#######################################
## Declare ROS messages and services ##
#######################################
## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetTargetPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES   
		std_msgs
		shape_msgs
		pcl_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		roscpp
		moveit_msgs
		moveit_ros_manipulation
		moveit_ros_move_group
		moveit_ros_planning_interface
		robot_io
		object_manipulation_msgs
		actionlib
		industrial_robot_client
		industrial_robot_simulator
		std_msgs
		shape_msgs
		pcl_ros
		pcl_msgs
		tf_conversions
		message_runtime
)

###########
## Build ##
###########
# 物体识别服务　节点
add_executable(target_recognition_service src/services/target_recognition_service.cpp)
add_dependencies(target_recognition_service ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(target_recognition_service ${catkin_LIBRARIES})
# 产生仿真障碍物点云数据　节点
add_executable(generate_cloud_node src/nodes/generate_point_cloud.cpp)
add_dependencies(generate_cloud_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(generate_cloud_node ${catkin_LIBRARIES})
# 主应用节点
add_executable(pick_and_place_node 
	src/nodes/pick_and_place_node.cpp
	src/tasks/move_to_wait_position.cpp
	src/tasks/set_gripper.cpp
	src/tasks/reset_world.cpp
	src/tasks/set_attached_object.cpp
	src/tasks/detect_box_pick.cpp
	src/tasks/create_pick_moves.cpp
  src/tasks/create_place_moves.cpp
	src/tasks/pickup_box.cpp
	src/tasks/place_box.cpp	
	src/tasks/create_motion_plan.cpp
	src/utilities/pick_and_place_utilities.cpp)
add_dependencies(pick_and_place_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pick_and_place_node ${catkin_LIBRARIES})


	
