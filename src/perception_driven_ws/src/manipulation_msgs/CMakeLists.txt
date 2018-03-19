cmake_minimum_required(VERSION 2.8.3)
project(manipulation_msgs)
#依赖组合
set(MSG_DEPS
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  household_objects_database_msgs
  )
#编译依赖
find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})
#消息文件组合
set(
  MSG_FILES
  CartesianGains.msg
  ClusterBoundingBox.msg
  GraspableObjectList.msg
  GraspableObject.msg
  Grasp.msg
  GraspPlanningErrorCode.msg
  GraspResult.msg
  GripperTranslation.msg
  ManipulationPhase.msg
  ManipulationResult.msg
  PlaceLocation.msg
  PlaceLocationResult.msg
  SceneRegion.msg
)
#服务文件组合
set(
  SRV_FILES
  GraspPlanning.srv
)
#行动文件组合
set(
  ACTION_FILES
  GraspPlanning.action)
#添加需要自动编译的文件
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})
add_action_files(DIRECTORY action FILES ${ACTION_FILES})
#添加生成 依赖
generate_messages(DEPENDENCIES ${MSG_DEPS})
#包依赖
catkin_package(
  CATKIN_DEPENDS message_generation ${MSG_DEPS}
  )
