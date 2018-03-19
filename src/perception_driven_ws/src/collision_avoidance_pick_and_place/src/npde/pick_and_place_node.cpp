/*
主　基于模式是别的　抓取放置　应用程序节点
*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

using namespace collision_avoidance_pick_and_place;

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  // 全局变量
  geometry_msgs::Pose box_pose;// 箱子位姿
  std::vector<geometry_msgs::Pose> pick_poses, place_poses;// 位姿态容器　抓取时的姿态　　放置时的姿态
  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Look into the pick_and_place_config:　'cfg' member of PickAndPlace to take notice of the parameters that
        are available for the rest of the program. */
  /* =========================================================================================*/
 
  // ros initialization　　初始化节点
  ros::init(argc,argv,"pick_and_place_node");

  ros::NodeHandle nh;//　节点句柄 
  ros::AsyncSpinner spinner(2);//多线程spin 开２个线程
  spinner.start();

  // creating pick and place application instance
  // 创建　应用类　实体对象 
  PickAndPlace application;

  // reading parameters　读取参数
  if(application.cfg.init())
  {
    ROS_INFO_STREAM("Parameters successfully read");
  }
  else
  {
    ROS_ERROR_STREAM("Parameters not found");
    return 0;
  }

  // marker publisher　发布　marker 消息
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // planning scene publisher　　规划发布器
  application.planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(
  		application.cfg.PLANNING_SCENE_TOPIC,1);

  // moveit interface
//     moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板本
  application.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroup(application.cfg.ARM_GROUP_NAME));
  application.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");

  // motion plan client 运动规划客户端　订阅规划服务器　的服务话题
  application.motion_plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(application.cfg.MOTION_PLAN_SERVICE);

  // transform listener　坐标变换监听
  application.transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());

  // marker publisher (rviz visualization)
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // target recognition client (perception)　目标识别　客户端
  application.target_recognition_client = nh.serviceClient<collision_avoidance_pick_and_place::GetTargetPose>(
		  application.cfg.TARGET_RECOGNITION_SERVICE);

  // grasp action client (vacuum gripper)　抓取行动　客户端
  application.grasp_action_client_ptr = GraspActionClientPtr(
		  new GraspActionClient(application.cfg.GRASP_ACTION_NAME,true));


  // 等待连接到　抓取行动服务发布者　waiting to establish connections
  while(ros::ok() &&
      !application.grasp_action_client_ptr->waitForServer(ros::Duration(3.0f)))//等待抓取行动服务
  {
    ROS_INFO_STREAM("Waiting for grasp action servers");
  }
  // 等待连接到　目标识别服务　　发布者
  if(ros::ok() && !application.target_recognition_client.waitForExistence(ros::Duration(3.0f)))//等待目标识别服务
  {
	  ROS_INFO_STREAM("Waiting for service'"<<application.cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }


  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/
  ROS_INFO_STREAM("Satrting Pick & Place Tasks ");

  // move to a "clear" position　　运动到　等候区 moveit实现
  application.move_to_wait_position();

  // turn off vacuum gripper　　　　　打开抓手  释放物体的状态
  application.set_gripper(false);

  // get the box position and orientation　　检测需要抓取的箱子　的　位姿
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "pick" the box　从当前位姿　到　抓取姿态　规划
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves　　　　　执行一系列位姿　到　抓取
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "place" the box　　　求解从抓取好箱子到放置位置　位姿
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves　　　　　　　　　　　　　　　　　放置物体
  application.place_box(place_poses,box_pose);

  // move back to the "clear" position　　　　　　　　　　　　　　返回　等候区
  application.move_to_wait_position();

  return 0;
}
