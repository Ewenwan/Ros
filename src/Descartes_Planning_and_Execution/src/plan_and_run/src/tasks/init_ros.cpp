/*
初始化ros系统
创建一个服客户端节点 来发S送 规划的路径点
marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);
可视化马卡消息（发送到rviz显示）
visualization_msgs::MarkerArray

等待 moveit 执行轨迹服务器 出现

*/
#include <plan_and_run/demo_application.h>

/* INIT ROS
  Goal:
    - Create a ros service client that will be used to send a robot path for execution.

  Hints:
*/

namespace plan_and_run
{

void DemoApplication::initRos()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // plan_and_run/demo_application.h  定义了类私有变量 节点 ros::NodeHandle nh_;
  // creating publisher for trajectory visualization
  // 可视化轨迹发布器
  marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

  /*  Fill Code:
   * Goal:
   * - Create a "moveit_msgs::ExecuteKnownTrajectory" client and assign it to the "moveit_run_path_client_"
   *    application variable.
   * Hint:
   * - Enter the service type moveit_msgs::ExecuteKnownTrajectory in between the "< >" arrow brackets of
   *   the "nh_.serviceClient" function call.
   */
  //moveit_run_path_client_;/* = nh_.serviceClient< [ COMPLETE HERE ] >(EXECUTE_TRAJECTORY_SERVICE,true); */
  // 客户端 
  moveit_run_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);

  // Establishing connection to server
  // 等待 moveit 执行轨迹服务器 出现
  if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit(-1);
  }

// 调用ac_需要在类构造函数实现部分 传递 ac_初始化列表
// demo_application.cpp
// 构造函数  带有　笛卡尔轨迹执行行动 初始化　列表
//DemoApplication::DemoApplication():ac_("joint_trajectory_action", true)
//{}

// 轨迹执行 行动 客户端
  // ac_("joint_trajectory_action", true);
  ROS_INFO_STREAM("Waiting for action server ...");
  ac_.waitForServer();

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
