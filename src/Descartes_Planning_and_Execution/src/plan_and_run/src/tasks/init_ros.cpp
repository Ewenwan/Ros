/*
初始化ros系统
创建一个服客户端节点 来发生规划的路径点
marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);
可视化马卡消息（发送到rviz显示）
visualization_msgs::MarkerArray
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
  // 等待 服务器 出现
  if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}

