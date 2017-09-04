#include <ros/ros.h>//ros系统文件
#include <actionlib/client/simple_action_client.h>//系统本身的 简单运动服务器库
#include <actionlib/client/terminal_state.h>      //系统本身的 终端状态库
#include <actionlib_lutorials/ShapeAction.h>      //catkin_make 生成的库文件

using namespace actionlib_lutorials;//使用  包名定义的actionlib_lutorials名字空间 下面一些类可省去 名字空间
typedef actionlib::SimpleActionClient<ShapeAction> Client; //定义运动客户端 请求

int main (int argc, char **argv)
{
  //////初始化ros系统 注册节点
  ros::init(argc, argv, "test_shape"); 

  // create the action client
  // true causes the client to spin it's own thread
  // 定义客户端  链接的服务器名字
  std::string Servername="turtle_shape";
   Client ac(Servername, true); 

  ROS_INFO_STREAM("Waiting for action server " << Servername << " to start.");
  //ROS_INFO("Waiting for action server %s to start.",Servername.toString().c_str());
  
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action 
  ShapeGoal goal;
  goal.edges = 6;    //goal.edges = 5 正五边形状
  goal.radius = 3.3; //半径大小
  ac.sendGoal(goal);  //发送目标
  
  //wait for the action to return
   //完成标志
  bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

  if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
  else  
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}