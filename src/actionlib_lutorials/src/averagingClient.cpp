/*
 * faveragingClient.cpp
 *
 *  Created on: March 16, 2017
 *      Author: EwenWan
 */
#include <ros/ros.h> //ros系统文件
#include <actionlib/client/simple_action_client.h>//系统本身的 简单运动服务器库
#include <actionlib/client/terminal_state.h>      //系统本身的 终端状态库
#include <actionlib_lutorials/AveragingAction.h>  //catkin_make 生成的库文件
#include <boost/thread.hpp>
using namespace actionlib_lutorials;//使用  包名定义的actionlib_lutorials名字空间 下面一些类可省去 名字空间
typedef actionlib::SimpleActionClient<AveragingAction> Client; //定义运动客户端 请求

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ////初始化ros系统 注册节点
  ros::init(argc, argv, "averagingClient");
  
  std::string Servername="averagingServer";
 // 定义客户端  链接的服务器名字 
  Client ac(Servername);
  boost::thread spin_thread(&spinThread);
  //ROS_INFO("Waiting for action server to start.");
  ROS_INFO_STREAM("Waiting for action server " << Servername << " to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  AveragingGoal goal;
  goal.samples = 100;
  ac.sendGoal(goal);  //发送目标

 //完成标志
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}