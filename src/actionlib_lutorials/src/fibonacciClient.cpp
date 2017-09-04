/*
 * fibonacciClient.cpp
 *
 *  Created on: March 16, 2017
 *      Author: EwenWan
 */
#include <ros/ros.h>  //ros系统文件
#include <actionlib/client/simple_action_client.h> //系统本身的 简单运动服务器库
#include <actionlib/client/terminal_state.h>       ////系统本身的 终端状态库
#include <actionlib_lutorials/FibonacciAction.h>   //添加actionlib_lutorials/FibonacciAction.h 文件 catkin_make 生成的库文件
using namespace actionlib_lutorials;//使用  包名定义的actionlib_lutorials名字空间 下面一些类可省去 名字空间
//actionlib::SimpleActionServer 来自<actionlib/server/simple_action_server.h>
//actionlib_lutorials::FibonacciAction 来自<actionlib_lutorials/FibonacciAction.h>
typedef actionlib::SimpleActionClient<FibonacciAction> Client; //定义运动客户端 请求

int main (int argc, char **argv)
{
ros::init(argc, argv, "test_fibonacci");//初始化ros系统 注册节点
// 定义客户端  链接的服务器名字   自动开始线程      
Client ac("fibonacci",true);
ROS_INFO("wait action server to start");
ac.waitForServer();
ROS_INFO("action server started,sending goal");

FibonacciGoal goal;
goal.order=20;
//发送 序列 目标 次序
ac.sendGoal(goal);
//完成标志
bool finish_before_timeout=ac.waitForResult(ros::Duration(30));

//完成了
if(finish_before_timeout){
  actionlib::SimpleClientGoalState state=ac.getState();
  ROS_INFO("action finished :%s",state.toString().c_str());

}
//未完成
else{
  ROS_INFO("action did not finished");
}
 return 0;
}


