/*
 * fibonacciclientprogress.cpp
 *
 *  Created on: Aug 5, 2016
 *      Author: unicorn
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <learning_actionlib/FibonacciAction.h>
using namespace learning_actionlib;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;
void doneCb(const actionlib::SimpleClientGoalState &state,
            const FibonacciResultConstPtr &result){
  ROS_INFO("finish in  state [%s]",state.toString().c_str());
  ROS_INFO("answer is %i",result->sequence.back());
  ros::shutdown();
}

void activeCb(){
  ROS_INFO("goal just become active");
}
void feedCb(const FibonacciFeedbackConstPtr &feedback){
  ROS_INFO("got length of fibonacci %lu",feedback->sequence.size());

}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_progress");
Client ac("fibonacci",true);
ROS_INFO("wait action server to start");
ac.waitForServer();
ROS_INFO("action server started,sending goal");

FibonacciGoal goal;
goal.order=21;

ac.sendGoal(goal,&doneCb,&activeCb,&feedCb);
ros::spin();


 return 0;
}

