// This program starts with an anonymous name, which
// allows multiple copies to execute at the same time ,
// without needing to manually create distinct names
// for each of them.
// 该带有匿名名称的节点可以同时启动多个
#include <ros/ros.h>
int main ( int argc, char **argv ) {
ros::init ( argc, argv, "anon",
ros::init_options::AnonymousName );
ros::NodeHandle nh ;
ros::Rate rate (1) ;
while (ros::ok( )) {
  ROS_INFO_STREAM("This message is from :"
     << ros::this_node::getName ( ));
    rate.sleep( );
   }
 }