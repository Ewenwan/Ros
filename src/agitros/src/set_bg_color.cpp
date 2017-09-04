 // This program waits for a turtlesim to start up , and
  // changes its background color .
  #include <ros/ros.h>
  #include <std_srvs/Empty.h>
 
  int main( int argc , char** argv ) {
  ros::init ( argc , argv , "set_bg_color") ;
  ros::NodeHandle nh ;

  // Wait until the clear service is available , which
  // indicates that turtlesim has started up , and has
  // set the background color parameters .
   ros::service::waitForService ("clear") ;
 
  // Set the background color for turtlesim ,
  // overriding the default blue color .
   ros::param::set ("background_r" , 255) ;
   ros::param::set ("background_g" , 255) ;
   ros::param::set ("background_b" , 0) ;
 
 // Get turtlesim to pick up the new parameter values.
 //rosservice call /clear
   ros::ServiceClient clearClient = nh.serviceClient <std_srvs::Empty>("/clear") ;
   std_srvs::Empty srv ;
   clearClient.call (srv); //更新
 }