 // This program publishes random velocity commands, using
 // a maximum linear velocity read from a parameter.
 #include <ros/ros.h>
 #include <geometry_msgs/Twist.h>
 #include <stdlib.h>

 int main( int argc , char** argv ) {
 ros::init ( argc , argv , "publish_velocity") ;
 ros::NodeHandle nh ;
 ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( "turtle1/cmd_vel", 1000) ;
 //生成产生随机数的种子点
 srand ( time (0) ) ; //初始化随机数 种子

// Get the maximum velocity parameter . ~代表私有参数 max_vel
  const std::string PARAM_NAME = "~max_vel" ;  //需要 设置参数值 rosparam set /publish_velocity/max_vel 0.1
  double maxVel;
  bool ok = ros::param::get (PARAM_NAME, maxVel) ;
  if (!ok) {
  ROS_FATAL_STREAM("Could not get parameter:"
  << PARAM_NAME) ;
  exit (1) ;
 }

  ros::Rate rate (2) ;
  while ( ros::ok () ) {
 // Create and send a random velocity command.
   geometry_msgs::Twist msg ;
   msg.linear.x = maxVel*double ( rand () )/double (RAND_MAX) ;  //线速度  为 0 到 1 之间的某个值 * maxVel
   msg.angular.z = 2*double ( rand () )/double (RAND_MAX) -1 ; //角速度  弧度制 为-1 到 1 之间的某个值
   //发布消息
   pub.publish(msg) ;
  // Wait untilit's time for another iteration .
  rate.sleep () ;
   }
 }