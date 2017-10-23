   // This program t oggles between rotation and translation
  // commands, based on calls to a service .
  #include <ros/ros.h>
  #include <std_srvs/Empty.h>      //服务数据类型           
  #include <geometry_msgs/Twist.h> //发布小乌龟速度消息类型  
 
  bool forward = true ;
  bool toggleForward (
                      std_srvs::Empty::Request &req ,
                      std_srvs::Empty::Response &resp ) {
     forward = !forward ; //改变前进 旋转标志 在主循环中判断
     ROS_INFO_STREAM("Now sending " << ( forward ?"forward" : " rotate ") << "  commands.") ;
     return true ;
   }
  
   int main( int argc , char** argv ) {
   ros::init ( argc , argv , "change_forward_rotate") ; //初始化 注册节点  change_forward_rotate
   ros::NodeHandle nh ;
  
   // Register our service with the master .            服务名称                   回调函数
   ros::ServiceServer server = nh.advertiseService ("change_forward_rotate" , &toggleForward ) ;
  
   // Publish commands, using the latest value for forward ,
   // until the node shuts down.
   ros::Publisher pub = nh.advertise <geometry_msgs::Twist>("turtle1/cmd_vel" , 1000) ;//发布话题
   ros::Rate rate (2) ;
   while ( ros::ok () ) {
   geometry_msgs::Twist msg ;
   msg.linear.x = forward ? 1.0 : 0.0 ;// forward 为1 前进
   msg.angular.z = forward ? 0.0 : 1.0 ;// forward 为0 旋转
   pub.publish (msg) ;
   ros::spinOnce () ;
   rate.sleep () ;
    }
 }
