// This program periodical ly generates log messages at various severity levels .
#include <ros/ros.h>
#include <log4cxx/logger.h>   //日志消息头文件
int main (int argc,char **argv) {
// Initialize the ROS system and become a node .
 ros::init(argc, argv, "count_and_log");   //建立  count_and_log节点
 ros::NodeHandle nh;

 // Generate log messages of varying severi ty regularly .
 ros::Rate rate (5);
 //设置日志消息级别
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
   ros::console::g_level_lookup[ros::console::levels::Warn]); //设置级别 Debug 这个标识当然可以替换为 Info、Warn、Error 或者 Fatal。
   ros::console::notifyLoggerLevelsChanged();  //启用改变
   
   
 for(int i=1; ros::ok(); i++){
  
   ROS_DEBUG_STREAM_ONCE( "This appears only once. " );
   ROS_INFO_STREAM_ONCE( "This appears only once. " );
   ROS_WARN_STREAM_ONCE( "This appears only once. " );
   ROS_ERROR_STREAM_ONCE( "This appears only once. " );
   ROS_FATAL_STREAM_ONCE( "This appears only once. " );
   
   ROS_ERROR_STREAM_THROTTLE(1.0, "This appears every 1.0 seconds.") ;
   
   ROS_INFO("the count unmber is %d",i);
   
   ROS_DEBUG_STREAM("Counted?to?"<<i);
   if((i%3)==0){
       ROS_INFO_STREAM(i<<"?is?divisible?by?3.");
    }
   if((i%5)==0){
       ROS_WARN_STREAM(i<<"?is?divisible?by?5.");
    }
   if((i%10)==0){
       ROS_ERROR_STREAM(i<<"?is?divisible?by?10.");
    }
   if((i%20)==0){
       ROS_FATAL_STREAM(i<<"?is?divisible?by?20.");
    }
   rate.sleep();
   }
 }