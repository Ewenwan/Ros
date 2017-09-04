// This program subscribes to turtle1/pose and shows its messages on the screen .
#include <ros/ros.h>         //ros系统头文件    
#include <turtlesim/Pose.h>  //订阅消息的 包名称和类型
#include <iomanip>           // for std::setprecision and std::fixed

// A callback function . Executed each time a new pose message arrives .
// 回调函数 接收消息  类似 对应敲门声的 终端程序（去开门 拿外卖/快递/艳遇。。。）
void poseMessageReceived ( const turtlesim::Pose& msg ) {
   /*
    ROS_INFO_STREAM( std::setprecision(2) << std::fixed
    << " position =(" << msg.x << " , " << msg.y << " ) "
    << " direction =" << msg.theta
    << "linear speed = "  << msg.linear_velocity
    << "angular speed = " << msg.angular_velocity) ;
    */
   ROS_INFO("Position = (%0.2f, %0.2f), direction = %0.2f, linear speed = %0.2f angular speed = %0.2f", msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity);
}

int main ( int argc , char** argv ) {
 // 初始化ros系统成为一个节点 .
 ros::init ( argc , argv , "subscribe_to_pose" ) ;
 ros::NodeHandle nh ;             //注册节点
 //创建一个订阅者sub     节点句柄         话题            缓存区    函数指针   &callbackfunc 得到
 ros::Subscriber sub = nh.subscribe( "turtle1/pose", 1000, &poseMessageReceived ) ;
 //休息
 //给ROS控制权 
  //ros::spin();     //给ROS控制权  订阅速度过快  占用cpu
 ros::Rate rate (1); //每秒订阅 3次 消息
 while ( ros::ok() ) {
    ros::spinOnce();  //给ROS控制权  可以调用一次回调函数
    rate.sleep();
 }

}