#include <ros/ros.h>               //ROS 库文件
#include <geometry_msgs/Twist.h>   //发布给小乌龟（turtlesim仿真器）的消息类型头文件库
#include <stdlib.h>                //标准库 产生rand()随机数

int main( int argc , char** argv ) {

ros::init( argc , argv , "publish_velocity" );  //初始化ROS系统  定义默认节点名字
ros::NodeHandle nh ;                            //在节点管理器中注册节点
//在节点上建立一个发布者 pub         发布的消息类型   发布的消息到话题（无前\ 相对名称）   消息序列的大小数据缓存区大小
//ros::Publisher pub = node_handle.advertise<message_type>(topic_name, queue_size);
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>( "turtle1/cmd_vel" , 1000 ) ;  //话题不能搞错了 多加空格也不行

//如果你想从同一个节点发布关于多个话题的消息,你需要为个话题创建一个独立的 ros::Publisher 对象。
//生成产生随机数的种子点
srand(time(0)) ;       //Seed the random number generator
ros::Rate rate (1);    //发布频率   控制 消息发布 频率   这个对象控制循环运行速度
while ( ros::ok() ) {  //ros::ok() 节点是否停止工作的检查  rosnode kill/终止信号(Ctrl-C)
     //定义消息变量 并以随机数赋值
     //若消息类型为数组类型(在rosmsg中用方括号示)在C++代码中是通过STL向量实现的。
     geometry_msgs::Twist msg;  
     msg.linear.x =      double( rand() ) / double(RAND_MAX);        //线速度  为 0 到 1 之间的某个值
     msg.angular.z = 2 * double( rand() ) / double(RAND_MAX) - 1;    //角速度  弧度制 为-1 到 1 之间的某个值
     //发布消息
     pub.publish( msg ) ;
     //在控制台显示发布的内容
     //演示了在输出中除了插入字符串还可以插入其他数据的能力
     ROS_INFO_STREAM( "Sending random velocity command : "            //ROS  information stream流
     << " linear speed = " << msg.linear.x << " angular speed = " << msg.angular.z);
     //休眠等待
     rate.sleep();
    }

}