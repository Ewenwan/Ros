  #include "ros/ros.h"
  #include "std_msgs/String.h" 
  #include <sstream>

 int main(int argc, char **argv)
{
     ros::init(argc, argv, "pub_hello_ros_node"); //初始化ros系统 ，在roscore节点管理器注册节点
     ros::NodeHandle nh;               //节点句柄
     ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("pub_hello_ros", 1000);//节点创建一个发布者
     ros::Rate  rate(5);              //发布频率
     int count = 0;                   //计数
     while (ros::ok())
     {
      std_msgs::String msg;           //消息类型变量
      std::stringstream ss;           //字符串输出流
      ss << "hello ROS at: " << count;
      msg.data = ss.str();            //生成消息
      ROS_INFO("%s", msg.data.c_str());//产生日至消息  输出之屏幕
      chatter_pub.publish(msg);        //发布话题
      //  ros::spinOnce();               //一般订阅者(需要回调函数）需要 ros::spinOnce();  给ros控制权
      rate.sleep();                    //其他时间休息
      ++count;
     }
   return 0;
 }