  #include "ros/ros.h"
  #include "std_msgs/String.h"
  //订阅话题的回调函数  类似中断函数
  //当消息到达chatter topic的时候就会被调用。消息是以共享指针形式传递
  void chatterCallback(const std_msgs::String::ConstPtr&  msg)
    {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
   
   int main(int argc, char **argv)
   { 
    ros::init(argc, argv, "hel_ros_listener_node");  //初始化ros系统 ，在roscore节点管理器注册节点
    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe("pub_hello_ros", 1000, chatterCallback);
    ros::Rate  rate(5);                //订阅频率
    ros::spin(); //给ros控制权（ros::spin() 无限权 占用较高cpu） 能够调用回调函数 类似 使能中断
     while(ros::ok()){
       ros::spinOnce();  //给ROS控制权  可以调用一次回调函数
       rate.sleep();
     }
     
    return 0;
    }