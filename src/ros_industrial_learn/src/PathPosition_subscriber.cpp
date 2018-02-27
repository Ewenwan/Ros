#include <ros/ros.h>
#include <ros_industrial_learn/PathPosition.h>//包含自动生成的 自定义消息头文件

void positionCallback(const ros_industrial_learn::PathPosition& msg)//常亮引用 不用复制（节省时间）
{
   ROS_INFO("New position: %.1f,%.1f,%.1f", msg.x, msg.y,
    msg.angle * 180.0 / 3.141592);
}

int main(int argc, char* argv[])
{
ros::init(argc, argv, "PathPosition_sub_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄  

//创建一个订阅者sub     节点句柄         话题     缓存区    函数指针   &callbackfunc 得到
ros::Subscriber sub = node.subscribe("position", 1000, positionCallback);

 //给ROS控制权 
  //ros::spin();     //给ROS控制权  订阅速度过快  占用cpu
ros::Rate loop_rate (1); //每秒订阅 1次 消息
while ( ros::ok() ) {
    ros::spinOnce();  //给ROS控制权  可以调用一次回调函数
    loop_rate.sleep();
}

return 0;
}
