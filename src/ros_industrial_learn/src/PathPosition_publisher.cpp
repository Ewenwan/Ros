#include <ros/ros.h>
#include <ros_industrial_learn/PathPosition.h>//包含自动生成的 自定义消息头文件
#include <stdlib.h>                           //标准库 产生rand()随机数

int main(int argc, char* argv[])
{
ros::init(argc, argv, "PathPosition_pub_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄 给节点权限
ros::Publisher pub = node.advertise<ros_industrial_learn::PathPosition>("position", 1000);//发布消息 队列大小

ros_industrial_learn::PathPosition pp_msg;

// 发送一次
/*
pp_msg.header.stamp = ros::Time::now();//时间戳

float angle = 180.0;
pp_msg.angle = angle * 3.141592 / 180.0;//角度 弧度制
pp_msg.x = 100.0 * cos(pp_msg.angle);//水平位置
pp_msg.y = 100.0 * sin(pp_msg.angle);//垂直位置
pub.publish(pp_msg);
ros::spinOnce();//给一次控制权
ROS_INFO("Published message %.1f, %.1f, %.1f", pp_msg.x, pp_msg.y, pp_msg.angle * 180.0 / 3.141592);
*/

//一秒发送一次
///*
// srand(time(0)) ;       //Seed the random number generator
ros::Rate loop_rate (1);    //发布频率   控制 消息发布 频率   这个对象控制循环运行速度
while(ros::ok()) {
  srand(time(0)) ; 
  pp_msg.header.stamp = ros::Time::now();//时间戳
  float angle =   float(rand())  /  RAND_MAX  * 180 ;        //角度  为 0 到 180 之间的某个值
  pp_msg.angle = angle * 3.141592 / 180.0;//角度 弧度制
  pp_msg.x = 100.0 * cos(pp_msg.angle);//水平位置
  pp_msg.y = 100.0 * sin(pp_msg.angle);//垂直位置
  pub.publish(pp_msg);
  //ros::spinOnce();//给一次控制权  发布消息不需要调用回调函数
  ROS_INFO("Published message %.1f, %.1f, %.1f", pp_msg.x, pp_msg.y, pp_msg.angle * 180.0 / 3.141592);

  loop_rate.sleep();//给节点运行权限（按照指定频率）
}
//*/

return 0;
}
