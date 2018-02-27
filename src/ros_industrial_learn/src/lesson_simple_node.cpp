#include <ros/ros.h>
int main(int argc, char* argv[])
{
ros::init(argc, argv, "lesson_simple_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄 给节点权限
ros::Rate loop_rate(1.0);//运行频率

int count = 0;
while(ros::ok()) {
  ROS_INFO_STREAM("Hello World, at " << count <<" times.");//输出信息
  ++count;
  loop_rate.sleep();//给节点运行权限（按照指定频率）
}
return 0;
}
