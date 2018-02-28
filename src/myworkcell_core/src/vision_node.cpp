/**
**  Simple ROS Node
**/
#include <ros/ros.h>// 系统头文件
// git clone https://github.com/jmeyer1292/fake_ar_publisher.git 安装
#include <fake_ar_publisher/ARMarker.h>// 自定义消息头文件

// 自定义类
class Localizer
{
public:
  // 类 初始化函数
  Localizer(ros::NodeHandle& nh)//节点句柄引用
  {
      // 订阅者                     消息类型                 话题名      队列大小
      ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
      &Localizer::visionCallback, this);// 回调函数指针  类自己
  }
  // 话题回调函数
  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)// 引用 常量指针
  {
      last_msg_ = msg;//复制一下放置 变化
      ROS_INFO_STREAM(last_msg_->pose.pose);//打印位置
  }

// 变量定义
  ros::Subscriber ar_sub_;// 订阅者
  fake_ar_publisher::ARMarkerConstPtr last_msg_;//常量指针
};



int main(int argc, char *argv[] ){
	// 初始化 ros节点
	ros::init(argc, argv, "vision_node");

	// 创建ros节点句柄
	ros::NodeHandle nh;
	Localizer localizer(nh);

	ROS_INFO("Vision node starting");

	// 节点 存活  rosnode list 可以一直看到
	ros::spin();

}
