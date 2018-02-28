/**
**  Simple ROS Node
**  ARserver.cpp  服务器 
**  加入坐标变换  监听
**/
#include <ros/ros.h>// 系统头文件
// // git clone https://github.com/jmeyer1292/fake_ar_publisher.git 安装
#include <fake_ar_publisher/ARMarker.h>// 自定义消息头文件
#include <myworkcell_core/LocalizePart.h>// 服务头文件
#include <tf/transform_listener.h>// 坐标变换  监听

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
      // 服务器订阅服务
      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }
  // 话题回调函数
  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)// 引用 常量指针
  {
      last_msg_ = msg;//复制一下放置 变化
     //  ROS_INFO_STREAM(last_msg_->pose.pose);//打印位置
  }

  // 服务回调函数
  bool localizePart(myworkcell_core::LocalizePart::Request& req,//请求
                      myworkcell_core::LocalizePart::Response& res)// 回应
  {
      // Read last message
      fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
      if (!p) return false;//空指针 无信息

      // res.pose = p->pose.pose;

      tf::Transform cam_to_target;// 参考坐标系 相机 到 目标坐标系的变换
      // geometry_msgs::Pose 转换到 tf::Transform object: 
      tf::poseMsgToTF(p->pose.pose, cam_to_target);

      // 监听 request.base_frame(客户端给的基坐标系) 到 ARMarker message  (which should be "camera_frame") 坐标变换
      tf::StampedTransform req_to_cam;
      listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);

      tf::Transform req_to_target;
      req_to_target = req_to_cam * cam_to_target;// 目标 在客户端给的基 坐标系下的 坐标变换

      tf::poseTFToMsg(req_to_target, res.pose);

      return true;
  }

// 变量定义
  ros::Subscriber ar_sub_;// 订阅者
  fake_ar_publisher::ARMarkerConstPtr last_msg_;//常量指针
  ros::ServiceServer server_;// 服务器
  tf::TransformListener listener_;// 坐标变换监听
};



int main(int argc, char *argv[] ){
	// 初始化 ros节点
	ros::init(argc, argv, "ARserver");

	// 创建ros节点句柄
	ros::NodeHandle nh;
	Localizer localizer(nh);

	ROS_INFO("Vision node starting");

	// 节点 存活  rosnode list 可以一直看到
	ros::spin();

}
