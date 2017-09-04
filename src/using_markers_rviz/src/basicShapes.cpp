#include <ros/ros.h>                    //系统文件 
#include <visualization_msgs/Marker.h>  // 可视化物体 marker类型 visualization_msgs::Marker

int main( int argc, char** argv )
{                        //节点名称
  ros::init(argc, argv, "basic_shapes"); //初始化ros系统 注册节点
  ros::NodeHandle nh;                    //在节点管理器中注册节点
  ros::Rate rate(2);                     //发布频率(控制 立体形状变换速度)
  //
  ros::Publisher marker_pub;             //创建发布者       话题名字             序列大小
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //立方体形状  shape
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    //可视化形状 消息 
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //坐标系
    marker.header.frame_id = "/my_frame";//立体形状的参考系   在rviz中 Fixed Frame 需要选择为/my_frame
    //时间戳
    marker.header.stamp = ros::Time::now();
// %Tag(NS_ID)%
    //           命名空间
    marker.ns = "basic_shapes";
    //标志号
    marker.id = 0;
//形状 初始 为//立方体形状   sphere 球体   arrow 箭头  cylinder 圆柱体
    marker.type = shape;  //形状标志变量
    //操作 添加ADD 去除DELETE   去除全部 DELETEALL
    marker.action = visualization_msgs::Marker::ADD;
    //姿态    位置position   方向orientation
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
//大小   单位米
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
//颜色 红绿蓝 透明度
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    //有效时间  ros::Duration();不会自动删除
    marker.lifetime = ros::Duration();
    // 发布 形状
    while (marker_pub.getNumSubscribers() < 1)
    {  //没有订阅者
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    switch (shape)
    {
    case visualization_msgs::Marker::CUBE://立方体
      shape = visualization_msgs::Marker::SPHERE;//球体
      break;
    case visualization_msgs::Marker::SPHERE://球体
      shape = visualization_msgs::Marker::ARROW;//箭头
      break;
    case visualization_msgs::Marker::ARROW://箭头
      shape = visualization_msgs::Marker::CYLINDER;//圆柱体
      break;
    case visualization_msgs::Marker::CYLINDER://圆柱体
      shape = visualization_msgs::Marker::CUBE;//立方体
      break;
    }
    rate.sleep();
  }
}