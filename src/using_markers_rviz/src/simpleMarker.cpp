#include <ros/ros.h> //系统文件
#include <interactive_markers/interactive_marker_server.h>
// 可视化交互 物体 interactive marker类型 visualization_msgs::InteractiveMarker

//鼠标操作等交互操作  回调函数 向控制台反馈 位置信息
void processFeedback(
          //
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  //初始化ros系统 注册节点  节点名字
  ros::init(argc, argv, "simple_marker");
  // 创建交互式marker服务器 发布的话题 命名空间simple_marker2 ！！！！update topic里面选择
  interactive_markers::InteractiveMarkerServer server("simple_marker2");  // /simple_marker2/update
   // 交互式marker消息类型
  visualization_msgs::InteractiveMarker interac_marker;
  //参考坐标
  interac_marker.header.frame_id = "base_link";
  //时间戳
  interac_marker.header.stamp=ros::Time::now();
  //显示的名字
  interac_marker.name = "my_marker";
  //描述
  interac_marker.description = "Simple 1-DOF Control";

  // 创建一个灰色 正方体 盒子 marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::SPHERE;  // CUBE
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 0.5; //透明度

  // 可视化 相关控制
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true; //控制操作可见
  box_control.markers.push_back( box_marker );//将控制操作附着与创建好的marker上

  // 创建可交互可控制的交互式marker
  interac_marker.controls.push_back( box_control );

 //平移控制
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";//x轴平移
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//添加具体的控制
  interac_marker.controls.push_back(rotate_control);
  /*
      while (interac_marker.getNumSubscribers() < 1)
    {  //没有订阅者
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    */
//调用回调函数 反馈 位置信息
  server.insert(interac_marker, &processFeedback);
//应用改变
  server.applyChanges();
//开始 主循环 给ros控制权
  ros::spin();
}