#include <ros/ros.h>                    //系统文件 
#include <visualization_msgs/Marker.h>  // 可视化物体 marker类型 visualization_msgs::Marker

#include <cmath>                        //数学库

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pointsAndLines"); //初始化ros系统 注册节点
  ros::NodeHandle nh;                      //在节点管理器中注册节点
  ros::Publisher marker_pub;               //创建发布者       话题名字           序列大小
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate rate(30);     //发布频率 控制线条摆动速度
  float f = 0.0;          //浮点变量
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%           点        线带（条）   线表（线段 排列）
    visualization_msgs::Marker points, line_strip, line_list;
    //参考坐标系
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    //时间戳
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    //命名空间
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    //操作 添加ADD 去除DELETE   去除全部 DELETEALL
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    //方向w
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    // 标志符号 身份标志
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    //形状
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // 点尺寸 类似直径大小
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    //线带尺寸 线条
    line_strip.scale.x = 0.1;
    //线表尺寸
    line_list.scale.x = 0.1;
    //点颜色
    points.color.g = 1.0f; //绿色
    points.color.a = 1.0;  //透明度
    //线带颜色
    line_strip.color.b = 1.0; //蓝色
    line_strip.color.a = 1.0;
    //线表颜色
    line_list.color.r = 1.0;  //红色
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 200; ++i)  //i 越大 线越长
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;//线性增长  
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // 两点确定一条直线  线段序列
      line_list.points.push_back(p);
      p.z += 3.0;//改变线段长度
      line_list.points.push_back(p);
    }
// %EndTag(HELIX)%

    while (marker_pub.getNumSubscribers() < 1)
    {  //没有订阅者
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    rate.sleep();

    f += 0.04;
  }
}
// %EndTag(FULLTEXT)%

