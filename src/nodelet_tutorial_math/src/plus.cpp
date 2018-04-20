/*
原理:
Nodelets旨在提供一种在单机器单进程运行多个算法而不会在进程中传递消息时产生复制成本的方法。
roscpp具有在同一节点内的发布和订阅调用之间进行零拷贝指针传递的优化
为了做到这一点，nodelet允许将类动态加载到同一个节点，然而它们提供了简单的单独命名空间，
	使得尽管nodelet在同一个进程中，它仍然像一个独立的节点
这进一步扩展了，它在运行时使用pluginlib是动态可加载的。
应用程序
高吞吐量数据流可以由许多节点组成，然后加载到同一进程，以避免复制和网络流量。


pluginlib是一个C++库，可以实现为一个ROS包动态的加载和卸载插件。
使用插件来扩展或者修改应用程序的功能非常方便，不用改动源码重新编译应用程序.
通过插件的动态加载即可完成功能的扩展和修改。

 */

#include <pluginlib/class_list_macros.h>//动态的加载和卸载插件
#include <nodelet/nodelet.h>// 高吞吐量数据 零拷贝指针传递
#include <ros/ros.h>
#include <std_msgs/Float64.h>//标准消息　64位float
#include <stdio.h>


#include <math.h> //fabs 绝对值
// 命名空间　nodelet_tutorial_math
namespace nodelet_tutorial_math
{
//　自定义类　　继承于　nodelet::Nodelet
class Plus : public nodelet::Nodelet
{
public:
  Plus()//默认构造函数
  : value_(0)//初始化为０
  {}

private:
  virtual void onInit()// 节点启动初始化函数
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();//私有节点句柄　引用
    private_nh.getParam("value", value_);// 私有节点　获取参数
    pub = private_nh.advertise<std_msgs::Float64>("out", 10);//　发布话题out
    sub = private_nh.subscribe("in", 10, &Plus::callback, this);// 订阅话题回调函数callback function Plus::callback
  }

  void callback(const std_msgs::Float64::ConstPtr& input)
  {
    std_msgs::Float64Ptr output(new std_msgs::Float64());
    output->data = input->data + value_;//相加
    NODELET_DEBUG("Adding %f to get %f", value_, output->data);//　打印信息
    pub.publish(output);//发布消息
  }

  ros::Publisher pub;//发布话题类
  ros::Subscriber sub;//订阅话题类
  double value_;//类内私有变量　
};
// pluginlib是一个C++库，可以实现为一个ROS包动态的加载和卸载插件。
// 使用插件来扩展或者修改应用程序的功能非常方便，不用改动源码重新编译应用程序.
// 通过插件的动态加载即可完成功能的扩展和修改。
PLUGINLIB_EXPORT_CLASS(nodelet_tutorial_math::Plus, nodelet::Nodelet)
}
