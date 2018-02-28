/**
**  Simple ROS Node
**  ARclient.cpp  客户端
**/
#include <ros/ros.h>// 系统头文件
#include <myworkcell_core/LocalizePart.h>// 服务头文件


// 自定义类
class ScanNPlan
{
public:
// 类 初始化函数
  ScanNPlan(ros::NodeHandle& nh)//节点句柄引用
  {
    // 客户端                             服务类型                      请求的服务名字
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }
// 执行函数
  void start()
  {
    // 打印信息
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;// 初始化 服务
    if (!vision_client_.call(srv))//调用服务 得到响应数据
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;// 私有变量 类内使用
};


int main(int argc, char **argv)
{
  // 初始化 ros节点
  ros::init(argc, argv, "ARclient");// 初始化 ros节点
  // 创建ros节点句柄
  ros::NodeHandle nh;

  ROS_INFO("ScanNPlan node has been initialized");

  ScanNPlan app(nh);// 客户端

  ros::Duration(.5).sleep();  // 等待客户端初始化完成 wait for the class to initialize
  app.start();// 请求服务

  ros::spin();// 节点 存活  rosnode list 可以一直看到
}

