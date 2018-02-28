/**
**  Simple ROS Node
**  moveit 接口 运动规划 
**/
#include <ros/ros.h>// 系统头文件
#include <myworkcell_core/LocalizePart.h>// 服务头文件

#include <tf/tf.h>// 坐标变换
#include <moveit/move_group_interface/move_group.h>//moveit 接口 新版 move_group_interface.h

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
  void start(const std::string& base_frame)
  {
    // 打印信息
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;// 初始化 服务
    srv.request.base_frame = base_frame; 
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    if (!vision_client_.call(srv))//调用服务 得到响应数据
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应

    // Plan for robot to move to part
    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");//运动规划组　配置文件里定义的　新版
    moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板　
    geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
    move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　目标位置
    move_group.move();// 规划　并　执行
    
    

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

  ros::NodeHandle private_node_handle("~");// 增加一个私有节点 获取目标对象坐标系参数

  ROS_INFO("ScanNPlan node has been initialized");

  std::string base_frame;// string 对象变量
  // 私有节点获取参数                      坐标系参数名  存储变量    默认值
  private_node_handle.param<std::string>("base_frame", base_frame ,"world"); 

// move_group.move() command requires use of an "asynchronous" spinner, 
// to allow processing of ROS messages during the blocking move() command.
  ros::AsyncSpinner async_spinner(1);

  ScanNPlan app(nh);// 客户端
  sleep(3); //alows for debugging
  ros::Duration(6).sleep();  // 等待客户端初始化完成 wait for the class to initialize
  async_spinner.start();
  app.start(base_frame);// 请求服务

  // ros::spin();// 节点 存活  rosnode list 可以一直看到
  ros::waitForShutdown();
}
