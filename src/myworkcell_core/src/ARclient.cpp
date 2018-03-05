/**
**  Simple ROS Node
**  moveit 接口 运动规划 　　　　　　　　　　　　　　　　　　　　　　　　　　运动到　工件位置
**  Descartes  笛卡尔 轨迹规划接口　使用轨迹跟踪执行　action行动执行　　　　工具在工件位置周围执行
**  相当于　moveit　轨迹规划
**  Descartes  笛卡尔  终端抓取规划
**/
#include <ros/ros.h>// 系统头文件
#include <myworkcell_core/LocalizePart.h>// 定位　服务头文件

#include <tf/tf.h>// 坐标变换
#include <moveit/move_group_interface/move_group.h>//moveit 接口 新版 move_group_interface.h

//　笛卡尔规划　接口
#include <actionlib/client/simple_action_client.h>// 动作　服务接口
#include <control_msgs/FollowJointTrajectoryAction.h>// 控制消息　　轨迹行动　根据轨迹点集合　执行　移动到个点
#include <myworkcell_core/PlanCartesianPath.h>// 笛卡尔规划　服务头文件　得到规划的轨迹点集合

// 自定义类
class ScanNPlan
{
public:
// 类 初始化函数
  //节点句柄引用 带有　笛卡尔轨迹初始化　列表
  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
  {
    // 定位　请求
    // 客户端                             服务类型                      请求的服务名字
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");

// 笛卡尔规划　 客户端　请求
    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
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
    // movieit 接口规划
    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");//运动规划组　配置文件里定义的　新版
    moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板本
    geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
    move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　目标位置
    move_group.move();// 规划　并　执行
    
    // 笛卡尔　规划　Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;// 服务
    cartesian_srv.request.pose = move_target;// 请求的目标位置
    if (!cartesian_client_.call(cartesian_srv))//　调用笛卡尔　规划 获取　轨迹点集
    {
      ROS_ERROR("Could not plan for path");
      return;
    }
    // 执行轨迹点集合　Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;// 行动中的　目标
    goal.trajectory = cartesian_srv.response.trajectory;// 目标轨迹
    ac_.sendGoal(goal);// 发送目标
    ac_.waitForResult();// 等待执行结果
    ROS_INFO("Done");//

  }

private://　成员变量
  // Planning components
  ros::ServiceClient vision_client_;// 私有变量 类内使用

// 笛卡尔规划　服务
  ros::ServiceClient cartesian_client_;//　客户端　请求笛卡尔规划　轨迹点集结果
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;// 行动请求
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
