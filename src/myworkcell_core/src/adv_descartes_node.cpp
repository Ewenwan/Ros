/*
使用　Descartes  笛卡尔　运动规划求解  正逆运动学求解器　demo
提供　返回笛卡尔规划　轨迹点集的服务
可以提供给　客户端
客户端再将轨迹发给　执行轨迹的　action去执行
*/
#include <ros/ros.h>
#include <ros/package.h>//获取包路径 ros::package::getPath("myworkcell_core")
#include "myworkcell_core/PlanCartesianPath.h"// 给目标点返回　轨迹点序列　的服务信息头文件（自动生成）

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <ur5_demo_descartes/ur5_robot_model.h>// Descartes  笛卡尔 ur5 模型
#include <descartes_planner/dense_planner.h>//　Descartes  笛卡尔 规划器　稠密求解器
#include <descartes_planner/sparse_planner.h>//　Descartes  笛卡尔 规划器　稀疏规划器
#include <descartes_trajectory/axial_symmetric_pt.h>//　对称 轨迹点
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_utilities/ros_conversions.h>// 轨迹点转换 到关节位姿
#include <eigen_conversions/eigen_msg.h>// eigen 类型转换
#include <fstream>//  文件流
#include <string>// 文件名字符串

// 获取当前关节位置
std::vector<double> getCurrentJointState(const std::string& topic)
{
  // 捕获关节位置
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

// 根据首末点位置　和　运动步长　生成　直线轨迹点　容器
EigenSTL::vector_Affine3d makeLine(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, double ds)
{
  EigenSTL::vector_Affine3d line;// 直线轨迹
  
  const Eigen::Vector3d travel = stop - start;//总向量 首位点位置差 向量
  const int steps = std::floor(travel.norm() / ds);// 向量长度/步长　得到步数

  // Linear interpolation
  for (int i = 0; i < steps; ++i)
  {
    double ratio = static_cast<float>(i) / steps;
    Eigen::Vector3d position = start + ratio * travel;//起点＋　每一步步长向量
    Eigen::Affine3d tr;
    tr = Eigen::Translation3d(position);
    line.push_back( tr );
  }

  return line;
}

// CartesianPlanner 笛卡尔规划器　类
class CartesianPlanner
{
public:

// 类 默认构造函数
  CartesianPlanner(ros::NodeHandle& nh)
  {
    // first init descartes
    if (!initDescartes())//初始化笛卡尔规划器
      throw std::runtime_error("There was an issue initializing Descartes");

    //　　初始化笛卡尔规划轨迹服务　订阅的服务名　　      服务回调函数　　　　　　类本体
    server_ = nh.advertiseService("adv_plan_path", &CartesianPlanner::planPath, this);
    // 初始化 可视化轨迹 发布 marker rviz会订阅 然后显示
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("puzzle_path", 0);
  }

// 初始化笛卡尔规划器　　１初始化机器人模型　２规划器初始化
  bool initDescartes()
  {
    // 1创建一个机器人模型 Create a robot model 
        //智能指针　显示消除　new  delete 内存的调研　方便　　　make_shared<T>()
    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
    // 1.1定义　机器人模型初始化变量   
    // Define the relevant "frames"
    const std::string robot_description = "robot_description";//　机器人描述
    const std::string group_name = "manipulator";// 轨迹规划群 "manipulator"
    const std::string world_frame = "world";// 世界坐标系
    const std::string tcp_frame = "part";//　末端坐标系 "tool0" 后接的工具 tool center point 
    // 1.2初始化机器人模型
    // Using the desired frames, let's initialize Descartes
    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("Descartes RobotModel failed to initialize");
      return false;
    }
     //设置检查障碍物
    model_->setCheckCollisions(true);
  
    //2 初始化笛卡尔规划器
    if (!planner_.initialize(model_))
    {
      ROS_WARN("Descartes Planner failed to initialize");
      return false;
    }

    return true;
  }

// 服务回调函数　　规划终端工具　路径
  bool planPath(myworkcell_core::PlanCartesianPathRequest& req, // 服务请求
                myworkcell_core::PlanCartesianPathResponse& res)// 服务响应
  {
    ROS_INFO("Recieved cartesian planning request");

    // Step 1: Generate path poses 
    // 产生　执行器末端　轨迹　　按照指定路径文件产生 轨迹点
   EigenSTL::vector_Affine3d tool_poses = makePuzzleToolPoses();//makeToolPoses();
    visualizePuzzlePath(tool_poses);//可视化

    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
    // 转换成笛卡尔　轨迹点
    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(tool_poses);

    // Step 3: Tell Descartes to start at the "current" robot position
    // 当前机器人位置设置为　起点
    std::vector<double> start_joints = getCurrentJointState("joint_states");
    descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints));
    path.front() = pt;

    // Step 4: Plan with descartes
    // 笛卡尔规划器　规划 
    if (!planner_.planPath(path))
    {
      return false;
    }
    // 规划结果
    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner_.getPath(result))
    {
      return false;
    }

    // Step 5: Convert the output trajectory into a ROS-formatted message
    // 转换　笛卡尔规划器规划结果到　标准ROS关节点信息
    res.trajectory.header.stamp = ros::Time::now();// 时间戳
    res.trajectory.header.frame_id = "world";      //　坐标系
    res.trajectory.joint_names = getJointNames();  //　关节名字　　controller_joint_names
	//# request 请求　目标位置
	//geometry_msgs/Pose pose
	//---
	//# response 响应　当前位置到目标位置　轨迹　点
	//trajectory_msgs/JointTrajectory trajectory
    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
    // 添加关节速度
    //addVel(res.trajectory);
    return true;
  }

// 按照指定路径文件产生 轨迹点  x,y,z,r,p,y 6dof的位姿 转换成 4*4的 变换矩阵
  EigenSTL::vector_Affine3d makePuzzleToolPoses()
  {
    //轨迹
    EigenSTL::vector_Affine3d path;
    std::ifstream indata;//输入文件流
    // 获取指定 轨迹文件路径 包下config/puzzle_bent.csv
    std::string filename = ros::package::getPath("myworkcell_core") + "/config/puzzle_bent.csv";
    // 打开文件
    indata.open(filename);
    std::string line;//每一行
    int lnum = 0;
    while (std::getline(indata, line))
    {
        ++lnum;
        if (lnum < 3)
          continue;//跳过前两行
        // 读物每一行数据 转换成 double 是6dof的位姿数据
        std::stringstream lineStream(line);//字符串流 每一行
        std::string  cell;
        Eigen::VectorXd xyzijk(6);//6dof的位姿数据
        int i = -2;
        while (std::getline(lineStream, cell, ','))//按，号分割
        {
          ++i;
          if (i == -1)//最后
            continue;

          xyzijk(i) = std::stod(cell);//转换成 double
        }

        Eigen::Vector3d pos = xyzijk.head<3>();//位置
        pos = pos / 1000.0;//换成m单位
        Eigen::Vector3d norm = xyzijk.tail<3>();//姿态
        norm.normalize();
        // x,y,z,r,p,y 6dof的位姿 转换成 4*4的 变换矩阵
        Eigen::Vector3d temp_x = (-1 * pos).normalized();
        Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
        Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
        Eigen::Affine3d pose;
        pose.matrix().col(0).head<3>() = x_axis;
        pose.matrix().col(1).head<3>() = y_axis;
        pose.matrix().col(2).head<3>() = norm;
        pose.matrix().col(3).head<3>() = pos;

        path.push_back(pose);
    }
    indata.close();//关闭文件
    return path;
  }
// 轨迹点 发布marker消息 在rviz中显示
  bool visualizePuzzlePath(EigenSTL::vector_Affine3d path)
  {
    int cnt = 0;
    visualization_msgs::MarkerArray marker_array;//可视化marker 数组
    for (auto &point : path)//范围for 遍历
    {
      Eigen::Vector3d pos = point.matrix().col(3).head<3>();//位置
      Eigen::Vector3d dir = point.matrix().col(2).head<3>();//姿态方向

      visualization_msgs::Marker marker;////可视化marker
      marker.header.frame_id = "part";//坐标系 相对于tcp（工件）的坐标系
      marker.header.stamp = ros::Time();//时间戳
      marker.header.seq = cnt;//序列
      marker.ns = "markers";//命令空间
      marker.id = cnt;// id
      marker.type = visualization_msgs::Marker::ARROW;//现状为箭头
      marker.action = visualization_msgs::Marker::ADD;//增加 
      marker.lifetime = ros::Duration(0);//生命周期
      marker.frame_locked = true;//固定坐标系
      marker.scale.x = 0.0002;//大小
      marker.scale.y = 0.0002;
      marker.scale.z = 0.0002;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;//绿色
      marker.color.b = 0.0;
      geometry_msgs::Point pnt1, pnt2;// 几何点 消息
      tf::pointEigenToMsg(pos, pnt1);
      tf::pointEigenToMsg(pos + (0.005*dir), pnt2);
      marker.points.push_back(pnt1);//点
      marker.points.push_back(pnt2);
      marker_array.markers.push_back(marker);
      ++cnt;
    }
// 发布消息
    vis_pub_.publish(marker_array);
  }

// 将EigenSTL::vector_Affine3d　位姿(4*4 T)　转换成　笛卡尔　轨迹点
  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const EigenSTL::vector_Affine3d& path)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;
    // 笛卡尔轨迹坐标 返回值r
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; //return value

    // 得到研磨工具手的位姿
    // need to get the transform between grinder_frame 研磨工具手 and base_link;
    tf::StampedTransform grinder_frame;
    Eigen::Affine3d gf;
    listener_.lookupTransform("world", "grinder_frame", ros::Time(0), grinder_frame);
    tf::transformTFToEigen(grinder_frame, gf);// 得到研磨工具手的位姿

    Frame wobj_base(gf);
    Frame tool_base = Frame::Identity();
    TolerancedFrame wobj_pt = Frame::Identity();

    for (auto& point : path)
    {
      auto p = point;//每个点
      TolerancedFrame tool_pt(p);//工件下的点
      tool_pt.orientation_tolerance.z_lower -= M_PI;
      tool_pt.orientation_tolerance.z_upper += M_PI;

      boost::shared_ptr<CartTrajectoryPt> pt(new CartTrajectoryPt(wobj_base, wobj_pt, tool_base, tool_pt, 0, M_PI/20.0));
      descartes_path.push_back(pt);
    }
    return descartes_path;
  }

  // 关节名字 HELPER
  std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    return names;
  }


  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;//机器人模型
  descartes_planner::DensePlanner planner_;//笛卡尔稠密规划器
  ros::ServiceServer server_;//服务器
  ros::NodeHandle nh_;//节点句柄
  tf::TransformListener listener_;//坐标变换监听
  ros::Publisher vis_pub_;//发布可视化路径消息
};



int main(int argc, char** argv)
{
  // 初始化节点
  ros::init(argc, argv, "adv_descartes_node");

  ros::NodeHandle nh;
  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");
  ros::spin();
}
