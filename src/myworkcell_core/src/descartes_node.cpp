/*
使用　Descartes  笛卡尔　运动规划求解  正逆运动学求解器　demo
提供　返回笛卡尔规划　轨迹点集的服务
可以提供给　客户端
客户端再将轨迹发给　执行轨迹的　action去执行
*/
#include <ros/ros.h>
#include "myworkcell_core/PlanCartesianPath.h" // 给目标点返回　轨迹点序列　的服务信息头文件（自动生成）

#include <ur5_demo_descartes/ur5_robot_model.h>// Descartes  笛卡尔 ur5 模型
#include <descartes_planner/dense_planner.h>   //　Descartes  笛卡尔 规划器　稠密求解器
#include <descartes_planner/sparse_planner.h>  //　Descartes  笛卡尔 规划器　稀疏规划器
#include <descartes_trajectory/axial_symmetric_pt.h> //　轨迹点
#include <descartes_trajectory/joint_trajectory_pt.h>//
#include <descartes_utilities/ros_conversions.h>//
#include <eigen_conversions/eigen_msg.h>//

// 获取关节位置
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
  
  const Eigen::Vector3d travel = stop - start;//首位点位置差 向量
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

// 类初始化函数　带入　ros节点句柄
  CartesianPlanner(ros::NodeHandle& nh)
  {
    // first init descartes　初始化节点
    if (!initDescartes())
      throw std::runtime_error("There was an issue initializing Descartes");

    // init services　　　　　　　　初始化服务　订阅的服务名　　服务回调函数　　　　　　类本体
    server_ = nh.advertiseService("plan_path", &CartesianPlanner::planPath, this);
  }


// 初始化笛卡尔规划器　　１初始化机器人模型　２规划器初始化
  bool initDescartes()
  {
    // Create a robot model
    // 智能指针　显示消除　new  delete 内存的调研　方便　　　make_shared<T>()
    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();// 创建模型变量
    
    // Define the relevant "frames"
    // 定义　机器人模型初始化变量
    const std::string robot_description = "robot_description";//　机器人描述
    const std::string group_name = "manipulator";// 轨迹规划群
    const std::string world_frame = "world"; // Frame in which tool poses are expressed
    const std::string tcp_frame = "tool0";//　末端　坐标系

    // Using the desired frames, let's initialize Descartes
    // 初始化　机器人模型变量　
    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("Descartes RobotModel failed to initialize");
      return false;
    }
    
    // 规划器　根据　机器人模型　初始化
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
    // 产生　执行器末端　轨迹　　　工件　一周　画圈
    EigenSTL::vector_Affine3d tool_poses = makeToolPoses();
    
    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
    // 转换成笛卡尔　轨迹点
    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(req.pose, tool_poses);

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
    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
    return true;
  }


//  规划末端执行机构　轨迹
  EigenSTL::vector_Affine3d makeToolPoses()
  {
    EigenSTL::vector_Affine3d path;// 轨迹

    // We assume that our path is centered at (0, 0, 0), so let's define the
    // corners of the AR marker

    // 机器人首先运动到　模块中心位置　　
    // 
    const double side_length = 0.25; // 目标模块边长　　All units are in meters (M)
    const double half_side = side_length / 2.0;
    const double step_size = 0.02;// 步长

    Eigen::Vector3d top_left  (half_side,  half_side,  0);// 右手定则　上左边
    Eigen::Vector3d bot_left  (-half_side, half_side,  0);// 下左边　ｘ方向为负
    Eigen::Vector3d bot_right (-half_side, -half_side, 0);// 下右边　均为负
    Eigen::Vector3d top_right (half_side,  -half_side, 0);//　上右　ｙ方向为负
    Eigen::Vector3d center (0, 0, 0);// 中心点   

    // Descartes requires you to guide it in how dense the points should be,
    // so you have to do your own "discretization".
    // NOTE that the makeLine function will create a sequence of points inclusive
    // of the start and exclusive of finish point, i.e. line = [start, stop)

    // TODO: Add the rest of the cartesian path
    auto segment1 = makeLine(top_left,  bot_left,  step_size);
    auto segment2 = makeLine(bot_left,  bot_right, step_size);
    auto segment3 = makeLine(bot_right, top_right, step_size);
    auto segment4 = makeLine(top_right, top_left,  step_size);

    path.insert(path.end(), segment1.begin(), segment1.end());
    path.insert(path.end(), segment2.begin(), segment2.end());
    path.insert(path.end(), segment3.begin(), segment3.end());
    path.insert(path.end(), segment4.begin(), segment4.end());

    return path;
  }


// 由参考点　将EigenSTL::vector_Affine3d　位置　转换成　笛卡尔　轨迹点
  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const geometry_msgs::Pose& reference,
                          const EigenSTL::vector_Affine3d& path)
  {
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // 返回值　return value

    // 参考位置
    Eigen::Affine3d ref;
    tf::poseMsgToEigen(reference, ref);//位置　转化成　EIGEN格式

    for (auto& point : path)//范围　for  path　引用
    {
      // TODO: make a Descartes "cartesian" point with some kind of constraints
      descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(ref * point);// 根据参考位置
      descartes_path.push_back(pt);
    }
    return descartes_path;
  }


// Eigen::Affine3d位置　生成　AxialSymmetricPt　笛卡尔轨迹
  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );// 关于Ｚ轴对称的点
// 详情　http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html
  }

// 获取　关节名字　controller_joint_names　获取　HELPER
  std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    return names;
  }

// 类内变量　
  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;// 笛卡尔机器人模型　指针指针　
  descartes_planner::DensePlanner planner_;// 稠密规划器
  // descartes_planner::SparsePlanner   稀疏规划器
  ros::ServiceServer server_;// 服务器
  ros::NodeHandle nh_;// 节点句柄
};


// 主函数
int main(int argc, char** argv)
{
  // 初始化节点
  ros::init(argc, argv, "descartes_node");
  // 节点句柄
  ros::NodeHandle nh;
  // 笛卡尔　规划器 
  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");

  ros::spin();
}


