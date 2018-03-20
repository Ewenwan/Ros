/*
 * demo_application.h
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#ifndef DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_
#define DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_

#include <ros/ros.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_trajectory/axial_symmetric_pt.h>// 笛卡尔轨迹 规划
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <ur5_demo_descartes/ur5_robot_model.h>
//　笛卡尔规划　行动服务接口
#include <actionlib/client/simple_action_client.h>// 动作　服务接口
#include <control_msgs/FollowJointTrajectoryAction.h>// 控制消息　　轨迹行动　根据轨迹点集合　执行　移动到个点

namespace plan_and_run
{

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";//机器人描述
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";//执行运动学轨迹的服务话题
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";//可视化轨迹 话题
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;//方向增量
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;//可视化轴长度
const double AXIS_LINE_WIDTH = 0.001;//可视化轴宽度
const std::string PLANNER_ID = "RRTConnectkConfigDefault";//规划器ID
const std::string HOME_POSITION_NAME = "home";//休息区 名字

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;//笛卡尔轨迹点 指针 数组
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FJTAction;
/*  =============================== Application Data Structure ===============================
 *
 * Holds the data used at various points in the application.  This structure is populated
 * from data found in the ros parameter server at runtime.
 *
 * 应用配置类型数据结构
 */
struct DemoConfiguration
{
  std::string group_name;                 /* 运动规划组 moveit_config  the manipulation group */
  std::string tip_link;                   /* 最后的部件 tool the last link in the kinematic chain of the robot */
  std::string base_link;                  /* 基部件 The name of the base link of the robot */
  std::string world_frame;                /* 世界坐标系 The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* 关节list A list with the names of the mobile joints in the robot */


  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  轨迹参数 trajectory/time_delay ... trajectory/seed_pose
   *  */
  double time_delay;              /* Time step between consecutive points in the robot path */
  double foci_distance;           /* Controls the size of the curve */
  double radius;                  /* 曲线半价 Controls the radius of the sphere on which the curve is projected */
  int num_points;                 /* 每个曲线的点数量 Number of points per curve */
  int num_lemniscates;            /* 曲线数量 Number of curves*/
  std::vector<double> center;     /* 双纽曲线 中心Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   * 可视化参数 visualization/min_point_distance
   */
  double min_point_distance;      /* 连续轨迹点 最小距离 Minimum distance between consecutive trajectory points. */
};


/*  =============================== Application Class ===============================
 *
 * Provides a group of functions for planning and executing a robot path using Moveit and
 * the Descartes Planning Library
 * 应用类
 */
class DemoApplication
{
public:
  /*  Constructor
   *    Creates an instance of the application class
   */
  DemoApplication();// 构造函数
  virtual ~DemoApplication();//虚 析构函数 可以继承

  /* Main Application Functions
   *  Functions that allow carrying out the various steps needed to run a
   *  plan an run application.  All these functions will be invoked from within
   *  the main routine.
   */

  void loadParameters();//载入参数
  void initRos();//初始化 ros系统
  void initDescartes();//初始化 笛卡尔规划
  void moveHome();// 移动到等候区（家位置）
  void generateTrajectory(DescartesTrajectory& traj);//产生笛卡尔规划轨迹 引用形参传回
  void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);//规划机器人路径
  void runPath(const DescartesTrajectory& path);//执行路径

protected:

  /* Support methods  私有函数（方法）
   *  Called from within the main application functions in order to perform convenient tasks.
   * 双纽曲线函数 创建轨迹
   */

  static bool createLemniscateCurve(double foci_distance, double sphere_radius,
                                    int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center,
                                    EigenSTL::vector_Affine3d& poses);
// 路径点类型转换
  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);
// 发布轨迹点 marker消息 到rviz进行显示
  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


protected:

  /* Application Data 私有变量
   *  Holds the data used by the various functions in the application.
   */
  DemoConfiguration config_;//应用配置类型数据结构

  /* Application ROS Constructs
   *  Components needed to successfully run a ros-node and perform other important
   *  ros-related tasks
   */
  ros::NodeHandle nh_;                        /* ros节点 Object used for creating and managing ros application resources*/
  ros::Publisher marker_publisher_;           /* 发布可视化消息到 rviz Publishes visualization message to Rviz */
  ros::ServiceClient moveit_run_path_client_; /* 客户端发送轨迹点去执行 Sends a robot trajectory to moveit for execution */

  /* Application Descartes Constructs  笛卡尔
   *  Components accessing the path planning capabilities in the Descartes library
   */
  descartes_core::RobotModelPtr robot_model_ptr_; /* 笛卡尔机器人模型 正逆运动学 碰撞检测 Performs tasks specific to the Robot
                                                     such IK, FK and collision detection*/
  descartes_planner::SparsePlanner planner_;      /* 笛卡尔稀疏规划器 Plans a smooth robot path given a trajectory of points */

 //FJTAction ac_;// 行动请求 客户端
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;// 行动请求 客户端
};

} /* namespace plan_and_run */

#endif /* DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_ */
