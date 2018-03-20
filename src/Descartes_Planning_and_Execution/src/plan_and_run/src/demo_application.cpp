/*
 * 应用实现函数
 * demo_application.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#include <plan_and_run/demo_application.h>
#include <descartes_utilities/ros_conversions.h>


namespace plan_and_run
{
// 构造函数  带有　笛卡尔轨迹执行行动 初始化　列表
DemoApplication::DemoApplication():ac_("joint_trajectory_action", true)
{}
//虚 析构函数 可以继承
DemoApplication::~DemoApplication()
{}

// 发布轨迹点 marker消息 到rviz进行显示
void DemoApplication::publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;
// LINE_LIST类型在每对点外创建一个不相连的线，也就是0连接1, 2连接3，等等。
  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;// 坐标轴 线列表序列
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";//命名空间
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;//增加
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);//生命周期
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;//坐标轴
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;//可视化轴宽度

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;//z轴蓝色
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;//y轴绿色
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;//x轴红色
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
// LINE_STRIP类型将每个点用作一个连接的线中的定点，其中点0和点1连接，1和2连接，等等。
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";//命名空间
  line.action = visualization_msgs::Marker::ADD;//增加
  line.lifetime = ros::Duration(0);//生命周期
  line.header.frame_id = config_.world_frame;//坐标轴
  line.scale.x = AXIS_LINE_WIDTH;//可视化轴宽度
  line.id = 0;
  line.color.r = 1;//红色
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;//两点间的距离
  Eigen::Affine3d prev = poses[0];//第一个点
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];//中间点的
    distance = (pose.translation() - prev.translation()).norm();//两点间的距离

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > config_.min_point_distance)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  marker_publisher_.publish(markers_msg);

}

void swap_segments(EigenSTL::vector_Affine3d& poses, unsigned npoints, unsigned idx1, unsigned idx2)
{
  auto n = npoints / 2;
  std::swap_ranges(poses.begin() + n * idx1, poses.begin() + n * (idx1 + 1),
                   poses.begin() + n * idx2);
}

void insert_segment(EigenSTL::vector_Affine3d& poses, const EigenSTL::vector_Affine3d& orig, unsigned npoints, unsigned idx)
{
  auto n = npoints / 2;
  poses.insert(poses.end(), orig.begin() + n * idx, orig.begin() + n * (idx + 1));
}

bool DemoApplication::createLemniscateCurve(double foci_distance, double sphere_radius,
                                  int num_points, int num_lemniscates,const Eigen::Vector3d& sphere_center,
                                  EigenSTL::vector_Affine3d& poses)
{
  double a = foci_distance;
  double ro = sphere_radius;
  int npoints = num_points;
  int nlemns = num_lemniscates;
  Eigen::Vector3d offset(sphere_center[0],sphere_center[1],sphere_center[2]);
  Eigen::Vector3d unit_z,unit_y,unit_x;

  // checking parameters
  if(a <= 0 || ro <= 0 || npoints < 10 || nlemns < 1)
  {
    ROS_ERROR_STREAM("Invalid parameters for lemniscate curve were found");
    return false;
  }

  // generating polar angle values
  std::vector<double> theta(npoints);

  // interval 1 <-pi/4 , pi/4 >
  double d_theta = 2*M_PI_2/(npoints - 1);
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[i] = -M_PI_4  + i * d_theta;
  }
  theta[0] = theta[0] + EPSILON;
  theta[npoints/2 - 1] = theta[npoints/2 - 1] - EPSILON;

  // interval 2 < 3*pi/4 , 5 * pi/4 >
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[npoints/2 + i] = 3*M_PI_4  + i * d_theta;
  }
  theta[npoints/2] = theta[npoints/2] + EPSILON;
  theta[npoints - 1] = theta[npoints - 1] - EPSILON;

  // generating omega angle (lemniscate angle offset)
  std::vector<double> omega(nlemns);
  double d_omega = M_PI/(nlemns);
  for(unsigned int i = 0; i < nlemns;i++)
  {
     omega[i] = i*d_omega;
  }

  // std::swap(omega[1], omega[2]);

  Eigen::Affine3d pose;
  double x,y,z,r,phi;

  poses.clear();
  poses.reserve(nlemns*npoints);
  for(unsigned int j = 0; j < nlemns;j++)
  {
    for(unsigned int i = 0 ; i < npoints;i++)
    {
      r = std::sqrt( std::pow(a,2) * std::cos(2*theta[i]) );
      phi = r < ro ? std::asin(r/ro):  (M_PI - std::asin((2*ro - r)/ro) );

      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
      z = ro * std::cos(phi);

      // determining orientation
      unit_z <<-x, -y , -z;
      unit_z.normalize();

      unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
      unit_y = (unit_z .cross(unit_x)).normalized();

      Eigen::Matrix3d rot;
      rot << unit_x(0),unit_y(0),unit_z(0)
         ,unit_x(1),unit_y(1),unit_z(1)
         ,unit_x(2),unit_y(2),unit_z(2);

      pose = Eigen::Translation3d(offset(0) + x,
                                  offset(1) + y,
                                  offset(2) + z) * rot;

      poses.push_back(pose);
    }
    std::reverse(poses.end() - npoints / 2, poses.end());
  }

  // Hacky optimization for purposes of smoother demo
  if (nlemns == 4) 
  {
    EigenSTL::vector_Affine3d other;
    insert_segment(other, poses, npoints, 0);
    insert_segment(other, poses, npoints, 1);
    insert_segment(other, poses, npoints, 5);
    insert_segment(other, poses, npoints, 4);
    insert_segment(other, poses, npoints, 7);
    insert_segment(other, poses, npoints, 6);
    insert_segment(other, poses, npoints, 2);
    insert_segment(other, poses, npoints, 3);
    poses = other;
  }

  return true;
}

// 添加关节速度
void addVel(trajectory_msgs::JointTrajectory& traj)
{
  if (traj.points.size() < 3) return;

  auto n_joints = traj.points.front().positions.size();//关节数量

  for (auto i = 0; i < n_joints; ++i)//对每个关节
  {
    for (auto j = 1; j < traj.points.size() - 1; j++)//的每个轨迹点
    {
      // For each point in a given joint
      double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
      double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
      double v = delta_theta / delta_time;
      traj.points[j].velocities[i] = v;// 添加关节速度
    } 
  }
}
// 转换 笛卡尔轨迹 到 moveit 机械臂关节 轨迹 并添加关节速度
void DemoApplication::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                                      trajectory_msgs::JointTrajectory& out_traj)
	{
	//  // Fill out information about our trajectory
	  out_traj.header.stamp = ros::Time::now();//时间戳
	  out_traj.header.frame_id = config_.world_frame;//坐标系
	  out_traj.joint_names = config_.joint_names;//关节名字 列表

	  descartes_utilities::toRosJointPoints(*robot_model_ptr_, in_traj, 0.4, out_traj.points);
	  addVel(out_traj);// 添加关节速度
	}

} /* namespace plan_and_run */
