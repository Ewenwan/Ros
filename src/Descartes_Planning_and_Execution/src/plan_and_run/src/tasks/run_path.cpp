/*
使用movieit 执行笛卡尔规划器规划的轨迹
1 由轨迹第一个点 得到机械臂 各个关节的位姿 first_point_ptr->getNominalJointPose(seed_pose, *robot_model_ptr_, start_pose);
2 机械臂运动到 初始的 启动状态 move_group.setJointValueTarget(start_pose);//设定关节位姿
3 转换 笛卡尔轨迹 到 moveit 机械臂关节 轨迹 并添加关节速度 fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
4 发送执行轨迹的请求 给 执行轨迹的服务器   
   moveit_msgs::ExecuteKnownTrajectory srv; 
   srv.request.trajectory = moveit_traj;
   moveit_run_path_client_.call(srv)

*/
#include <plan_and_run/demo_application.h>

/* MOVE HOME
  Goal:
    - Move the robot to from its current location to the first point in the robot path.
    - Execute the robot path.

  Hints:
    - Use the "first_point_ptr->getNominalJointPose(...)" method in order to get the joint values at that position.
    - Execute the robot path by sending it to the moveit_msgs::ExecuteKnownTrajectory server by calling the
      "moveit_run_path_client_.call(srv)" client method.

*/

namespace plan_and_run
{

void DemoApplication::runPath(const DescartesTrajectory& path)
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // creating move group to move the arm in free space
  // moveit 执行器 接口
  moveit::planning_interface::MoveGroup move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);// id

  // creating goal joint pose to start of the path
  /*  Fill Code:
   * Goal:
   * - Retrieve the first point in the path.
   * - Save the joint values of the first point into "start_pose".
   * Hint:
   * - The first argument to "first_point_ptr->getNominalJointPose(...)" is a "std::vector<double>" variable
   *    where the joint values are to be stored.
   */
  // 由轨迹第一个点 得到机械臂 各个关节的位姿
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());// 机器人 初始关节位姿
  std::vector<double> start_pose;//机器人启动位姿
  //descartes_core::TrajectoryPtPtr first_point_ptr /* [ COMPLETE HERE ]: =  path[??]*/;
  /*[ COMPLETE HERE ]: first_point_ptr->getNominalJointPose(??,*robot_model_ptr_,start_pose); */
  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];//第一个点
  first_point_ptr->getNominalJointPose(seed_pose, *robot_model_ptr_, start_pose);

  // moving arm to joint goal
  // 机械臂运动到 初始的 启动状态
  move_group.setJointValueTarget(start_pose);//设定关节位姿
  move_group.setPlanningTime(10.0f);//
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // 转换 笛卡尔轨迹 到 moveit 机械臂关节 轨迹  并添加关节速度
  // creating Moveit trajectory from Descartes Trajectory
  moveit_msgs::RobotTrajectory moveit_traj;
  // demo_application.c
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);

  // sending robot path to server for execution
  /*  Fill Code:
   * Goal:
   * - Complete the service request by placing the "moveit_msgs::RobotTrajectory" trajectory in the request object
   * - Use the service client to send the trajectory for execution.
   * Hint:
   * - The "srv.request.trajectory" can be assigned the Moveit trajectory.
   * - The "moveit_run_path_client_.call(srv)" sends a trajectory execution request.
   */
  // 发送执行轨迹的请求 给 执行轨迹的服务器
  //  服务的提供节点 为 ros系统 /move_group 节点提供服务
  moveit_msgs::ExecuteKnownTrajectory srv;
  //srv.request.trajectory ; /* [ COMPLETE HERE ]: = ?? */;
  srv.request.trajectory = moveit_traj;
  srv.request.wait_for_execution = true;

  ROS_INFO_STREAM("Robot path sent for execution");
  //if(false /* [ COMPLETE HERE ]: moveit_run_path_client_.??( ?? ) */)
  if(moveit_run_path_client_.call(srv))
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
