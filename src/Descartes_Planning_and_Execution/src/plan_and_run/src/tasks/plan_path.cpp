/*
规划机器人路径
优化首尾点位姿
规划器 按轨迹点规划出 机器人笛卡尔轨迹 planner_.planPath(input_traj);
得到规划器 规划的 笛卡尔轨迹 succeeded = planner_.getPath(output_path);
*/
#include <plan_and_run/demo_application.h>

/* PLAN AND RUN
  Goal:
    - Plan a robot path from a trajectory using the path planner
    - Observe how to modify the start and end points in the trajectory so that
      unique joint poses at those points are used instead.  Skipping this step would cause planning
      for multiple path possibilities (multiple combinations of start and end), which
      could take a very long time.
*/

namespace plan_and_run
{

void DemoApplication::planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path)
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // modifying start and end points
  /*  Inspect Code:
   * Goal:
   * - Look into the use of the "getClosestJointPose(...)" method to obtain a single valid joint pose
   *    from a rotationally free cartesian point that is closest to a desired joint pose.
   * - Observe how the computed start_pose and end_pose joint poses are used to turn the start and end points
   *    in the trajectory to JointTrajectoryPts.
   * Hint:
   * - This manipulation of the trajectory allows guiding the planner so that the resulting path
   *    begins and ends near desirable joint configurations.  It also saves time by not having the
   *    planner search through potential path solutions whose start and end configurations are less adequate.
   */
  // 按照机器人模型 和 轨迹点集序列的首尾点 找到一个最近的合适位置 为新的 首尾点位姿
  std::vector<double> start_pose, end_pose;
  if(input_traj.front()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,start_pose) &&
      input_traj.back()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,end_pose))
  {
    ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");

    // Creating Start JointTrajectoryPt from start joint pose
    descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(start_pose));

    // Creating End JointTrajectoryPt from end joint pose
    descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(end_pose));

    // Modifying start and end of the trajectory.
    input_traj[0] = start_joint_point;
    input_traj[input_traj.size() - 1 ] = end_joint_point;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
    exit(-1);
  }

  // planning robot path
  /*  Fill Code:
   * Goal:
   * - Call the "planner_.planPath(...)" method in order to plan a robot path from the trajectory.
   * - Save the result of the planPath(...) call into the succeeded variable in order to verify that
   *     a valid robot path was generated.
   * Hint:
   * - The "planner_.planPath(...)" can take the "input_traj" Trajectory as an input argument.
   */
  //bool succeeded = false /* [ COMPLETE HERE ]*/;
  // 类头文件定义  descartes_planner::SparsePlanner planner_; // 笛卡尔稀疏规划器 
  // 初始化 笛卡尔规划器 bool succeeded = planner_.initialize(robot_model_ptr_);
  // 规划器 按轨迹点规划出 机器人笛卡尔轨迹
  bool succeeded = planner_.planPath(input_traj);

  if (succeeded)
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // retrieving robot path
  /*  Fill Code:
   * Goal:
   * - Call the "planner_.getPath(...)" in order to retrieve the planned robot path.
   * - Save the result of the planPath(...) call into the succeeded variable in order to verify that
   *     a valid robot path was generated.
   * Hint:
   * - The "planner_.getPath(...)" can take the "output_path" variable as an output argument.
   */
  //succeeded = false /* [ COMPLETE HERE ]*/;
  // 得到规划器 规划的 笛卡尔轨迹
  succeeded = planner_.getPath(output_path);

  if(!succeeded || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
}

}

