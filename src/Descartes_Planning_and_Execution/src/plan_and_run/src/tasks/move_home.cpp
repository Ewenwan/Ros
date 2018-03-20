/*
移动到等候区（家位置）
使用moveit 运动规划接口执行

*/
#include <plan_and_run/demo_application.h>

/* MOVE HOME
  Goal:
    - Use the moveit MoveGroup interface to move the arm to a pre-recorded positions saved in the moveit config package.
    - Verify that the arm reached the target.

  Hints:
    - Call the "move_group_interface::MoveGroup::move()" method to move the arm.
    - The "result.val" is an integer flag which indicates either success or an error condition.
*/

namespace plan_and_run
{

void DemoApplication::moveHome()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // 创建moveit规划接口 creating move group interface for planning simple moves
  moveit::planning_interface::MoveGroup move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);//设置规划器ID 区别

  // 设置目标位置（家的位置） setting home position as target 
  if(!move_group.setNamedTarget(HOME_POSITION_NAME))
  {
    ROS_ERROR_STREAM("Failed to set home '"<<HOME_POSITION_NAME<<"' position");
    exit(-1);
  }

  // moving home
  /*  Fill Code:
   * Goal:
   * - Call the move_group.move() and save the returned flag into the "result" variable
   * - Verify that the robot reached the goal.
   * Hint:
   * - The "result.val" and "result.SUCCESS" flags can be used to verify that the move was completed
   * -
   */
  //moveit_msgs::MoveItErrorCodes result /* [ COMPLETE HERE ]: = move_group.??() ;*/;
  //if(false /* [ COMPLETE HERE ]: result.?? != result.?? */)
// 
// 规划执行 并返回结果
  moveit_msgs::MoveItErrorCodes result  = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to move to "<<HOME_POSITION_NAME<<" position");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached home position");
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
/*
moveit_msgs/MoveItErrorCodes.msg
int32 val

# overall behavior
int32 SUCCESS=1
int32 FAILURE=99999

int32 PLANNING_FAILED=-1
int32 INVALID_MOTION_PLAN=-2
int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
int32 CONTROL_FAILED=-4
int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
int32 TIMED_OUT=-6
int32 PREEMPTED=-7

# planning & kinematics request errors
int32 START_STATE_IN_COLLISION=-10
int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11

int32 GOAL_IN_COLLISION=-12
int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
int32 GOAL_CONSTRAINTS_VIOLATED=-14

int32 INVALID_GROUP_NAME=-15
int32 INVALID_GOAL_CONSTRAINTS=-16
int32 INVALID_ROBOT_STATE=-17
int32 INVALID_LINK_NAME=-18
int32 INVALID_OBJECT_NAME=-19

# system errors
int32 FRAME_TRANSFORM_FAILURE=-21
int32 COLLISION_CHECKING_UNAVAILABLE=-22
int32 ROBOT_STATE_STALE=-23
int32 SENSOR_INFO_STALE=-24

# kinematics errors
int32 NO_IK_SOLUTION=-31

*/
