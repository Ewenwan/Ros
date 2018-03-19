/*
运动到等候区　moveit 实现 


*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the "move_group" interface to move the robot to the "wait" target.
    - Observe how we verify that the move was completed

  Hints:
    - "cfg.WAIT_POSE_NAME" contains the name of the wait target.
    - Once the target is set you can call the "move" method in order to go to that target.
*/

void collision_avoidance_pick_and_place::PickAndPlace::move_to_wait_position()
{
 //  ROS_ERROR_STREAM("move_to_wait_position is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success; // saves the move result  规划执行结果

  /* Fill Code:
   * Goal:
   * - Set robot wait target
   * Hints:
   * - Use the "setNamedTarget" method in the "move_group" object.
   * - Look in the "cfg.WAIT_POSE_NAME" object for the name of the target.
   */
  // 设置规划目标位置(按位置命名设置)
  move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);
  // geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
   //move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　按位置设置　目标位置
  /* ========  ENTER CODE HERE ======== */
  // 
  std::cout << "Move to waitting place..." << std::endl;
  // set allowed planning time　设置规划运行的执行时间
  move_group_ptr->setPlanningTime(60.0f);

  /* Fill Code:
   * Goal:
   * - Move the robot
   * Hints:
   * - Use the "move" method in the "move_group" object and save the result
   *  in the "success" variable
   */
  success = move_group_ptr->move();// 执行　运动规划　并返回执行结果　
  /* ========  ENTER CODE HERE ======== */
  if(success)
  {
    ROS_INFO_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Failed");
    exit(1);
  }
}


