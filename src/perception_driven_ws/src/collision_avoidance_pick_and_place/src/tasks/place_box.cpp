/*
放置物体 moveit实现
拿住货物  set_attached_object(true,box_pose,robot_state);
moveit   运动规划执行
到卸货区 放下货物 set_attached_object(false,geometry_msgs::Pose(),robot_state);

*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot through the entire place motion.
    - Open gripper after reaching the release pose.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void collision_avoidance_pick_and_place::PickAndPlace::place_box(std::vector<geometry_msgs::Pose>& place_poses,
		const geometry_msgs::Pose& box_pose)
{
  //ROS_ERROR_STREAM("place_box is not implemented yet.  Aborting."); exit(1);

  // 任务变量 task variables
  bool success;

  /* Fill Code:
   * Goal:
   * - Set the ReferenceFrame and EndEffectorLink
   * Hints:
   * - Use the "setEndEffectorLink" and "setPoseReferenceFrame" methods of "move_group_ptr"
   */
  /* ========  ENTER CODE HERE ======== */
  /* ========  ENTER CODE HERE ======== */

  // 设置参考坐标系
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);
  // 设置moveit规划组末端link
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

  // 允许的最大规划时间 set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
  	moveit_msgs::RobotState robot_state;

  	if(i==0 || i == 1)// 起始点 目标点
  	{
      // attaching box  拿住 物体
      set_attached_object(true,box_pose,robot_state);
      show_box(true);
  	}
  	else//终点 放货
  	{
      // detaching box 到卸货区 放下货物
      set_attached_object(false,geometry_msgs::Pose(),robot_state);
      show_box(false);
  	}

  	//  moveit 规划并执行 create motion plan
    moveit::planning_interface::MoveGroup::Plan plan;
    success = create_motion_plan(place_poses[i],robot_state,plan) && move_group_ptr->execute(plan);

    if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      set_gripper(false);
      exit(1);
    }


    if(i == 1)
    {
	/* Fill Code:
	 * Goal:
	 * - Turn off gripper suction after the release pose is reached.
	 * Hints:
	 * - Call the "set_gripper" function to turn on suction.
	 * - The input to the set_gripper method takes a "true" or "false"
	 * 	  boolean argument.
	 */
	 /* ========  ENTER CODE HERE ======== */
      set_gripper(true);

    }

  }
   std::cout << "Place box succecc ..."<< std::endl;
}



