/*
执行拿起箱子 使用 moveit 
将手腕终端机构 运动到 目标物体附近
起点->目标点->打开抓手->终点

*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Use the "move_group" object to set the wrist as the end-effector link
    - Use the "move_group" object to set the planning reference frame.
    - Move the robot through the pick motion.
    - Close the gripper when appropriate.

  Hints:
    - The "move_group" interface has useful methods such as "setEndEffectorLink"
    	and "setPoseReferenceFrame" that can be used to prepare the robot for planning.
    - The "setPoseTarget" method allows you to set a "pose" as your target
    	to move the robot.
*/
void collision_avoidance_pick_and_place::PickAndPlace::pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose)
{
	//  ROS_ERROR_STREAM("pickup_box is not implemented yet.  Aborting."); exit(1);

	  // 任务变量 task variables
	  bool success;

	  /* Fill Code:
	   * Goal:
	   * - Set the wrist as the end-effector link
	     	 - The robot will try to move this link to the specified pose
	     	 - If not specified, moveit will use the last link in the arm group.
	   * Hints:
	   * - Use the "setEndEffectorLink" in the "move_group_ptr" object.
	   * - The WRIST_LINK_NAME field in the "cfg" configuration member contains
	   * 	the name for the arm's wrist link.
	   */
	  /* ========  ENTER CODE HERE ======== */
 	  // 设置moveit规划组末端link
	  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

	  // set allowed planning time 允许的最大规划时间
	  move_group_ptr->setPlanningTime(60.0f);


	  /* Fill Code:
	   * Goal:
	   * - Set world frame as the reference
	    	- The target position is specified relative to this frame
	   	  	- If not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
	   * Hints:
	   * - Use the "setPoseReferenceFrame" in the "move_group_ptr" object.
	   * - The WORLD_FRAME_ID in the "cfg" configuration member contains the name
	   * 	for the reference frame.
	   */
	  /* ========  ENTER CODE HERE ======== */
	  // 设置参考坐标系
	  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);
	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {
	  	moveit_msgs::RobotState robot_state;

	  /* Inspect Code:
	   * Goal:
	   * - Look in the "set_attached_object()" method to understand
	   * 	how to attach a payload using moveit.
	   */
		// /src/tasks/set_attached_object.cpp　　// 不吸取
		 set_attached_object(false,geometry_msgs::Pose(),robot_state);
		//set_attached_object(true,box_pose,robot_state);//吸附抓取的对象 装货

	  /* Inspect Code:
	   * Goal:
	   * - Look in the "create_motion_plan()" method to observe how an
	   * 	entire moveit motion plan is created.
	   */
             //规划并执行
            	moveit::planning_interface::MoveGroup::Plan plan;
	        success = create_motion_plan(pick_poses[i], robot_state,plan) &&
					     move_group_ptr->execute(plan);

	    // verifying move completion
	        if(success)
	        {
	          ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
	         }
	        else
	        {
	         ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
	         set_gripper(false);
	         exit(1);
	       }
   

	    if(i == 0)
	    {

		/* Fill Code:
		 * Goal:
		 * - Turn on gripper suction after approach pose is reached.
		 * Hints:
		 * - Call the "set_gripper" function to turn on suction.
		 * - The input to the set_gripper method takes a "true" or "false"
		 * 	  boolean argument.
		 */
	     /* ========  ENTER CODE HERE ======== */
		set_gripper(true);//
	    }

	  }
          std::cout << "Pickup box succecc ..."<< std::endl;

}

