/*
获取放置物体 运动规划的路径 位姿
设置工具到 放置物体位姿
获取tcp路径位姿
tcp到wrist的坐标变换 
转换到wrist下 
*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    CREATE PLACE MOVES
  Goal:
    - 设置工具到 放置物体位姿 Set the pose of the tcp at the box place pose
    - 获取tcp路径位姿Create tcp poses for the place motion (approach, release, retreat).
    - tcp到wrist的坐标变换 Find transform of the wrist in tcp coordinates
    - 转换到wrist下 Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Use the "create_manipulation_poses" function to create the tcp poses between each place move
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_place_poses" array.
*/

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_place_moves()
{
  // ROS_ERROR_STREAM("create_place_moves is not implemented yet.  Aborting."); exit(1);

  // 任务变量 坐标变换 task variables
  tf::Transform world_to_tcp_tf;// wd->tcp
  tf::StampedTransform tcp_to_wrist_tf;// tcp->wrist
  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;// tcp规划位姿 wrist规划位姿


  /* Fill Code:
   * Objective:
   * - Find the desired tcp pose at box place
   * Hints:
   * - Use the "setOrigin" method to set the position of "world_to_tcp_tf"
   * 	using cfg.BOX_PLACE_TF.
   * - cfg.BOX_PLACE_TF is a tf::Transform object so it provides a getOrigin() method.
   */
  /* ========  ENTER CODE HERE ======== */
  // TCP位置设置为目标点位置
  world_to_tcp_tf.setOrigin(cfg.BOX_PLACE_TF.getOrigin());

  /* Fill Code:
   * Goal:
   * - Reorient the tool so that the tcp points towards the box.
   * Hints:
   * - Use the "setRotation" to set the orientation of "world_to_tcp_tf".
   * - The quaternion value "tf::Quaternion(M_PI, 0, M_PI/2.0f)" will point
   * 	the tcp's direction towards the box.
   */
  /* ========  ENTER CODE HERE ======== */
  // tcp方向设置为 目标物体方向
   world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI/2.0f));

  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the "create_manipulation_poses" and save results to "tcp_place_poses".
   * - Look in the "cfg" object to find the corresponding retreat and approach distance
   * 	values.
   */
  /* ========  ENTER CODE HERE ======== */
  // 获取tcp路径位姿
  // pick_and_place_utilities.c中实现 
  // 工具 指定位置姿态后 获取 运动路径 位姿点 起点 目标点 终点 
  // 起点根据 接近距离确定
  // 终点根据 离开距离确定
  // 目标点是 目标位置的位姿
  tcp_place_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, 
  		  			      cfg.APPROACH_DISTANCE,
					      world_to_tcp_tf);

  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method in the transform listener.
   * */
  //等待坐标变换
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), ros::Duration(3.0f));
  /* ========  ENTER CODE HERE ======== */
  // 获取工具到 手腕的坐标变换
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,
 			    	 	  ros::Time(0.0f), tcp_to_wrist_tf);

  /* Fill Code:
   * Goal:
   * - Transform list of place poses from the tcp to the wrist coordinate frame.
   * Hints:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_place_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_place_poses"
   * 	into "wrist_place_poses".
   */
  /* ========  ENTER CODE HERE ======== */
  // 工具TCP路径位姿点 转换到 手腕路径位姿点
  wrist_place_poses = transform_from_tcp_to_wrist( tcp_to_wrist_tf, tcp_place_poses);

  std::cout << "Creating place move poses succecc ..."<< std::endl;

  // printing results
  ROS_INFO_STREAM("tcp position at place: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at place: "<<wrist_place_poses[1].position);

  return wrist_place_poses;
}

