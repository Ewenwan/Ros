/*
创建从当前位置到抓取点的 手腕位姿路径
首先获取工具的 位姿
监听工具到 手腕的坐标变换
变换到手腕 的位姿

TCP tool center point 工具中心点
是机械手臂前端工具的中心点　　是工具坐标系TCS Tool Coordinate System 的坐标系原点
常规tcp: 
	tcp跟随机器人本体一起运动，TCS的xy平面在机器人第六轴的法兰盘平面上，TCS原点与法兰盘中心重合。
	显然TCP在法兰盘中心，　ABB把TCP称为　tool0, REIS机器人称为　_tnull
        在实际使用时，常将TCP点定义到工具末端
*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* CREATE PICK MOVES
  Goal:
    - Set the pose for the tcp( tool center point 工具中心点) at the box pick.
    - Create tcp poses for the pick motions (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * Moveit's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Look into the "create_manipulation_poses" function and observe how each pick pose is created.
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_pick_poses" array.
*/

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_pick_moves(geometry_msgs::Pose &box_pose)
{
  // ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // 任务变量　task variables
  tf::Transform world_to_tcp_tf;// tool center point 工具中心点 坐标系 坐标变换
  tf::Transform world_to_box_tf;// 箱子坐标系  坐标变换
  tf::StampedTransform tcp_to_wrist_tf;//  工具 到 机械手腕 坐标变换
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;//工具、机械手腕  抓取 位姿 

  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the "setOrigin" to set the position of "world_to_tcp_tf".
   */
  tf::poseMsgToTF(box_pose,world_to_box_tf);//目标箱子位姿转换到 世界坐标系下
  tf::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  /* ========  ENTER CODE HERE ======== */
  world_to_tcp_tf.setOrigin(box_position);// 工具原点定义为 目标物体位置
  /* Setting tcp orientation 工具指向目标物体 方向
	   * Inverting the approach direction so that the tcp points towards the box instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));

  // create all the poses for tcp's pick motion (approach, pick and retreat) 工具 操作位姿
  // pick_and_place_utilities.c中实现 
  // 工具 指定位置姿态后 获取 运动路径 位姿点 起点 目标点 终点 
  // 起点根据 接近距离确定
  // 终点根据 离开距离确定
  // 目标点是 目标位置的位姿
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);

  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method in the transform listener.
   */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,ros::Time::now(),ros::Duration(3.0f));//等待坐标变换
  /* ========  ENTER CODE HERE ======== */
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);// 工具到 手腕的坐标变换

  /* Fill Code:
   * Goal:
   * - Transform list of pick poses from tcp frame to wrist frame
   * Hint:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_pick_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_pick_poses"
   * 	into "wrist_pick_poses".
   */
  /* ========  ENTER CODE HERE ======== */
  // pick_and_place_utilities.c中实现 
  // 工具TCP路径位姿点 转换到 手腕路径位姿点
  //TCP点->wd点  wd->tcp->wrist  wd点 × wd->wrist = wrist点
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);//转换 工具的 位姿 到手腕 的位姿

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);

  std::cout << "Creating pick poses succecc ..."<< std::endl;

  return wrist_pick_poses;
}


