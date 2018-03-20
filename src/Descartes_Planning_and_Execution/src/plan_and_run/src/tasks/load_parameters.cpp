/*载入参数
使用私有节点获取参数 ros::NodeHandle ph("~");
*/
#include <plan_and_run/demo_application.h>

/* LOAD PARAMETERS
  Goal:
    - Load missing application parameters into the from the ros parameter server.
    - Use a private NodeHandle in order to load parameters defined in the node's namespace.

  Hints:
    - Look at how the 'config_' structure is used to save the parameters.
    - A private NodeHandle can be created by passing the "~" string to its constructor.
*/

namespace plan_and_run
{

void DemoApplication::loadParameters()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);



  /*  Fill Code:
   * Goal:
   * - Create a private handle by passing the "~" string to its constructor
   * Hint:
   * - Replace the string in ph below with "~" to make it a private node.
   */
  //ros::NodeHandle ph("[ COMPLETE HERE ]");
  ros::NodeHandle ph("~");

  // creating handle with public scope
  ros::NodeHandle nh;


  /*  Fill Code:
   * Goal:
   * - Read missing parameters 'tip_link' and 'world_frame' from ros parameter server
   * 类私有结构体变量 DemoConfiguration config_;
   */
  if(ph.getParam("group_name",config_.group_name) &&

      //ph.getParam("[ COMPLETE HERE ]",config_.base_link) &&
      ph.getParam("tip_link",config_.tip_link) &&

      ph.getParam("base_link",config_.base_link) &&

      //ph.getParam("[ COMPLETE HERE ]",config_.base_link) &&
      ph.getParam("world_frame",config_.world_frame) &&

      ph.getParam("trajectory/time_delay",config_.time_delay) &&
      ph.getParam("trajectory/foci_distance",config_.foci_distance) &&
      ph.getParam("trajectory/radius",config_.radius) &&
      ph.getParam("trajectory/num_points",config_.num_points) &&
      ph.getParam("trajectory/num_lemniscates",config_.num_lemniscates) &&
      ph.getParam("trajectory/center",config_.center) &&
      ph.getParam("trajectory/seed_pose",config_.seed_pose) &&
      ph.getParam("visualization/min_point_distance",config_.min_point_distance) &&
      nh.getParam("controller_joint_names",config_.joint_names) )
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }
// __FUNCTION__ 为函数名
  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
