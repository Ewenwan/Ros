#include <ros/ros.h> //系统文件
#include <interactive_markers/interactive_marker_server.h>
//// 可视化交互 物体 interactive marker类型 visualization_msgs::InteractiveMarker
#include <interactive_markers/menu_handler.h>
//鼠标下拉菜单库
#include <tf/transform_broadcaster.h>//坐标变换库 
#include <tf/tf.h>
#include <math.h>  //数学函数库

using namespace visualization_msgs;  //使用可视化消息命名空间 调用变量时 省去visualization_msgs::


// 交互式marker服务器类型的 共享指针 服务器 多个（）
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
//visualization_msgs类下  interactive_markers::MenuHandler 鼠标下拉菜单变量
interactive_markers::MenuHandler menu_handler;

// // 创建 灰色 正方体 盒子 marker 函数
Marker makeBox( InteractiveMarker &msg )
{
  //msg 为  visualization_msgs::InteractiveMarker msg
 //visualization_msgs类下::Marker
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  return marker;
}
//  // 创建 可视化 相关控制
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  //msg 为  visualization_msgs::InteractiveMarker msg
  //visualization_msgs类下::InteractiveMarkerControl
  InteractiveMarkerControl control;
  control.always_visible = true;    //可见
  control.markers.push_back( makeBox(msg) ); //先创建盒子 再创建控制
  msg.controls.push_back( control );         //控制附着交互式marker
  return msg.controls.back();
}
// %EndTag(Box)%

//坐标系的回调函数
// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;        //计数器
  static tf::TransformBroadcaster br; //创建坐标广播
  tf::Transform tf;                    //创建坐标转换变量
  ros::Time time = ros::Time::now();  //时间戳
  tf.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0)); //与参考坐标系的相对位置
  tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));                   //四元素
    //发送坐标变换                     变换内容  时间戳  父坐标系（参考坐标系）   子坐标系
  br.sendTransform(tf::StampedTransform(tf, time, "base_link", "moving_frame"));//广播转换
 
  tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(tf, time, "base_link", "rotating_frame"));

  counter++;
}
// %EndTag(frameCallback)%

//鼠标操作等交互操作  回调函数 向控制台反馈 位置信息
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s; //创建总信息反馈
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;  //创建鼠标位置反馈信息
  if( feedback->mouse_point_valid )   //鼠标位置有效
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK: //点击
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:  //菜单
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE: //更新位置
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN: //鼠标按下 确定
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:   //鼠标松开 弹起
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges(); //可交互marker服务器 根据鼠标选择 更新（应用）变换（位置、方向等） 
}
// %EndTag(processFeedback)%

// 排列 marker
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  geometry_msgs::Pose pose = feedback->pose;         //姿态   位置 x y z  方向 w x y z
  pose.position.x = round(pose.position.x-0.5)+0.5;  //位置标准化 取为最近 的整数位置点
  pose.position.y = round(pose.position.y-0.5)+0.5;  //
//从之前的位置 排列 到现在的位置
  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x          
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose ); //更新姿态信息
  server->applyChanges();                         //应用改变
}
// %EndTag(alignMarker)%
//产生 min ~ max 随机数 函数
double rand( double min, double max )
{
  //确保 max >= min
  double temp;
  if(min>max) {temp=max; max=min; min=temp; }
  //产生0~1随机数
  double t = (double)rand() / (double)RAND_MAX;
  //产生min~max随机数
  return min + t*(max-min);
}
//
void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);//保存
  server->setCallback(int_marker.name, &processFeedback);//调用改变 反馈信息
}

// 六自由度
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
   //visualization_msgs类下::InteractiveMarker
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link"; //参考坐标系
  tf::pointTFToMsg(position, int_marker.pose.position);//与左边变换相关联  可变换int_marker.header.frame_id
  int_marker.scale = 1; //尺寸

  int_marker.name = "simple_6dof";//名字
  int_marker.description = "Simple 6-DOF Control";// 描述
  // insert a box
  makeBoxControl(int_marker);//// 创建 可视化 相关控制 盒子 marker
  int_marker.controls[0].interaction_mode = interaction_mode; //交互模式
   //交互模式 visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
   //visualization_msgs类下::InteractiveMarkerControl
  InteractiveMarkerControl control;

  if ( fixed ) //坐标系是否固定
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;//方向固定
  }
//确定交互模式  MOVE_AXIS（沿轴移动 ）MOVE_3D （任意方向移动，方向不变） ROTATE_3D（任意方向旋转，位置不可动）  MOVE_ROTATE_3D（任意移动+方向可变）
//show_6dof 的模式没有 要自己搭建
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)//交互模式非空
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      //
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }
  
//show_6dof 的模式没有 要自己搭建 仅仅x y z三个方向的移动和x y z三个轴的旋转
  if(show_6dof)
  {
    //方向 x轴旋转 x 轴移动
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;//旋转
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;//移动
    int_marker.controls.push_back(control);
    //方向 z轴旋转  
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;//旋转
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;//移动
    int_marker.controls.push_back(control);
   //方向 x轴旋转
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;//旋转
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;//移动
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);//保存
  server->setCallback(int_marker.name, &processFeedback);//调用改变 反馈信息
  
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%



// 创建随机六轴模式的交互式marker 
void makeRandomDofMarker( const tf::Vector3& position )
{
   //visualization_msgs类下::InteractiveMarker
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link"; //参考坐标系
  tf::pointTFToMsg(position, int_marker.pose.position);//与左边变换相关联  可变换int_marker.header.frame_id
  int_marker.scale = 1;//尺寸

  int_marker.name = "6dof_random_axes";// 名字
  int_marker.description = "6-DOF\n(Arbitrary Axes)";// 描述

  makeBoxControl(int_marker); // 创建 可视化 相关控制 盒子 marker

  InteractiveMarkerControl control; //创建 可视化 相关控制

  for ( int i=0; i<3; i++ ) //随机六轴
  {
    control.orientation.w = rand(-1,1);//随机
    control.orientation.x = rand(-1,1);
    control.orientation.y = rand(-1,1);
    control.orientation.z = rand(-1,1);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);//保存
  server->setCallback(int_marker.name, &processFeedback); //调用改变 反馈信息
}
// %EndTag(RandomDof)%


//  任意方向移动 + 面对使用者方向的 旋转
void makeViewFacingMarker( const tf::Vector3& position )
{
  //visualization_msgs类下::InteractiveMarker
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link"; //参考坐标系
  tf::pointTFToMsg(position, int_marker.pose.position);//与左边变换相关联  可变换int_marker.header.frame_id
  int_marker.scale = 1;//尺寸

  int_marker.name = "view_facing";// 名字
  int_marker.description = "View Facing 6-DOF";// 描述

  InteractiveMarkerControl control; //创建 可视化 相关控制

  // make a control that rotates around the view axis 旋转
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane. 移动
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%


// 直升机 maker    //xoy平面旋转+平移+ z轴上下移动
void makeQuadrocopterMarker( const tf::Vector3& position )
{
  //visualization_msgs类下::InteractiveMarker
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link"; //参考坐标系
  tf::pointTFToMsg(position, int_marker.pose.position);//与左边变换相关联  可变换int_marker.header.frame_id
  int_marker.scale = 1;//尺寸

  int_marker.name = "quadrocopter";//名字
  int_marker.description = "Quadrocopter";//描述

  makeBoxControl(int_marker);//创建盒子

  InteractiveMarkerControl control;//创建控制

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;//轴旋转  ？ Y->Z  Z->Y??
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE; //xoy平面旋转+平移
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;  //z轴上下移动
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%  xoy平面移动
void makeChessPieceMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%   两个方向的旋转
void makePanTiltMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(PanTilt)%

// %Tag(Menu)% 创建带有 鼠标菜单 的marker
void makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "context_menu";//名字
  int_marker.description = "Context Menu\n(Right Click)";//描述

  InteractiveMarkerControl control;//控制

  control.interaction_mode = InteractiveMarkerControl::MENU;//控制模式
  control.name = "menu_only_control";//控制名字

  Marker marker = makeBox( int_marker );//创建盒子
  control.markers.push_back( marker );//应用控制模式
  control.always_visible = true;      //永久可视化
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Button)% 创建带有 鼠标点击的marker
void makeButtonMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Button)%

// 移动的marker
void makeMovingMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS; //x轴方向旋转 control.orientation.x = 1
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;   //任意方向移动
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");  //初始化ros系统  注册节点 节点名字为 basic_controls
  ros::NodeHandle nh;

  // create a timer to update the published transforms 参考系更新frameCallback
  ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert( "First Entry", &processFeedback );
  menu_handler.insert( "Second Entry", &processFeedback );
  //子菜单句柄sub_menu_handle
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  tf::Vector3 position;
  position = tf::Vector3(-3, 3, 0); //位置
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
  position = tf::Vector3( 0, 3, 0);
  make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
  position = tf::Vector3( 3, 3, 0);
  makeRandomDofMarker( position );
  position = tf::Vector3(-3, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );
  position = tf::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
  position = tf::Vector3( 3, 0, 0);
  make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false );
  position = tf::Vector3(-3,-3, 0);
  makeViewFacingMarker( position );
  position = tf::Vector3( 0,-3, 0);
  makeQuadrocopterMarker( position );
  position = tf::Vector3( 3,-3, 0);
  makeChessPieceMarker( position );
  position = tf::Vector3(-3,-6, 0);
  makePanTiltMarker( position );
  position = tf::Vector3( 0,-6, 0);
  makeMovingMarker( position );
  position = tf::Vector3( 3,-6, 0);
  makeMenuMarker( position );
  position = tf::Vector3( 0,-9, 0);
  makeButtonMarker( position );

  server->applyChanges(); //应用改变

  ros::spin(); //给ros控制权

  server.reset();
}
// %EndTag(main)%