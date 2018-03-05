	# ros 工业机器人 实战

	参考　
	http://ros-industrial.github.io/industrial_training/_source/session4/Introduction-to-Perception.html

	安装ar 仿真工件　测试包
	sudo apt install ros-indigo-calibration-msgs
	cd ~/catkin_ws/src
	git clone https://github.com/jmeyer1292/fake_ar_publisher.git

	source devel/setup.bash 
	rospack find fake_ar_publisher  //找到安装包

	创建包
	cd ~/catkin_ws/src
	catkin_create_pkg myworkcell_core roscpp

	cd myworkcell_core
	gedit package.xml

	// 修改
	  <build_depend>roscpp</build_depend> 编译依赖
	  <build_depend>fake_ar_publisher</build_depend>
	  <run_depend>fake_ar_publisher</run_depend> 运行依赖
	  <run_depend>roscpp</run_depend>


	gedit CMakeList.txt
	// 修改
	add_compile_options(-std=c++11) # 支持c++ 11 新标准

	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  fake_ar_publisher#依赖其他的包
	)

	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES myworkcell_core
	  CATKIN_DEPENDS 
	    roscpp
	    fake_ar_publisher#运行依赖 其他的包
	#  DEPENDS system_lib
	)

	add_executable(vision_node src/vision_node.cpp) # 可执行文件
	#依赖
	add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	#连接
	target_link_libraries(vision_node ${catkin_LIBRARIES})

	查看信息列表
	rosmsg list
	显示信息详情
	rosmsg show fake_ar_publisher/ARMarker
	>>>
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	uint32 id
	geometry_msgs/PoseWithCovariance pose
	  geometry_msgs/Pose pose
	    geometry_msgs/Point position
	      float64 x
	      float64 y
	      float64 z
	    geometry_msgs/Quaternion orientation
	      float64 x
	      float64 y
	      float64 z
	      float64 w
	  float64[36] covariance
	uint32 confidence



	/**
	**  Simple ROS Node
	**  vision_node.cpp
	**/
	#include <ros/ros.h>// 系统头文件
	#include <fake_ar_publisher/ARMarker.h>// 自定义消息头文件

	// 自定义类
	class Localizer
	{
	public:
	  // 类 初始化函数
	  Localizer(ros::NodeHandle& nh)//节点句柄引用
	  {
	      // 订阅者                     消息类型                 话题名      队列大小
	      ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
	      &Localizer::visionCallback, this);// 回调函数指针  类自己
	  }
	  // 话题回调函数
	  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)// 引用 常量指针
	  {
	      last_msg_ = msg;//复制一下放置 变化
	      ROS_INFO_STREAM(last_msg_->pose.pose);//打印位置
	  }

	// 变量定义
	  ros::Subscriber ar_sub_;// 订阅者
	  fake_ar_publisher::ARMarkerConstPtr last_msg_;//常量指针
	};

	int main(int argc, char *argv[] ){
		// 初始化 ros节点
		ros::init(argc, argv, "vision_node");

		// 创建ros节点句柄
		ros::NodeHandle nh;
		Localizer localizer(nh);

		ROS_INFO("Vision node starting");

		// 节点 存活  rosnode list 可以一直看到
		ros::spin();

	}

	运行 
	roscore
	rosrun fake_ar_publisher fake_ar_publisher_node
	rosrun workcell_core vision_node
	查看节点间关系
	rqt_graph

	###############################################################################
	服务 类似函数调用------------------------------------------------------
	srv/LocalizePart.srv
	#request 请求
	string base_frame #目标基坐标系 名称
	---
	#response 回应
	geometry_msgs/Pose pose # 目标坐标系位姿 位置 和 姿态

	---------------------------------------------------------------------
	gedit package.xml
	// 修改
	  <build_depend>roscpp</build_depend>
	  <build_depend>fake_ar_publisher</build_depend>
	  <build_depend>message_generation</build_depend>
	  <build_depend>geometry_msgs</build_depend>

	  <run_depend>roscpp</run_depend>
	  <run_depend>fake_ar_publisher</run_depend>
	  <run_depend>message_runtime</run_depend>
	  <run_depend>geometry_msgs</run_depend>
	-------------------------------------------------------------------
	gedit CMakeList.txt
	// 修改

	# 包编译依赖
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  fake_ar_publisher#依赖其他的包
	  message_generation
	  geometry_msgs
	)

	#添加服务文件
	## Generate services in the 'srv' folder
	 add_service_files(
	   FILES
	   LocalizePart.srv
	#   Service1.srv
	#   Service2.srv
	 )

	#信息生成 依赖
	## Generate added messages and services with any dependencies listed here
	 generate_messages(
	   DEPENDENCIES
	   std_msgs  # Or other packages containing msgs
	   geometry_msgs
	 )

	#包运行依赖
	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES myworkcell_core
	  CATKIN_DEPENDS 
	    roscpp
	    fake_ar_publisher#运行依赖 其他的包
	    message_runtime
	    geometry_msgs
	#  DEPENDS system_lib
	)

	# 提供获取目标位置 服务的节点 订阅话题监控 服务
	add_executable(ARserver_node src/ARserver.cpp) # 可执行文件
	#依赖
	add_dependencies(ARserver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	#连接
	target_link_libraries(ARserver_node ${catkin_LIBRARIES})


	# 客户端 请求获取目标位置的服务
	add_executable(ARclient_node src/ARclient.cpp) # 可执行文件
	#依赖
	add_dependencies(ARclient_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	#连接
	target_link_libraries(ARclient_node ${catkin_LIBRARIES})

	---------------------------------------------------------------------
	/**
	**  Simple ROS Node
	**  ARserver.cpp   服务器
	**/
	#include <ros/ros.h>// 系统头文件
	// git clone https://github.com/jmeyer1292/fake_ar_publisher.git 安装
	#include <fake_ar_publisher/ARMarker.h>// 自定义消息头文件
	#include <myworkcell_core/LocalizePart.h>// 服务头文件

	// 自定义类
	class Localizer
	{
	public:
	  // 类 初始化函数
	  Localizer(ros::NodeHandle& nh)//节点句柄引用
	  {
	      // 订阅者                     消息类型                 话题名      队列大小
	      ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
	      &Localizer::visionCallback, this);// 回调函数指针  类自己
	      // 服务器订阅服务
	      server_ = nh.advertiseService("localize_part", &Localizer::localizerPart, this);
	  }
	  // 话题回调函数
	  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)// 引用 常量指针
	  {
	      last_msg_ = msg;//复制一下放置 变化
	     //  ROS_INFO_STREAM(last_msg_->pose.pose);//打印位置
	  }

	  // 服务回调函数
	  bool localizePart(myworkcell_core::LocalizePart::Request& req,//请求
			      myworkcell_core::LocalizePart::Response& res)// 回应
	  {
	      // Read last message
	      fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
	      if (!p) return false;//空指针 无信息

	      res.pose = p->pose.pose;
	      return true;
	  }

	// 变量定义
	  ros::Subscriber ar_sub_;// 订阅者
	  fake_ar_publisher::ARMarkerConstPtr last_msg_;//常量指针
	  ros::ServiceServer server_;// 服务器
	};


	int main(int argc, char *argv[] ){
		// 初始化 ros节点
		ros::init(argc, argv, "ARserver");

		// 创建ros节点句柄
		ros::NodeHandle nh;
		Localizer localizer(nh);

		ROS_INFO("Vision node starting");

		// 节点 存活  rosnode list 可以一直看到
		ros::spin();

	}

	----------------------------------------------------------

	/**
	**  Simple ROS Node
	**  ARclient.cpp  客户端
	**/
	#include <ros/ros.h>// 系统头文件
	#include <myworkcell_core/LocalizePart.h>// 服务头文件

	// 自定义类
	class ScanNPlan
	{
	public:
	// 类 初始化函数
	  ScanNPlan(ros::NodeHandle& nh)//节点句柄引用
	  {
	    // 客户端                             服务类型                      请求的服务名字
	    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
	  }
	// 执行函数
	  void start()
	  {
	    // 打印信息
	    ROS_INFO("Attempting to localize part");
	    // Localize the part
	    myworkcell_core::LocalizePart srv;// 初始化 服务
	    if (!vision_client_.call(srv))//调用服务 得到响应数据
	    {
	      ROS_ERROR("Could not localize part");
	      return;
	    }
	    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应
	  }

	private:
	  // Planning components
	  ros::ServiceClient vision_client_;// 私有变量 类内使用
	};


	int main(int argc, char **argv)
	{
	  // 初始化 ros节点
	  ros::init(argc, argv, "ARclient");// 初始化 ros节点
	  // 创建ros节点句柄
	  ros::NodeHandle nh;

	  ROS_INFO("ScanNPlan node has been initialized");

	  ScanNPlan app(nh);

	  ros::Duration(.5).sleep();  // wait for the class to initialize
	  app.start();

	  ros::spin();// 节点 存活  rosnode list 可以一直看到
	}
	-----------------------------------------------------------------------
	运行
	roscore
	rosrun fake_ar_publisher fake_ar_publisher_node
	rosrun myworkcell_core ARserver_node
	rosrun myworkcell_core ARclient_node

	---------------------------------------------------------------------


	###################################################
	##########################################################
	Action 行动   长周期的 任务  复杂序列 任务 


	######################################################
	launch 文件 多个节点启动
	launch/workcell.launch
	<launch>
		<node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
		<node name="ARserver_node" pkg="myworkcell_core" type="ARserver_node" />
		<node name="ARclient_node" pkg="myworkcell_core" type="ARclient_node"  output="screen" />
	</launch>

	运行 
	roslaunch myworkcell_core workcell.launch 
	------------------------------------------------------------------


	#############################################################
	参数 
	param
	/**
	**  Simple ROS Node
	**  ARclient.cpp  客户端
	**/
	#include <ros/ros.h>// 系统头文件
	#include <myworkcell_core/LocalizePart.h>// 服务头文件

	// 自定义类
	class ScanNPlan
	{
	public:
	// 类 初始化函数
	  ScanNPlan(ros::NodeHandle& nh)//节点句柄引用
	  {
	    // 客户端                             服务类型                      请求的服务名字
	    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
	  }
	// 执行函数
	  void start(const std::string& base_frame)
	  {
	    // 打印信息
	    ROS_INFO("Attempting to localize part");
	    // Localize the part
	    myworkcell_core::LocalizePart srv;// 初始化 服务
	    srv.request.base_frame = base_frame; 
	    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

	    if (!vision_client_.call(srv))//调用服务 得到响应数据
	    {
	      ROS_ERROR("Could not localize part");
	      return;
	    }
	    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应
	  }

	private:
	  // Planning components
	  ros::ServiceClient vision_client_;// 私有变量 类内使用
	};

	int main(int argc, char **argv)
	{
	  // 初始化 ros节点
	  ros::init(argc, argv, "ARclient");// 初始化 ros节点
	  // 创建ros节点句柄
	  ros::NodeHandle nh;

	  ros::NodeHandle private_node_handle("~");// 增加一个私有节点 获取目标对象坐标系参数

	  ROS_INFO("ScanNPlan node has been initialized");

	  std::string base_frame;// string 对象变量
	  // 私有节点获取参数                      坐标系参数名  存储变量    默认值
	  private_node_handle.param<std::string>("base_frame", base_frame ,"world"); 


	  ScanNPlan app(nh);// 客户端
	  ros::Duration(.5).sleep();  // 等待客户端初始化完成 wait for the class to initialize
	  app.start(base_frame);// 请求服务

	  ros::spin();// 节点 存活  rosnode list 可以一直看到
	}
	---------------------------------------------------------------
	launch文件中设置参数
	launch/workcell.launch
	<launch>
		<node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
		<node name="ARserver_node" pkg="myworkcell_core" type="ARserver_node" />
		<node name="ARclient_node" pkg="myworkcell_core" type="ARclient_node"  output="screen">
		    <param name="base_frame" value="world"/>
		</node>
	</launch>

	运行 
	roslaunch myworkcell_core workcell.launch 

	-----------------------------------------------------------


	###################################################
	urdf文件 机器人描述语言
	urdf/workcell.urdf
	------------------------------------------------------------------
	<?xml version="1.0" ?>
	<robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">

	<!-- Add the world frame as a "virtual link" (no geometry) -->
	  <link name="world"/>

	<!-- the table frame  -->
	  <link name="table">
	    <visual>
	      <geometry>
		<box size="1.0 1.0 0.05"/>
	      </geometry>
	    </visual>
	    <collision>
	      <geometry>
		<box size="1.0 1.0 0.05"/>
	      </geometry>
	    </collision>
	  </link>

	<!-- Add the camera_frame frame as another virtual link (no geometry)  -->
	  <link name="camera_frame"/>

	<!-- joint -->
	  <joint name="world_to_table" type="fixed">
	    <parent link="world"/>
	    <child link="table"/>
	    <!-- up to z 0.5m -->
	    <origin xyz="0 0 0.5" rpy="0 0 0"/>
	  </joint>

	  <joint name="world_to_camera" type="fixed">
	    <parent link="world"/>
	    <child link="camera_frame"/>
	    <origin xyz="-0.25 -0.5 1.25" rpy="0 3.14159 0"/>
	  </joint>
	</robot>
	--------------------------------------------------------------

	roslaunch urdf_tutorial display.launch model:=workcell.urdf

	################################################################
	XACRO语言
	// 安装 ur机器人  丹麦UR优傲机械臂  http://www.ur5.cc/UR10.html
	cd ~/catkin_ws/src
	git clone https://github.com/ros-industrial/universal_robot.git
	catkin build
	source ~/catkin_ws/devel/setup.bash


	urdf/workcell.xacro

	----------------------------------------------------------
	<?xml version="1.0" ?>
	<robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">

	  <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />

	<!-- http://ros-industrial.github.io/industrial_training/_source/session3/Workcell-XACRO.html -->
	<!-- Add the world frame as a "virtual link" (no geometry) -->
	  <link name="world"/>

	<!-- the table frame  -->
	  <link name="table">
	    <visual>
	      <geometry>
		<box size="1.0 1.0 0.05"/>
	      </geometry>
	    </visual>
	    <collision>
	      <geometry>
		<box size="1.0 1.0 0.05"/>
	      </geometry>
	    </collision>
	  </link>

	<!-- Add the camera_frame frame as another virtual link (no geometry)  -->
	  <link name="camera_frame"/>


	<!-- need to call the macro to create the robot links and joints.  -->
	  <xacro:ur5_robot prefix="" joint_limited="true"/>

	<!-- world  to table -->
	  <joint name="world_to_table" type="fixed">
	    <parent link="world"/>
	    <child link="table"/>
	    <origin xyz="0 0 0.5" rpy="0 0 0"/>
	  </joint>

	<!-- world to camera -->
	  <joint name="world_to_camera" type="fixed">
	    <parent link="world"/>
	    <child link="camera_frame"/>
	    <origin xyz="-0.25 -0.5 1.25" rpy="0 3.14159 0"/>
	  </joint>

	<!-- table to ur5_robot 
	Connect the UR5 base_link to your existing static geometry with a fixed link.
	-->
	  <joint name="table_to_robot" type="fixed">
	    <parent link="table"/>
	    <child link="base_link"/>
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	  </joint>
	</robot>

	-----------------------------------------------------------------
	launch文件
	launch/urdf.launch
	--------------------------------
	<launch>
	  <arg name="gui" default="true"/>
	  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find myworkcell_core)/urdf/workcell.xacro'" />
	  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
	  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">  
	// 关节角度发布节点 
	    <param name="use_gui" value="$(arg gui)"/>   // 动态反馈显示
	  </node>
	  <node name="rviz" pkg="rviz" type="rviz" if="$(arg gui)"/> // rviz显示
	</launch>
	-------------------------------------------------------
	启动
	roslaunch myworkcell_core urdf.launch

	切换 Fixed Frame 为 world
	添加显示
	Add ---> rviz------> RobotModel and TF 


	---------------------------------------------------------------------------------
	########################################################################
	TF Coordinate Tranforms  坐标变换 发布

	---------------------------------------------------------------------
	gedit package.xml
	// 修改
	  <build_depend>roscpp</build_depend>
	  <build_depend>fake_ar_publisher</build_depend>
	  <build_depend>message_generation</build_depend>
	  <build_depend>geometry_msgs</build_depend>
	  <build_depend>tf</build_depend>   // 新增

	  <run_depend>roscpp</run_depend>
	  <run_depend>fake_ar_publisher</run_depend>
	  <run_depend>message_runtime</run_depend>
	  <run_depend>geometry_msgs</run_depend>
	  <run_depend>tf</run_depend>       // 新增
	-------------------------------------------------------------------
	gedit CMakeList.txt
	// 修改

	# 包编译依赖
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  fake_ar_publisher#依赖其他的包
	  message_generation
	  geometry_msgs
	  tf // 新增
	)

	#添加服务文件
	## Generate services in the 'srv' folder
	 add_service_files(
	   FILES
	   LocalizePart.srv
	 )

	#信息生成 依赖
	## Generate added messages and services with any dependencies listed here
	 generate_messages(
	   DEPENDENCIES
	   std_msgs  # Or other packages containing msgs
	   geometry_msgs
	 )

	#包运行依赖
	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES myworkcell_core
	  CATKIN_DEPENDS 
	    roscpp
	    fake_ar_publisher#运行依赖 其他的包
	    message_runtime
	    geometry_msgs
	    tf // 新增
	#  DEPENDS system_lib
	)

	---------------------------------------------------------------------
	/**
	**  Simple ROS Node
	**  ARserver.cpp  服务器 
	**  加入坐标变换  监听
	**/
	#include <ros/ros.h>// 系统头文件
	// // git clone https://github.com/jmeyer1292/fake_ar_publisher.git 安装
	#include <fake_ar_publisher/ARMarker.h>// 自定义消息头文件
	#include <myworkcell_core/LocalizePart.h>// 服务头文件
	#include <tf/transform_listener.h>// 坐标变换  监听

	// 自定义类
	class Localizer
	{
	public:
	  // 类 初始化函数
	  Localizer(ros::NodeHandle& nh)//节点句柄引用
	  {
	      // 订阅者                     消息类型                 话题名      队列大小
	      ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
	      &Localizer::visionCallback, this);// 回调函数指针  类自己
	      // 服务器订阅服务
	      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
	  }
	  // 话题回调函数
	  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)// 引用 常量指针
	  {
	      last_msg_ = msg;//复制一下放置 变化
	     //  ROS_INFO_STREAM(last_msg_->pose.pose);//打印位置
	  }

	  // 服务回调函数
	  bool localizePart(myworkcell_core::LocalizePart::Request& req,//请求
			      myworkcell_core::LocalizePart::Response& res)// 回应
	  {
	      // Read last message
	      fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
	      if (!p) return false;//空指针 无信息

	      // res.pose = p->pose.pose;

	      tf::Transform cam_to_target;// 参考坐标系 相机 到 目标坐标系的变换
	      // geometry_msgs::Pose 转换到 tf::Transform object: 
	      tf::poseMsgToTF(p->pose.pose, cam_to_target);

	      // 监听 request.base_frame(客户端给的基坐标系) 到 ARMarker message  (which should be "camera_frame") 坐标变换
	      tf::StampedTransform req_to_cam;
	      listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);

	      tf::Transform req_to_target;
	      req_to_target = req_to_cam * cam_to_target;// 目标 在客户端给的基 坐标系下的 坐标变换

	      tf::poseTFToMsg(req_to_target, res.pose);

	      return true;
	  }

	// 变量定义
	  ros::Subscriber ar_sub_;// 订阅者
	  fake_ar_publisher::ARMarkerConstPtr last_msg_;//常量指针
	  ros::ServiceServer server_;// 服务器
	  tf::TransformListener listener_;// 坐标变换监听
	};



	int main(int argc, char *argv[] ){
		// 初始化 ros节点
		ros::init(argc, argv, "ARserver");

		// 创建ros节点句柄
		ros::NodeHandle nh;
		Localizer localizer(nh);

		ROS_INFO("Vision node starting");

		// 节点 存活  rosnode list 可以一直看到
		ros::spin();

	}

	--------------------------------------------------------------

	运行
	roslaunch myworkcell_core urdf.launch       机械臂
	roslaunch myworkcell_core workcell.launch   目标物体


	--------------------------------------------------------------
	########################################################
	Moveit  使用
	会生成新的 moveit 配置包  myworkcell_moveit_config
	配置一个机械臂 规划组  manipulator  包含 一个运动学规划链  从 UR5的基坐标系 base_link 到 末端 tool0

	1 启动配置助手
	roslaunch moveit_setup_assistant setup_assistant.launch
	2 选择 Create New MoveIt Configuration Package 
	3 勾选 Enable Jade+ xacro extensions 载入 workcell.xacro
	4 避免碰撞 矩阵 myworkcell_moveit_config/config/joint_names.yaml
	5 添加固定虚拟关节 Add a fixed virtual base joint.
		name = 'FixedBase' (arbitrary)
		child = 'world' (should match the URDF root link)
		parent = 'world' (reference frame used for motion planning)
		type = 'fixed'
	6 创建运动规划组 planning group
	   Group Name        ： manipulator
	   Kinematics Solver :  KDLKinematicsPlugin

	Add Kin.Chain: base_link 到 tool0


	7 添加确定位置
	  zero  关节全0
	  home  垂直举高

	8 末端执行机构 effectors/grippers
	  暂无

	9 被动关节 passive joints
	  暂无

	10 作者信息
	  需要添加  邮箱不会验证

	11 生成配置文件
	/home/ewenwan/ewenwan/catkin_ws/src/myworkcell_moveit_config

	12 运行
	roslaunch myworkcell_moveit_config demo.launch

	---------------------------------------------------------------

	在实际的硬件 UR5上实验
	需要在 配置文件下 myworkcell_moveit_config/config 新建一些配置文件
	1 创建 controllers.yaml 
	------------------------
	controller_list:
	  - name: ""
	    action_ns: joint_trajectory_action
	    type: FollowJointTrajectory
	    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
	---------------------

	2 创建joint_names.yaml
	-----------------------------
	controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint] 
	-----------------------------

	3 更新　myworkcell_moveit_config/launch/myworkcell_moveit_controller_manager.launch.xml
	------------------------------------
	<launch>
	  <arg name="moveit_controller_manager"
	       default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
	  <param name="moveit_controller_manager"
		 value="$(arg moveit_controller_manager)"/>

	  <rosparam file="$(find myworkcell_moveit_config)/config/controllers.yaml"/>
	</launch>
	---------------------------------------------

	4 创建　新文件　myworkcell_moveit_config/launch/myworkcell_planning_execution.launch
	--------------------------------------------------
	<launch>
	  <!-- The planning and execution components of MoveIt! configured to run -->
	  <!-- using the ROS-Industrial interface. -->

	  <!-- Non-standard joint names:
	       - Create a file [robot_moveit_config]/config/joint_names.yaml
		   controller_joint_names: [joint_1, joint_2, ... joint_N] 
	       - Update with joint names for your robot (in order expected by rbt controller)
	       - and uncomment the following line: -->
	  <rosparam command="load" file="$(find myworkcell_moveit_config)/config/joint_names.yaml"/>

	  <!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
	  <!--  - if sim=false, a robot_ip argument is required -->
	  <arg name="sim" default="true" />
	  <arg name="robot_ip" unless="$(arg sim)" />

	  <!-- load the robot_description parameter before launching ROS-I nodes -->
	  <include file="$(find myworkcell_moveit_config)/launch/planning_context.launch" >
	    <arg name="load_robot_description" value="true" />
	  </include>

	  <!-- run the robot simulator and action interface nodes -->
	  <group if="$(arg sim)">
	    <include file="$(find industrial_robot_simulator)/launch/robot_interface_simulator.launch" />
	  </group>

	  <!-- run the "real robot" interface nodes -->
	  <!--   - this typically includes: robot_state, motion_interface, and joint_trajectory_action nodes -->
	  <!--   - replace these calls with appropriate robot-specific calls or launch files -->
	  <group unless="$(arg sim)">
	    <include file="$(find ur_bringup)/launch/ur5_bringup.launch" />
	  </group>

	  <!-- publish the robot state (tf transforms) -->
	  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

	  <include file="$(find myworkcell_moveit_config)/launch/move_group.launch">
	    <arg name="publish_monitored_planning_scene" value="true" />
	  </include>

	  <include file="$(find myworkcell_moveit_config)/launch/moveit_rviz.launch">
	    <arg name="config" value="true"/>
	  </include>

	</launch>
	------------------------------------------------------------------------------

	运行　
	roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch


	-----------------------------------------------------------------------------------

	######################################################################
	rviz moveit 运动规划

	roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch

	1　显示　Displays
	    Scene Robot -> Show Robot Visual　　　　　勾选
	    Scene Robot -> Show Robot Collision　　　×
	    Planning Request -> Query Start State　×
	    Planning Request -> Query Goal State　　勾选

	2　Motion Planning >> Planning
	   Query  >>>> Goal State 选择　random valid　　点击　Update
	   Commands >>> Plan / Plan and Execute

	3 显示规划　Displays -> Motion Planning -> Planned Path -> Show Trail

	4 更换　规划算法　Context >>> OMPL
	  RRTkConfigDefault  默认算法　较快

	5 添加障碍物
	Scene Objects >>>> Import File 

	https://raw.githubusercontent.com/ros-industrial/industrial_training/indigo/training/orig/3.5/src/I-Beam.dae

	-------------------------------------------------------------
	#################################################################
	使用　moveit  C++ 接口　运动规划
	---------------------------------------------------------------------
	gedit package.xml
	// 修改
	  <build_depend>roscpp</build_depend>
	  <build_depend>fake_ar_publisher</build_depend>
	  <build_depend>message_generation</build_depend>
	  <build_depend>geometry_msgs</build_depend>
	  <build_depend>tf</build_depend>
	  <build_depend>moveit_ros_planning_interface</build_depend>　　#　新增

	  <run_depend>roscpp</run_depend>
	  <run_depend>fake_ar_publisher</run_depend>
	  <run_depend>message_runtime</run_depend>
	  <run_depend>geometry_msgs</run_depend>
	  <run_depend>tf</run_depend>
	  <run_depend>moveit_ros_planning_interface</run_depend>　　　　　　#　新增
	-------------------------------------------------------------------
	gedit CMakeList.txt
	// 修改
	# 包编译依赖
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  fake_ar_publisher#依赖其他的包
	  message_generation
	  geometry_msgs
	  tf
	  moveit_ros_planning_interface  #新增
	)

	#包运行依赖
	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES myworkcell_core
	  CATKIN_DEPENDS 
	    roscpp
	    fake_ar_publisher#运行依赖 其他的包
	    message_runtime   
	    geometry_msgs
	    tf
	    moveit_ros_planning_interface　　#新增
	#  DEPENDS system_lib
	)

	-----------------------------------------------
	/**
	**  Simple ROS Node
	**  moveit 接口 运动规划 
	**/
	#include <ros/ros.h>// 系统头文件
	#include <myworkcell_core/LocalizePart.h>// 服务头文件

	#include <tf/tf.h>// 坐标变换
	#include <moveit/move_group_interface/move_group.h>//moveit 接口 新版 move_group_interface.h

	// 自定义类
	class ScanNPlan
	{
	public:
	// 类 初始化函数
	  ScanNPlan(ros::NodeHandle& nh)//节点句柄引用
	  {
	    // 客户端                             服务类型                      请求的服务名字
	    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
	  }
	// 执行函数
	  void start(const std::string& base_frame)
	  {
	    // 打印信息
	    ROS_INFO("Attempting to localize part");
	    // Localize the part
	    myworkcell_core::LocalizePart srv;// 初始化 服务
	    srv.request.base_frame = base_frame; 
	    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

	    if (!vision_client_.call(srv))//调用服务 得到响应数据
	    {
	      ROS_ERROR("Could not localize part");
	      return;
	    }
	    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应

	    // Plan for robot to move to part
	    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");//运动规划组　配置文件里定义的　新版
	    moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板　
	    geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
	    move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　目标位置
	    move_group.move();// 规划　并　执行

	  }

	private:
	  // Planning components
	  ros::ServiceClient vision_client_;// 私有变量 类内使用
	};


	int main(int argc, char **argv)
	{
	  // 初始化 ros节点
	  ros::init(argc, argv, "ARclient");// 初始化 ros节点
	  // 创建ros节点句柄
	  ros::NodeHandle nh;

	  ros::NodeHandle private_node_handle("~");// 增加一个私有节点 获取目标对象坐标系参数

	  ROS_INFO("ScanNPlan node has been initialized");

	  std::string base_frame;// string 对象变量
	  // 私有节点获取参数                      坐标系参数名  存储变量    默认值
	  private_node_handle.param<std::string>("base_frame", base_frame ,"world"); 

	// move_group.move() command requires use of an "asynchronous" spinner, 
	// to allow processing of ROS messages during the blocking move() command.
	  ros::AsyncSpinner async_spinner(1);

	  ScanNPlan app(nh);// 客户端
	  sleep(3); //alows for debugging
	  ros::Duration(6).sleep();  // 等待客户端初始化完成 wait for the class to initialize
	  async_spinner.start();
	  app.start(base_frame);// 请求服务

	  // ros::spin();// 节点 存活  rosnode list 可以一直看到
	  ros::waitForShutdown();
	}


	-----------------------------------------------------------
	运行　
	roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
	roslaunch myworkcell_core workcell.launch
	可以看到　机械臂移动到　一个固定的地方(目标物体　fake_ar_publisher的位置)


	-----------------------------------------------------
	zai RVIZ中显示　添加的　目标物体　fake_ar_publisher

	Add >>> By topic >>>> /ar_pose_visual >>> Marker 

	-----------------------------------------------------------------------
	待解决
	Try updating the workcell_node's start method to automatically move back to 
	the allZeros position after moving to the AR_target position. 
	See here for a list of move_group's available methods.
	----------------------------------------------------------------------------


	##################################################################
	Descartes  笛卡尔　运动规划求解  正逆运动学求解器　solve forward and inverse kinematics
	终端　规划求解器
	机器人模型　robot model 
	轨迹点　trajectory points　　　　vector 存储
	规划器　planner
	http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes


	１】descartes_core
	descartes_core::TrajectoryPt　　　　笛卡尔轨迹点　　　　　descartes trajectory point
	descartes_core::RobotModel 　　　　　笛卡尔规划机器人模型　descartes robot model　机器人运动学模型　kinematics of the robot　正逆运动学求解　IF/FK
	descartes_core::PathPlannerBase　笛卡尔规划器　　　　　descartes path planner


	２】descartes_moveit
	descartes_moveit::MoveitStateAdapter      descartes_core::RobotModel using Moveit 



	３】descartes_planner
	descartes_planner::DensePlanner  dense_planner   稠密求解器
	descartes_planner::SparsePlanner sparse_planner  稀疏规划器



	４】descartes_trajectory
	descartes_trajectory::JointTrajectoryPt

	descartes_trajectory::CartTrajectoryPt  依赖于　descartes_core::TrajectoryPt

	对称点
	descartes_trajectory::AxialSymmetricPt　　依赖于　descartes_trajectory::CartTrajectoryPt



	５】descartes_utilities
	descartes_utilities::toRosJointPoints  笛卡尔规划结果　转化为　标准ROS关节信息



	------------------------------------------------
	安装
	cd src
	git clone https://github.com/ros-industrial-consortium/descartes.git

	rosdep install -r -y --from-paths src --ignore-src
	catkin_make
	-------------------------------------------------
	复制ur5使用　descartes　的demo例子
	cp -r ~/industrial_training/exercises/4.1/src/ur5_demo_descartes catkin_ws/src

	-------------------------------------------------------
	复制未完成的节点　到　包目录
	cp ~/industrial_training/exercises/4.1/src/descartes_node_unfinished.cpp myworkcell_core/src/descartes_node.cpp
	------------------------------------------------------------
	修改　package.xml文件
	添加
	  <build_depend>ur5_demo_descartes</build_depend>
	  <run_depend>ur5_demo_descartes</run_depend>

	  <build_depend>descartes_trajectory</build_depend>
	  <run_depend>descartes_trajectory</run_depend>

	  <build_depend>descartes_planner</build_depend>
	  <run_depend>descartes_planner</run_depend>

	  <build_depend>descartes_utilities</build_depend>
	  <run_depend>descartes_utilities</run_depend>

	  <build_depend>trajectory_msgs</build_depend>　　　// 新服务　消息需要
	  <run_depend>trajectory_msgs</run_depend>

	--------------------------------------------------------------------------

	修改　CMakeLists.txt文件

	# 包编译依赖
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  fake_ar_publisher#依赖其他的包
	  message_generation
	  geometry_msgs
	  tf
	  moveit_ros_planning_interface

	  ur5_demo_descartes
	  descartes_trajectory
	  descartes_planner
	  descartes_utilities
	  trajectory_msgs
	)

	#添加服务文件
	## Generate services in the 'srv' folder
	 add_service_files(
	   FILES
	   LocalizePart.srv
	   PlanCartesianPath.srv#笛卡尔　轨迹点
	 )

	#信息生成 依赖
	## Generate added messages and services with any dependencies listed here
	 generate_messages(
	   DEPENDENCIES
	   std_msgs  # Or other packages containing msgs
	   geometry_msgs# 坐标系位姿
	   trajectory_msgs# 笛卡尔　轨迹点 消息
	 )

	#包运行依赖
	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES myworkcell_core
	  CATKIN_DEPENDS 
	    roscpp
	    fake_ar_publisher#运行依赖 其他的包
	    message_runtime   
	    geometry_msgs
	    tf
	    moveit_ros_planning_interface

	    ur5_demo_descartes
	    descartes_trajectory
	    descartes_planner
	    descartes_utilities
	    trajectory_msgs
	)

	＃执行文件
	# descartes　笛卡尔轨迹规划　使用示例节点　
	add_executable(descartes_node src/descartes_node.cpp)
	add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(descartes_node ${catkin_LIBRARIES})


	--------------------------------------------------------------
	添加服务文件
	workcell_core/PlanCartesianPath.srv
	# request 请求　目标位置
	geometry_msgs/Pose pose

	---

	# response 响应　当前位置到目标位置　轨迹　点
	trajectory_msgs/JointTrajectory trajectory


	-------------------------------------------------
	修改　descartes_node.cpp文件　　修改TODO部分

	/*
	使用　Descartes  笛卡尔　运动规划求解  正逆运动学求解器　demo
	提供　返回笛卡尔规划　轨迹点集的服务
	可以提供给　客户端
	客户端再将轨迹发给　执行轨迹的　action去执行
	*/
	#include <ros/ros.h>
	#include "myworkcell_core/PlanCartesianPath.h" // 给目标点返回　轨迹点序列　的服务信息头文件（自动生成）

	#include <ur5_demo_descartes/ur5_robot_model.h>// Descartes  笛卡尔 ur5 模型
	#include <descartes_planner/dense_planner.h>   //　Descartes  笛卡尔 规划器　稠密求解器
	#include <descartes_planner/sparse_planner.h>  //　Descartes  笛卡尔 规划器　稀疏规划器
	#include <descartes_trajectory/axial_symmetric_pt.h> //　轨迹点
	#include <descartes_trajectory/joint_trajectory_pt.h>//
	#include <descartes_utilities/ros_conversions.h>//
	#include <eigen_conversions/eigen_msg.h>//

	// 获取关节位置
	std::vector<double> getCurrentJointState(const std::string& topic)
	{
	  // 捕获关节位置
	  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
	  if (!state) throw std::runtime_error("Joint state message capture failed");
	  return state->position;
	}

	// 根据首末点位置　和　运动步长　生成　直线轨迹点　容器
	EigenSTL::vector_Affine3d makeLine(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, double ds)
	{
	  EigenSTL::vector_Affine3d line;// 直线轨迹

	  const Eigen::Vector3d travel = stop - start;//首位点位置差 向量
	  const int steps = std::floor(travel.norm() / ds);// 向量长度/步长　得到步数

	  // Linear interpolation
	  for (int i = 0; i < steps; ++i)
	  {
	    double ratio = static_cast<float>(i) / steps;
	    Eigen::Vector3d position = start + ratio * travel;//起点＋　每一步步长向量
	    Eigen::Affine3d tr;
	    tr = Eigen::Translation3d(position);
	    line.push_back( tr );
	  }

	  return line;
	}

	// CartesianPlanner 笛卡尔规划器　类
	class CartesianPlanner
	{
	public:

	// 类初始化函数　带入　ros节点句柄
	  CartesianPlanner(ros::NodeHandle& nh)
	  {
	    // first init descartes　初始化节点
	    if (!initDescartes())
	      throw std::runtime_error("There was an issue initializing Descartes");

	    // init services　　　　　　　　初始化服务　订阅的服务名　　服务回调函数　　　　　　类本体
	    server_ = nh.advertiseService("plan_path", &CartesianPlanner::planPath, this);
	  }


	// 初始化笛卡尔规划器　　１初始化机器人模型　２规划器初始化
	  bool initDescartes()
	  {
	    // Create a robot model
	    // 智能指针　显示消除　new  delete 内存的调研　方便　　　make_shared<T>()
	    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();// 创建模型变量

	    // Define the relevant "frames"
	    // 定义　机器人模型初始化变量
	    const std::string robot_description = "robot_description";//　机器人描述
	    const std::string group_name = "manipulator";// 轨迹规划群
	    const std::string world_frame = "world"; // Frame in which tool poses are expressed
	    const std::string tcp_frame = "tool0";//　末端　坐标系

	    // Using the desired frames, let's initialize Descartes
	    // 初始化　机器人模型变量　
	    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
	    {
	      ROS_WARN("Descartes RobotModel failed to initialize");
	      return false;
	    }

	    // 规划器　根据　机器人模型　初始化
	    if (!planner_.initialize(model_))
	    {
	      ROS_WARN("Descartes Planner failed to initialize");
	      return false;
	    }
	    return true;
	  }


	// 服务回调函数　　规划终端工具　路径
	  bool planPath(myworkcell_core::PlanCartesianPathRequest& req, // 服务请求
			myworkcell_core::PlanCartesianPathResponse& res)// 服务响应
	  {
	    ROS_INFO("Recieved cartesian planning request");

	    // Step 1: Generate path poses 
	    // 产生　执行器末端　轨迹　　　工件　一周　画圈
	    EigenSTL::vector_Affine3d tool_poses = makeToolPoses();

	    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
	    // 转换成笛卡尔　轨迹点
	    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(req.pose, tool_poses);

	    // Step 3: Tell Descartes to start at the "current" robot position
	    // 当前机器人位置设置为　起点
	    std::vector<double> start_joints = getCurrentJointState("joint_states");
	    descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints));
	    path.front() = pt;

	    // Step 4: Plan with descartes
	    // 笛卡尔规划器　规划 
	    if (!planner_.planPath(path))
	    {
	      return false;
	    }
	    // 规划结果
	    std::vector<descartes_core::TrajectoryPtPtr> result;
	    if (!planner_.getPath(result))
	    {
	      return false;
	    }

	    // Step 5: Convert the output trajectory into a ROS-formatted message
	    // 转换　笛卡尔规划器规划结果到　标准ROS关节点信息
	    res.trajectory.header.stamp = ros::Time::now();// 时间戳
	    res.trajectory.header.frame_id = "world";      //　坐标系
	    res.trajectory.joint_names = getJointNames();  //　关节名字　　controller_joint_names
	    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
	    return true;
	  }


	//  规划末端执行机构　轨迹
	  EigenSTL::vector_Affine3d makeToolPoses()
	  {
	    EigenSTL::vector_Affine3d path;// 轨迹

	    // We assume that our path is centered at (0, 0, 0), so let's define the
	    // corners of the AR marker

	    // 机器人首先运动到　模块中心位置　　
	    // 
	    const double side_length = 0.25; // 目标模块边长　　All units are in meters (M)
	    const double half_side = side_length / 2.0;
	    const double step_size = 0.02;// 步长

	    Eigen::Vector3d top_left  (half_side,  half_side,  0);// 右手定则　上左边
	    Eigen::Vector3d bot_left  (-half_side, half_side,  0);// 下左边　ｘ方向为负
	    Eigen::Vector3d bot_right (-half_side, -half_side, 0);// 下右边　均为负
	    Eigen::Vector3d top_right (half_side,  -half_side, 0);//　上右　ｙ方向为负
	    Eigen::Vector3d center (0, 0, 0);// 中心点   

	    // Descartes requires you to guide it in how dense the points should be,
	    // so you have to do your own "discretization".
	    // NOTE that the makeLine function will create a sequence of points inclusive
	    // of the start and exclusive of finish point, i.e. line = [start, stop)

	    // TODO: Add the rest of the cartesian path
	    auto segment1 = makeLine(top_left,  bot_left,  step_size);
	    auto segment2 = makeLine(bot_left,  bot_right, step_size);
	    auto segment3 = makeLine(bot_right, top_right, step_size);
	    auto segment4 = makeLine(top_right, top_left,  step_size);

	    path.insert(path.end(), segment1.begin(), segment1.end());
	    path.insert(path.end(), segment2.begin(), segment2.end());
	    path.insert(path.end(), segment3.begin(), segment3.end());
	    path.insert(path.end(), segment4.begin(), segment4.end());

	    return path;
	  }


	// 由参考点　将EigenSTL::vector_Affine3d　位置　转换成　笛卡尔　轨迹点
	  std::vector<descartes_core::TrajectoryPtPtr>
	  makeDescartesTrajectory(const geometry_msgs::Pose& reference,
				  const EigenSTL::vector_Affine3d& path)
	  {
	    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // 返回值　return value

	    // 参考位置
	    Eigen::Affine3d ref;
	    tf::poseMsgToEigen(reference, ref);//位置　转化成　EIGEN格式

	    for (auto& point : path)//范围　for  path　引用
	    {
	      // TODO: make a Descartes "cartesian" point with some kind of constraints
	      descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(ref * point);// 根据参考位置
	      descartes_path.push_back(pt);
	    }
	    return descartes_path;
	  }


	// Eigen::Affine3d位置　生成　AxialSymmetricPt　笛卡尔轨迹
	  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
	  {
	    using namespace descartes_core;
	    using namespace descartes_trajectory;
	    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );// 关于Ｚ轴对称的点
	// 详情　http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html
	  }

	// 获取　关节名字　controller_joint_names　获取　HELPER
	  std::vector<std::string> getJointNames()
	  {
	    std::vector<std::string> names;
	    nh_.getParam("controller_joint_names", names);
	    return names;
	  }

	// 类内变量　
	  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;// 笛卡尔机器人模型　指针指针　
	  descartes_planner::DensePlanner planner_;// 稠密规划器
	  // descartes_planner::SparsePlanner   稀疏规划器
	  ros::ServiceServer server_;// 服务器
	  ros::NodeHandle nh_;// 节点句柄
	};


	// 主函数
	int main(int argc, char** argv)
	{
	  // 初始化节点
	  ros::init(argc, argv, "descartes_node");
	  // 节点句柄
	  ros::NodeHandle nh;
	  // 笛卡尔　规划器 
	  CartesianPlanner planner (nh);

	  ROS_INFO("Cartesian planning node starting");

	  ros::spin();
	}

	--------------------------------------------------------------------------
	修改　ARclient.cpp　增加　终端工具　动作规划

	/**
	**  Simple ROS Node
	**  moveit 接口 运动规划 　　　　　　　　　　　　　　　　　　　　　　　　　　运动到　工件位置
	**  Descartes  笛卡尔 轨迹规划接口　使用轨迹跟踪执行　action行动执行　　　　工具在工件位置周围执行
	**  相当于　moveit　轨迹规划
	**  Descartes  笛卡尔  终端抓取规划
	**/
	#include <ros/ros.h>// 系统头文件
	#include <myworkcell_core/LocalizePart.h>// 定位　服务头文件

	#include <tf/tf.h>// 坐标变换
	#include <moveit/move_group_interface/move_group.h>//moveit 接口 新版 move_group_interface.h

	//　笛卡尔规划　接口
	#include <actionlib/client/simple_action_client.h>// 动作　服务接口
	#include <control_msgs/FollowJointTrajectoryAction.h>// 控制消息　　轨迹行动　根据轨迹点集合　执行　移动到个点
	#include <myworkcell_core/PlanCartesianPath.h>// 笛卡尔规划　服务头文件　得到规划的轨迹点集合

	// 自定义类
	class ScanNPlan
	{
	public:
	// 类 初始化函数
	  //节点句柄引用 带有　笛卡尔轨迹初始化　列表
	  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
	  {
	    // 定位　请求
	    // 客户端                             服务类型                      请求的服务名字
	    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");

	// 笛卡尔规划　 客户端　请求
	    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
	  }
	// 执行函数
	  void start(const std::string& base_frame)
	  {
	    // 打印信息
	    ROS_INFO("Attempting to localize part");
	    // Localize the part
	    myworkcell_core::LocalizePart srv;// 初始化 服务
	    srv.request.base_frame = base_frame; 
	    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

	    if (!vision_client_.call(srv))//调用服务 得到响应数据
	    {
	      ROS_ERROR("Could not localize part");
	      return;
	    }
	    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应

	    // Plan for robot to move to part
	    // movieit 接口规划
	    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");//运动规划组　配置文件里定义的　新版
	    moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板本
	    geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
	    move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　目标位置
	    move_group.move();// 规划　并　执行

	    // 笛卡尔　规划　Plan cartesian path
	    myworkcell_core::PlanCartesianPath cartesian_srv;// 服务
	    cartesian_srv.request.pose = move_target;// 请求的目标位置
	    if (!cartesian_client_.call(cartesian_srv))//　调用笛卡尔　规划 获取　轨迹点集
	    {
	      ROS_ERROR("Could not plan for path");
	      return;
	    }
	    // 执行轨迹点集合　Execute descartes-planned path directly (bypassing MoveIt)
	    ROS_INFO("Got cart path, executing");
	    control_msgs::FollowJointTrajectoryGoal goal;// 行动中的　目标
	    goal.trajectory = cartesian_srv.response.trajectory;// 目标轨迹
	    ac_.sendGoal(goal);// 发送目标
	    ac_.waitForResult();// 等待执行结果
	    ROS_INFO("Done");//

	  }

	private://　成员变量
	  // Planning components
	  ros::ServiceClient vision_client_;// 私有变量 类内使用

	// 笛卡尔规划　服务
	  ros::ServiceClient cartesian_client_;//　客户端　请求笛卡尔规划　轨迹点集结果
	  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;// 行动请求
	};


	int main(int argc, char **argv)
	{
	  // 初始化 ros节点
	  ros::init(argc, argv, "ARclient");// 初始化 ros节点
	  // 创建ros节点句柄
	  ros::NodeHandle nh;

	  ros::NodeHandle private_node_handle("~");// 增加一个私有节点 获取目标对象坐标系参数

	  ROS_INFO("ScanNPlan node has been initialized");

	  std::string base_frame;// string 对象变量
	  // 私有节点获取参数                      坐标系参数名  存储变量    默认值
	  private_node_handle.param<std::string>("base_frame", base_frame ,"world"); 

	// move_group.move() command requires use of an "asynchronous" spinner, 
	// to allow processing of ROS messages during the blocking move() command.
	  ros::AsyncSpinner async_spinner(1);

	  ScanNPlan app(nh);// 客户端
	  sleep(3); //alows for debugging
	  ros::Duration(6).sleep();  // 等待客户端初始化完成 wait for the class to initialize
	  async_spinner.start();
	  app.start(base_frame);// 请求服务

	  // ros::spin();// 节点 存活  rosnode list 可以一直看到
	  ros::waitForShutdown();
	}

	----------------------------------------------------------
	新建　setup.launch
	<launch>
		<include file = "$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch" /> // UR5moveit配置文件
		<node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" /> // 目标工件发布
		<node name="ARserver_node"  pkg="myworkcell_core" type="ARserver_node" />　　　　　　　　　　　　　　　// 目标工件定位服务
		<node name="descartes_node" pkg="myworkcell_core" type="descartes_node" output="screen"/>// 笛卡尔　规划　服务
		<node name="ARclient_node"  pkg="myworkcell_core" type="ARclient_node"  output="screen">// 获取工件位置　移动过去　在工件　周围　运动
		    <param name="base_frame" value="world"/>
		</node>
	</launch>


	roslaunch myworkcell_core setup.launch
