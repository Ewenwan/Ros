# 机械臂三维感知应用1 避免碰撞抓取和放置
[参考](http://ros-industrial.github.io/industrial_training/_source/demo1/index.html)

[Github项目](https://github.com/Ewenwan/Ros/tree/master/src/perception_driven_ws/src/collision_avoidance_pick_and_place)

[原github目录](https://github.com/ros-industrial/industrial_training/tree/kinetic/exercises/Perception-Driven_Manipulation/solution_ws/src/collision_avoidance_pick_and_place)

        三维感知驱动的　操作　　     Perception-Driven_Manipulation
        模式识别驱动的　操作手示例　　pick and place task
        
        
        目的是实现一个ROS节点，该节点通过一系列动作和动作驱动机器人完成一个选择和放置任务。
        此外，它们还将作为如何将各种软件功能（感知、控制器驱动程序、I/O、反向运动学、路径规划、碰撞避免等）
        集成到基于ROS的工业应用程序中的示例。
        
        目标：
         1. 了解真实或模拟机器人应用程序的组件和结构。
         2. 学习如何使用 moveit 命 令机器人移动！.
         3. 学习如何 移动手臂 到 关节 或 笛卡尔位置。
         4. 利用 感知能力，包括 AR标签识别 和 PCL。
         5. 为 拾取 和 放置任务 规划 无碰撞路径。
         6. 控制机器人外围设备，如抓手。

# 包含内容：
        perception　　　　　　模式识别
        controller drivers　　控制驱动
        I/O		　　　I/O口
        inverse kinematics　　逆运动学
        path planning　　　　 运动规划
        collision avoidance　碰撞检测

[获取　文件](https://github.com/ros-industrial/industrial_training)

        三维感知驱动的　操作
        cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/template_ws ~/perception_driven_ws
        cd ~/perception_driven_ws
        source /opt/ros/kinetic/setup.bash
        catkin init
        
        安装依赖 安装文件 src/.rosinstall
        cd ~/perception_driven_ws/src/
        wstool update

        进入避免碰撞　包
        cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/

## 项目分析
        launch文件夹　/launch：
          1. ur5_setup.launch: 启动整个系统　Brings up the entire ROS system 
                  (MoveIt!, rviz, perception, ROS-I drivers, robot I/O peripherals)
          2. ur5_pick_and_place.launch   : 抓取放置 节点　Runs your pick and place node.


        头文件　/include/collision_avoidance_pick_and_place:
          1. pick_and_place.h
          2. pick_and_place_utilities.h　　应用程序变量定义


        参数配置文件夹 /config/ur5/
          1. pick_and_place_parameters.yaml    :　　　抓取节点所需参数
          2. rviz_config.rviz   : 　　　　　　　　　　　Rviz 　显示参数配置
          3. target_recognition_parameters.yaml    : 目标识别服务　参数配置　从传感器数据中　检测出 box
          4. test_cloud_obstacle_descriptions.yaml  : 用来产生仿真传感器数据的　参数　
          5. collision_obstacles.txt   : 　　　　　　　仿真模式下　的　障碍物　范围　　参数


        源文件　/src：
        /src/nodes:　主应用节点
          1. pick_and_place_node.cpp :   主应用线程!!!!　包括消息头header  和　函数调用
          2. generate_point_cloud.cpp　:      生成点云
          3. collision_object_publisher.py  ：碰撞物体　发布　

        /src/services: 服务节点
          1. target_recognition_service.cpp　　　目标识别服务
          2. simulation_recognition_service.py　仿真识别服务


        /src/tasks: 实际任务
          1. create_motion_plan.cpp　　　　运动规划
          2. create_pick_moves.cpp 　　　　抓取规划
          3. create_place_moves.cpp　　　　放置规划
          4. detect_box_pick.cpp　　　　　检测 物体抓取
          5. pickup_box.cpp　　　　　　　　抓起 箱子
          6. place_box.cpp　　　　　　　　 放置 箱子
          7. move_to_wait_position.cpp　 运动 到 等待区域
          8. set_attached_object.cpp　　 设置要 抓取的对象
          9. set_gripper.cpp　　　　　　　设置抓手 打开释放
          10.reset_world.cpp             重置环境


        /src/utilities:  
         - pick_and_place_utilities.cpp : 核心函数
         
        服务数据 srv/
          1. GetTargetPose.srv  获取目标位置
## 运行
### 建立包的依赖环境
        cd ~/perception_driven_ws
        catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
        source devel/setup.bash


###  启动 UR5仿真环境  目标识别服务等
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch　　
            默认都是仿真的
            
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false　　　
            真实的传感器Kinect 假的机器人
            
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]　  
            真实的机器人　传递ip　假的传感器
            
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
            真实的传感器　真是的机器人　真实的抓取手
        --------------------------------------------------------------------
#### 文件分析
        ur5环境启动　目标识别服务等
        ur5_setup.launch
        ------------------------------
        <?xml version="1.0"?>
        <launch>
        // 参数　标志
          <arg name="sim_robot" default="true"/>
          <arg name="sim_sensor" default="true"/>
          <arg name="sim_gripper" default="true"/>
          <arg name="robot_ip" unless="$(arg sim_robot)"/>

          <!-- moveit components 运动配置文件-->
          <include file="$(find ur5_collision_avoidance_moveit_config)/launch/moveit_planning_execution.launch">
            <arg name="sim" value="$(arg sim_robot)"/>
            <arg unless="$(arg sim_robot)" name="robot_ip" value="$(arg robot_ip)"/>
          </include>
        // 如果是仿真机械手的话　启动仿行动服务
          <!-- simulated robot mode nodes-->
          <group if="$(arg sim_gripper)">
            <!-- grasp action service (simulated) -->
            <node pkg="robot_io" type="simulated_grasp_action_server" name="gripper_action_server" output="screen"/>
          </group>

        // 如果是不是仿真　的话　unless="$(arg sim_gripper)"　　启动机器人接口　节点
          <!-- robot interface (real robot) nodes -->
          <group unless="$(arg sim_gripper)">
             <!-- grasp action service for vacuum gripper -->
            <node pkg="robot_io" type="suction_gripper_action_server" name="gripper_action_server" output="screen"/>
            <param name="suction_on_output_channel" value="0"/>
            <param name="suction_check_output_channel" value="1"/>
            <param name="use_sensor_feedback" value="false"/>
          </group>

        // 如果　是仿真的传感器if="$(arg sim_sensor)"　发布虚假的点云数据
         <!-- simulated sensor mode -->
          <group if="$(arg sim_sensor)">	
            <!-- static ar_tag frame publisher for simulation 目标件　ar -->
           <node pkg="tf" type="static_transform_publisher" name="world_to_tag" args="-0.8 0.2 0.17 0.785 0 0 world_frame ar_tag 100"/>
            <!-- detection node 识别检测节点　-->
           <include file="$(find collision_avoidance_pick_and_place)/launch/ur5_target_recognition.launch"/>
            <!-- simulated sensor data 仿真障碍物点云数据节点-->
           <include file="$(find collision_avoidance_pick_and_place)/launch/ur5_generate_test_cloud_obstacles.launch"/>	
          </group>

        // 如果　是由真实的传感器unless="$(arg sim_sensor)"　　启动kinect 节点　
        <!-- real sensor mode -->
          <group unless="$(arg sim_sensor)">
            <!-- sensor setup 启动传感器　-->
            <include file="$(find sensor_config)/launch/ur5_sensor_setup.launch"/>
            <!-- detection node 检测节点　-->
            <include file="$(find collision_avoidance_pick_and_place)/launch/ur5_target_recognition.launch"/>

            <!-- ar tag detection node -->	
            <node name="ar_pose" pkg="ar_pose" type="ar_multi" respawn=


#### 初始化和全局变量分析
        include/collision_avoidance_pick_and_place/pick_and_place_utilities.h
```c
    ARM_GROUP_NAME  = "manipulator";       // 机械臂规划组  名字
    TCP_LINK_NAME   = "tcp_frame";         // 终端 工具中点 名字
    MARKER_TOPIC = "pick_and_place_marker";
    PLANNING_SCENE_TOPIC = "planning_scene";           // 场景
    TARGET_RECOGNITION_SERVICE = "target_recognition"; // 目标识别服务
    MOTION_PLAN_SERVICE = "plan_kinematic_path";       // 运动学规划服务
    WRIST_LINK_NAME = "ee_link";                       // 终端 关节 名
    ATTACHED_OBJECT_LINK_NAME = "attached_object_link";// 接触目标 关节
    WORLD_FRAME_ID  = "world_frame";                   // 世界坐标系 id
    HOME_POSE_NAME  = "home";
    WAIT_POSE_NAME  = "wait";
    AR_TAG_FRAME_ID    = "ar_frame";
    GRASP_ACTION_NAME = "grasp_execution_action";
    BOX_SIZE        = tf::Vector3(0.1f, 0.1f, 0.1f);
    BOX_PLACE_TF    = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()));
    TOUCH_LINKS = std::vector<std::string>();
    RETREAT_DISTANCE  = 0.05f;
    APPROACH_DISTANCE = 0.05f;
```
        
       变量访问
       ROS_INFO_STREAM("world frame: " << application.cfg.WORLD_FRAME_ID)
       
       
#### 初始化节点
     src/npde/pick_and_place_node.cpp
     
```c
/*
主　基于模式是别的　抓取放置　应用程序节点
*/
#include <collision_avoidance_pick_and_place/pick_and_place.h>

using namespace collision_avoidance_pick_and_place;

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  // 全局变量
  geometry_msgs::Pose box_pose;// 箱子位姿
  std::vector<geometry_msgs::Pose> pick_poses, place_poses;// 位姿态容器　抓取时的姿态 放置时的姿态

  // ros initialization　　初始化节点
  ros::init(argc,argv,"pick_and_place_node");

  ros::NodeHandle nh;//　节点句柄 
  ros::AsyncSpinner spinner(2);//多线程spin 开2个线程
  spinner.start();

  // 创建　应用类　实体对象 获取参数
  PickAndPlace application;

  // reading parameters　读取参数
  if(application.cfg.init())
  {
    ROS_INFO_STREAM("Parameters successfully read");
  }
  else
  {
    ROS_ERROR_STREAM("Parameters not found");
    return 0;
  }

  // marker publisher　发布　marker 消息
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // planning scene publisher　　规划场景 发布器
  application.planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(
  		application.cfg.PLANNING_SCENE_TOPIC,1);

  // moveit 接口
 //  moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板本
  application.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroup(application.cfg.ARM_GROUP_NAME));
 
  application.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");

  // motion plan client 运动规划客户端　订阅规划服务器　的服务话题
  application.motion_plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(application.cfg.MOTION_PLAN_SERVICE);

  // transform listener　坐标变换监听
  application.transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());

  // marker publisher (rviz visualization) 发布　marker 消息
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // target recognition client (perception)　目标识别　客户端
  application.target_recognition_client = nh.serviceClient<collision_avoidance_pick_and_place::GetTargetPose>(
		  application.cfg.TARGET_RECOGNITION_SERVICE);

  // grasp action client (vacuum gripper)　抓取行动　客户端
  application.grasp_action_client_ptr = GraspActionClientPtr(
		  new GraspActionClient(application.cfg.GRASP_ACTION_NAME,true));


  // 等待连接到　抓取行动服务发布者　waiting to establish connections
  while(ros::ok() &&
      !application.grasp_action_client_ptr->waitForServer(ros::Duration(3.0f)))//等待抓取行动服务
  {
    ROS_INFO_STREAM("Waiting for grasp action servers");
  }
  // 等待连接到　目标识别服务　　发布者
  if(ros::ok() && !application.target_recognition_client.waitForExistence(ros::Duration(3.0f)))//等待目标识别服务
  {
	  ROS_INFO_STREAM("Waiting for service'"<<application.cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }


// 抓取 移动 放置 任务================================================
  ROS_INFO_STREAM("Satrting Pick & Place Tasks ");

  // move to a "clear" position　　运动到　等候区 moveit实现
  application.move_to_wait_position();

  // turn off vacuum gripper　　　　　打开抓手  释放物体的状态
  application.set_gripper(false);

  // get the box position and orientation 检测需要抓取的箱子　的　位姿
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "pick" the box　从当前位姿　到　抓取姿态　规划
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves　 执行一系列位姿　到　抓取
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "place" the box 求解从抓取好箱子到放置位置　位姿
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves　 放置物体
  application.place_box(place_poses,box_pose);

  // move back to the "clear" position　 返回　等候区
  application.move_to_wait_position();

  return 0;
}

```




     
     
     
       
