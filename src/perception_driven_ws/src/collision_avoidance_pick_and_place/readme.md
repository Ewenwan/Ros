        #########################################################
        ###############################################################
        应用　　【１】
        三维感知驱动的　操作　　Perception-Driven_Manipulation
        模式识别驱动的　操作手示例　　pick and place task

        包含内容：
        perception　　　　　　　　　　模式识别
        controller drivers　　控制驱动
        I/O		　　　　I/O口
        inverse kinematics　　逆运动学
        path planning　　　　　　　运动规划
        collision avoidance　碰撞检测

        获取　文件
        https://github.com/ros-industrial/industrial_training
        三维感知驱动的　操作
        cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/template_ws ~/perception_driven_ws
         更新
        cd ~/perception_driven_ws/src/
        wstool update

        进入避免碰撞　包
        cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/

        launch文件夹　/launch：
        ur5_setup.launch     : 启动整个系统　Brings up the entire ROS system 
                  (MoveIt!, rviz, perception, ROS-I drivers, robot I/O peripherals)
        ur5_pick_and_place.launch   : 抓取节点　Runs your pick and place node.


        头文件　/include/collision_avoidance_pick_and_place:
         - pick_and_place.h
         - pick_and_place_utilities.h　　应用程序变量定义




        参数配置文件夹 /config :
        ur5/
         - pick_and_place_parameters.yaml    :　　　　　　　　抓取节点所需参数
         - rviz_config.rviz   : 　　　　　　　　　　　　　　　　　　　　　　Rviz 　显示参数配置
         - target_recognition_parameters.yaml    : 　　　目标识别服务　参数配置　从传感器数据中　检测出　ｂｏｘ
         - test_cloud_obstacle_descriptions.yaml    : 用来产生仿真传感器数据的　参数　
         - collision_obstacles.txt   : 　　　　　　　　　　　　　　　仿真模式下　的　障碍物　范围　　参数


        源文件　/src：
        /src/nodes:　主应用节点
         - pick_and_place_node.cpp : 主应用线程　包括消息头header  和　函数调用
         - generate_point_cloud.cpp　　　　　　　: 生成点云
         - collision_object_publisher.py  ：碰撞物体　发布　

        /src/services: 服务节点
        target_recognition_service.cpp　　　　　目标识别服务
        simulation_recognition_service.py　　仿真识别服务


        /src/tasks: Source files with incomplete function definitions. 
         - create_motion_plan.cpp　　　　运动规划
         - create_pick_moves.cpp 　　　　抓取规划
         - create_place_moves.cpp　　　　放置规划
         - detect_box_pick.cpp　　　　　　　检测物体抓取
         - pickup_box.cpp　　　　　　　　　　　　抓起箱子
         - place_box.cpp　　　　　　　　　　　　　放置箱子
         - move_to_wait_position.cpp　运动到等待区域
         - set_attached_object.cpp　　　设置要抓取的对象
         - set_gripper.cpp　　　　　　　　　　　设置抓手　　　打开释放
         - reset_world.cpp           重置环境


        /src/utilities:  
         - pick_and_place_utilities.cpp : 核心　Contains support functions that will help you complete the exercise.


        建立包的依赖环境
        cd ~/perception_driven_ws
        catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
        source devel/setup.bash


        启动 UR5仿真环境  目标识别服务等
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch　　默认都是仿真的
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false　　　真实的传感器Kinect 假的机器人
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]　真实的机器人　传递ip　假的传感器
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
        真实的传感器　真是的机器人　真实的抓取手
        --------------------------------------------------------------------
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
            <node name="ar_pose" pkg="ar_pose" type="ar_multi" respawn="false">
              <remap from="/usb_cam/image_raw" to="/kinect/rgb/image_color"/>
              <remap from="/usb_cam/camera_info" to="/kinect/rgb/camera_info"/>
              <param name="marker_pattern_list" type="string" value="$(find ar_pose)/data/target_info"/>
              <param name="threshold" type="int" value="100"/>
            </node>
          </group>


          <!-- rviz 可视化-->
          <node name="$(anon rviz)" pkg="rviz" type="rviz" respawn="false" 
                args="-d $(find collision_avoidance_pick_and_place)/config/ur5/rviz_config.rviz" output="screen" launch-prefix="nice">
            <rosparam command="load" file="$(find ur5_collision_avoidance_moveit_config)/config/kinematics.yaml"/>
          </node>

        </launch>
        --------------------------------------------------------------------------------
        ---------------------------------------------------------------------------
        pick_and_place抓取放置　应用节点　启动
        ur5_pick_and_place.launch
        -----------------------------------------
        <?xml version="1.0"?>
        <launch>

          <!-- pick and place node 启动抓取放置节点　-->
          <node pkg="collision_avoidance_pick_and_place" type="pick_and_place_node" name="pick_and_place_node" output="screen" required="true">
            <rosparam command="load" file="$(find collision_avoidance_pick_and_place)/config/ur5/pick_and_place_parameters.yaml"/> //　载入参数文件
          </node>

          <include file="$(find ur5_collision_avoidance_moveit_config)/launch/move_group.launch">
            <arg name="publish_monitored_planning_scene" value="true" />
          </include>

          <rosparam command="load" file="$(find ur5_collision_avoidance_moveit_config)/config/joint_names.yaml"/>

          <include file="$(find ur5_collision_avoidance_moveit_config)/launch/default_warehouse_db.launch" />

        </launch>

        -----------------------------------------------------------------------------

        应用程序变量　定义　

        /include/collision_avoidance_pick_and_place/pick_and_place_utilities.h　　
        应用程序变量定义
        -----------------------------------------
        定义了一个配置类
        公开全局变量
        class pick_and_place_config
        {
        public:

          // =============================== Parameters ===============================
          std::string ARM_GROUP_NAME;  // 运动规划组　与moveit配置时一置
          ...
        --------------------------------------------------
          pick_and_place_config()
          {
            ARM_GROUP_NAME  = "manipulator";// moveit 规划组名字
            TCP_LINK_NAME   = "tcp_frame";  // 通讯
            MARKER_TOPIC    = "pick_and_place_marker";
            PLANNING_SCENE_TOPIC       = "planning_scene";//规划　话题
            TARGET_RECOGNITION_SERVICE = "target_recognition";// 目标识别服务
            MOTION_PLAN_SERVICE        = "plan_kinematic_path";// 运动学规划服务
            WRIST_LINK_NAME            = "ee_link";// 末端关节名字
            ATTACHED_OBJECT_LINK_NAME  = "attached_object_link";//
            WORLD_FRAME_ID     = "world_frame";//世界坐标系，名称
            HOME_POSE_NAME     = "home";// 家　位姿
            WAIT_POSE_NAME     = "wait";//　等待区位姿
            AR_TAG_FRAME_ID    = "ar_frame";
            GRASP_ACTION_NAME  = "grasp_execution_action";//抓取行动
            BOX_SIZE           = tf::Vector3(0.1f, 0.1f, 0.1f);//箱子尺寸
            BOX_PLACE_TF       = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()));//箱子坐标位置
            TOUCH_LINKS        = std::vector<std::string>();
            RETREAT_DISTANCE   = 0.05f;
            APPROACH_DISTANCE  = 0.05f;
          }


        ----------------------------------------------
        pick_and_place.h　中定义了　命名空间　collision_avoidance_pick_and_place
        下PickAndPlace类　

        namespace collision_avoidance_pick_and_place
        {
          class PickAndPlace
          {
          public:
          // ================ constructor　构造函数 ========================
            PickAndPlace()
            {

            }

          // ================ global members　包含全局类对象 ===================
            pick_and_place_config cfg;// pick_and_place_config 类
                       ...
          // =============================== Task Functions ===============================
                       void move_to_wait_position();//移动到等候区
                       ...
               }
        }

        ----------------------------------------------------------------------
        而在　
        /src/nodes/pick_and_place_node.cpp : 主应用线程　包括消息头header  和　函数调用
        下
        使用命名空间  using namespace collision_avoidance_pick_and_place;
        定义PickAndPlace 类对象　application
        PickAndPlace application;

        所以可以使用
        application.cfg.WORLD_FRAME_ID　　访问修参数
        //　访问打印参数信息
        ROS_INFO_STREAM("world frame: " << application.cfg.WORLD_FRAME_ID)

        ---------------------------------------------------------------------------

        /* ========================================*/
          /* Pick & Place Tasks                      */
          /* ========================================*/

          // move to a "clear" position　　运动到　等候区
          application.move_to_wait_position();

          // turn off vacuum gripper　　　　　打开抓手
          application.set_gripper(false);

          // get the box position and orientation　　检测需要抓取的箱子　的　位姿
          box_pose = application.detect_box_pick();

          // build a sequence of poses to "pick" the box　从当前位姿　到　抓取姿态　规划
          pick_poses = application.create_pick_moves(box_pose);

          // plan/execute the sequence of "pick" moves　　　　　执行一系列位姿　到　抓取
          application.pickup_box(pick_poses,box_pose);

          // build a sequence of poses to "place" the box　　　求解从抓取好箱子到放置位置　位姿
          place_poses = application.create_place_moves();

          // plan/execute the "place" moves　　　　　　　　　　　　　　　　　放置物体
          application.place_box(place_poses,box_pose);

          // move back to the "clear" position　　　　　　　　　　　　　　返回　等候区

        -------------------------------------------------------------------------
        -------------------------------------------------------------------------

         运动到　等候区　函数实现　
          // move to a "clear" position　　运动到　等候区
          application.move_to_wait_position();
        实现函数：
        /src/tasks/move_to_wait_position.cpp 

          bool success; // saves the move result  规划执行结果
          // 设置规划目标位置(按位置命名设置)
          move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);
           // geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
           //move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　按位置设置　目标位置
          // set allowed planning time　设置规划运行的执行时间
          move_group_ptr->setPlanningTime(60.0f);
          // 执行
          success = move_group_ptr->move();

        pick_and_place.h
        // typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;//轨迹　规划组 指针
        // MoveGroupPtr move_group_ptr;// moveit运动规划指针　　访问需要使用　->  



        -------------------------------------------------------------------------------
        ----------------------------------------------------------------------------------
        设置抓手状态　  application.set_gripper(false);
        实现文件　/src/tasks/set_gripper.cpp
        void collision_avoidance_pick_and_place::PickAndPlace::set_gripper(bool do_grasp)

        过程较长　为　action 行动 类型
        自定义文件
        /perception_driven_ws/src/object_manipulation_msgs/action/GraspHandPostureExecution.action
        ----------------------------------------------------------------
        int32 PRE_GRASP=1
        # requests that the hand execute the actual grasp　抓取
        int32 GRASP=2
        # requests that the hand open to release the object 释放
        int32 RELEASE=3
        int32 goal＃目标
        # the max contact force to use (<=0 if no desired max)
        float32 max_contact_force

        ---
        # the result of the action　　结果
        ManipulationResult result
        ---
        # empty for now　反馈为空
        -------------------------------------------------------------------
        会自动生成 头文件
        GraspHandPostureExecutionAction.h 
        GraspHandPostureExecutionActionGoal.h
        GraspHandPostureExecutionActionResult.h
        GraspHandPostureExecutionActionFeedback.h
        -------------------------------------------------------

        pick_and_place.h
        typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
        GraspActionClientPtr grasp_action_client_ptr;//抓手抓取　释放　action对象　指针　
        -----------------------------------------------------------------

        实现文件　/src/tasks/set_gripper.cpp
        void collision_avoidance_pick_and_place::PickAndPlace::set_gripper(bool do_grasp)

        //　任务变量
        object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;//抓手对象  msg消息类型　任务变量
          bool success;
          if (do_grasp)
            grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;//抓取状态
          else
            grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;//释放状态
        grasp_action_client_ptr->sendGoal(grasp_goal);// 发送行动给行动服务器　　让行动服务器执行
        // 等待行动服务被执行　返回结果
        success = grasp_action_client_ptr->waitForResult(ros::Duration(4.0f));

        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        检测 目标箱子的位姿
        box_pose = application.detect_box_pick();
        实现 
        /src/tasks/detect_box_pick.cpp
        detect_box_pick()

          //定义一个箱子，并且给出相应的外形信息
          shape_msgs::SolidPrimitive shape;
          shape.type = shape_msgs::SolidPrimitive::BOX;//箱子
          shape.dimensions.resize(3);
          shape.dimensions[0] = cfg.BOX_SIZE.getX();
          shape.dimensions[1] = cfg.BOX_SIZE.getY();
          shape.dimensions[2] = cfg.BOX_SIZE.getZ();

        调研 目标识别的服务获取目标物体的 坐标
        (target_recognition_client.call(srv))

        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        获取到目标箱子的 位姿路径
        pick_poses = application.create_pick_moves(box_pose);
        实现 
        /src/tasks/create_pick_moves.cpp

        创建从当前位置到抓取点的 手腕位姿路径
        首先获取工具（tcp_pick_poses）的 位姿路径
        监听工具到 手腕的坐标变换（tcp_to_wrist_tf）
        变换到手腕 的位姿路径（wrist_pick_poses）

        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        执行抓取箱子
          application.pickup_box(pick_poses,box_pose);
        实现 
        /src/tasks/pickup_box.cpp
        执行拿起箱子 使用 moveit 
        将手腕终端机构 运动到 目标物体附近
        起点->目标点->打开抓手->终点
        // 设置moveit规划组末端link
        move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

        // set allowed planning time 允许的最大规划时间
        move_group_ptr->setPlanningTime(60.0f);

        // 设置参考坐标系
            move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);
        //规划并执行
                      moveit::planning_interface::MoveGroup::Plan plan;
                  success = create_motion_plan(pick_poses[i],robot_state,plan) && 
                       move_group_ptr->execute(plan);
        set_gripper(true);//

        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        获取放置物体到目标位置 的 位姿 路径
        place_poses = application.create_place_moves();
        实现 
        /src/tasks/create_place_moves.cpp

        获取放置物体 运动规划的路径 位姿
        设置工具到 放置物体位姿
        获取tcp路径位姿
        tcp到wrist的坐标变换 
        转换到wrist下 

        // TCP位置设置为目标点位置
        world_to_tcp_tf.setOrigin(cfg.BOX_PLACE_TF.getOrigin());
        // tcp方向设置为 目标物体方向
           world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI/2.0f));
         // 获取tcp路径位姿 工具 指定位置姿态后 获取 运动路径 位姿点 起点 目标点 终点 
          tcp_place_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, 
                            cfg.APPROACH_DISTANCE,
                        world_to_tcp_tf);
          //等待坐标变换
          transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), ros::Duration(3.0f));
          /* ========  ENTER CODE HERE ======== */
          // 获取工具到 手腕的坐标变换
          transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,
                        ros::Time(0.0f), tcp_to_wrist_tf);
         // 工具TCP路径位姿点 转换到 手腕路径位姿点
          wrist_place_poses = transform_from_tcp_to_wrist( tcp_to_wrist_tf, tcp_place_poses);

        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        执行放置物体
        application.place_box(place_poses,box_pose);
        实现 
        /src/tasks/place_box.cpp

        放置物体 moveit实现
        拿住货物  set_attached_object(true,box_pose,robot_state);
        moveit   运动规划执行
        到卸货区 放下货物 set_attached_object(false,geometry_msgs::Pose(),robot_state);



        --------------------------------------------------------------------------
        ========================================================================================
        ======================================================================================
        返回到等候区
         application.move_to_wait_position();
        实现 
        /src/tasks/


        bool success; // saves the move result  规划执行结果
          // 设置规划目标位置(按位置命名设置)
          move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);
           // geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
           //move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　按位置设置　目标位置
          // set allowed planning time　设置规划运行的执行时间
          move_group_ptr->setPlanningTime(60.0f);
          // 执行
          success = move_group_ptr->move();

        pick_and_place.h
        // typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;//轨迹　规划组 指针
        // MoveGroupPtr move_group_ptr;// moveit运动规划指针　　访问需要使用　->  

        整体实验：
        启动环境 服务等
        roslaunch collision_avoidance_pick_and_place ur5_setup.launch
        启动任务
        roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch 
