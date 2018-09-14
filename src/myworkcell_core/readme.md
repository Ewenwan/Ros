# 参考　
          http://ros-industrial.github.io/industrial_training/_source/session4/Introduction-to-Perception.html
	  
	  ur3 
	  https://github.com/ros-industrial/industrial_training/issues/147
	  
	  #include <descartes_moveit/ikfast_moveit_state_adapter.h>
	  robot_model_ptr_.reset(new IkFastMoveitStateAdapter());

# 安装ar 仿真工件　测试包
          sudo apt install ros-indigo-calibration-msgs
          cd ~/catkin_ws/src
          git clone https://github.com/jmeyer1292/fake_ar_publisher.git

          source devel/setup.bash 
          rospack find fake_ar_publisher  //找到安装包

# 创建包
          cd ~/catkin_ws/src
          catkin_create_pkg myworkcell_core roscpp

          cd myworkcell_core
## gedit package.xml

          // 修改
            <build_depend>roscpp</build_depend> 编译依赖
            <build_depend>fake_ar_publisher</build_depend>
            <run_depend>fake_ar_publisher</run_depend> 运行依赖
            <run_depend>roscpp</run_depend>


## gedit CMakeList.txt
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

### 查看信息列表
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

##  Simple ROS Node

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

##   运行 
          roscore
          rosrun fake_ar_publisher fake_ar_publisher_node
          rosrun workcell_core vision_node
##  查看节点间关系
          rqt_graph

          ###############################################################################
##       服务 类似函数调用------------------------------------------------------
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
          
## ARserver.cpp   服务器
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
##       Action 行动   长周期的 任务  复杂序列 任务 


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
##  参数 
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
##    urdf文件 机器人描述语言
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
##     XACRO语言
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
##     启动
          roslaunch myworkcell_core urdf.launch

          切换 Fixed Frame 为 world
          添加显示
          Add ---> rviz------> RobotModel and TF 


          ---------------------------------------------------------------------------------
          ########################################################################
##    TF Coordinate Tranforms  坐标变换 发布

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

##      运行
          roslaunch myworkcell_core urdf.launch       机械臂
          roslaunch myworkcell_core workcell.launch   目标物体


          --------------------------------------------------------------
          ########################################################
##    Moveit  使用
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

##    在实际的硬件 UR5上实验
          需要在 配置文件下 myworkcell_moveit_config/config 新建一些配置文件
###        1 创建 controllers.yaml 
          ------------------------
          controller_list:
            - name: ""
              action_ns: joint_trajectory_action
              type: FollowJointTrajectory
              joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
          ---------------------

###       2 创建joint_names.yaml
          -----------------------------
          controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint] 
          -----------------------------

###      3 更新　myworkcell_moveit_config/launch/myworkcell_moveit_controller_manager.launch.xml
          ------------------------------------
          <launch>
            <arg name="moveit_controller_manager"
                 default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
            <param name="moveit_controller_manager"
                   value="$(arg moveit_controller_manager)"/>

            <rosparam file="$(find myworkcell_moveit_config)/config/controllers.yaml"/>
          </launch>
          ---------------------------------------------

###     4 创建　新文件　myworkcell_moveit_config/launch/myworkcell_planning_execution.launch
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

###       运行　
          roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch


          -----------------------------------------------------------------------------------

          ######################################################################
##      rviz moveit 运动规划

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
###      使用　moveit  C++ 接口　运动规划
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
###     运行　
          roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
          roslaunch myworkcell_core workcell.launch
          可以看到　机械臂移动到　一个固定的地方(目标物体　fake_ar_publisher的位置)


          -----------------------------------------------------
###     在RVIZ中显示　添加的　目标物体　fake_ar_publisher

          Add >>> By topic >>>> /ar_pose_visual >>> Marker 

          -----------------------------------------------------------------------
          待解决
          Try updating the workcell_node's start method to automatically move back to 
          the allZeros position after moving to the AR_target position. 
          See here for a list of move_group's available methods.
          ----------------------------------------------------------------------------


          ##################################################################
##      Descartes  笛卡尔　运动规划求解  正逆运动学求解器　solve forward and inverse kinematics
          终端　规划求解器
          机器人模型　robot model 
          轨迹点　trajectory points　　　　vector 存储
          规划器　planner
          http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes


###      １】descartes_core
          descartes_core::TrajectoryPt　　　　笛卡尔轨迹点　　　　　descartes trajectory point
          descartes_core::RobotModel 　　　　　笛卡尔规划机器人模型　descartes robot model　机器人运动学模型　kinematics of the robot　正逆运动学求解　IF/FK
          descartes_core::PathPlannerBase　笛卡尔规划器　　　　　descartes path planner


###       ２】descartes_moveit
          descartes_moveit::MoveitStateAdapter      descartes_core::RobotModel using Moveit 


###       ３】descartes_planner
          descartes_planner::DensePlanner  dense_planner   稠密求解器
          descartes_planner::SparsePlanner sparse_planner  稀疏规划器



###      ４】descartes_trajectory 笛卡尔轨迹点
          descartes_trajectory::JointTrajectoryPt

          descartes_trajectory::CartTrajectoryPt  依赖于　descartes_core::TrajectoryPt

          对称点
          descartes_trajectory::AxialSymmetricPt　　依赖于　descartes_trajectory::CartTrajectoryPt



###      ５】descartes_utilities
          descartes_utilities::toRosJointPoints  笛卡尔规划结果　转化为　标准ROS机器人关节信息



          ------------------------------------------------
##          安装
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
###          修改　package.xml文件
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

 ###         修改　CMakeLists.txt文件

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
 ###         添加服务文件
          workcell_core/PlanCartesianPath.srv
          # request 请求　目标位置
          geometry_msgs/Pose pose

          ---

          # response 响应　当前位置到目标位置　轨迹　点
          trajectory_msgs/JointTrajectory trajectory


          -------------------------------------------------
 ###         修改　descartes_node.cpp文件　　修改TODO部分

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
              // 添加关节速度
              addVel(res.trajectory);
              return true;
            }

          // 添加关节速度
          void addVel(trajectory_msgs::JointTrajectory& traj)
          {
            if (traj.points.size() < 3) return;

            auto n_joints = traj.points.front().positions.size();//关节数量

            for (auto i = 0; i < n_joints; ++i)//对每个关节
            {
              for (auto j = 1; j < traj.points.size() - 1; j++)//的每个轨迹点
              {
                // For each point in a given joint
                double delta_theta = -traj.points[j - 1].positions[i] + traj.points[j + 1].positions[i];
                double delta_time = -traj.points[j - 1].time_from_start.toSec() + traj.points[j + 1].time_from_start.toSec();
                double v = delta_theta / delta_time;
                traj.points[j].velocities[i] = v;// 添加关节速度
              } 
            }
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
###          修改　ARclient.cpp　增加　终端工具　动作规划

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
###          新建　setup.launch
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
	  
	============================================================
## 高级笛卡尔进阶  读取指定路径 csv文件 创建 笛卡尔路径规划
[文件来源](https://github.com/ros-industrial/industrial_training/tree/kinetic-devel/exercises/5.0/src)

	myworkcell_core/src 增加
	adv_descartes_node.cpp//高级笛卡尔服务节点
	adv_myworkcell_node.cpp//高级笛卡尔应用节点
	需要增加配置 路径文件
	myworkcell_core/config
	puzzle_bent.csv

	--------------------------------
	myworkcell_core/urdf下 增加
	grinder.xacro 工具手  
	puzzle_mount.xacro 操作轨迹

	需要mesh文件
	myworkcell_core/mesh
	grinder_collision.dae
	grinder_visual.dae
	puzzle_bent.dae
	puzzle_mount.stl
	------------------------------
### 修改 workcell.xacro

	<?xml version="1.0" ?>
	<robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">
	  <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
	<!-- git clone https://github.com/ros-industrial/universal_robot.git  -->
	<!-- http://ros-industrial.github.io/industrial_training/_source/session3/Workcell-XACRO.html -->
	  <!-- grinder traj 新增-->
	  <xacro:include filename="$(find myworkcell_core)/urdf/grinder.xacro" />
	  <!-- puzzle tool 新增-->
	  <xacro:include filename="$(find myworkcell_core)/urdf/puzzle_mount.xacro" />

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
	<!-- world  to grinder 新增-->
	  <joint name="world_to_grinder" type="fixed">
	    <parent link="world"/>
	    <child link="grinder_frame"/>
	    <origin xyz="0.0 -0.4 0.6" rpy="0 3.14159 0"/>
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

	<!-- robot tool0 to puzzle tool  新增 -->
	  <joint name="robot_tool" type="fixed">
	    <parent link="tool0"/>
	    <child link="ee_mount"/>
	    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
	  </joint>

	</robot>

	--------------------------------------------------
	CMakeList.txt
### 增加编译
	# adv_descartes　高级笛卡尔轨迹规划 提供从轨迹点csv文件生成轨迹点的服务　 
	add_executable(adv_descartes_node src/adv_descartes_node.cpp)
	add_dependencies(adv_descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(adv_descartes_node ${catkin_LIBRARIES})
	-----------------------------------------------------------------
### adv_descartes_node.cpp//高级笛卡尔服务节点
	----------------------------------------
	/*
	使用　Descartes  笛卡尔　运动规划求解  正逆运动学求解器　demo
	提供　返回笛卡尔规划　轨迹点集的服务
	可以提供给　客户端
	客户端再将轨迹发给　执行轨迹的　action去执行
	*/
	#include <ros/ros.h>
	#include <ros/package.h>//获取包路径 ros::package::getPath("myworkcell_core")
	#include "myworkcell_core/PlanCartesianPath.h"// 给目标点返回　轨迹点序列　的服务信息头文件（自动生成）

	#include <tf/tf.h>
	#include <tf_conversions/tf_eigen.h>
	#include <tf/transform_listener.h>
	#include <ur5_demo_descartes/ur5_robot_model.h>// Descartes  笛卡尔 ur5 模型
	#include <descartes_planner/dense_planner.h>//　Descartes  笛卡尔 规划器　稠密求解器
	#include <descartes_planner/sparse_planner.h>//　Descartes  笛卡尔 规划器　稀疏规划器
	#include <descartes_trajectory/axial_symmetric_pt.h>//　对称 轨迹点
	#include <descartes_trajectory/joint_trajectory_pt.h>
	#include <descartes_utilities/ros_conversions.h>// 轨迹点转换 到关节位姿
	#include <eigen_conversions/eigen_msg.h>// eigen 类型转换
	#include <fstream>//  文件流
	#include <string>// 文件名字符串

	// 获取当前关节位置
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

	  const Eigen::Vector3d travel = stop - start;//总向量 首位点位置差 向量
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

	// 类 默认构造函数
	  CartesianPlanner(ros::NodeHandle& nh)
	  {
	    // first init descartes
	    if (!initDescartes())//初始化笛卡尔规划器
	      throw std::runtime_error("There was an issue initializing Descartes");

	    //　　初始化笛卡尔规划轨迹服务　订阅的服务名　　      服务回调函数　　　　　　类本体
	    server_ = nh.advertiseService("adv_plan_path", &CartesianPlanner::planPath, this);
	    // 初始化 可视化轨迹 发布 marker rviz会订阅 然后显示
	    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("puzzle_path", 0);
	  }

	// 初始化笛卡尔规划器　　１初始化机器人模型　２规划器初始化
	  bool initDescartes()
	  {
	    // 1创建一个机器人模型 Create a robot model 
		//智能指针　显示消除　new  delete 内存的调研　方便　　　make_shared<T>()
	    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
	    // 1.1定义　机器人模型初始化变量   
	    // Define the relevant "frames"
	    const std::string robot_description = "robot_description";//　机器人描述
	    const std::string group_name = "puzzle";// 轨迹规划群 "manipulator"
	    const std::string world_frame = "world";// 世界坐标系
	    const std::string tcp_frame = "part";//　末端坐标系 "tool0"  tool center point 
	    // 1.2初始化机器人模型
	    // Using the desired frames, let's initialize Descartes
	    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
	    {
	      ROS_WARN("Descartes RobotModel failed to initialize");
	      return false;
	    }
	     //设置检查障碍物
	    model_->setCheckCollisions(true);

	    //2 初始化笛卡尔规划器
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
	    // 产生　执行器末端　轨迹　　按照指定路径文件产生 轨迹点
	   EigenSTL::vector_Affine3d tool_poses = makePuzzleToolPoses();//makeToolPoses();
	    visualizePuzzlePath(tool_poses);//可视化

	    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
	    // 转换成笛卡尔　轨迹点
	    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(tool_poses);

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
		//# request 请求　目标位置
		//geometry_msgs/Pose pose
		//---
		//# response 响应　当前位置到目标位置　轨迹　点
		//trajectory_msgs/JointTrajectory trajectory
	    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
	    // 添加关节速度
	    //addVel(res.trajectory);
	    return true;
	  }

	// 按照指定路径文件产生 轨迹点  x,y,z,r,p,y 6dof的位姿 转换成 4*4的 变换矩阵
	  EigenSTL::vector_Affine3d makePuzzleToolPoses()
	  {
	    //轨迹
	    EigenSTL::vector_Affine3d path;
	    std::ifstream indata;//输入文件流
	    // 获取指定 轨迹文件路径 包下config/puzzle_bent.csv
	    std::string filename = ros::package::getPath("myworkcell_core") + "/config/puzzle_bent.csv";
	    // 打开文件
	    indata.open(filename);
	    std::string line;//每一行
	    int lnum = 0;
	    while (std::getline(indata, line))
	    {
		++lnum;
		if (lnum < 3)
		  continue;//跳过前两行
		// 读物每一行数据 转换成 double 是6dof的位姿数据
		std::stringstream lineStream(line);//字符串流 每一行
		std::string  cell;
		Eigen::VectorXd xyzijk(6);//6dof的位姿数据
		int i = -2;
		while (std::getline(lineStream, cell, ','))//按，号分割
		{
		  ++i;
		  if (i == -1)//最后
		    continue;

		  xyzijk(i) = std::stod(cell);//转换成 double
		}

		Eigen::Vector3d pos = xyzijk.head<3>();//位置
		pos = pos / 1000.0;//换成m单位
		Eigen::Vector3d norm = xyzijk.tail<3>();//姿态
		norm.normalize();
		// x,y,z,r,p,y 6dof的位姿 转换成 4*4的 变换矩阵
		Eigen::Vector3d temp_x = (-1 * pos).normalized();
		Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
		Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
		Eigen::Affine3d pose;
		pose.matrix().col(0).head<3>() = x_axis;
		pose.matrix().col(1).head<3>() = y_axis;
		pose.matrix().col(2).head<3>() = norm;
		pose.matrix().col(3).head<3>() = pos;

		path.push_back(pose);
	    }
	    indata.close();//关闭文件
	    return path;
	  }
	// 轨迹点 发布marker消息 在rviz中显示
	  bool visualizePuzzlePath(EigenSTL::vector_Affine3d path)
	  {
	    int cnt = 0;
	    visualization_msgs::MarkerArray marker_array;//可视化marker 数组
	    for (auto &point : path)//范围for 遍历
	    {
	      Eigen::Vector3d pos = point.matrix().col(3).head<3>();//位置
	      Eigen::Vector3d dir = point.matrix().col(2).head<3>();//姿态方向

	      visualization_msgs::Marker marker;////可视化marker
	      marker.header.frame_id = "part";//坐标系 相对于tcp（工件）的坐标系
	      marker.header.stamp = ros::Time();//时间戳
	      marker.header.seq = cnt;//序列
	      marker.ns = "markers";//命令空间
	      marker.id = cnt;// id
	      marker.type = visualization_msgs::Marker::ARROW;//现状为箭头
	      marker.action = visualization_msgs::Marker::ADD;//增加 
	      marker.lifetime = ros::Duration(0);//生命周期
	      marker.frame_locked = true;//固定坐标系
	      marker.scale.x = 0.0002;//大小
	      marker.scale.y = 0.0002;
	      marker.scale.z = 0.0002;
	      marker.color.a = 1.0;
	      marker.color.r = 0.0;
	      marker.color.g = 1.0;//绿色
	      marker.color.b = 0.0;
	      geometry_msgs::Point pnt1, pnt2;// 几何点 消息
	      tf::pointEigenToMsg(pos, pnt1);
	      tf::pointEigenToMsg(pos + (0.005*dir), pnt2);
	      marker.points.push_back(pnt1);//点
	      marker.points.push_back(pnt2);
	      marker_array.markers.push_back(marker);
	      ++cnt;
	    }
	// 发布消息
	    vis_pub_.publish(marker_array);
	  }

	// 将EigenSTL::vector_Affine3d　位姿(4*4 T)　转换成　笛卡尔　轨迹点
	  std::vector<descartes_core::TrajectoryPtPtr>
	  makeDescartesTrajectory(const EigenSTL::vector_Affine3d& path)
	  {
	    using namespace descartes_core;
	    using namespace descartes_trajectory;
	    // 笛卡尔轨迹坐标 返回值r
	    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; //return value

	    // 得到研磨工具手的位姿
	    // need to get the transform between grinder_frame 研磨工具手 and base_link;
	    tf::StampedTransform grinder_frame;
	    Eigen::Affine3d gf;
	    listener_.lookupTransform("world", "grinder_frame", ros::Time(0), grinder_frame);
	    tf::transformTFToEigen(grinder_frame, gf);// 得到研磨工具手的位姿

	    Frame wobj_base(gf);
	    Frame tool_base = Frame::Identity();
	    TolerancedFrame wobj_pt = Frame::Identity();

	    for (auto& point : path)
	    {
	      auto p = point;//每个点
	      TolerancedFrame tool_pt(p);//工件下的点
	      tool_pt.orientation_tolerance.z_lower -= M_PI;
	      tool_pt.orientation_tolerance.z_upper += M_PI;

	      boost::shared_ptr<CartTrajectoryPt> pt(new CartTrajectoryPt(wobj_base, wobj_pt, tool_base, tool_pt, 0, M_PI/20.0));
	      descartes_path.push_back(pt);
	    }
	    return descartes_path;
	  }

	  // 关节名字 HELPER
	  std::vector<std::string> getJointNames()
	  {
	    std::vector<std::string> names;
	    nh_.getParam("controller_joint_names", names);
	    return names;
	  }


	  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;//机器人模型
	  descartes_planner::DensePlanner planner_;//笛卡尔稠密规划器
	  ros::ServiceServer server_;//服务器
	  ros::NodeHandle nh_;//节点句柄
	  tf::TransformListener listener_;//坐标变换监听
	  ros::Publisher vis_pub_;//发布可视化路径消息
	};



	int main(int argc, char** argv)
	{
	  // 初始化节点
	  ros::init(argc, argv, "adv_descartes_node");

	  ros::NodeHandle nh;
	  CartesianPlanner planner (nh);

	  ROS_INFO("Cartesian planning node starting");
	  ros::spin();
	}

	----------------------------------------------------------------------------
	==============================================================
### adv_myworkcell_node.cpp//高级笛卡尔应用节点
	----------------------------------------------
	
	/**
	**  Simple ROS Node
	**  moveit 接口 运动规划 　　　　　　　　　　　　　　　　　　　　　　运动到　工件位置
	**  Descartes  笛卡尔 轨迹规划接口　使用轨迹跟踪执行　action行动执行　工具在工件位置周围执行
	**  相当于　moveit　轨迹规划
	**  Descartes  笛卡尔  终端抓取规划
	**/
	#include <ros/ros.h>// 系统头文件
	#include <myworkcell_core/LocalizePart.h>// 定位　服务头文件
	#include <tf/tf.h>// 坐标变换
	#include <moveit/move_group_interface/move_group.h>//moveit 接口 新版 move_group_interface.h
	#include <moveit_msgs/ExecuteKnownTrajectory.h>
	//　笛卡尔规划　接口
	#include <actionlib/client/simple_action_client.h>// 动作　服务接口
	#include <control_msgs/FollowJointTrajectoryAction.h>// 控制消息　　轨迹行动　根据轨迹点集合　执行　移动到个点
	#include <myworkcell_core/PlanCartesianPath.h>// 笛卡尔规划　服务头文件　得到规划的轨迹点集合


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
	    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("adv_plan_path");
	  }

	// 执行函数
	  void start(const std::string& base_frame)
	  {
	    ROS_INFO("Attempting to localize part");

	    // 定位工件 Localize the part
	    myworkcell_core::LocalizePart srv;// 初始化 定位服务
	    srv.request.base_frame = base_frame;//基坐标系
	    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

	    if (!vision_client_.call(srv))//调用服务 得到响应数据
	    {
	      ROS_ERROR("Could not localize part");
	      return;
	    }
	    ROS_INFO_STREAM("part localized: " << srv.response);// 打印响应

	    // Plan for robot to move to part
	    //moveit::planning_interface::MoveGroupInterface move_group("manipulator");//运动规划组　配置文件里定义的　新版
	    moveit::planning_interface::MoveGroup move_group("manipulator");//运动规划组　配置文件里定义的 老板本
	    geometry_msgs::Pose move_target = srv.response.pose;//目标　位姿
	    move_group.setPoseTarget(move_target);// 设置　moveit 运动规划　目标位置
	    if(!move_group.move()){//运动到指定位置
	      ROS_ERROR("Could not plan for path in moveit planning");
	       return;
	     }

	    // 规划笛卡尔路径  Plan cartesian path
	    myworkcell_core::PlanCartesianPath cartesian_srv;// 服务
	    cartesian_srv.request.pose = move_target;// 请求的目标位置
	    if (!cartesian_client_.call(cartesian_srv))//　调用笛卡尔　规划 获取　轨迹点集
	    {
	      ROS_ERROR("Could not plan for path");
	      return;
	    }

	    // Execute descartes-planned path directly (bypassing MoveIt)
	    ROS_INFO("Got cart path, executing");
	    control_msgs::FollowJointTrajectoryGoal goal;// 行动目标
	    goal.trajectory = cartesian_srv.response.trajectory;// 笛卡尔服务目标轨迹
	    ac_.sendGoal(goal);//发生目标
	    ac_.waitForResult();//等待执行结果
	    ROS_INFO("Done");
	  }

	private://　成员变量
	  // Planning components
	  ros::ServiceClient vision_client_;//可视化 请求
	  ros::ServiceClient cartesian_client_;//笛卡尔规划请求
	  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;//笛卡尔轨迹执行 行动 请求
	};

	int main(int argc, char **argv)
	{
	// 初始化节点
	  ros::init(argc, argv, "adv_myworkcell_node");
	  ros::NodeHandle nh;//节点句柄
	  ros::NodeHandle private_node_handle("~");//私有节点 用于获取参数
	  ros::AsyncSpinner async_spinner (1);//开启一个线程

	  ROS_INFO("ScanNPlan node has been initialized");

	  std::string base_frame;//基坐标系
	// 私有节点获取参数
	  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

	  ScanNPlan app(nh);//应用类
	  ros::Duration(.5).sleep(); //等待初始化成功 wait for the class to initialize

	  async_spinner.start();//线程开启
	  app.start(base_frame);//应用开启

	  ROS_INFO("ScanNPlan node has been initialized");

	  ros::waitForShutdown();
	}

	---------------------------------------------------------------------

### 运行 rviz配置环境 
	roslaunch myworkcell_moveit_config demo.launch
	桌子上多了一个工件
	机械臂末端多了一个工具

	----------------------------------
	因为机器人模型变了
	末端多了一个 工具
### 重新配置 moveit 配置文件
	----------------------------
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

	Add Kin.Chain: base_link 到 part


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
### 在实际的硬件 UR5上实验
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

### 12 运行
	roslaunch myworkcell_moveit_config demo.launch
	--------------------------------------------------------
	============================================================
	更新 setup.launch  增加标志参数 选择起点 普通 笛卡尔应用节点ARclient_node /高级笛卡尔应用节点adv_myworkcell_node
	两个服务都启动，根据不同任务启动不同应用节点
	----------------------------------
	<?xml version="1.0" ?>
	<launch>
		<arg name="adv" default="0" />
		<include file = "$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch" >
		</include>
		<node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
		<node name="ARserver_node"  pkg="myworkcell_core" type="ARserver_node" />
		<node name="descartes_node" pkg="myworkcell_core" type="descartes_node" output="screen" />
		<node name="adv_descartes_node" pkg="myworkcell_core" type="adv_descartes_node" output="screen" />
		<node name="ARclient_node"  pkg="myworkcell_core" type="ARclient_node"  output="screen" unless="$(arg adv)">
		    <param name="base_frame" value="world"/>
		</node>
		<node name="adv_myworkcell_node"  pkg="myworkcell_core" type="adv_myworkcell_node"  output="screen" if="$(arg adv)" >
		    <param name="base_frame" value="world"/>
		</node>
	</launch>


	========================================================
### 启动
	roslaunch myworkcell_core setup.launch adv:=true





          ##############################################################
##        pcl　点云数据　模式识别

          pcl_ros PCL（Point Cloud Library）ROS接口堆，
          PCL_ROS是在ROS中涉及n维点云和3D几何处理的3D应用的首选桥梁。
          这个包提供运行ROS和PCL的接口和工具，包括nodelets、nodes和c++接口

          http://www.cnblogs.com/qixianyu/p/6607440.html


###          示例桌子　点云数据
          /industrial_training/gh_pages/_downloads/table.pcd　

          命令行显示数据　　　pcl_viewer table.pcd　

          rviz显示：
          数据格式转换话题　　pcd格式转化到　ros pointcloud　　　　
          rosrun pcl_ros pcd_to_pointcloud table.pcd 0.1 _frame_id:=map cloud_pcd:=orig_cloud_pcd
           格式转化　并发布数据到　话题 /orig_cloud_pcd 上　参考坐标系frame_id　 为 　map　
            格式为　x y z rgb

          运行rviz
          rviz
          ADD --> By topic ----> /orig_cloud_pcd  ----->  ointCloud2

###          pcl 算法库应用

####          １】　采样一致性分割算法　
          主要是从原点云中提取目标模型，比如说面，球体，圆柱等等，从而为后续的目标识别或者点云匹配等等做准备。
          使用此算法之前应该先熟悉PCL中的采样一致性（sample consensus）模块，
          里边包含了模型（平面、直线等）和采样方法（RANSAC、LMedS等）的一些枚举变量，
          一些核心的算法也包含其中，我们可以使用不同的方法提取不同的模型。

          参数输入输出：
                  此类由基类PCLBase派生，生成对象方式也很简单，如下：
          Pcl:: SACSegmentation<PointT> sac
          成员函数：
          a) inline void setModelType (int model) 所提取目标模型的属性（平面plane、球、圆柱等等）。
          b) inline void setMethodType (int method)采样方法（RANSAC、LMedS等）。
          c) inline void setDistanceThreshold (doublethreshold) 查询点到目标模型的距离阈值（如果大于此阈值，则查询点不在目标模型上，默认值为0）。
          d) inline void setMaxIterations (intmax_iterations) 最大迭代次数（默认值为50）。
          e) inline void setProbability (double probability) 至少一个样本不包含离群点的概率（默认值为0.99）。
          f) virtual void segment (PointIndices &inliers,ModelCoefficients &model_coefficients) 输出提取点的索引和目标模型的参数。

          ---------------------------------------------------------------------------
#####          命令行　示例

          1 获取桌面
          分割平面　　　　　　　　　　　原来的　　　新生成的　　　　分割阈值
          pcl_sac_segmentation_plane table.pcd only_table.pcd -thresh 0.01
          显示　pcl_viewer only_table.pcd
          rviz显示
          rosrun pcl_ros pcd_to_pointcloud only_table.pcd 0.1 _frame_id:=map cloud_pcd:=only_table_cloud
          rviz

          2 获取桌面上的物品　　　　　　　　　　　　　　　　　　　　　　　　　　　　剩余最大的cluster
          pcl_sac_segmentation_plane table.pcd object_on_table.pcd -thresh 0.01 -neg 1
          显示 pcl_viewer object_on_table.pcd
          rviz显示
          rosrun pcl_ros pcd_to_pointcloud object_on_table.pcd 0.1 _frame_id:=map cloud_pcd:=object_on_table_cloud
          rviz

####          2】　点云滤波算法　　
          http://blog.csdn.net/qq_33933704/article/details/78649728
          一）使用VoxelGrid滤波器对点云进行下采样　
          体素化网格方法实现下采样，即减少点的数量 减少点云数据，并同时保存点云的形状特征，
          在提高配准，曲面重建，形状识别等算法速度中非常实用，PCL是实现的VoxelGrid类通过输入的点云数据
          创建一个　三维体素栅格，容纳后每个体素内用体素中所有点的重心来近似显示体素中其他点，这样该体素内所有点都用
          一个重心点最终表示，对于所有体素处理后得到的过滤后的点云，这种方法比用体素中心逼近的方法更慢，
          但是对于采样点对应曲面的表示更为准确。

          二）使用statisticalOutlierRemoval滤波器移除离群点
          使用统计分析技术，从一个点云数据中集中移除测量噪声点（也就是离群点）

          比如：激光扫描通常会产生密度不均匀的点云数据集，另外测量中的误差也会产生稀疏的离群点，
          使效果不好，估计局部点云特征（例如采样点处法向量或曲率变化率）的运算复杂，
          这会导致错误的数值，反过来就会导致点云配准等后期的处理失败。

          解决办法：每个点的邻域进行一个统计分析，并修剪掉一些不符合一定标准的点，
          稀疏离群点移除方法基于在输入数据中对点到临近点的距离分布的计算，对每一个点，
          计算它到它的所有临近点的平均距离，，假设得到的结果是一个高斯分布，其形状是由均值和标准差决定，
          平均距离在标准范围之外的点，可以被定义为离群点并可从数据中去除。


#####          1 下采样示例
          示例　　　　　　　　　　　　　　　　　　　　　　　设置滤波时创建的体素体积为 5cm 的立方体  
          pcl_voxel_grid table.pcd table_downsampled5cm.pcd -leaf 0.05,0.05,0.05
          显示　pcl_viewer table_downsampled5cm.pcd
          rviz显示
          rosrun pcl_ros pcd_to_pointcloud table_downsampled5cm.pcd 0.1 _frame_id:=map cloud_pcd:=table_downsampled5cm_cloud
          rviz
                                 设置滤波时创建的体素体积为 3cm 的立方体  
          pcl_voxel_grid table.pcd table_downsampled3cm.pcd -leaf 0.03,0.03,0.03
          显示　pcl_viewer table_downsampled3cm.pcd
          rviz显示
          rosrun pcl_ros pcd_to_pointcloud table_downsampled3cm.pcd 0.1 _frame_id:=map cloud_pcd:=table_downsampled3cm_cloud
          rviz


#####          2 移除　离群点示例
          pcl_outlier_removal table.pcd table_outlier_removal.pcd -method statistical
          显示　pcl_viewer table_outlier_removal.pcd
          rviz显示
          rosrun pcl_ros pcd_to_pointcloud table_outlier_removal.pcd 0.1 _frame_id:=map cloud_pcd:=table_outlier_removal
          rviz


####          3】　估计点云的法向量
          估计平面的法向量　　　　平面估计　大小半价参数　
          pcl_normal_estimation only_table.pcd table_normals.pcd -radius 0.1
          查看　pcl_viewer table_normals.pcd -normals  20

####          4】曲面重建
          pcl_marching_cubes_reconstruction table_normals.pcd table_mesh.vtk -grid_res 50
          pcl_viewer table_mesh.vtk

          MarchingCubes(移动立方体)算法是目前三围数据场等值面生成中最常用的方法。
          它实际上是一个分而治之的方法，把等值面的抽取分布于每个体素中进行。对于每个被处理的体素，
          以三角面片逼近其内部的等值面片。每个体素是一个小立方体，构造三角面片的处理过程对每个体素都“扫描”一遍，
          就好像一个处理器在这些体素上移动一样，由此得名移动立方体算法。

          MC算法主要有三步：
          1.将点云数据转换为体素网格数据；
          2.使用线性插值对每个体素抽取等值面；
          3.对等值面进行网格三角化


          #########################################################
          ###############################################################
 #         应用　　【１】
 #         三维感知驱动的　操作　　Perception-Driven_Manipulation
          模式识别驱动的　操作手示例　　pick and place task

          包含内容：
          perception　　　　　　　　　　模式识别
          controller drivers　　控制驱动
          I/O		　　　　I/O口
          inverse kinematics　　逆运动学
          path planning　　　　　　　运动规划
          collision avoidance　碰撞检测

##          获取　文件
          https://github.com/ros-industrial/industrial_training
          三维感知驱动的　操作
          cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/template_ws ~/perception_driven_ws
           更新
          cd ~/perception_driven_ws/src/
          wstool update

          进入避免碰撞　包
          cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/

 ###         launch文件夹　/launch：
          ur5_setup.launch     : 启动整个系统　Brings up the entire ROS system 
                    (MoveIt!, rviz, perception, ROS-I drivers, robot I/O peripherals)
          ur5_pick_and_place.launch   : 抓取节点　Runs your pick and place node.


          头文件　/include/collision_avoidance_pick_and_place:
           - pick_and_place.h
           - pick_and_place_utilities.h　　应用程序变量定义




  ###        参数配置文件夹 /config :
          ur5/
           - pick_and_place_parameters.yaml    :　　　　　　　　抓取节点所需参数
           - rviz_config.rviz   : 　　　　　　　　　　　　　　　　　　　　　　Rviz 　显示参数配置
           - target_recognition_parameters.yaml    : 　　　目标识别服务　参数配置　从传感器数据中　检测出　ｂｏｘ
           - test_cloud_obstacle_descriptions.yaml    : 用来产生仿真传感器数据的　参数　
           - collision_obstacles.txt   : 　　　　　　　　　　　　　　　仿真模式下　的　障碍物　范围　　参数


   ###       源文件　/src：
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


###       建立包的依赖环境
          cd ~/perception_driven_ws
          catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
          source devel/setup.bash


 ###         启动 UR5仿真环境  目标识别服务等
          roslaunch collision_avoidance_pick_and_place ur5_setup.launch　　默认都是仿真的
          roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false　　　真实的传感器Kinect 假的机器人
          roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]　真实的机器人　传递ip　假的传感器
          roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
          真实的传感器　真是的机器人　真实的抓取手
          --------------------------------------------------------------------
####   ur5环境启动　目标识别服务等
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

###    应用程序变量　定义　

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
#### 而在　
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

####   运动到　等候区　函数实现　
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
####   设置抓手状态　  application.set_gripper(false);
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
####   检测 目标箱子的位姿
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
####     获取到目标箱子的 位姿路径
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
####     执行抓取箱子
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
####    获取放置物体到目标位置 的 位姿 路径
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
####  执行放置物体
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
####   返回到等候区
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


###      整体实验：
          启动环境 服务等
          roslaunch collision_avoidance_pick_and_place ur5_setup.launch
          启动任务
          roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch 


          ########################################################################################
          ###################################################################################
#     应用2 笛卡尔规划
          http://ros-industrial.github.io/industrial_training/_source/demo2/Application-Structure.html

          获取　文件
          https://github.com/ros-industrial/industrial_training

          cd ~/industrial_training/exercises/Descartes_Planning_and_Execution
          cp -r template_ws ~/descartes_ws
          cd ~/descartes_ws/src
          catkin_init_workspace
          cd ..
          rosinstall src
          catkin build
          source devel/setup.bash

##  规划执行文件夹/
          plan_and_run/

              src : 应用程序源文件 Application source files.
              src/demo_application.cpp : 类实现数据结构、成员函数 源文件.
              src/plan_and_run.cpp :  主应用节点  
              src/tasks :  任务文件
              include : 头文件
              include/plan_and_run/demo_application.h : 全局变量
              launch: Launch文件气动 应用
              launch/demo_setup.launch : 起点运行环境
              launch/demo_run.launch :   起点任务节点
              config: 配置文件


##       主应用节点  
          plan_and_run/src/plan_and_run_node.cpp

          可能需要修改 机器人描述语言文件
          /Descartes_Planning_and_Execution/src/ur5_demo_support/urdf/ur5_demo_robot.xacro

            <xacro:property name="PI" value="3.14159265359"/>
            <xacro:property name="pi" value="3.14159265359"/>
            <xacro:property name="pi_2" value="1.5707963267948966"/>

          ==================================================================
##    仿真启动
          环境
          roslaunch plan_and_run demo_setup.launch
          应用节点 精确轨迹规划
          roslaunch plan_and_run demo_run.launch
          ==============================================

##    环境+真实机器人
          roslaunch plan_and_run demo_setup.launch sim:=false robot_ip:=000.000.0.00 
          应用节点 精确轨迹规划
          roslaunch plan_and_run demo_run.launch
          -------------------------------------------------
          launch文件 启动应用节点 设置参数 demo_run.launch

          <?xml version="1.0" ?>
          <launch>
            <node name="plan_and_run_node" type="plan_and_run_node" pkg="plan_and_run" output="screen">
              <param name="group_name" value="manipulator"/>
              <param name="tip_link" value="tool"/>
              <param name="base_link" value="base_link"/>
              <param name="world_frame" value="world"/>
              <param name="trajectory/time_delay" value="0.1"/>
              <param name="trajectory/foci_distance" value="0.07"/>
              <param name="trajectory/radius" value="0.08"/>
              <param name="trajectory/num_points" value="200"/>
              <param name="trajectory/num_lemniscates" value="4"/>
              <rosparam param="trajectory/center">[0.36, 0.2, 0.1]</rosparam>
              <rosparam param="trajectory/seed_pose">[0.0, -1.03, 1.57 , -0.21, 0.0, 0.0]</rosparam>
              <param name="visualization/min_point_distance" value="0.02"/>
            </node>
          </launch>


          ====================================================
###    载入参数

          主节点 plan_and_run_node.cpp
            // creating application plan_and_run命令空间下 创建类
            plan_and_run::DemoApplication application;
            // loading parameters 载入参数
            application.loadParameters();

          实现函数为  私有节点 获取参数
          src/tasks/load_parameters.cpp
          -------------------------------
            ros::NodeHandle ph("~");

            // creating handle with public scope
            ros::NodeHandle nh;
            // plan_and_run/demo_application.h 
            // 类私有结构体变量 DemoConfiguration config_;
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
          ==================================================================
###      初始化ROS系统
          主节点 plan_and_run_node.cpp
            // initializing ros components 初始化 ros系统
            application.initRos();

          实现函数为 
          src/tasks/init_ros.cpp

          可视化马卡消息（发送到rviz显示）
          visualization_msgs::MarkerArray
          // 可视化轨迹发布器 可视化马卡消息（发送到rviz显示）
            marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

####      // 创建一个服客户端节点 来发送 规划的路径点 客户端 轨迹发布
          moveit_run_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);

          等待 moveit 执行轨迹服务器  /execute_kinematic_path 出现
          moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT))




####     执行轨迹还可以使用  轨迹跟随行动 实现
          类头文件 demo_application.h
          添加头文件
          //　笛卡尔规划　行动服务接口
          #include <actionlib/client/simple_action_client.h>// 动作　服务接口
          #include <control_msgs/FollowJointTrajectoryAction.h>// 控制消息　　轨迹行动　根据轨迹点集合　执行　移动到个点

          创建类私有变量  轨迹执行 行动 客户端
          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;// 行动请求 客户端

          类实现文件 demo_application.cpp
          类 构造函数 传递 客户端初始化列表
          // 构造函数  带有　笛卡尔轨迹执行行动 初始化　列表
          //DemoApplication::DemoApplication():ac_("joint_trajectory_action", true)
          //{}
          初始化ROS系统 init_ros.cpp 中添加 
          等待行动服务器 （可省略）
            ROS_INFO_STREAM("Waiting for action server ...");
            ac_.waitForServer();
          然后在 轨迹执行函数中 发出行动请求 run_path.cpp



          ===============================================================
###         初始化笛卡尔规划器
          主节点 plan_and_run_node.cpp
            // initializing descartes    初始化 笛卡尔规划
            application.initDescartes();

          实现函数为 
          src/tasks/init_descartes.cpp
            // descartes_core::RobotModelPtr robot_model_ptr_;
            // 笛卡尔机器人模型 正逆运动学 碰撞检测 Performs tasks specific to the Robot
            // Instantiating a robot model 创建一个机器人模型 用来初始化 笛卡尔规划器
            robot_model_ptr_.reset(new ur5_demo_descartes::UR5RobotModel());
          // 初始化机器人模型
            if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,//机器人描述 string
                                            config_.group_name,//规划组名字
                                            config_.world_frame,//世界坐标系
                                            config_.tip_link))//末端执行 关节

            // 类头文件定义  descartes_planner::SparsePlanner planner_; // 笛卡尔稀疏规划器 
            // 初始化 笛卡尔规划器
            bool succeeded = planner_.initialize(robot_model_ptr_);
          ===============================================================================


###        移动到等候区（家位置）
          主节点 plan_and_run_node.cpp
            // moving to home position 移动到等候区（家位置）
            application.moveHome();
          实现函数为 
          src/tasks/move_home.cpp

          // 创建moveit规划接口 creating move group interface for planning simple moves
            moveit::planning_interface::MoveGroup move_group(config_.group_name);
            move_group.setPlannerId(PLANNER_ID);//设置规划器ID 区别

            // 设置目标位置（家的位置） setting home position as target 
            if(!move_group.setNamedTarget(HOME_POSITION_NAME))

          // 规划执行 并返回结果
            moveit_msgs::MoveItErrorCodes result  = move_group.move();

          /*
          moveit_msgs/MoveItErrorCodes.msg
          int32 val

          # overall behavior
          int32 SUCCESS=1
          int32 FAILURE=99999

          int32 PLANNING_FAILED=-1
          int32 INVALID_MOTION_PLAN=-2
          int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
          int32 CONTROL_FAILED=-4
          int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
          int32 TIMED_OUT=-6
          int32 PREEMPTED=-7

          # planning & kinematics request errors
          int32 START_STATE_IN_COLLISION=-10
          int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11

          int32 GOAL_IN_COLLISION=-12
          int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
          int32 GOAL_CONSTRAINTS_VIOLATED=-14

          int32 INVALID_GROUP_NAME=-15
          int32 INVALID_GOAL_CONSTRAINTS=-16
          int32 INVALID_ROBOT_STATE=-17
          int32 INVALID_LINK_NAME=-18
          int32 INVALID_OBJECT_NAME=-19

          # system errors
          int32 FRAME_TRANSFORM_FAILURE=-21
          int32 COLLISION_CHECKING_UNAVAILABLE=-22
          int32 ROBOT_STATE_STALE=-23
          int32 SENSOR_INFO_STALE=-24

          # kinematics errors
          int32 NO_IK_SOLUTION=-31

          */

          ============================================================================
###         产生笛卡尔规划轨迹 引用形参传回
          // 类头文件 类型定义
          //typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;//笛卡尔轨迹 点 数组
            // generating trajectory  产生笛卡尔规划轨迹 引用形参传回
            plan_and_run::DescartesTrajectory traj;
            application.generateTrajectory(traj);


          实现函数为 
          src/tasks/generate_trajectory.cpp
          产生精确的笛卡尔轨迹
          双纽曲线函数 创建轨迹  Eigen::Vector3d
          descartes_trajectory::AxialSymmetricPt(); 创建笛卡尔轨迹（带有姿态）

          ==========================================================================
###        规划 笛卡尔轨迹  
            // planning robot path   规划机器人路径
            plan_and_run::DescartesTrajectory output_path;
            application.planPath(traj,output_path);

          实现函数为 
          src/tasks/plan_path.cpp

          规划机器人路径
          优化首尾点位姿
          规划器 按轨迹点规划出 机器人笛卡尔轨迹 planner_.planPath(input_traj);
          得到规划器 规划的 笛卡尔轨迹 succeeded = planner_.getPath(output_path);


          ==========================================================================
###        执行路径
            // running robot path  执行路径
            application.runPath(output_path);
          实现函数为 
          src/tasks/run_path.cpp


####         1 使用movieit  /execute_kinematic_path服务实现 运动
           执行笛卡尔规划器规划的轨迹
          1 由轨迹第一个点 得到机械臂 各个关节的位姿 first_point_ptr->getNominalJointPose(seed_pose, *robot_model_ptr_, start_pose);
          2 机械臂运动到 初始的 启动状态 move_group.setJointValueTarget(start_pose);//设定关节位姿
          3 转换 笛卡尔轨迹 到 moveit 机械臂关节 轨迹 并添加关节速度 fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
          4 发送执行轨迹的请求 给 执行轨迹的服务器   
             moveit_msgs::ExecuteKnownTrajectory srv; //  服务的提供节点 为 ros系统 /move_group 节点提供服务
             srv.request.trajectory = moveit_traj;
             moveit_run_path_client_.call(srv)

          执行的服务为 /execute_kinematic_path
          需要在moveit配置文件的 move_group.launch启动文件中 开启功能
          move_group.launch

              <param name="capabilities" value="move_group/MoveGroupExecuteService
                      " />
          更多的能力：
              <param name="capabilities" value="move_group/MoveGroupCartesianPathService
                        move_group/MoveGroupExecuteService
                        move_group/MoveGroupKinematicsService
                        move_group/MoveGroupMoveAction
                        move_group/MoveGroupPickPlaceAction
                        move_group/MoveGroupPlanService
                        move_group/MoveGroupQueryPlannersService
                        move_group/MoveGroupStateValidationService
                        move_group/MoveGroupGetPlanningSceneService
                        move_group/ClearOctomapService
                        " />
          ----------------------------------------------------------
####        2 也可用 执行轨迹行动action实现 

              trajectory_msgs::JointTrajectory RBjoint_trajectory;
              // demo_application.c
              fromDescartesToMoveitTrajectory(path, RBjoint_trajectory);
              control_msgs::FollowJointTrajectoryGoal goal;// 行动中的　目标
              goal.trajectory = RBjoint_trajectory;// 目标轨迹
              ac_.sendGoal(goal);// 发送目标
              bool FLAG = ac_.waitForResult();// 等待执行结果   bool = ac_.waitForResult();
              if(FLAG) ROS_INFO_STREAM("Robot path execution completed");
              else{ ROS_ERROR_STREAM("Failed to run robot path with error "); exit(-1);}



          =================================================================================


