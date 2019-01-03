# 机械臂三维感知应用1 避免碰撞抓取和放置
[参考](http://ros-industrial.github.io/industrial_training/_source/demo1/index.html)

[Github项目](https://github.com/Ewenwan/Ros/tree/master/src/perception_driven_ws/src/collision_avoidance_pick_and_place)

        三维感知驱动的　操作　　     Perception-Driven_Manipulation
        模式识别驱动的　操作手示例　　pick and place task

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
         更新
        cd ~/perception_driven_ws/src/
        wstool update

        进入避免碰撞　包
        cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/

## 项目分析
        launch文件夹　/launch：
        ur5_setup.launch: 启动整个系统　Brings up the entire ROS system 
                  (MoveIt!, rviz, perception, ROS-I drivers, robot I/O peripherals)
                  
        ur5_pick_and_place.launch   : 抓取节点　Runs your pick and place node.


        头文件　/include/collision_avoidance_pick_and_place:
         - pick_and_place.h
         - pick_and_place_utilities.h　　应用程序变量定义


        参数配置文件夹 /config :
        ur5/
         - pick_and_place_parameters.yaml    :　　　抓取节点所需参数
         - rviz_config.rviz   : 　　　　　　　　　　　Rviz 　显示参数配置
         - target_recognition_parameters.yaml    : 　目标识别服务　参数配置　从传感器数据中　检测出　ｂｏｘ
         - test_cloud_obstacle_descriptions.yaml   : 用来产生仿真传感器数据的　参数　
         - collision_obstacles.txt   : 　　　　　　　 仿真模式下　的　障碍物　范围　　参数


        源文件　/src：
        /src/nodes:　主应用节点
         - pick_and_place_node.cpp :       主应用线程　包括消息头header  和　函数调用
         - generate_point_cloud.cpp　:      生成点云
         - collision_object_publisher.py  ：碰撞物体　发布　

        /src/services: 服务节点
        target_recognition_service.cpp　　　目标识别服务
        simulation_recognition_service.py　仿真识别服务


        /src/tasks: Source files with incomplete function definitions. 
         - create_motion_plan.cpp　　　　运动规划
         - create_pick_moves.cpp 　　　　抓取规划
         - create_place_moves.cpp　　　　放置规划
         - detect_box_pick.cpp　　　　　检测物体抓取
         - pickup_box.cpp　　　　　　　　抓起箱子
         - place_box.cpp　　　　　　　　 放置箱子
         - move_to_wait_position.cpp　 运动到等待区域
         - set_attached_object.cpp　　 设置要抓取的对象
         - set_gripper.cpp　　　　　　　设置抓手　　　打开释放
         - reset_world.cpp             重置环境


        /src/utilities:  
         - pick_and_place_utilities.cpp : 核心　Contains support functions that will help you complete the exercise.

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




