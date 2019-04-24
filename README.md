# Learn ROS

[中国大学MOOC《机器人操作系统入门》课程代码示例](https://github.com/Ewenwan/ROS-Academy-for-Beginners)

[ROS 1 和 ROS 2 的前世、今生、安装使用说明与资料汇总](https://blog.csdn.net/zhangrelay/article/details/78418393)

[ROS(1和2)机器人操作系统相关书籍、资料和学习路径](https://blog.csdn.net/zhangrelay/article/details/78179097)

[move_base的全局路径规划代码研究1](https://www.cnblogs.com/shhu1993/p/6337004.html)

[move_base的全局路径规划代码研究2](https://www.cnblogs.com/shhu1993/p/6337004.html)

[move_base代码学习一](https://www.cnblogs.com/shhu1993/p/6323699.html)

[octomap中3d-rrt路径规划](https://www.cnblogs.com/shhu1993/p/7062099.html)

[ROS多个master消息互通](https://www.cnblogs.com/shhu1993/p/6021396.html)

[roscpp源码阅读](https://www.cnblogs.com/shhu1993/p/5573926.html)

[ros的源码阅读](https://www.cnblogs.com/shhu1993/p/5573925.html)

[Gazebo Ros入门](https://www.cnblogs.com/shhu1993/p/5067749.html)

[ROS源代码分析、笔记和注释](https://github.com/Ewenwan/ROS----ros_comm)

[ROS学习资料汇总 ](https://github.com/sychaichangkun/ROS_Resources)

[古月居](https://mp.weixin.qq.com/s?__biz=MzIyMzkxODg0Mw==&mid=2247484445&idx=1&sn=8f10fb4ee78da414588ffabd3eb721a6&chksm=e817ab89df60229f5888a2ec660649d81f371f16f7eff60b982e78fea0a6fe1c0762bc433e15&mpshare=1&scene=1&srcid=1023JPEqq835Iu6CamiVpO2R&pass_ticket=GUYqMrcaykeEbRgrCw0aeD%2BfAzY39PVt%2Bi56mOUARZhCrsvWuLlkpUmDb3YAV5LN#rd)

[ros下开发工具 脚本](https://github.com/carlosmccosta/ros_development_tools)

[ros编程书籍 c++ !!!!推荐](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook)

[Mastering-ROS-for-Robotics-Programming-Second-Edition 代码](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Second-Edition)

[第十四届全国大学生智能汽车竞赛室外光电竞速创意赛,ART-Racecar  ros 激光雷达+IMU建图导航](https://github.com/Ewenwan/racecar)

# 一、消息

## 1. 发布 字符串 消息
```c
#include "ros/ros.h"
#include "std_msgs/String.h"// 字符串消息 其他 int.h
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example1a");// 节点初始化
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("message", 100);// 发布消息到 message 话题，100个数据空间
  ros::Rate loop_rate(10);// 发送频率
 
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello World!"; // 生成消息
    msg.data = ss.str();
    pub.publish(msg);// 发布
    
    ros::spinOnce();// 给ros控制权
    loop_rate.sleep();// 时间没到，休息
  }
  return 0;
}

```
## 2. 订阅消息
```c

#include "ros/ros.h"
#include "std_msgs/String.h"

// 订阅消息的回调函数
void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Thanks: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example1b");
  ros::NodeHandle n;
  // 订阅话题，消息，接收到消息就会 调用 回调函数  messageCallback
  ros::Subscriber sub = n.subscribe("message", 100, messageCallback);
  ros::spin();
  return 0;
}

```
## 3. 发布自定义消息 msg
```c
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg.h" // 项目 msg文件下

// msg/chapter2_msg.msg  包含3个整数的消息
// int32 A
// int32 B
// int32 C

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example2a");
  ros::NodeHandle n;
  // 发布自定义消息====
  ros::Publisher pub = n.advertise<chapter2_tutorials::chapter2_msg>("chapter2_tutorials/message", 100);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    chapter2_tutorials::chapter2_msg msg;
    msg.A = 1;
    msg.B = 2;
    msg.C = 3;
    
    pub.publish(msg);
    ros::spinOnce();
   
    loop_rate.sleep();
  }
  return 0;
}

```


## 4. 订阅自定义消息 msg
```c
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg.h"

void messageCallback(const chapter2_tutorials::chapter2_msg::ConstPtr& msg)
{
  ROS_INFO("I have received: [%d] [%d] [%d]", msg->A, msg->B, msg->C);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example3_b");
  ros::NodeHandle n;
  // 订阅自定义消息===
  ros::Subscriber sub = n.subscribe("chapter2_tutorials/message", 100, messageCallback);
  ros::spin();
  return 0;
}

```

## 5. 发布自定义服务 srv
```c
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv.h" // 项目 srv文件下
// chapter2_srv.srv
// int32 A    请求
// int32 B 
// --- 
// int32 sum  响应---该服务完成求和服务

// 服务回调函数==== 服务提供方具有 服务回调函数
bool add(chapter2_tutorials::chapter2_srv::Request  &req, // 请求
         chapter2_tutorials::chapter2_srv::Response &res) // 回应
{
  res.sum = req.A + req.B; // 求和服务
  ROS_INFO("Request: A=%d, B=%d", (int)req.A, (int)req.B);
  ROS_INFO("Response: [%d]", (int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adder_server");
  ros::NodeHandle n;
  // 发布服务(打广告) 广而告之 街头叫卖   等待被撩.jpg
  ros::ServiceServer service = n.advertiseService("chapter2_tutorials/adder", add);
  ROS_INFO("adder_server has started");
  ros::spin();

  return 0;
}

```

## 6. 订阅服务 获取服务 强撩.jpg
```c
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adder_client");
  if (argc != 3)
  {
    ROS_INFO("Usage: adder_client A B ");
    return 1;
  }

  ros::NodeHandle n;
  // 服务客户端，需求端，调用服务
  ros::ServiceClient client = n.serviceClient<chapter2_tutorials::chapter2_srv>("chapter2_tutorials/adder");
  
  //创建服务类型
  chapter2_tutorials::chapter2_srv srv;
  
  // 设置请求内容
  srv.request.A = atoll(argv[1]);
  srv.request.B = atoll(argv[2]);
  
  // 调用服务===
  if (client.call(srv))
  {
    // 打印服务带有的响应数据====
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service adder_server");
    return 1;
  }

  return 0;
}

```

CMakeLists.txt

```c
cmake_minimum_required(VERSION 2.8.3)
project(chapter2_tutorials) # 项目名称
## 依赖包===========
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation  # 生成自定义消息的头文件
  dynamic_reconfigure
)
## 自定义消息文件====
add_message_files(
	FILES
	chapter2_msg.msg
)

## 自定义服务文件====
add_service_files(
	FILES
	chapter2_srv.srv
)

## 生成消息头文件
generate_messages(
   DEPENDENCIES
   std_msgs
)
## 依赖
catkin_package(
CATKIN_DEPENDS message_runtime
)

## 编译依赖库文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(example1a src/example_1a.cpp)
add_executable(example1b src/example_1b.cpp)

add_executable(example2a src/example_2a.cpp)
add_executable(example2b src/example_2b.cpp)

add_executable(example3a src/example_3a.cpp)
add_executable(example3b src/example_3b.cpp)
## 添加依赖
add_dependencies(example1a chapter2_tutorials_generate_messages_cpp)
add_dependencies(example1b chapter2_tutorials_generate_messages_cpp)

add_dependencies(example2a chapter2_tutorials_generate_messages_cpp)
add_dependencies(example2b chapter2_tutorials_generate_messages_cpp)

add_dependencies(example3a chapter2_tutorials_generate_messages_cpp)
add_dependencies(example3b chapter2_tutorials_generate_messages_cpp)

# 动态链接库
target_link_libraries(example1a ${catkin_LIBRARIES})
target_link_libraries(example1b ${catkin_LIBRARIES})

target_link_libraries(example2a ${catkin_LIBRARIES})
target_link_libraries(example2b ${catkin_LIBRARIES})

target_link_libraries(example3a ${catkin_LIBRARIES})
target_link_libraries(example3b ${catkin_LIBRARIES})

```

# 二、行动action类型 参数服务器 坐标变换 tf可视化 安装插件 gazebo仿真
[插件1]
(https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/tree/master/Chapter03/chapter3_tutorials/nodelet_hello_ros)

[插件2](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter03/chapter3_tutorials/pluginlib_tutorials/src/polygon_plugins.cpp)

## 1. 发布行动 action
// 类似于服务，但是是应对 服务任务较长的情况，避免客户端长时间等待，

// 以及服务结果是一个序列，例如一件工作先后很多步骤完成
```c
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> // action 服务器
#include <actionlib_tutorials/FibonacciAction.h>   // 自定义的 action类型 产生斐波那契数列 
// action/Fibonacci.action
// #goal definition        任务目标
// int32 order
// ---
// #result definition      最终 结果
// int32[] sequence
// ---
// #feedback               反馈 序列 记录中间 递增 序列
// int32[] sequence

//  定义的一个类========================
class FibonacciAction
{
// 私有=============
protected:
  ros::NodeHandle nh_; // 节点实例
  
  // 节点实例必须先被创建 NodeHandle instance 
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // 行动服务器，输入自定义的模板类似
  std::string action_name_;// 行动名称
  
  // 行动消息，用来发布的 反馈feedback / 结果result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
  
// 公开==================
public:
  // 类构造函数=============
  FibonacciAction(std::string name) :
    // 行动服务器 需要绑定 行动回调函数===FibonacciAction::executeCB====
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();// 启动
  }
  // 类析构函数========
  ~FibonacciAction(void)
  {
  }
  //  行动回调函数=========
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
  
    ros::Rate r(1);// 频率
    bool success = true;// 标志

    /* the seeds for the fibonacci sequence */
    feedback_.sequence.clear();// 结果以及反馈
    feedback_.sequence.push_back(0); // 斐波那契数列
    feedback_.sequence.push_back(1);

    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    /* start executing the action */
    for(int i=1; i<=goal->order; i++)// order 为序列数量
    {
      /* check that preempt has not been requested by the client */
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
		
        /* set the action state to preempted */
        as_.setPreempted();
        success = false;
        break;
      }
      // 产生后一个数 
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      
	  /* publish the feedback */
      as_.publishFeedback(feedback_);// 发布
      /* this sleep is not necessary, however, the sequence is computed at 1 Hz for demonstration purposes */
      r.sleep();
    }

    if(success)
    {
      // 最终结果
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      
	  /* set the action state to succeeded */
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci server");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}

```


## 2. 行动客户端 类似 服务消费者
```c
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>// action 客户端
#include <actionlib/client/terminal_state.h>      // action 状态
#include <actionlib_tutorials/FibonacciAction.h>  // 自定义行动类型

int main (int argc, char **argv)
{
  ros::init(argc, argv, "fibonacci client");

  /* create the action client
     "true" causes the client to spin its own thread */
  //  action 客户端 =====
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  
  /* will be  waiting for infinite time */
  ac.waitForServer(); // 等待 行动服务器启动

  ROS_INFO("Action server started, sending goal.");
  
  // 发布任务目标 产生20个数量的 斐波那契数列序列
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);// 发给 行动服务器=====

  // 等待 行动 执行结果
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();// 状态
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action doesnot finish before the time out.");

  return 0;
}

```

CMakeLists.txt
```c
cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)
# add_compile_options(-std=c++11)
# 找到包依赖
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
## 行动自定义文件
add_action_files(
   DIRECTORY action
   FILES Fibonacci.action
 )
## 生成行动类型 头文件
generate_messages(
 DEPENDENCIES actionlib_msgs std_msgs
)
## 包依赖
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES actionlib_tutorials
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
  DEPENDS system_lib
)
## 包含
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 编译 连接 
add_executable(fibonacci_server src/fibonacci_server.cpp)
add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries(fibonacci_server ${catkin_LIBRARIES})
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})

add_dependencies(fibonacci_server ${actionlib_tutorials_EXPORTED_TARGETS})
add_dependencies(fibonacci_client ${actionlib_tutorials_EXPORTED_TARGETS})

```


## 3. 参数服务器 parameter_server 
```c
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>// 动态参数 调整
#include <parameter_server_tutorials/parameter_server_Config.h> // 自定义的 配置参数列表

// cfg/parameter_server_tutorials.cfg===========
/*
# coding:utf-8
#!/usr/bin/env python
PACKAGE = "parameter_server_tutorials" # 包名

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()# 参数生成器

# 参数列表 ====================
gen.add("BOOL_PARAM",   bool_t,   0, "A Boolean  parameter",  True) # BOOL量类型参数
gen.add("INT_PARAM",    int_t,    0, "An Integer Parameter",  1,   0, 100) # 整形量参数
gen.add("DOUBLE_PARAM", double_t, 0, "A Double   Parameter",  0.01, 0,   1)# 浮点型变量参数
gen.add("STR_PARAM",    str_t,    0, "A String   parameter",  "Dynamic Reconfigure") # 字符串类型变量参数

#  自定义 枚举常量 类型 ==========
size_enum = gen.enum([ gen.const("Low",        int_t,  0, "Low : 0"),
                       gen.const("Medium",     int_t,  1, "Medium : 1"),
                       gen.const("High",       int_t,  2, "Hight :2")],
                       "Selection List")
# 添加自定义 变量类型
gen.add("SIZE", int_t, 0, "Selection List", 1, 0, 3, edit_method=size_enum)

# 生成 动态参数配置 头文件   以 parameter_server_ 为前缀
exit(gen.generate(PACKAGE, "parameter_server_tutorials", "parameter_server_"))

*/


// 参数改变后 的回调函数，parameter_server_Config 为参数头
void callback(parameter_server_tutorials::parameter_server_Config &config, uint32_t level)
{

  ROS_INFO("Reconfigure Request: %s %d %f %s %d", 
            config.BOOL_PARAM?"True":"False", 
            config.INT_PARAM, 
            config.DOUBLE_PARAM, 
            config.STR_PARAM.c_str(),
            config.SIZE);

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "parameter_server_tutorials");

  dynamic_reconfigure::Server<parameter_server_tutorials::parameter_server_Config> server;// 参数服务器
  dynamic_reconfigure::Server<parameter_server_tutorials::parameter_server_Config>::CallbackType f;// 参数改变 回调类型
  
  // 绑定回调函数
  f = boost::bind(&callback, _1, _2);
  // 参数服务器设置 回调器
  server.setCallback(f);

  ROS_INFO("Spinning");
  ros::spin();// 启动
  return 0;
}

```


CMakeLists.txt
```c
cmake_minimum_required(VERSION 2.8.3)
project(parameter_server_tutorials)
# add_compile_options(-std=c++11)

# 找到包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
)
# 动态参数配置文件
generate_dynamic_reconfigure_options(
  cfg/parameter_server_tutorials.cfg
)
# 依赖
catkin_package(
CATKIN_DEPENDS message_runtime
)

# 包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(parameter_server_tutorials src/parameter_server_tutorials.cpp)
add_dependencies(parameter_server_tutorials parameter_server_tutorials_gencfg)
target_link_libraries(parameter_server_tutorials ${catkin_LIBRARIES})

```


## 4. 坐标变换发布 tf_broadcaster 
```c
#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // 坐标变换发布/广播
#include <turtlesim/Pose.h>// 小乌龟位置类型

std::string turtle_name;

// 小乌龟 位姿 话题 回调函数 =======
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
  static tf::TransformBroadcaster br;// 坐标变换广播
  tf::Transform transform;// 坐标变换 
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );// 坐标位置
  tf::Quaternion q;// 位姿四元素
  q.setRPY(0, 0, msg->theta);// 按照 rpy 姿态向量形式设置 平面上只有 绕Z轴的旋转 偏航角
  transform.setRotation(q);// 姿态
  // 广播位姿变换消息=====
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  // 订阅小乌龟 位姿 话题数据  绑定回调函数 poseCallback
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
}


```


## 5. 坐标变换监听 tf_listener 
```c
#include <ros/ros.h>
#include <tf/transform_listener.h>// 坐标变换监听
#include <geometry_msgs/Twist.h>  // 消息类型
#include <turtlesim/Spawn.h>// 生成一个小乌龟

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");// 等待 生成小乌龟的服务到来
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn"); // 服务客户端
  turtlesim::Spawn srv;
  add_turtle.call(srv); // 调用服务
  
  // 发布小乌龟运动指令=====
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
  
  // 左边变换监听
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform; // 得到的坐标变换消息
    try
    {
      // 两个小乌龟坐标变换消息 之差 左边变换??
      // 有两个  坐标变换发布器 一个发布 /turtle1  一个发布 /turtle2
      listener.lookupTransform("/turtle2", "/turtle1",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    // 根据位姿差，发布 命令 让 小乌龟2 追赶上 小乌龟1
    geometry_msgs::Twist vel_msg;
    // 位置差值 计算角度
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    // 位置直线距离，关联到速度
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    // 发布速度命令
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
}

```

CMakeLists.txt
```c
cmake_minimum_required(VERSION 2.8.3)
project(tf_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

```

start_demo.launch
```c
<launch>
    <!-- Turtlesim Node 小乌龟1-->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
    <!--  小乌龟1 键盘控制 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    
    <!-- Axes -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>
    <!--  发布 小乌龟1 位姿 ->
    <node pkg="tf_tutorials" type="turtle_tf_broadcaster"
          args="/turtle1" name="turtle1_tf_broadcaster" />
    <!--  发布 小乌龟2 位姿 ->	  
    <node pkg="tf_tutorials" type="turtle_tf_broadcaster"
          args="/turtle2" name="turtle2_tf_broadcaster" />
    <!-- 监听两者位姿变换 让小乌龟2 追上 小乌龟1 ->	  
    <node pkg="tf_tutorials" type="turtle_tf_listener"
          name="listener" />

  </launch>


```

## 6. 可视化 插件
[rviz 插件](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter03/chapter3_tutorials/rviz_plugin_tutorials/src/imu_display.cpp)

[gazebo 插件](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter03/chapter3_tutorials/gazebo_plugin_tutorial/hello_world.cc)


# 三、日志 + 话题/服务/参数/action/发布图像/发布点云/发布marker


## 1. 定义 ROS_DEBUG 
```c
#include <ros/ros.h>
#include <ros/console.h> // 控制台

#define OVERRIDE_NODE_VERBOSITY_LEVEL 0

int main( int argc, char **argv )
{

  ros::init( argc, argv, "program1" );

#if OVERRIDE_NODE_VERBOSITY_LEVEL
  /* Setting the logging level manually to DEBUG */
  // 日志等级 Debug
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
#endif

  ros::NodeHandle nh;

  const double val = 3.14;

// ros 打印日志
  ROS_DEBUG( "We are looking DEBUG message" );

  ROS_DEBUG( "We are looking DEBUG message with an argument: %f", val );

  ROS_DEBUG_STREAM("We are looking DEBUG stream message with an argument: " << val);

  ros::spinOnce();

  return EXIT_SUCCESS;

}


```

## 2. 各种消息接口 名字消息 条件消息 过滤消息 单次消息 频率消息
```c
#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{

  ros::init( argc, argv, "program2" );

  ros::NodeHandle n;

  const double val = 3.14;

  /* Basic messages: 基本消息 */
  ROS_INFO( "ROS INFO message." ); // 
  ROS_INFO( "ROS INFO message with argument: %f", val ); // 相当于c中的printf; 
  ROS_INFO_STREAM( "ROS INFO stream message with argument: " << val); // 相当于c++中的cout; 

  /* Named messages: 为调试信息命名 */ 
  // 表示为这段信息命名，为了更容易知道这段信息来自那段代码．
  ROS_INFO_STREAM_NAMED("named_msg","ROS named INFO stream message; val = " << val);

  /* Conditional messages: 条件消息*/
  ROS_INFO_STREAM_COND(val < 0., "ROS conditional INFO stream message; val (" << val << ") < 0");
  ROS_INFO_STREAM_COND(val >= 0.,"ROS conditional INFO stream message; val (" << val << ") >= 0");

  /* Conditional Named messages: 条件 名字消息*/
  ROS_INFO_STREAM_COND_NAMED(val < 0., "cond_named_msg","ROS conditional INFO stream message; val (" << val << ") < 0");
  ROS_INFO_STREAM_COND_NAMED(val >= 0., "cond_named_msg","ROS conditional INFO stream message; val (" << val << ") >= 0");

  /* Filtered messages: 滤波消息*/
  struct ROSLowerFilter : public ros::console::FilterBase 
  {
    ROSLowerFilter( const double& val ) : value( val ) {}

    inline virtual bool isEnabled()
    {
      return value < 0.;// 小于0
    }

    double value;
  };

  struct ROSGreaterEqualFilter : public ros::console::FilterBase
  {
    ROSGreaterEqualFilter( const double& val ) : value( val ) {}

    inline virtual bool isEnabled()
    {
      return value >= 0.; // 大于0
    }
  
    double value;
  };

  ROSLowerFilter filter_lower(val);// 小于0的消息
  ROSGreaterEqualFilter filter_greater_equal(val);// 大于0的消息
   
   // 宏定义接口传入 过滤消息实例================
  ROS_INFO_STREAM_FILTER(
    &filter_lower,
    "ROS filter INFO stream message; val (" << val << ") < 0"
  );
  ROS_INFO_STREAM_FILTER(
    &filter_greater_equal,
    "ROS filter INFO stream message; val (" << val << ") >= 0"
  );

  /* Once messages: 单次显示*/
  for( int i = 0; i < 10; ++i ) {
  // 在循环中让信息只输出一次 
    ROS_INFO_STREAM_ONCE(
      "ROS once INFO stream message; i = " << i
    );
  }

  /* Throttle messages: 设置显示频率 */
  for( int i = 0; i < 10; ++i ) {
  // THROTTLE表示节流的意思， 代码运行两次输出一次INFO throttle message． 
    ROS_INFO_STREAM_THROTTLE(
      2,
      "ROS throttle INFO stream message; i = " << i
    );
    ros::Duration(1).sleep();
  }

  ros::spinOnce();

  return EXIT_SUCCESS;

}


```


## 3. debug  info  warn  error  fatal 
```c

#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{

    ros::init( argc, argv, "program3" );

    ros::NodeHandle nh;

    ros::Rate rate(1);

    while(ros::ok())
    {

        ROS_DEBUG_STREAM( "ROS DEBUG message.");  // debug 等级消息
        ROS_INFO_STREAM ( "ROS INFO message.");   // info  普通正常消息
        ROS_WARN_STREAM ( "ROS WARN message." );  // warn  警告消息
        ROS_ERROR_STREAM( "ROS ERROR message." ); // error 错误消息
        ROS_FATAL_STREAM( "ROS FATAL message." ); // fatal 验证错误消息

        ROS_INFO_STREAM_NAMED( "named_msg", "ROS INFO named message." );// 名字消息

        ROS_INFO_STREAM_THROTTLE(2, "ROS INFO Throttle message." );     // 频率消息

        ros::spinOnce();
        rate.sleep();
    }
    return EXIT_SUCCESS;
}


```


## 4. 自定义服务消息 客户端 + 日志打印   北京瘫.jpg
```c
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter4_tutorials/SetSpeed.h> // 自定义服务消息类型
// srv/SetSpeed.srv-----------
// float32 desired_speed   // 请求，期望速度
// ---
// float32 previous_speed  // 反馈，上一次的速度
// float32 current_speed   // 当前速度
// bool stalled            // 设置完成标志

int main( int argc, char **argv )
{

    ros::init( argc, argv, "program4" );

    ros::NodeHandle nh;
    
    // 发布温度数据
    ros::Publisher pub_temp = nh.advertise< std_msgs::Int32 >( "temperature", 1000 );// 普通整形数据话题，温度数据
    
    // 发布加速度消息 1*3 向量
    ros::Publisher pub_accel = nh.advertise< geometry_msgs::Vector3 >( "acceleration", 1000 );
    
    // 服务客户端，请求服务，获取服务，消费者
    ros::ServiceClient srv_speed = nh.serviceClient< chapter4_tutorials::SetSpeed>( "speed" );

    std_msgs::Int32 msg_temp;// 温度数据
    geometry_msgs::Vector3 msg_accel;// 三轴加速度消息
    
    chapter4_tutorials::SetSpeed msg_speed;// 服务消息

    int i = 0;

    ros::Rate rate( 1 );// 频率为1
    while( ros::ok() ) 
    {

        msg_temp.data = i;// 温度数据======

        msg_accel.x = 0.1 * i;// 三轴加速度消息===== 
        msg_accel.y = 0.2 * i;
        msg_accel.z = 0.3 * i;
        
        // 服务数据，设置 期望值，消费者提出的服务标准====
        msg_speed.request.desired_speed = 0.01 * i;// 期望速度===

        pub_temp.publish( msg_temp );// 发布温度数据
        pub_accel.publish( msg_accel );// 发布加速度消息
        
	// 服务消费者，调用服务，享受服务===
        if( srv_speed.call( msg_speed ) )// 服务数据中携带，服务反馈值
        {
	    // 日志消息打印，服务数据反馈值========================
            ROS_INFO_STREAM(
                        "SetSpeed response:\n" <<
                        "Previous speed = " << msg_speed.response.previous_speed << "\n" <<
                        "Current  speed = " << msg_speed.response.current_speed      << "\n" <<
                        "Motor stalled  = " << (msg_speed.response.stalled ? "true" : "false" )
                        );
        }
        else
        {
            /* Note that this might happen at the beginning, because
               the service server could have not started yet! */
	    // 暂时无服务，获取服务提供错误
            ROS_ERROR_STREAM( "Call to speed service failed!" );
        }

        ++i;

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;

}

```


## 5. 自定义服务消息 服务端 + 日志打印 上街叫卖.jpg
```c
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter4_tutorials/SetSpeed.h>

// 全局变量，记录前后两次的速度=====
float previous_speed = 0.;
float current_speed  = 0.;

// 订阅 温度数据话题，回调函数
void callback_temperature( const std_msgs::Int32::ConstPtr& msg )
{
    // 日志打印收到的消息
    ROS_INFO_STREAM( "Temperature = " << msg->data );
}

// 订阅加速度数据话题，回调函数
void callback_acceleration( const geometry_msgs::Vector3::ConstPtr& msg )
{
    // 日志打印收到的消息
    ROS_INFO_STREAM("Acceleration = (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
}
// 话题数据======是生产者主导==============被动消费=====容易爆仓========生产导向=================

// 服务话题回调函数=====消费者主导==========主动消费=====主动权在手=====顾客是上帝=====需求导向====
bool callback_speed(chapter4_tutorials::SetSpeed::Request  &req, // 服务请求，消费者主动发来的
                    chapter4_tutorials::SetSpeed::Response &res) // 服务反馈，提供者，完成服务后的反馈信息
{
    // 打印 服务客户端发来的 服务请求，服务要求，期望速度
    ROS_INFO_STREAM("Speed service request: desired speed = " << req.desired_speed);

    current_speed = 0.9 * req.desired_speed;// 当前速度，仿真

    res.previous_speed = previous_speed;
    res.current_speed  = current_speed;
    res.stalled        = current_speed < 0.1;

    previous_speed = current_speed;// 迭代======

    return true;
}


int main( int argc, char **argv )
{

    ros::init( argc, argv, "program5" );

    ros::NodeHandle nh;

    // 订阅话题，直接购买商品，有多少我要多少=====土豪脸.jpg
    // 温度数据 话题
    ros::Subscriber sub_temp = nh.subscribe( "temperature", 1000, callback_temperature);
    // 加速度数据话题
    ros::Subscriber sub_accel = nh.subscribe( "acceleration", 1000, callback_acceleration);
    
    // 发布服务，广播消息，打广告，请把需求砸过来!!!!!!!    可爱脸.jpg 
    ros::ServiceServer srv_speed = nh.advertiseService( "speed", callback_speed );

    ros::spin();
    
    return EXIT_SUCCESS;
}

```


## 6. 动态参数配置 + 日志
```c
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <chapter4_tutorials/DynamicParamConfig.h>// 自定义 参数
// cfg/DynamicParam.cfg-----------------------
/*
# coding: utf-8
#!/usr/bin/env python

PACKAGE='chapter4_tutorials' # 包名

from math import pi
from dynamic_reconfigure.parameter_generator_catkin import *
from dynamic_reconfigure.msg import SensorLevels

gen = ParameterGenerator() # 参数生成

gen.add('BOOL', bool_t, SensorLevels.RECONFIGURE_RUNNING,
        'Bool param', True)
gen.add('INT', int_t, SensorLevels.RECONFIGURE_STOP,
        'Int param', 0, -10, 10)
gen.add('DOUBLE', double_t, SensorLevels.RECONFIGURE_CLOSE,
        'Double param', 0.0, -pi, pi)
# 常量
foo = gen.const('ros', str_t,  'Ros',   'ROS')
bar = gen.const('cook', str_t, 'Cook', 'COOK')
baz = gen.const('book', str_t, 'Book', 'BOOK')
# 枚举变量
strings = gen.enum([foo, bar, baz], 'Strings')
# 添加自定义的枚举变量
gen.add('STRING', str_t, SensorLevels.RECONFIGURE_RUNNING,
        'String param', 'Ros', edit_method = strings)
	
# 生成消息 头文件
exit(gen.generate(PACKAGE, PACKAGE, 'DynamicParam'))
*/ 
// ------------------------------------

// 动态参数服务器
class DynamicParamServer
{
public:
    DynamicParamServer()
    {
    // 动态参数配置服务器设置，参数改变后响应的 回调函数
        _cfg_server.setCallback(boost::bind(&DynamicParamServer::callback, this, _1, _2));
    }

    void callback(chapter4_tutorials::DynamicParamConfig& config, uint32_t level)
    {
    // 打印动态配置后的参数
        ROS_INFO_STREAM(
                    "New configuration received with level = " << level << ":\n" <<
                    "BOOL   = " << config.BOOL << "\n" <<
                    "INT    = " << config.INT<< "\n" <<
                    "DOUBLE = " << config.DOUBLE << "\n" <<
                    "STRING = " << config.STRING
                    );
    }

private:
    // 接收 参数类型后实例化的 动态参数配置服务器对象
    dynamic_reconfigure::Server<chapter4_tutorials::DynamicParamConfig> _cfg_server;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "program6");

    DynamicParamServer dps;// 定义参数服务器类，修改参数后，回调函数会指定执行

    while(ros::ok())
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}

```


## 7. diagnostic_updater 诊断
[ diagnostic_updater/diagnostic_updater.h 诊断???](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter04/chapter4_tutorials/src/program7.cpp)


## 8. 发布图像消息 + 日志
```c
#include <ros/ros.h>

#include <image_transport/image_transport.h> // 图像发送
#include <cv_bridge/cv_bridge.h>// opencv 图像 转换成 ros图像
#include <sensor_msgs/image_encodings.h> // 图像编码

#include <opencv2/highgui/highgui.hpp>// opencvgui

int main( int argc, char **argv )
{
    ros::init( argc, argv, "program8" );

    ros::NodeHandle nh;

    /*Open camera with CAMERA_INDEX (webcam is typically #0).*/
    const int CAMERA_INDEX = 0; // 摄像头id
    cv::VideoCapture capture( CAMERA_INDEX );// opencv打开相机

    if(not capture.isOpened() )
    {// 打开相机发生错误
        ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
        ros::shutdown();
    }
    
    // 图像信息发送器
    image_transport::ImageTransport it(nh);
    // 发布图像消息
    image_transport::Publisher pub_image = it.advertise( "camera", 1 );
    
    // opencv 图像 带 时间戳
    cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    while( ros::ok() ) {
        capture >> frame->image;// 图像域

        if( frame->image.empty() )
        {
            ROS_ERROR_STREAM( "Failed to capture frame!" );
            ros::shutdown();
        }

        frame->header.stamp = ros::Time::now();// 时间戳
        pub_image.publish( frame->toImageMsg() );// 转换成 ros图像消息后发布出去====

        cv::waitKey( 3 );

        ros::spinOnce();
    }

    capture.release();// 释放相机=======

    return EXIT_SUCCESS;
}

```


## 9. 发布点云消息 + 日志 
```c
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>       // rviz可视化图像/marker

#include <sensor_msgs/PointCloud2.h>         // 点云消息
#include <pcl_conversions/pcl_conversions.h> // pcl类型转换成 rospcl类型
#include <pcl/point_cloud.h>// 点云
#include <pcl/point_types.h>// 点类型

int main( int argc, char **argv )
{
  ros::init( argc, argv, "program9" );

  ros::NodeHandle n;

  // 发布marker消息
  ros::Publisher pub_marker = n.advertise< visualization_msgs::Marker >( "marker", 1000 );
  // 发布点云消息
  ros::Publisher pub_pc = n.advertise< sensor_msgs::PointCloud2 >( "pc", 1000 );
  
  // 可视化marker消息----------------------------------------------------
  visualization_msgs::Marker msg_marker;
  msg_marker.header.frame_id = "/frame_world"; // 消息头，坐标系id
  msg_marker.ns = "shapes"; // 所属命名空间
  msg_marker.id = 0;        // id
  msg_marker.type = visualization_msgs::Marker::CUBE;  // 形状类型，正方体
  msg_marker.action = visualization_msgs::Marker::ADD; // 叠加

  msg_marker.pose.position.x = 0.;// 位置
  msg_marker.pose.position.y = 1.;
  msg_marker.pose.position.z = 2.;
  msg_marker.pose.orientation.x = 0.;// 姿态 四元素类型
  msg_marker.pose.orientation.y = 0.;
  msg_marker.pose.orientation.z = 0.;
  msg_marker.pose.orientation.w = 1.;

  msg_marker.scale.x = 1.;// 尺寸
  msg_marker.scale.y = 1.;
  msg_marker.scale.z = 1.;

  msg_marker.color.r = 1.; // 颜色
  msg_marker.color.g = 0.;
  msg_marker.color.b = 0.;
  msg_marker.color.a = 1.; // 透明度，不透明

  msg_marker.lifetime = ros::Duration();// 声生命周期

  ROS_INFO_STREAM( "Marker Created." );


// 点云消息--------------------------------------------
  sensor_msgs::PointCloud2 msg_pc;// rospcl 类型
  pcl::PointCloud< pcl::PointXYZ > pc;// pcl XYZ类型点云

  pc.width  = 300;
  pc.height = 200; // 有序点云
  pc.is_dense = false;// 有nan点
  pc.points.resize( pc.width * pc.height );
  // 随机生成假的点云数据
  for( size_t i = 0; i < pc.height; ++i ) {
    for( size_t j = 0; j < pc.width; ++j ) {
      const size_t k = pc.width * i + j;
      pc.points[k].x = 0.1 * i;
      pc.points[k].y = 0.2 * j;
      pc.points[k].z = 1.5;
    }
  }

  ROS_INFO_STREAM( "Point Cloud Created." );

  ros::Rate rate( 1 );
  
  while( ros::ok() )
  {
    msg_marker.header.stamp = ros::Time::now(); // marker时间戳
    msg_marker.pose.position.x += 0.01; // 位置在移动
    msg_marker.pose.position.y += 0.02;
    msg_marker.pose.position.z += 0.03;

    for( size_t i = 0; i < pc.height; ++i ) {
      for( size_t j = 0; j < pc.width; ++j ) {
        const size_t k = pc.width * i + j;

        pc.points[k].z -= 0.1; // z方向位置在移动
      }
    }

    pcl::toROSMsg( pc, msg_pc );// pcl点云类型 转换成 rospcl类型

    msg_pc.header.stamp = msg_marker.header.stamp;// 时间戳
    msg_pc.header.frame_id = "/frame_robot";// 坐标系

    pub_marker.publish( msg_marker );// 发布marker
    pub_pc.publish( msg_pc );        // 发布 点云

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}

```


## 10. 交互式marker +日志
```c
#include <ros/ros.h>
#include <tf/tf.h>

#include <interactive_markers/interactive_marker_server.h> // 交互式marker 可以响应鼠标

// 有交互后的回调函数----------------------------------------------------------------------------
void feedback_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(feedback->pose.orientation, q);// 获取四元素姿态
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);// 对应的姿态向量 
    
    // 打印marker的位置 和 姿态
    ROS_INFO_STREAM(
                feedback->marker_name << "position (x, y, z) = (" <<
                feedback->pose.position.x << ", " <<
                feedback->pose.position.y << ", " <<
                feedback->pose.position.z << "), orientation (roll, pitch, yaw) = (" <<
                roll << ", " << pitch << ", " << yaw << ")"
                );
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "program10");
    
    // 交互式marker服务器
    interactive_markers::InteractiveMarkerServer server("marker");

    visualization_msgs::InteractiveMarker marker;// 交互式marker 类型
    marker.header.frame_id = "base_link";// 头，坐标系
    marker.name = "marker";// 名字
    marker.description = "2-DOF Control";// 自我介绍

    /* Box marker */
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE; // 正方体
    box_marker.scale.x = 0.5;// 尺寸
    box_marker.scale.y = 0.5;
    box_marker.scale.z = 0.5;
    box_marker.color.r = 0.5;// 颜色
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    /* Non-interactive control which contains the box */
    visualization_msgs::InteractiveMarkerControl box_control;// 交互式marker控制
    box_control.always_visible = true;// 一直显示
    box_control.markers.push_back(box_marker);// 设置控制对象

    /* Controls to move the box */
    visualization_msgs::InteractiveMarkerControl move_x_control, rotate_z_control;
    move_x_control.name = "move_x";
    move_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;// 沿轴方向移动

    rotate_z_control.name = "rotate_z";
    rotate_z_control.orientation.w = 1;
    rotate_z_control.orientation.y = 1;
    rotate_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;// 沿轴方向旋转

// 交互式marker设置可 交互方式
    marker.controls.push_back(box_control);
    marker.controls.push_back(move_x_control);
    marker.controls.push_back(rotate_z_control);

// 交互式marker服务器吗，设置携带交互方式的 交互式marker
    server.insert(marker, &feedback_callback);
    server.applyChanges();

    ros::spin();
}

```

CMakeLists.txt
```c

cmake_minimum_required(VERSION 2.8.3)
project(chapter4_tutorials)

set(ROS_BUILD_TYPE Debug) # 编译模式

# 找到依赖包
find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

# 找依赖库
find_package(OpenCV)
find_package(PCL REQUIRED)

# 自定义服务类型
add_service_files(FILES SetSpeed.srv)
# 生成服务类型 的 头文件
generate_messages(DEPENDENCIES std_msgs)
# 生成动态参数配置参数 的头文件
generate_dynamic_reconfigure_options(cfg/DynamicParam.cfg)

# 设置包
catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      dynamic_reconfigure
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)
# 添加依赖库 
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

# 编译
add_executable(program1 src/program1.cpp)
target_link_libraries(program1 ${catkin_LIBRARIES})

add_executable(program1_dump src/program1_dump.cpp)
target_link_libraries(program1_dump ${catkin_LIBRARIES})

add_executable(program1_mem src/program1_mem.cpp)
target_link_libraries(program1_mem ${catkin_LIBRARIES})

add_executable(program2 src/program2.cpp)
target_link_libraries(program2 ${catkin_LIBRARIES})

add_executable(program3 src/program3.cpp)
target_link_libraries(program3 ${catkin_LIBRARIES})

add_executable(program4 src/program4.cpp)
add_dependencies(program4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(program4 ${catkin_LIBRARIES})

add_executable(program5 src/program5.cpp)
add_dependencies(program5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(program5 ${catkin_LIBRARIES})

add_executable(program6 src/program6.cpp)
add_dependencies(program6 ${PROJECT_NAME}_gencfg)
target_link_libraries(program6 ${catkin_LIBRARIES})

add_executable(program7 src/program7.cpp)
target_link_libraries(program7 ${catkin_LIBRARIES})

add_executable(program8 src/program8.cpp)
target_link_libraries(program8 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(program9 src/program9.cpp)
target_link_libraries(program9 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(program10 src/program10.cpp)
target_link_libraries(program10 ${catkin_LIBRARIES})

```


# 四、发布 雷达数据 坐标变换 里程计数据

## 1. 发布雷达数据
```c
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // 雷达扫描数据

int main(int argc, char** argv)
{
 ros::init(argc, argv, "laser_scan_publisher");
 ros::NodeHandle n;
 
 // 话题 发布 雷达扫描数据
 ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

 unsigned int num_readings = 100;  // 一周数据点??
 double laser_frequency = 40;      // 频率
 double ranges[num_readings];      // 范围
 double intensities[num_readings]; // 密度
 int count = 0;


 ros::Rate r(1.0);

 while(n.ok()){

    // 生成假的雷达数据=============
    for(unsigned int i = 0; i < num_readings; ++i)
    {
     ranges[i] = count;            // 距离数据
     intensities[i] = 100 + count; // 密度数据，反射强度??
    }
    
    
    // 准备雷达数据=========================
    ros::Time scan_time = ros::Time::now();
    sensor_msgs::LaserScan scan;// 定义雷达数据
    scan.header.stamp = scan_time;// 时间戳
    scan.header.frame_id = "base_link";// 坐标系
    scan.angle_min = -1.57; // 扫描最小角度 -90度
    scan.angle_max = 1.57;  // 扫描最大角度 +90度
    scan.angle_increment = 3.14 / num_readings; // 180度 100个数据，角度分辨率
    scan.time_increment = (1 / laser_frequency) / (num_readings);// 每一个扫描需要的时间，时间增量
    scan.range_min = 0.0;    // 数据范围
    scan.range_max = 100.0;
    scan.ranges.resize(num_readings); // 距离范围数据
    scan.intensities.resize(num_readings);// 强度数据??

    for(unsigned int i = 0; i < num_readings; ++i)
    {
     // 填充距离数据 和 强度数据
     scan.ranges[i] = ranges[i];
     scan.intensities[i] = intensities[i];
    }
    
    // 发布雷达数据
    scan_pub.publish(scan);
    ++count;
    r.sleep();

 }

}


```

## 2. 发布里程计数据
```c
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>   // 关节状态??
#include <tf/transform_broadcaster.h> // 左边变换 广播
#include <nav_msgs/Odometry.h>        // 导航下的里程计消息


int main(int argc, char** argv)
{
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	
	// 发布里程计消息
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	// 初始2d位姿
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	// 速度 velocity
	double vx = 0.4; // 前进线速度
	double vy = 0.0;
	double vth = 0.4;// 旋转角速度

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();// 当前时间
	last_time = ros::Time::now();   // 上次时间

	tf::TransformBroadcaster broadcaster; // 位姿 广播
	ros::Rate loop_rate(20);// 频率

	const double degree = M_PI/180; // 度转 弧度

	// message declarations
	geometry_msgs::TransformStamped odom_trans; // 坐标变换消息
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while (ros::ok()) {
		current_time = ros::Time::now(); // 当前时间

		double dt = (current_time - last_time).toSec();// 两次时间差
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		
		//     \vy y  /vx
		//      \  | /
		//       \ |/
		//        -------x-------
		//

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat;// 四元素位姿	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);// rpy转换到 四元素

		// 更新左边变换消息，tf广播发布==================
		odom_trans.header.stamp = current_time; // 当前时间
		odom_trans.transform.translation.x = x; // 位置 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);// 位姿

		// 更新 里程计消息
		nav_msgs::Odometry odom;//  里程计消息
		odom.header.stamp = current_time;// 当前时间
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// 位置 position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat; // 位姿

		// 速度 velocity
		odom.twist.twist.linear.x = vx;// 线速度
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0; // 小速度
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vth;

		last_time = current_time;// 迭代消息

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);// 发布坐标变换消息 =====
		odom_pub.publish(odom);// 发布里程计消息====

		loop_rate.sleep();
	}
	return 0;
}

```

## 3. 发布 目标位置 action
```c
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> // 移动底盘 action消息
#include <actionlib/client/simple_action_client.h>// action 客户端，发布目标
#include <tf/transform_broadcaster.h>// 坐标变换广播
#include <sstream>

// action 客户端==========
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");
	
        // action 客户端
	MoveBaseClient ac("move_base", true);
        
	// 等待action服务 启动
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server");
	}
        
	// action 目标信息 目标位置
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";// 坐标系
	goal.target_pose.header.stamp = ros::Time::now();// 时间戳

	goal.target_pose.pose.position.x = 1.0;// 目标位置
	goal.target_pose.pose.position.y = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;// 姿态

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);// 发送 action 目标

	ac.waitForResult(); // 等待 action服务端 完成action

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	return 0;
}

```

# 五、综合 应用

## 1. 三维重建 opencv pcl g2o 
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter08/chapter8_tutorials/opencv_candidate/src/reconst3d/reconstruction.cpp)


## 2. RGBD数据处理
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter08/chapter8_tutorials/opencv_candidate/src/rgbd/src/odometry.cpp)



## 3. 机器人控制  pid 笛卡尔臂控制 差分底盘控制
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter08/chapter8_tutorials/robot_controllers/robot_controllers/src/pid.cpp)



## 4. 智能抓取
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/blob/master/Chapter08/chapter8_tutorials/smart_grasping_sandbox/smart_grasping_sandbox/src/smart_grasping_sandbox/smart_grasper.py)


## 5. universal_robot  UR机械臂 UR3 UR5 UR10 MOVI配置 gazebo 运动学
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/tree/master/Chapter08/chapter8_tutorials/universal_robot)


## 6. 发布自定义消息 msg
[multi sensor fusion EKF多传感器融合框架 ](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/tree/master/Chapter09/chapter9_tutorials/ethzasl_msf)


## 7. 地理信息系统
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/tree/master/Chapter09/chapter9_tutorials/geographic_info)


## 8. 无人机仿真
[参考](https://github.com/PacktPublishing/Robot-Operating-System-Cookbook/tree/master/Chapter09/chapter9_tutorials/rotors_simulator)
