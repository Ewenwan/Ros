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


# 三、日志


## 7. 发布自定义消息 msg
```c


```

## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```

## 3. 发布自定义消息 msg
```c


```

## 3. 发布自定义消息 msg
```c


```

## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```

## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```


## 3. 发布自定义消息 msg
```c


```

