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
## 1. 发布自定义消息 msg
```c


```


## 2. 发布自定义消息 msg
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

