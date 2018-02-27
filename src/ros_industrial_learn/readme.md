ros 工业机器人 学习

教程地址：
industrial_training
===================
Training material
 - [Kinetic](http://ros-industrial.github.io/industrial_training/).

Training material before Kinetic
================================
 - [Indigo](http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/)
 - [Hydro](http://aeswiki.datasys.swri.edu/rositraining/hydro/Exercises/)


【1】===================================================== 
linux基础

Ctrl+H 显示隐藏文件

结束正常运行程序 Control+C

结束跑飞的程序：
查看程序运行ID  ps ax | grep 程序名 
结束程序   
kill <id> 
kill -SIGKILL <id>

查看CPU和内存使用情况：
top
Shift+P   显示CPU使用情况排序
Shift+M   显示内存使用情况排序
q 结束

【2】ros基础 
创建包
catkin_create_pkg pkg_name dep1 dep2
到包文件夹
roscd package_name

rospack
rospack find package_name 找到包文件夹
rospack list              所有安装的包 的列表 
rospack depends package_name 包依赖关系


节点程序对比

简单C++程序

            //
            #include <iostream>
            int main(int argc, char* argv[]) 
            {
            std::cout<< "Hello World!";
            return 0;
            }
            //

简单C++ ROS节点
//
#include <ros/ros.h>
int main(int argc, char* argv[])
{
ros::init(argc, argv, "hello");// 节点初始化
ros::NodeHandle node;//节点句柄 给节点权限
ROS_INFO_STREAM("Hello World!");//输出信息
return 0;
}
//


运行节点
rosrun package_name node_name

rosnode
rosnode list 查看运行节点的列表
rosnode info node_name 查看节点的详细信息（发布 订阅 的消息  服务）
rosnode kill node_name 结束节点

////////////////////////
创建包
catkin_create_pkg lesson_simple roscpp
修改   CMakeLists.txt
add_executable(lesson_simple_node src/lesson_simple_node.cpp)
target_link_libraries(lesson_simple_node ${catkin_LIBRARIES})
新建源文件
/src/lesson_simple_node.cpp

#include <ros/ros.h>
int main(int argc, char* argv[])
{
ros::init(argc, argv, "lesson_simple_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄 给节点权限
ros::Rate loop_rate(1.0);//运行频率

int count = 0;
while(ros::ok()) {
  ROS_INFO_STREAM("Hello World, at " << count <<" times.");//输出信息
  ++count;
  loop_rate.sleep();//给节点运行权限（按照指定频率）
}
return 0;
}

编译
catkin_make
运行
roscore
rosrun lesson_simple lesson_simple_node



标准消息 Standard data primitives-------------------
– 布尔量
Boolean:          bool
– 整数 (有符号)
Integer:          int8,int16,int32,int64
– 无符号整数
Unsigned Integer: uint8,uint16,uint32,uint64
– 浮点数
Floating Point:   float32, float64
– 字符串
String:           string
• 固定长度数组
Fixed length arrays:  bool[16]
• 可变长度数组
Variable length arrays:  int32[]
• 其他类型
Other: Nest message types for more complex data structure


自定义消息类型 -----------------------------------------
以 .msg文件结尾 编译后形成 自定义消息头文件 放在 /msg文件下
位置消息类型 PathPosition.msg
# A 2D position and orientation 注释
Header  header
float64 x     # X coordinate 水平方向坐标 
float64 y     # Y coordinate 垂直方向坐标
float64 angle # Orientation  方向

修改 package.xml  支持 自定义消息类型生成
<build_depend>message_generation</build_depend>  // 编译依赖
<run_depend>message_runtime</run_depend>         // 运行依赖

修改 CMakeLists.txt
增加包生成（编译）依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)
增加需要生成的消息 的消息文件
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PathPosition.msg
#   Message1.msg
#   Message2.msg
 )

增加新增消息的 依赖消息类型 标准消息/之前自定义的消息
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

增加包 运行依赖
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros_industrial_learn
  CATKIN_DEPENDS 
    roscpp rospy
    message_runtime
#  DEPENDS system_lib
)

增加消息头文件依赖 确保 节点运行前 自定义的消息已经自动生成头文件 可能有些问题
可以先生成 消息头文件  再编译需要消息头文件的 源文件
 add_dependencies(simple_subscriber ${PROJECT_NAME}_generate_messages_cpp)
 add_dependencies(simple_publisher ${PROJECT_NAME}_generate_messages_cpp)

切换到主目录下 编译catkin_make  
可以看到在 devel/include/lesson_simple/PathPosition.h 生成了 自定义消息类型的头文件


rosmsg list 话题发布的消息列表
rosmsg package <package>
rosmsg show <package>/<message_type> 显示消息
rosmsg info <package>/<message_type>


rostopic list 话题 列表
rostopic type <topic> 话题发布的消息类型
rostopic info <topic> 话题消息 类型 发布者 订阅者 
rostopic echo <topic> 打印话题消息
rostopic find <message_type> 查询发布指定消息类型的 话题

创建 PathPosition消息发布者 PathPosition_publisher.cpp 
#include <ros/ros.h>
#include <ros_industrial_learn/PathPosition.h>//包含自动生成的 自定义消息头文件
#include <stdlib.h>                           //标准库 产生rand()随机数

int main(int argc, char* argv[])
{
ros::init(argc, argv, "PathPosition_pub_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄 给节点权限
ros::Publisher pub = node.advertise<ros_industrial_learn::PathPosition>("position", 1000);//发布消息 队列大小

ros_industrial_learn::PathPosition pp_msg;

// 发送一次
/*
pp_msg.header.stamp = ros::Time::now();//时间戳

float angle = 180.0;
pp_msg.angle = angle * 3.141592 / 180.0;//角度 弧度制
pp_msg.x = 100.0 * cos(pp_msg.angle);//水平位置
pp_msg.y = 100.0 * sin(pp_msg.angle);//垂直位置
pub.publish(pp_msg);
ros::spinOnce();//给一次控制权
ROS_INFO("Published message %.1f, %.1f, %.1f", pp_msg.x, pp_msg.y, pp_msg.angle * 180.0 / 3.141592);
*/

//一秒发送一次
///*
// srand(time(0)) ;       //Seed the random number generator
ros::Rate loop_rate (1);    //发布频率   控制 消息发布 频率   这个对象控制循环运行速度
while(ros::ok()) {
  srand(time(0)) ; 
  pp_msg.header.stamp = ros::Time::now();//时间戳
  float angle =   float(rand())  /  RAND_MAX  * 180 ;        //角度  为 0 到 180 之间的某个值
  pp_msg.angle = angle * 3.141592 / 180.0;//角度 弧度制
  pp_msg.x = 100.0 * cos(pp_msg.angle);//水平位置
  pp_msg.y = 100.0 * sin(pp_msg.angle);//垂直位置
  pub.publish(pp_msg);
  //ros::spinOnce();//给一次控制权  发布消息不需要调用回调函数
  ROS_INFO("Published message %.1f, %.1f, %.1f", pp_msg.x, pp_msg.y, pp_msg.angle * 180.0 / 3.141592);

  loop_rate.sleep();//给节点运行权限（按照指定频率）
}
//*/

return 0;
}

修改   CMakeLists.txt
add_executable(PathPosition_pub_node src/PathPosition_publisher.cpp)
target_link_libraries(PathPosition_pub_node ${catkin_LIBRARIES})

编译
catkin_make
运行
roscore
rosrun lesson_simple PathPosition_pub_node


创建 PathPosition消息订阅者 PathPosition_subscriber.cpp 

#include <ros/ros.h>
#include <ros_industrial_learn/PathPosition.h>//包含自动生成的 自定义消息头文件

void positionCallback(const ros_industrial_learn::PathPosition& msg)//常量引用 不用复制（节省时间）
{
   ROS_INFO("New position: %.1f,%.1f,%.1f", msg.x, msg.y,
    msg.angle * 180.0 / 3.141592);
}

int main(int argc, char* argv[])
{
ros::init(argc, argv, "PathPosition_sub_node");// 节点初始化
ros::NodeHandle node;//创建节点句柄  

//创建一个订阅者sub     节点句柄         话题     缓存区    函数指针   &callbackfunc 得到
ros::Subscriber sub = node.subscribe("position", 1000, positionCallback);
 //给ROS控制权 
  //ros::spin();     //给ROS控制权  订阅速度过快  占用cpu
ros::Rate loop_rate (1); //每秒订阅 1次 消息
while ( ros::ok() ) {
    ros::spinOnce();  //给ROS控制权  可以调用一次回调函数
    loop_rate.sleep();
}
return 0;
}
修改   CMakeLists.txt
add_executable(PathPosition_sub_node src/PathPosition_subscriber.cpp)
target_link_libraries(PathPosition_sub_node ${catkin_LIBRARIES})

编译
catkin_make
运行
roscore
rosrun lesson_simple PathPosition_sub_node


rqt_grapt 查看 话题 节点 关系 




参数 Parameters    全局变量
参数服务器 Parameter Server
配置文件 Config File 
节点 Node
参数服务器 从 配置文件 获取参数值 提供给 节点
可配置的参数有：
--- robot kinematics          机器人运动学参数
--- workcell description      工作空间 描述参数
--- algorithm limits / tuning 算法限制参数/调优



