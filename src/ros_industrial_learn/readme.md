# ros工业机器人

      首先切换到自己的ROS工作空间： $ cd ~/catkin_ws
      如果你想编译整个工作空间里面的包：$ catkin_make
      如果你只想编译工作空间某一个包：$ catkin_make  -DCATKIN_WHITELIST_PACKAGES= "包名"
      例如：
      $ catkin_make  -DCATKIN_WHITELIST_PACKAGES="beginner_tutorials"


# 节点编译三部曲
      #添加可执行文件
      add_executable(ARclient_node src/ARclient.cpp) # 可执行文件
      #依赖　　解决依赖　消息　服务　action头文件　问题
      add_dependencies(ARclient_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      #连接
      target_link_libraries(ARclient_node ${catkin_LIBRARIES})


      ------------------------------------------------------------------------------------------------
      // ros::spin()是最简单的单线程自旋, 它会一直调用直到结束                     用法:  ros::spin();
      // 另一个单线程spinning是ros::spinOnce(),它定期调用等待在那个点上的所有回调  用法:  ros::spinOnce();
      // 简单的我们自己实现一个用法相同的ros::spin()
      // 这样:  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
      // ros::spinonce

      //这样:  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
       // ros::AsyncSpinner async_spinner(1);

# 那么spin到底做了什么呢?
      首先, 当我们调用ros::spin时, 会有一个互斥锁, 把你的回调队列加锁, 防止执行混乱. 
      然后, 检测如果回调队列不为空, 则读取回调队列
      最后,当while(nh.ok())为true时, 调用当前队列中的所有函数,如果有不满足的, 会重新放回队列中

      所以listener中, 就一直执行这ros::spin来监听话题了.从这样看来,spin和spinOnce的区别之一,
      就是while(nh::ok())执行块的大小了. 另一个是等待时间, spin在执行时, 
      会指定一个返回前可以等待调用的时间. spin会等待0.1s而spinonce不会

      spinOnce使得pub/sub为非阻塞锁 spin是客户端的, 因此是阻塞的.

      这样就很好理解talker要用SpinOnce,有需要talk的时候发出,没有的时候不发送.
      而listener一直在阻塞着听,ros::spin来监听话题

      回调函数一直等待在回调队列中, 只要条件一满足就会发生回调, 而spin的作用, 
      只是创建了线程给这个回调函数去执行它, 这样多线程就不会影响其他的作业.

      之所以用spin, 是因为rospy不愿指定线程模型, 在程序中将线程暴露出来, 
      而用spin来把它封装起来. 但你可以用多线程调用任意数量的回调函数.

      没有用户订阅, 服务和回调是不会被调用的.

###### 多线程Spinning ########

      roscpp内部支持调用多线程, 有两个:
      1  ros::MultiThreadedSpinner
      ros::MultiThreadedSpinner是阻塞微调, 类似于ros::spin(), 你可以在它的构造函数中指定线程数量,
       但如果不指定或者设为0, 它会根据你的CPU内核数创建线程.
        ros::MultiThreadedSpinner spinner(4); // 使用 4 线程
        spinner.spin();　　　　　　　　　　　　　　　　　　　　　 // spin() 不会返回　直到节点结束

      ２ros::AsyncSpinner (since 0.10)
      一个更有用的线程spinner是AsyncSpinner. 与阻塞的spin()不同, 
      它有start()和stop()调用, 并且在销毁时自动停止

      ros::AsyncSpinner spinner(4); // 使用 4 线程
      spinner.start();
      ros::waitForShutdown();







# 教程地址：
      industrial_training
      ===================
      Training material
[Kinetic](http://ros-industrial.github.io/industrial_training/).

      Training material before Kinetic
      ================================
[Indigo](http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/)
[Hydro](http://aeswiki.datasys.swri.edu/rositraining/hydro/Exercises/)


# 安装ar 仿真测试包
      sudo apt install ros-kinetic-calibration-msgs
      cd ~/catkin_ws/src
      git clone https://github.com/jmeyer1292/fake_ar_publisher.git

      source devel/setup.bash 
      rospack find fake_ar_publisher  //找到安装包


      【1】===================================================== 
# linux基础

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

#    【2】ros基础 
      创建包
      catkin_create_pkg pkg_name dep1 dep2
      到包文件夹
      roscd package_name

      rospack
      rospack find package_name 找到包文件夹
      rospack list              所有安装的包 的列表 
      rospack depends package_name 包依赖关系


# 节点程序对比

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


#  运行节点
      rosrun package_name node_name

      rosnode
      rosnode list 查看运行节点的列表
      rosnode info node_name 查看节点的详细信息（发布 订阅 的消息  服务）
      rosnode kill node_name 结束节点

      ////////////////////////
#  创建包
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

#  编译
      catkin_make
      运行
      roscore
      rosrun lesson_simple lesson_simple_node



#   标准消息 Standard data primitives-------------------
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


#   自定义消息类型 -----------------------------------------
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

#    修改 CMakeLists.txt
      增加包生成（编译）依赖
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
      )
#   增加需要生成的消息 的消息文件
      ## Generate messages in the 'msg' folder
       add_message_files(
         FILES
         PathPosition.msg
      #   Message1.msg
      #   Message2.msg
       )

#   增加新增消息的 依赖消息类型 标准消息/之前自定义的消息
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


#     rosmsg list 话题发布的消息列表
      rosmsg package <package>
      rosmsg show <package>/<message_type> 显示消息
      rosmsg info <package>/<message_type>


#    rostopic list 话题 列表
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

#  创建 PathPosition消息订阅者 PathPosition_subscriber.cpp 

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


#   rqt_grapt 查看 话题 节点 关系 



      #######################################################################
#   参数 Parameters    全局变量
      参数服务器 Parameter Server
      配置文件 Config File 
      节点 Node
      参数服务器 从 配置文件 获取参数值 提供给 节点
      可配置的参数有：
      --- robot kinematics          机器人运动学参数
      --- workcell description      工作空间 描述参数
      --- algorithm limits / tuning 算法限制参数/调优


#   参数配置来源--------------------------------------------------

##   1 YAML Files 参数文件 

      manipulator_kinematics:
      solver: kdl_plugin/KDLKinematics
      search_resolution: 0.005
      timeout: 0.005
      attempts: 3

##    2 命令行 Command Line

      rosrun my_pkg load_robot _ip:="192.168.1.21"
      rosparam set "/debug" true

##    3 程序 program

      nh.setParam("name", "left");

##     参数形式----------------------------------------------------
      基本数据结构  int, real, boolean， string
      列表 向量 Lists (vectors)
           混合类型 [1, str, 3.14159]
           单一类型 [1.1, 1.2, 1.3]
##   结构体 structures
       /box/weight
       /box/center/x
       /box/center/y

##    命令行  rosparam  --------------------------------
      rosparam set <key> <value>  设置参数值
      rosparam get <key>          获取参数值
      rosparam delete <key>       删除参数
      rosparam list               列出以设置的参数
      rosparam load <filename>  [<namespace>] 从参数文件 载入参数


##    C++ API -----------------------------------------
      ros::NodeHandle relative;//节点句柄未赋值
      relative.getParam("test");// 相对 命名空间参数 "/<ns>/test"

      ros::NodeHandle fixed("/myApp");//节点句柄赋值
      fixed.getParam("test");// 绝对 固定 命名空间参数 "/myApp/test"

      ros::NodeHandle priv("~");//节点句柄 ~
      priv.getParam("test");// 私有命名空间参数 "/myNode/test"

###     节点句柄 的方法有 ---------------------------------------
      nh.hasParam(key)           // 如果参数存在 返回 true
      nh.getParam(key, &value)   // 获取参数值到 value上 ，存在返回true
      nh.param(key, &value, default)// 获取参数值到 value上 ，存在返回true, 不存在 设置默认值
      nh.setParam(key, value)    // 设置参数值
      nh.deleteParam(key)        // 删除参数

###    参数动态配置  dynamic_reconfigure


###     // test.yaml
      global_integer_value: 10

      simple_parameters: {
       integer_value: 127,
       point: {
        x: 13.4,
        y: 31.8
       },
       simple_string: Test Value
      }

###     命令行载入参数
      rosparam load src/ros_industrial_learn/config/test.yaml
      rosparam get /global_integer_value  //获取参数 >>> 10
      rosparam get /simple_parameters/simple_string
      rosparam get /simple_parameters/point
      >>> {x: 13.4, y: 31.8}

      rosparam set /simple_parameters/point/x 1.75   //设置参数
      rosparam set /global_new_value "New String"    //设置新参数
      rosparam get /global_new_value                 //查看新参数

###      // C++ 程序获取参数
      ros::NodeHandle node("/simple_parameters");
      int integer_value;
      if(!node.hasParam("integer_value")) {
       ROS_WARN_STREAM("  integer_value is not set");
      } else if(!node.getParam("integer_value", integer_value)) {
         ROS_ERROR_STREAM("  integer_value is wrong type");
      } else {
         ROS_INFO_STREAM("  integer_value is " << integer_value);
      }

###     // C++ 程序设置参数
      double x, y;
      node.param("point/x", x, 0.0);
      node.param("point/y", y, 0.0);
      node.setParam("/global_sum", x + y);
      ROS_INFO_STREAM("  Sum of point values " << x << " & " << y << ": "
              << x + y)


      ##############################################################################
##   服务 Services  类似于 函数调用  有需求时才返回数据

      应用程序 客户端client   >>>>>> 请求Rrquest >>>>> 各个关键姿态   Server服务器 正运动学求解器  请求回调函数
                             末端位置 <<<<<< 回应Response <<<<<
##     典型应用 
      算法 ： 运动学、感知
      闭环命令 ： 移动到某一位置  打开抓手

###   服务定义  定义 请求和回应的数据 类型
      AddTwoInts.srv
      #Add Integers   // 注释
      int64 a         // 请求的数据类型
      int64 b
      ---             // 分割线
      int64 sum       // 回应的数据类型

      放入 /srv文件下

      同 msg自定义消息一样 需要修改package.xml 
      修改 package.xml  支持 服务数据 生成------------------------------
      <build_depend>message_generation</build_depend>  // 编译依赖
      <run_depend>message_runtime</run_depend>         // 运行依赖


###   修改   CMakeLists.txt---------------------------

      ## Generate services in the 'srv' folder
       add_service_files(
         FILES
         AddTwoInts.srv
      #   Service1.srv
      #   Service2.srv
       )

###    ## 自定义服务
      # 客户端client 发布服务（有需求） 请求
      add_executable(add_two_ints_client src/add_two_ints_client.cpp)
      target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
      add_dependencies(add_two_ints_client ${PROJECT_NAME}_generate_messages_cpp)#自定义服务 头文件依赖
      # 服务器端server 提供服务 服务回调函数 响应
      add_executable(add_two_ints_server src/add_two_ints_server.cpp)
      target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
      add_dependencies(add_two_ints_server ${PROJECT_NAME}_generate_messages_cpp)#自定义服务 头文件依赖



      // 服务器 add_two_ints_server.cpp
      #include "ros/ros.h"
      //添加srv/AddTwoInts.srv 文件 后有catkin_make 自动生成的库文件  
      #include "ros_industrial_learn/AddTwoInts.h"

####    //服务器回应 客户端服务请求的 回调函数
      //       包名      服务数据类型
      bool add(ros_industrial_learn::AddTwoInts::Request  &req,//引用 不用复制 时间短
               ros_industrial_learn::AddTwoInts::Response &res)
      {
             res.sum = req.a + req.b;
             ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
             ROS_INFO("sending back response: [%ld]", (long int)res.sum);
            return true;
      }

      int main(int argc, char **argv)
       {
          //初始化ros  和节点
         ros::init(argc, argv, "add_two_ints_server");
         ros::NodeHandle nh;//节点句柄
         //在节点创建服务端                                 订阅服务名称   回调函数
         ros::ServiceServer service = nh.advertiseService("add_two_ints", &add);
         ROS_INFO("Ready to add two ints.");
         ros::Rate rate(2) ;
         while ( ros::ok () ) {
           ros::spinOnce();
           rate.sleep();
         }
         return 0;
       }


      // 客户端  add_two_ints_client.cpp
       #include "ros/ros.h"
      //添加srv/AddTwoInts.srv 文件 后有catkin_make 自动生成的库文件     
      #include "ros_industrial_learn/AddTwoInts.h"
      #include <cstdlib>// 字符串 转成long long 类型 数
      int main(int argc, char **argv)
      {
         ros::init(argc, argv, "add_two_ints_client");//初始化
         if (argc != 3)
          {
           ROS_INFO("usage: add_two_ints_client X Y");
           return 1; 
          }
          ros::NodeHandle nh;//节点句柄
          //        客户端                                         服务类型               服务名称
          ros::ServiceClient client = nh.serviceClient<ros_industrial_learn::AddTwoInts>("add_two_ints");
          ros_industrial_learn::AddTwoInts srv;//自定义的服务
          srv.request.a = atoll(argv[1]);// 字符串 转成long long int类型 数
          srv.request.b = atoll(argv[2]);
          if (client.call(srv)) //等待服务器相应
            {   
              ROS_INFO("Sum: %ld", (long int)srv.response.sum);
            }
          else
            {
             ROS_ERROR("Failed to call service add_two_ints");
             return 1;
            } 
          return 0;
      }


###    编译
      catkin_make
      运行
      roscore
      rosrun ros_industrial_learn add_two_ints_server
      rosrun ros_industrial_learn add_two_ints_client 121 121

###      查看服务类型
      rossrv show ros_industrial_learn/AddTwoInts
      int64 a
      int64 b
      ---
      int64 sum


      ###############################################################
##     行动 动作 Action   处理长周期 运行的 任务 Long-Running Tasks

      客户端 client,
      服务端 server
      目标 Goal,      客户端 请求 目标，  服务端 接收 目标  
      结果 Result，   服务端 生成结果 ，  发生给 客户端
      反馈 FeedBack， 服务端 生成 反馈    客户端对反馈监控，可以取消 本次 action

###    典型应用 ：
        长周期 任务 ： 机器人运动 Robot Motion, 路径规划 Path Planning
        复杂序列    ： 拿起箱子 Pick Up Box, 排序任务 Sort Widgets

###    行动 动作 Action 定义 
      CalcPi.action
      #Calculate Pi   #注释
      int32 digits    #目标 Goal
      ---
      string pi       #结果 Result
      ---
      string pi       #反馈 FeedBack
      int32 iter

###    放在 action文件夹下
      action/CalcPi.action

      修改 package.xml 文件-------------------------
         <build_depend>message_generation</build_depend>
         <build_depend>actionlib</build_depend>
         <build_depend>actionlib_msgs</build_depend>

         <run_depend>message_runtime</run_depend>
         <run_depend>actionlib</run_depend>
         <run_depend>actionlib_msgs</run_depend>


      修改   CMakeLists.txt---------------------------
      #包编译依赖
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        actionlib
        actionlib_msgs
      )

      #添加行动文件
       add_action_files(
         FILES
          CalcPi.action
       )

      # 生成信息 依赖
       generate_messages(
         DEPENDENCIES
         std_msgs  # Or other packages containing msgs
         actionlib_msgs
       )

      #包运行依赖
      catkin_package(
        INCLUDE_DIRS include
      #  LIBRARIES ros_industrial_learn
        CATKIN_DEPENDS 
          roscpp rospy
          message_runtime
          actionlib
          actionlib_msgs
      #  DEPENDS system_lib
      )

      # 可执行文件
        add_executable(calcPi_server src/calcPi_server.cpp)
        target_link_libraries(calcPi_server ${catkin_LIBRARIES})
        add_dependencies(calcPi_server ${PROJECT_NAME}_generate_messages_cpp)

        add_executable(calcPi_client src/calcPi_client.cpp)
        target_link_libraries(calcPi_client ${catkin_LIBRARIES})
        add_dependencies(calcPi_client ${PROJECT_NAME}_generate_messages_cpp)
