 #include <ros/ros.h>                  // 系统
 #include <tf/transform_broadcaster.h> // 坐标变换
 #include <nav_msgs/Odometry.h>        // 里程记
 int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher"); // 建节点
    ros::NodeHandle nh;                          // 节点句柄
    //  创建发布者                            消息类型            话题   缓存区大小
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_tf_bc;            // 创建里程记 的坐标变换广播变量
    double x = 0.0;   // 坐标 x轴
    double y = 0.0;   // 坐标 y轴
    double th = 0.0;  // 偏航角度
    double vx = 0.1;  // 机器人 x轴（前向）速度 分量  来源与 轮子编码器 或者 惯性测量单元 这里 虚拟创建一个
    double vy = -0.1; // 机器人 y轴（左向）速度 分量
    double vth = 0.1; // 角速度
    // 或者对应速度控制命令 (vx, vy, vth) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)
    ros::Time current_time, last_time;           // 时间 变量
    current_time = ros::Time::now();             // 当前时间 时间戳
    last_time = ros::Time::now();                // 上次时间
    ros::Rate rate(1.0);                         // 发布频率
    while(nh.ok()){
       ros::spinOnce();                          // 检查
       current_time = ros::Time::now();          // 当前时间
       //计算 根据 机器人自身 线速度 和 角速度 得到 机器人位置坐标和偏行角
       double dt = (current_time - last_time).toSec();       // 单位时间 时间差
       double delta_x = (vx * cos(th) - vy * sin(th)) * dt;  // 等效到 map固定坐标系上 的前向速度vx * cos(th) - vy * sin(th)
       double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
       double delta_th = vth * dt;
       x += delta_x;   //位置
       y += delta_y;
       th += delta_th; //偏行角
       // 根据偏行角得到四元素信息
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); //根据偏行角创建 四元素
       // 创建坐标变换信息
       geometry_msgs::TransformStamped odom_trans; // 创建带时间戳的 坐标变换变量
       odom_trans.header.stamp = current_time;     // 添加header 时间戳
       odom_trans.header.frame_id = "odom";        // 添加header 坐标系
       odom_trans.child_frame_id = "base_link";    // 添加坐标变换 子坐标系
       odom_trans.transform.translation.x = x;     // 里程记坐标系到 机器人底座坐标系的一个变换
       odom_trans.transform.translation.y = y;
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = odom_quat;  // 添加四元素信息
       // 广播坐标变换信息
       odom_tf_bc.sendTransform(odom_trans);       // 发送
       // 创建 里程记信息
       nav_msgs::Odometry odom;                    // 创建里程记信息变量
       odom.header.stamp = current_time;           // 添加header 时间戳
       odom.header.frame_id = "odom";              // 添加header 坐标系
            // 设置姿态
       odom.pose.pose.position.x = x;              // 添加位置坐标
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = odom_quat;     // 添加方位信息 四元素 
            // 设置速度
       odom.child_frame_id = "base_link";          // 添加子坐标系
       odom.twist.twist.linear.x = vx;             // 机器人自身的 坐标线速度
       odom.twist.twist.linear.y = vy;             //
       odom.twist.twist.angular.z = vth;           // 自身叫角速度
       //发 布里程记消息
       odom_pub.publish(odom);
       last_time = current_time;                   // 更新时间
       rate.sleep();                               // 睡觉
     }
  }