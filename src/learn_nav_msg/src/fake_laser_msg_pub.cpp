 #include <ros/ros.h>  // 系统
 #include <sensor_msgs/LaserScan.h> // 传感器消息类型
 
  int main(int argc, char** argv){
    ros::init(argc, argv, "laser_scan_publisher"); // 创建节点
    ros::NodeHandle nh;                            // 节点句柄
    //  创建发布者                               消息类型             话题     缓存区大小
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    unsigned int num_readings = 100;               // 激光雷达消息 量
    double laser_frequency = 40;                   // 频率 计算时间增量等信息
    double ranges[num_readings];                   // 距离数据
    double intensities[num_readings];              // 数据密度
    
    int count = 0;                                 // 计数 数据
    ros::Rate rate(1.0);                           // 消息发布频率
    while(nh.ok()){
        
       for(unsigned int i = 0; i < num_readings; ++i){
           ranges[i] = count;                      // 产生假的激光雷达数据
           intensities[i] = 100 + count;
        }
       ros::Time scan_time = ros::Time::now();     // 时间戳
       sensor_msgs::LaserScan scan;                // 创建雷达消息
       scan.header.stamp = scan_time;              // 定义消息头文件里的时间戳
       scan.header.frame_id = "laser_frame";       // 定义消息头文件里的 坐标
       scan.angle_min = -1.57;                     // 开始的扫描 角度 弧度  -90
       scan.angle_max = 1.57;                      // 结束的扫描 角度 弧度  +90
       scan.angle_increment = 3.14 / num_readings; // 角度增量 总共旋转 180度 分 100个数据
       scan.time_increment = (1 / laser_frequency) / (num_readings); //发送一次数据的时间 为1 / laser_frequency 总共100个数据
       scan.range_min = 0.0;                       // 最小距离范围
       scan.range_max = 100.0;                     // 最大距离范围
       scan.ranges.resize(num_readings);           // 数据量规整 调整大小
       scan.intensities.resize(num_readings);
       // 将产生的数据 赋值给 激光雷达数据
       for(unsigned int i = 0; i < num_readings; ++i){
          scan.ranges[i] = ranges[i];
          scan.intensities[i] = intensities[i];
        }
       scan_pub.publish(scan);                      // 发布消息
       ++count;                                     // 数据 加1
       rate.sleep();                                // 休息
    }
 }