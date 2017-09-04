 #include <ros/ros.h>   // 系统
 #include <sensor_msgs/PointCloud.h>// 消息头文件
 int main(int argc, char** argv){
 ros::init(argc, argv, "point_cloud_publisher");// 创建节点
 ros::NodeHandle nh;                            // 节点句柄
 //  创建发布者                               消息类型              话题     缓存区大小
 ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 50);
 unsigned int num_points = 100;                 // 总点数
 int count = 0;                                 // 数据
 ros::Rate rate(1.0);                           // 发布频率
 while(nh.ok()){
     sensor_msgs::PointCloud cloud;             // 创建点云 数据变量
     cloud.header.stamp = ros::Time::now();     // 创建header 时间戳
     cloud.header.frame_id = "sensor_frame";    // 创建header 坐标州
     cloud.points.resize(num_points);           // 点数据
     cloud.channels.resize(1);                  // 点附加消息 点密度 一个 "intensity" channel
     cloud.channels[0].name = "intensities";    // 附加信息名字
     cloud.channels[0].values.resize(num_points);//附加信息值
     for(unsigned int i = 0; i < num_points; ++i){
          cloud.points[i].x = 1 + count;        //产生假的 点 坐标数据
          cloud.points[i].y = 2 + count;
          cloud.points[i].z = 3 + count;
          cloud.channels[0].values[i] = 100 + count; //产生假的 点 附加信息 密度信息
        }
     cloud_pub.publish(cloud);                  //发布点云消息
     ++count;
     rate.sleep();
    }
}