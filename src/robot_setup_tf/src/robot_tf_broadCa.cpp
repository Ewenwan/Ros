 #include <ros/ros.h> //系统文件
 #include <tf/transform_broadcaster.h>  //坐标变换
 
 int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher"); //创建节点
    ros::NodeHandle nh;                          //节点句柄
    ros::Rate rate(100);                         //频率
    tf::TransformBroadcaster bc;                 //创建坐标广播
    while(nh.ok()){
        //发送坐标变换                             变换内容 
       bc.sendTransform(                                                         //相对位置
           tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        //   时间戳   父坐标系（参考坐标系 底座）   子坐标系（雷达）
           ros::Time::now(),"base_link", "base_laser"));
       rate.sleep();
      }
  }