 #include <ros/ros.h> //ros系统库文件
 #include <tf/transform_broadcaster.h>//坐标广播类型头文件

 int main(int argc, char** argv){
    
//初始化ros  注册节点  名字 my_tf_broadcaster_node
 ros::init(argc, argv, "add_frame_tf_broadcaster");
 // 节点句柄
 ros::NodeHandle nh;
 //定义一个 坐标变换广播
 tf::TransformBroadcaster br;
 //定义一个 坐标变换 变量
 tf::Transform transform;
 
 //每秒订阅 10次 消息
 ros::Rate rate(10.0);
 
 while (nh.ok()){
 //初始化坐标变换 变量    与父坐标系的坐标差别固  carrot1 在 turtle1 左边2 米(垂直小乌龟肚子)
// transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
 //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
 //初始化坐标变换 变量与父坐标系的坐标差别
  transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
 
 //发送坐标变换                          变换内容      时间戳    父坐标系（参考坐标系）   子坐标系
 br.sendTransform( // //带有时间戳类型的坐标变换内容类型
                  tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1")
                  );
 rate.sleep();
 }
 return 0;
};