#include <ros/ros.h> //ros系统库文件
#include <tf/transform_broadcaster.h>//坐标广播类型头文件
#include <turtlesim/Pose.h>          //广播的消息类型 小乌龟的位置

std::string turtle_name;             //需要被广播坐标的坐标系名字变量

void poseCallback(const turtlesim::PoseConstPtr& msg){
  //定义一个 坐标变换广播
  static tf::TransformBroadcaster br; //对应 #include <tf/transform_broadcaster.h>
  //定义一个 坐标变换 变量
  tf::Transform transform;
  //初始化为坐标原点
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  //坐标变换中间变量 四元素
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  //得到坐标变换内容
  transform.setRotation(q);
  //发送坐标变换                             变换内容      时间戳    父坐标系（参考坐标系）   子坐标系
  br.sendTransform(       //带有时间戳类型的坐标变换内容类型
                       tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name)
                   );
}

int main(int argc, char** argv){
  //初始化ros  注册节点  名字 my_tf_broadcaster_node
  ros::init(argc, argv, "my_tf_broadcaster_node");
  //argument count 至有一个 坐标系 名字
  if (argc != 2){
    ROS_ERROR("need turtle name as argument");
    return -1;
    };
  
  turtle_name = argv[1];
  //节点句柄
  ros::NodeHandle nh;
  //创建一个订阅者 订阅被广播者的 位置话题信息                 列队大小  订阅者的回调函数
  ros::Subscriber sub = nh.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::Rate rate (10); //每秒订阅 10次 消息
 //当nh节点没有结束   或者 nh.ok()
 while ( ros::ok() ) {
    ros::spinOnce();  //给ROS控制权  可以调用一次回调函数
    rate.sleep();
 }
 // ros::spin();  //给ros控制权 即使能中断 可以无限次调用回调函数
  return 0;
};
