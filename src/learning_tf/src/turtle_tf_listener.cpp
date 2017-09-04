#include <ros/ros.h>//ros系统库文件
#include <tf/transform_listener.h>//坐标监听类型头文件
#include <geometry_msgs/Twist.h>  //发布到小乌龟2 的 几何坐标速度等的话题上
#include <turtlesim/Spawn.h>      //新生成 一个小乌龟所调用的服务文件库

int main(int argc, char** argv){
  //初始化ros  注册节点  名字 my_tf_listener_node
  ros::init(argc, argv, "my_tf_listener_node");
  //节点句柄
  ros::NodeHandle nh;
  //等待 spawn（重生）服务可用 
  ros::service::waitForService("spawn");
  //通过调用服务spawn  添加一个小乌龟  默认为 turtle2 命名空间
  ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  //创建发布者                                 消息类型               话题名        队列大小
  ros::Publisher turtle_vel =nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
  //创建坐标系变换监听
  tf::TransformListener listener;
  //监听频率
  ros::Rate rate(10.0);
  //当nh节点没有结束   或者 ros::ok()
  while (nh.ok()){
    // 创建带有时间戳类型的坐标变换内容 变量 与 Broadcaster 发布的对应
    tf::StampedTransform transform;
    try{ //监听两个小乌龟的坐标 得到两坐标差别  /turtle2紧跟/turtle1 列队中最新可用变换 時間
      //listener.lookupTransform("/turtle2", "/turtle1",ros::Time(0), transform);
                                                            //会出错 waitForTransform 
      //listener.lookupTransform("/turtle2", "/turtle1",ros::Time::now(), transform);
      
      //监听/turtle2小乌龟 和/turtle1小乌龟旁2米的子坐标系/carrot1 得到两坐标差别  /turtle2紧跟/turtle1
      //listener.lookupTransform("/turtle2", "/carrot1",ros::Time(0), transform);
      
      //等待变换
      /*
      ros::Time now = ros::Time::now();                    //最长等待時間 3s
    listener.waitForTransform("/turtle2", "/turtle1", now, ros::Duration(3.0));
    listener.lookupTransform("/turtle2", "/turtle1", now, transform);
                             */
        // 过去時間     有问题
        /*
     ros::Time past = ros::Time::now() - ros::Duration(5.0);
    listener.waitForTransform("/turtle2", "/turtle1", past, ros::Duration(1.0));
    listener.lookupTransform("/turtle2", "/turtle1", past, transform);
    */
        //
    ros::Time now = ros::Time::now();
    ros::Time past = now - ros::Duration(5.0);
    listener.waitForTransform("/turtle2", now,
                              "/turtle1", past,
                              "/world", ros::Duration(1.0));
    listener.lookupTransform("/turtle2", now,
                             "/turtle1", past,
                             "/world", transform);
                             
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  //turtle2/cmd_vel 上的消息类型  #include <geometry_msgs/Twist.h> 
    geometry_msgs::Twist vel_msg;
  // 角速度
    vel_msg.angular.z = 4* atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
  //线速度
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
  //发布消息
    turtle_vel.publish(vel_msg);
  //空闲时间休息 可以做其它事情
    rate.sleep();
  }
  return 0;
};
