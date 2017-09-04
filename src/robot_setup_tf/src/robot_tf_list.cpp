   #include <ros/ros.h>  //系统
   #include <geometry_msgs/PointStamped.h> //带有时间戳的位子
   #include <tf/transform_listener.h>      //坐标变换监听
   
   //回调函数
   void transformPoint(const tf::TransformListener& listener){
       geometry_msgs::PointStamped laser_point;     //激光雷达位置坐标
       laser_point.header.frame_id = "base_laser";  //坐标系为  base_laser
       laser_point.header.stamp = ros::Time();      //最新时间 的坐标变换
       //just an arbitrary point in space
       laser_point.point.x = 1.0;                   //雷达的位置
       laser_point.point.y = 0.2;
       laser_point.point.z = 0.0;
       try{
        geometry_msgs::PointStamped base_point;     //底座的位置
         //由雷达的位置和 监听得到的base_link坐标系与base_laser坐标系的哦坐标变换 得到 底座的位置坐标
        listener.transformPoint("base_link", laser_point, base_point);
        ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
           laser_point.point.x, laser_point.point.y, laser_point.point.z,
           base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
      }
      catch(tf::TransformException& ex){
       ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
      }
   }
  
   int main(int argc, char** argv){
       ros::init(argc, argv, "robot_tf_listener"); //注册 创建节点
       ros::NodeHandle nh;                         // 节点句柄
       tf::TransformListener listener(ros::Duration(10)); //监听等待时间
       //we'll transform a point once every second
       ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
      ros::spin();
      
     }