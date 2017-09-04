   // This program subscribes to turtle1/cmd_vel and
   // republishes on turtle1/cmd_vel_reversed ,
   // with the signs inverted .
   #include <ros/ros.h>
   #include <geometry_msgs/Twist.h> //消息类型头文
   ros::Publisher *pubPtr;
   
  //订阅turtle1/cmd_vel话题的回调函数
   void commandVelocityReceived (const geometry_msgs::Twist& msgIn ) {
   geometry_msgs::Twist msgOut;
   msgOut.linear.x = -msgIn.linear.x;
   msgOut.angular.z = -msgIn.angular.z;
   pubPtr->publish(msgOut);
   }
   
   int main( int argc , char** argv ) {
   ros::init ( argc , argv , "reverse_velocity");
   ros::NodeHandle nh;
   
     //把反转的消息发布到turtle1/cmd_vel_reversed 新话题上
   pubPtr = new ros::Publisher ( nh.advertise <geometry_msgs::Twist>("turtle1/cmd_vel_reversed",1000) ) ;
    //订阅turtle1/cmd_vel话题上的消息（键盘控制发来的）
   ros::Subscriber sub = nh.subscribe ("turtle1/cmd_vel" , 1000 ,&commandVelocityReceived ) ;
 
   ros::spin () ;
 
   delete pubPtr ;
  }