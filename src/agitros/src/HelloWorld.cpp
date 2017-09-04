//ROS Hello,world
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_ros");    //initialize the ROS system
    ros::NodeHandle nh;                    //establish this program/cpp file as a ROS node in node handle
    int count=0;
    ros::Rate rate (1);
    while ( ros::ok() ) { 
    ROS_INFO_STREAM("Hello, ROS!"
                     <<" TIME IS : " << count << " s");
    count+=1;
    rate.sleep() ;
    }
}