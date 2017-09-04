#include "ros/ros.h"
//添加srv/AddTwoInts.srv 文件 后有catkin_make 生成的库文件
#include "agitros/AddTwoInts.h"

//服务器相应 客户端的 回调函数
//       包名      服务数据类型
bool add(agitros::AddTwoInts::Request  &req,
         agitros::AddTwoInts::Response &res)
    {
       res.sum = req.a + req.b;
       ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
       ROS_INFO("sending back response: [%ld]", (long int)res.sum);
      return true;
    }
    
    
int main(int argc, char **argv)
 {
    //初始化ros  和节点
   ros::init(argc, argv, "add_two_ints_server");
   ros::NodeHandle nh;
   //在节点创建服务端                                    服务名称
   ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::Rate rate (2) ;
    while ( ros::ok () ) {
     ros::spinOnce();
     rate.sleep();
    }
    return 0;
 }