#include "ros/ros.h"
//添加srv/AddTwoInts.srv 文件 后有catkin_make 生成的库文件
//           包      
#include "agitros/AddTwoInts.h"
#include <cstdlib>
 int main(int argc, char **argv)
    {
   ros::init(argc, argv, "add_two_ints_client");
   if (argc != 3)
    {
     ROS_INFO("usage: add_two_ints_client X Y");
     return 1; }
    ros::NodeHandle nh;
    //        客户端                                                     服务名称
    ros::ServiceClient client = nh.serviceClient<agitros::AddTwoInts>("add_two_ints");
    agitros::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv)) //等待服务器相应
      {   ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      }
    else
      {
       ROS_ERROR("Failed to call service add_two_ints");
       return 1;
      } 
    return 0;
   }