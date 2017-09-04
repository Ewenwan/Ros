// This program spawns a new turtlesim turtle by calling
   // the appropriate service .
   #include <ros/ros.h>
   // The srv class for the service .
   #include <turtlesim/Spawn.h>
   int main( int argc , char ** argv ) {
   ros::init ( argc , argv , "spawn_turtle") ;
   ros::NodeHandle nh ;

   // Create a client object for the spawn service . This
   // needs to know the data type of the service and its  name.
   ros::ServiceClient spawnClient = nh.serviceClient <turtlesim::Spawn>("spawn") ;
   //Ros:: ServiceClient client =node_handle.advertise<service_type>(service_name, true); 可以创建一个持续的服务客户端 不推荐
   // Create the request and response objects.
   turtlesim::Spawn::Request req ;
   turtlesim::Spawn::Response resp ;
   // Fill in the request data members.
   req.x = 2;
   req.y = 3;
   req.theta = M_PI/2;  //90度
   req.name = "my_spa_turtle" ;
   // Actually call the service. This won't return until the service is complete .
   bool success = spawnClient.call( req , resp ) ;
   //调用服务 一旦拥有了一个 ServiceClient、一个完整的 Request 以及 Response,我们就可以调用服务了
   //这个方法实际上完成了定位服务器节点、传输请求数据、等待响应和存储响应数据等一系列工作。
   //这个调用方法返回一个布尔值来表明服务调用是否成功完成。
   //若调用失败通常意味着另一只有着被请求名称的海龟已经存在。
   
   // Check for success and use the response .
   if ( success ) {
   ROS_INFO_STREAM("Spawned a turtle named: "
   << resp.name) ;
   } else {
   ROS_ERROR_STREAM("Failed  to  spawn.") ; //当服务调用失败的时候,可以调用 ROS_ERROR_STREAM 输出错误信息。
   }  
  }