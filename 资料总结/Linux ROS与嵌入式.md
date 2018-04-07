# 1.根据ros wiki的官方教程 学习即可，并且有许多例子可供学习

 [ros_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials)
      
# 2. Linux ROS与嵌入式的串口通信  ros 串口通信
# 3.用ASIO读写设备串行口
     ASIO不仅支持网络通信还能支持串口通信。如何让两个设备使用串口通讯，最重要的是设置好正确的参数，
     那么串口的参数就包括：
     波特率、
     奇偶校验位、
     停止位、
     字符大小和
     流量控制，
     两个设备只有设置相同的参数才能相互间进行数据交换。
     
     ASIO提供了boost::asio::serial_port类， 
     它有一个
     set_option(const SettableSerialPortOption& option)方法就是用于设置上面列举的这些参数的，
     其中的option 可以是：
     
     serial_port::baud_rate       波特率，  构造参数为unsigned int
     serial_port::parity          奇偶校验，构造参数为serial_port::parity::type，enum类型，可以是none, odd, even。
     serial_port::flow_control    流量控制，构造参数为serial_port::flow_control::type，enum类型，可以是none software hardware
     serial_port::stop_bits       停止位，  构造参数为serial_port::stop_bits::type，enum类型，可以是one onepointfive two
     serial_port::character_size  字符大小，构造参数为unsigned int
     
     向串口发送数据时，采用boost::asio:serial_port下含有write字样的函数将数据写入串口，
     接受串口数据时，  用read函数从串口读取数据，
     那么比如用串口对象调用write_some(),read_some(),
     这类函数属于serial_port的成员函数，
     还有在函数内部指明串口对象的write()，read()函数，
     这些函数是不属于serial_port类的成员函数，但他们是boost::asio成员的函数，
     一般情况下我们都会用指明串口的函数比如：

     write(sp, buffer("Hello world", 12));
     第一个参数SP表示serial_port对象，
     第二个参数是写向串口的数据，
     第三个参数数据长度，
     但如果write函数在传输数据错误时会自动抛出异常boost::sysytem::error_code 
     那么这句话的意思就是向串口的写入长度为12 的字符串“Helloworld”

     char buf[12]            这句话的额意思很明显是声明数据的格式及长度;
     read(sp, buffer(buf));  本句话的意思就是从串口读取数据，这里在读取数据一般是我们知道发送数据的长度，
     就是说这句话的是从串口读取12个字符（读满才返回）才会返回，
     
     那么用read读取数据流必须读满内存变量后才会返回，然而返回的有时候是乱码，就会阻塞后面程序的执行，
     此时可以采用异步读取/接受串口的方式，就算没有完全读取/接受串口数据，
     异步读取函数依旧会马上返回执行后面的代码，等串口数据读取完毕或者发生异常时，
     
     io_service::run() 函数会等待异步读取串口的数据操作，然后调用异步函数制定的回调函数。 
     
     那么一步读写操作包含三个部分：异步操作函数，异步函数以形参的方式指定到回调函数，
     io_service::run() 函数的调用，这三个部分的执行流程是：当程序执行到异步操作函数时，异步操作立即返回，
     程序继续执行后续的代码，异步操作函数功能完成或者异常时，
     io_service::run()函数会自动的调用异步操作函数指定到回调函数。
     
## 示例程序
     #include <string>
     #include <ros/ros.h>                     // 包含ROS的头文件
     #include <sensor_msgs/JointState.h>
     #include <tf/transform_broadcaster.h>
     #include <nav_msgs/Odometry.h>
     #include <boost/asio.hpp>                //包含boost库函数
     #include <boost/bind.hpp>
     #include <math.h>
     #include "std_msgs/String.h"             //ros定义的String数据类型

     using namespace std;
     using namespace boost::asio;             //定义一个命名空间，用于后面的读写操作

     unsigned char buf[24];                   //定义字符串长度

     int main(int argc, char** argv) {

        ros::init(argc, argv, "boost");      //初始化节点
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); //定义发布消息的名称及sulv
        ros::Rate loop_rate(10);             //执行频率

        io_service iosev;                    // io异步串口读写 服务
        serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
        sp.set_option(serial_port::baud_rate(9600));   
        sp.set_option(serial_port::flow_control());
        sp.set_option(serial_port::parity());
        sp.set_option(serial_port::stop_bits());
        sp.set_option(serial_port::character_size(8));

        while (ros::ok()) {
          // write(sp, buffer(buf1, 6));  //write the speed for cmd_val    
          //write(sp, buffer("Hello world", 12));  
          read (sp,buffer(buf));
          string str(&buf[0],&buf[22]);   //将数组转化为字符串
          //if(buf[0]=='p' && buf[21] == 'a')
          // {
            std_msgs::String msg;
            std::stringstream ss;
            ss << str;
            msg.data = ss.str();

          ROS_INFO("%s", msg.data.c_str());//打印接受到的字符串
          chatter_pub.publish(msg);   //发布消息
          ros::spinOnce();
          loop_rate.sleep();
          //  }
         }
         iosev.run(); // io异步串口读写 服务
         return 0;
     }
     
     
     
