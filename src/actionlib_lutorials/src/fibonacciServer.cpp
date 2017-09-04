#include <ros/ros.h>                               //ros系统文件
#include <actionlib/server/simple_action_server.h>//系统本身的 简单运动服务器库
#include <actionlib_lutorials/FibonacciAction.h>   //添加actionlib_lutorials/FibonacciAction.h 文件 catkin_make 生成的库文件

//using namespace actionlib_lutorials;  //使用  包名定义的actionlib_lutorials名字空间 下面一些类可省去 名字空间
//使用名字 空间 可能会造成 类调用不明确（来源不明确）
//actionlib::SimpleActionServer 来自<actionlib/server/simple_action_server.h>
//actionlib_lutorials::FibonacciAction 来自<actionlib_lutorials/FibonacciAction.h>
typedef actionlib::SimpleActionServer<actionlib_lutorials::FibonacciAction> Server; //定义运动服务器

//自定义类
class FibonacciAction{
  //受保护的 自定义类型
protected:
  ros::NodeHandle nh_;  //节点句柄  自定义类变量末尾带下划线 而调用时则不用
  Server as_;           //运动服务器
  std::string action_name_;//运动 服务器名字
 actionlib_lutorials::FibonacciFeedback feedback_; //反馈  goal 由 Client 提供
 actionlib_lutorials::FibonacciResult result_;     //结果

public:
  //定义运动服务器 函数结构   服务器名字
  FibonacciAction(std::string name):
    //服务器 节点 名字                  可执行函数
     as_(nh_,name,boost::bind(&FibonacciAction::executeCB,this,_1),false),
     action_name_(name)
      {
        as_.start();
      }
   //私有函数
   ~FibonacciAction(void)
    {}
  //可执行函数                                 目标
  void executeCB(const actionlib_lutorials::FibonacciGoalConstPtr &goal){
    ros::Rate rate(1);   //频率
   bool success=true; //标志
   feedback_.sequence.clear(); //反馈序列初始化 清零
   feedback_.sequence.push_back(0); //反馈序列初始化 加入种子点
   feedback_.sequence.push_back(1);
   ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
   //生成序列
   for(int i=0;i<=goal->order;i++){
    //确保Preempt没被 客户端请求
     if(as_.isPreemptRequested()||!ros::ok()){
       ROS_INFO("%s: Preempted", action_name_.c_str());
       as_.setPreempted();
       success=false;
       break;
     }
     //
     feedback_.sequence.push_back(feedback_.sequence[i]+feedback_.sequence[i-1]);
     as_.publishFeedback(feedback_);

     rate.sleep();
   }
   
   if(success){
     ROS_INFO("%s: Succeeded", action_name_.c_str());
     result_.sequence=feedback_.sequence;
     //同志客户端 生成 序列成功
     as_.setSucceeded(result_);
   }

  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");//初始化节点
  // 服务器函数                     服务器名字
  FibonacciAction fibonacci(ros::this_node::getName()); //若 using namespace actionlib_lutorials; 可能会造成调用不明确
  ros::spin();
  return 0;
}
