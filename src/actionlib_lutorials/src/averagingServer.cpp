#include <ros/ros.h>              //ros系统文件
#include <std_msgs/Float32.h>     //生成随即数 话题的消息类型
#include <actionlib/server/simple_action_server.h>//系统本身的 简单运动服务器库
#include <actionlib_lutorials/AveragingAction.h>  //

//using namespace actionlib_lutorials;//使用  包名定义的actionlib_lutorials名字空间 下面一些类可省去 名字空间  服务器不能用
typedef actionlib::SimpleActionServer<actionlib_lutorials::AveragingAction> Server; //定义运动服务器
//自定义类
class AveragingAction
{
  //受保护的 自定义类内的数据类型
  protected:   
  ros::NodeHandle nh_; ////节点句柄  自定义类变量末尾带下划线 
  Server as_;          //运动服务器
  std::string action_name_;////运动 服务器名字
  int data_count_, goal_;   //
  float sum_, sum_sq_;
  actionlib_lutorials::AveragingFeedback feedback_; ////反馈    goal 由 Client 提供
  actionlib_lutorials::AveragingResult result_;     //结果
  ros::Subscriber sub_;        // 


public:
   //定义运动服务器 函数结构   服务器名字   
  AveragingAction(std::string name) :
  //无 executeCB 函数     boost::bind(&FibonacciAction::executeCB,this,_1)
    as_(nh_, name, false),
    action_name_(name)
  {
    //注册 register the goal and preemp callbacks
    as_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    as_.start();
  }

  ~AveragingAction(void)
  {
  }

  void goalCB()
  {
    // reset helper variables
    data_count_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    // accept the new goal
    goal_ = as_.acceptNewGoal()->samples;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;     //"/random_number" 随机数
    //compute the std_dev and mean of the data 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2))); //标准差
    as_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    } 
  }
};



int main(int argc, char** argv)
{
  //初始化节点
  ros::init(argc, argv, "averagingServer");
 // 服务器函数                     服务器名字
  AveragingAction averaging(ros::this_node::getName());
  ros::spin();

  return 0;
}