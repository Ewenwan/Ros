#include <ros/ros.h>          //ros系统文件
#include <turtlesim/Pose.h>   // "/turtle1/pose"话题消息类型 turtlesim/Pose
#include <actionlib/server/simple_action_server.h>//系统本身的 简单运动服务器库
#include <cmath>
#include <math.h>
#include <angles/angles.h>

#include <geometry_msgs/Twist.h> ///turtle1/cmd_vel话题消息类型 geometry_msgs/Twist
#include <actionlib_lutorials/ShapeAction.h>      //catkin_make 生成的库文件

typedef actionlib::SimpleActionServer<actionlib_lutorials::ShapeAction> Server; //定义运动服务器
// This class computes the command_velocities of the turtle to draw regular polygons 
class ShapeAction
{
  //受保护的 自定义类内的数据类型
  protected:
  ros::NodeHandle nh_; ////节点句柄  自定义类变量末尾带下划线
  Server as_;          //运动服务器
  std::string action_name_;  ////运动 服务器名字
  double radius_, apothem_, interior_angle_, side_len_; //相关变量
  double start_x_, start_y_, start_theta_;
  double dis_error_, theta_error_;
  int edges_ , edge_progress_;
  bool start_edge_;
  geometry_msgs::Twist command_;        //控制小乌龟速度的命令
  actionlib_lutorials::ShapeResult result_;////结果
  ros::Subscriber sub_;                 //订阅者  /turtle1/pose 位置
  ros::Publisher pub_;                  //发布者  /turtle1/cmd_vel 速度
   
public:
  //定义运动服务器 函数结构   服务器名字  
  ShapeAction(std::string name) :
   //无 executeCB 函数     boost::bind(&FibonacciAction::executeCB,this,_1)
    as_(nh_, name, false),
    action_name_(name)
  {
    // 注册 register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ShapeAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ShapeAction::preemptCB, this));

    //subscribe to the data topic of interest
    //订阅位置
    sub_ = nh_.subscribe("/turtle1/pose", 1, &ShapeAction::controlCB, this);
    //发布 速度
    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    as_.start();
  }

  ~ShapeAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    actionlib_lutorials::ShapeGoal goal = *as_.acceptNewGoal();
    //save the goal as private variables
    edges_ = goal.edges;
    radius_ = goal.radius;

    // reset helper variables
    interior_angle_ = ((edges_-2)*M_PI)/edges_;
    apothem_ = radius_*cos(M_PI/edges_);
    //compute the side length of the polygon
    side_len_ = apothem_ * 2* tan( M_PI/edges_);
    //store the result values
    result_.apothem = apothem_;
    result_.interior_angle = interior_angle_;
    edge_progress_ =0;
    start_edge_ = true;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void controlCB(const turtlesim::Pose::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
  
    if (edge_progress_ < edges_)
    {
      // scalar values for drive the turtle faster and straighter
      double l_scale = 6.0;
      double a_scale = 6.0;
      double error_tol = 0.00001;

      if (start_edge_)
      {
        start_x_ = msg->x;
        start_y_ = msg->y;
        start_theta_ = msg->theta;
        start_edge_ = false;
      }

      // compute the distance and theta error for the shape
      dis_error_ = side_len_ - fabs(sqrt((start_x_- msg->x)*(start_x_-msg->x) + (start_y_-msg->y)*(start_y_-msg->y)));
      theta_error_ = angles::normalize_angle_positive(M_PI - interior_angle_ - (msg->theta - start_theta_));
     
      if (dis_error_ > error_tol)
      {
        command_.linear.x = l_scale*dis_error_;
        command_.angular.z = 0;
      }
      else if (dis_error_ < error_tol && fabs(theta_error_)> error_tol)
      { 
        command_.linear.x = 0;
        command_.angular.z = a_scale*theta_error_;
      }
      else if (dis_error_ < error_tol && fabs(theta_error_)< error_tol)
      {
        command_.linear.x = 0;
        command_.angular.z = 0;
        start_edge_ = true;
        edge_progress_++;
      }  
      else
      {
        command_.linear.x = l_scale*dis_error_;
        command_.angular.z = a_scale*theta_error_;
      } 
      // publish the velocity command
      pub_.publish(command_);
      
    } 
    else
    {          
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }   
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_shape");

  ShapeAction shape(ros::this_node::getName());
  ros::spin();

  return 0;
}