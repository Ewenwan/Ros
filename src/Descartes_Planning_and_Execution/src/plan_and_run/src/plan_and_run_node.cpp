/*
 * plan_and_run.cpp :  主应用节点  
 *
 *  Created on: Apr 10, 2015
 *      Author: Jorge Nicho
 */

#ifdef __i386__
  #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
  #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif

#include <plan_and_run/demo_application.h>

int main(int argc,char** argv)
{
  // 初始化ros系统 节点 plan_and_run
  ros::init(argc,argv,"plan_and_run");

  ros::AsyncSpinner spinner(2);//开启两个线程
  spinner.start();

  // creating application plan_and_run命令空间下 创建类
  plan_and_run::DemoApplication application;

  // loading parameters 载入参数
  application.loadParameters();

  // initializing ros components 初始化 ros系统 创建一个服客户端节点 来发送规划的路径点
  // 服务的提供节点 为 ros系统 /move_group 节点提供服务
  application.initRos();

  // initializing descartes    初始化 笛卡尔规划
  application.initDescartes();

  // moving to home position 移动到等候区（家位置）
  application.moveHome();

  // generating trajectory  产生笛卡尔规划轨迹 引用形参传回
// typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;//笛卡尔轨迹点 指针 数组
  plan_and_run::DescartesTrajectory traj;
  application.generateTrajectory(traj);


  // planning robot path   规划机器人路径
  plan_and_run::DescartesTrajectory output_path;
  application.planPath(traj,output_path);

  // running robot path  执行路径
  application.runPath(output_path);

  // exiting ros node 退出线程
  spinner.stop();



  return 0;
}

