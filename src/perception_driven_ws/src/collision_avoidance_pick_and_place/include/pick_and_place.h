/*
类头文件　PickAndPlace
所属命名空间　namespace collision_avoidance_pick_and_place{}
*/
#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>//行动服务头文件
#include <moveit/move_group_interface/move_group.h>//moveit接口头文件
#include <moveit_msgs/PlanningScene.h>//moveit消息
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>// 包自定义的消息
#include <tf/transform_listener.h>//坐标变换监听
#include <tf_conversions/tf_eigen.h>//坐标变换转换 基于eigen
#include <collision_avoidance_pick_and_place/pick_and_place_utilities.h>// 包核心消息头文件
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>// moveit规划监视
#include <moveit/robot_model_loader/robot_model_loader.h>//moveit　机器人模型载入
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>//运动学
#include <geometric_shapes/shape_operations.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;//轨迹　规划组 指针
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;//坐标变换监听

namespace collision_avoidance_pick_and_place
{
	class PickAndPlace
	{
	public:
	// =============================== constructor 类构造函数　==============================
		PickAndPlace(){}
	// =============================== global members 全局成员变量 ==========================
		pick_and_place_config cfg;//类配置类　主要参数
		ros::Publisher marker_publisher;//仿真块发布者
		ros::Publisher planning_scene_publisher;//　规划路径发布者
		ros::ServiceClient target_recognition_client;//　目标识别 客户端
		ros::ServiceClient motion_plan_client;//运动规划　客户端
		GraspActionClientPtr grasp_action_client_ptr;//抓手抓取　释放　action对象　指针　
		MoveGroupPtr move_group_ptr;// moveit运动规划　指针　　MoveGroup对象　访问需要使用　->  
		TransformListenerPtr transform_listener_ptr;//坐标变换监听　指针

	// =============================== Task Functions 任务函数　=============================
		void move_to_wait_position();//　运动到等候区
		void set_gripper(bool do_grasp);// 打开抓手
		// 设置抓住物体
		void set_attached_object(bool attach,
				const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state);
		// 重置环境世界
		void reset_world(bool refresh_octomap = true);
		// 检测箱子的姿态 target_recognition_client.call(srv) 
		geometry_msgs::Pose detect_box_pick();
		//　抓取规划的路径位姿点　容器
		std::vector<geometry_msgs::Pose> create_pick_moves(geometry_msgs::Pose &box_pose);
		std::vector<geometry_msgs::Pose> create_place_moves();
		// 拿起箱子
		void pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose);
		// 放下箱子
		void place_box(std::vector<geometry_msgs::Pose>& place_poses,const geometry_msgs::Pose& box_pose);
		// 执行运动规划
		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
        const moveit_msgs::RobotState &start_robot_state,moveit::planning_interface::MoveGroup::Plan &plan);
		// 更新箱子　marker显示
		void show_box(bool show=true)
		{
			// updating marker action
			cfg.MARKER_MESSAGE.action = show ? visualization_msgs::Marker::ADD : 							    visualization_msgs::Marker::DELETE;
			// publish messages
			marker_publisher.publish(cfg.MARKER_MESSAGE);
		}

	};
}

#endif /* PICK_AND_PLACE_H_ */
