/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JOINT_TRAJTORY_ACTION_H
#define JOINT_TRAJTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

class JointTrajectoryAction
{

public:
  /**
   * \brief Constructor
   *
   */
  JointTrajectoryAction();

  /**
   * \brief Destructor
   *
   */
  ~JointTrajectoryAction();

  /**
     * \brief Begin processing messages and publishing topics.
     */
    void run() { ros::spin(); }

private:

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;

  /**
   * \brief Internal ROS node handle
   */
  ros::NodeHandle node_;

  /**
   * \brief Internal action server
   */
  JointTractoryActionServer action_server_;

  /**
   * \brief Publishes desired trajectory (typically to the robot driver)
   */
  ros::Publisher pub_trajectory_command_;

  /**
   * \brief Subscribes to trajectory feedback (typically published by the
   * robot driver).
   */
  ros::Subscriber sub_trajectory_state_;

  /**
   * \brief Subscribes to the robot status (typically published by the
   * robot driver).
   */
  ros::Subscriber sub_robot_status_;

  /**
   * \brief Watchdog time used to fail the action request if the robot
   * driver is not responding.
   */
  ros::Timer watchdog_timer_;

  /**
    * \brief Controller was alive during the last watchdog interval
    */
  bool controller_alive_;

  /**
   * \brief Indicates action has an active goal
   */
  bool has_active_goal_;

  /**
   * \brief Indicates that the robot has been in a moving state at least once since
   * starting the current active trajectory
   */
  bool has_moved_once_;

  /**
   * \brief Cache of the current active goal
   */
  JointTractoryActionServer::GoalHandle active_goal_;
  /**
   * \brief Cache of the current active trajectory
   */
  trajectory_msgs::JointTrajectory current_traj_;

  /**
   * \brief The default goal joint threshold see(goal_threshold). Unit
   * are joint specific (i.e. radians or meters).
   */
  static const double DEFAULT_GOAL_THRESHOLD_;// = 0.01;

  /**
   * \brief The goal joint threshold used for determining if a robot
   * is near it final destination.  A single value is used for all joints
   *
   * NOTE: This value is used in conjunction with the robot inMotion
   * status (see industrial_msgs::RobotStatus) if it exists.
   */
  double goal_threshold_;

  /**
   * \brief The joint names associated with the robot the action is
   * interfacing with.  The joint names must be the same as expected
   * by the robot driver.
   */
  std::vector<std::string> joint_names_;

  /**
   * \brief Cache of the last subscribed feedback message
   */
  control_msgs::FollowJointTrajectoryFeedbackConstPtr last_trajectory_state_;

  /**
   * \brief Cache of the last subscribed status message
   */
  industrial_msgs::RobotStatusConstPtr last_robot_status_;

  /**
   * \brief Time at which to start checking for completion of current 
   * goal, if one is active
   */
  ros::Time time_to_check_;

  /**
   * \brief The watchdog period (seconds)
   */
  static const double WATCHDOG_PERIOD_;// = 1.0;

  /**
   * \brief Watch dog callback, used to detect robot driver failures
   *
   * \param e time event information
   *
   */
  void watchdog(const ros::TimerEvent &e);

  /**
   * \brief Action server goal callback method
   *
   * \param gh goal handle
   *
   */
  void goalCB(JointTractoryActionServer::GoalHandle gh);

  /**
   * \brief Action server cancel callback method
   *
   * \param gh goal handle
   *
   */

  void cancelCB(JointTractoryActionServer::GoalHandle gh);
  /**
   * \brief Controller state callback (executed when feedback message
   * received)
   *
   * \param msg joint trajectory feedback message
   *
   */
  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

  /**
   * \brief Controller status callback (executed when robot status
   *  message received)
   *
   * \param msg robot status message
   *
   */
  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);

  /**
   * \brief Aborts the current action goal and sends a stop command
   * (empty message) to the robot driver.
   *
   *
   */
  void abortGoal();

  /**
   * \brief Controller status callback (executed when robot status
   *  message received)
   *
   * \param msg trajectory feedback message
   * \param traj trajectory to test against feedback
   *
   * \return true if all joints are within goal contraints
   *
   */
  bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                             const trajectory_msgs::JointTrajectory & traj);
};

} //joint_trajectory_action
} //industrial_robot_client

#endif /* JOINT_TRAJTORY_ACTION_H */
