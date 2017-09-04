/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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


#ifndef JOINT_HANDLER_H
#define JOINT_HANDLER_H

#include <string>
#include <vector>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/joint_message.h"


namespace industrial_robot_client
{
namespace joint_relay_handler
{

using industrial::joint_message::JointMessage;
using industrial::simple_message::SimpleMessage;

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
* \brief Constructor
*/
  JointRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  * \param joint_names list of joint-names for msg-publishing.
  *   - Count and order should match data from robot connection.
  *   - Use blank-name to exclude a joint from publishing.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string> &joint_names);

protected:

  std::vector<std::string> all_joint_names_;

  ros::Publisher pub_joint_control_state_;
  ros::Publisher pub_joint_sensor_state_;
  ros::NodeHandle node_;

  /**
   * \brief Convert joint message into publish message-types
   *
   * \param[in] msg_in Joint message from robot connection
   * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * \param[out] sensor_state JointState message for ROS publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(JointMessage& msg_in,
                               control_msgs::FollowJointTrajectoryFeedback* control_state,
                               sensor_msgs::JointState* sensor_state);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pos_in joint positions, exactly as passed from robot connection.
   * \param[out] pos_out transformed joint positions (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    *pos_out = pos_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select specific joints for publishing
   *
   * \param[in] all_joint_pos joint positions, in count/order matching robot connection
   * \param[in] all_joint_names joint names, matching all_joint_pos
   * \param[out] pub_joint_pos joint positions selected for publishing
   * \param[out] pub_joint_names joint names selected for publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<double>& all_joint_pos, const std::vector<std::string>& all_joint_names,
                      std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names);

  /**
   * \brief Callback executed upon receiving a joint message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(JointMessage& in);

private:
 /**
  * \brief Callback executed upon receiving a message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(SimpleMessage& in);
};//class JointRelayHandler

}//joint_relay_handler
}//industrial_robot_cliet


#endif /* JOINT_HANDLER_H */
