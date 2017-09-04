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


#ifndef ROBOT_STATUS_RELAY_HANDLER_H
#define ROBOT_STATUS_RELAY_HANDLER_H

#include "ros/ros.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/robot_status_message.h"


namespace industrial_robot_client
{
namespace robot_status_relay_handler
{

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class RobotStatusRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
* \brief Constructor
*/
  RobotStatusRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

protected:

  ros::Publisher pub_robot_status_;
  ros::NodeHandle node_;

  /**
   * \brief Callback executed upon receiving a robot status message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::robot_status_message::RobotStatusMessage & in);

private:
 /**
  * \brief Callback executed upon receiving a message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage& in);
};

}
}


#endif /* ROBOT_STATUS_RELAY_HANDLER_H */
