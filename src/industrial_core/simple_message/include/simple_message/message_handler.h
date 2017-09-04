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

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/smpl_msg_connection.h"
#else
#include "simple_message.h"
#include "smpl_msg_connection.h"
#endif

namespace industrial
{
namespace message_handler
{

/**
 * \brief Interface definition for message handlers.  The interface defines the
 * callback function that should execute when a message is received.
 */
//* MessageHandler
/**
 * Defines the interface used for function callbacks when a message is received.
 * When used in conjunction with a message_manager (link) generic message handling
 * can be achieved.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class MessageHandler

{
public:

  /**
   * \brief Constructor
   */
  MessageHandler();

  /**
   * \brief Destructor
   */
  ~MessageHandler();

  /**
   * \brief Callback function that should be executed when a message arrives
   * DO NOT OVERRIDE THIS FUNCTION.  It performs message validation before the
   * internal callback (which should be overridden) is called.  If one is required
   * the callback sends a message reply
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool callback(industrial::simple_message::SimpleMessage & in);

  /**
   * \brief Gets message type that callback expects
   *
   * \return message type
   */
  int getMsgType()
  {
    return this->msg_type_;
  }
  ;

protected:
  /**
   * \brief Gets connectoin for message replies
   *
   * \return connection reference
   */
  industrial::smpl_msg_connection::SmplMsgConnection*getConnection()
  {
    return this->connection_;
  }
  ;

  /**
   * \brief Class initializer
   *
   * \param msg_type type of message expected
   * \param connection simple message connection that will be used to send replies.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection);

private:

  /**
   * \brief Reference to reply connection (called if incoming message requires a reply)
   */
  industrial::smpl_msg_connection::SmplMsgConnection* connection_;

  /**
   * \brief Message type expected by callback
   */
  int msg_type_;

  /**
   * \brief Virtual callback function
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  virtual bool internalCB(industrial::simple_message::SimpleMessage & in)=0;

  /**
   * \brief Validates incoming message for processing by internal callback
   *
   * \param in incoming message
   *
   * \return true on if valid, false otherwise
   */
  bool validateMsg(industrial::simple_message::SimpleMessage & in);

  /**
   * \brief Sets connection for message replies
   *
   * \param connection connection reference
   */
  void setConnection(industrial::smpl_msg_connection::SmplMsgConnection* connection)
  {
    this->connection_ = connection;
  }
  ;

  /**
   * \brief Sets message type that callback expects
   *
   * \param msg_type message type
   */
  void setMsgType(int msg_type)
  {
    this->msg_type_ = msg_type;
  }
  ;

};

} //namespace message_handler
} //namespace industrial

#endif //MESSAGE_HANDLER_H
