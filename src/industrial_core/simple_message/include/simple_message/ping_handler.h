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


#ifndef PING_HANDLER_H
#define PING_HANDLER_H

#ifndef FLATHEADERS
#include "simple_message/message_handler.h"
#else
#include "message_handler.h"
#endif


namespace industrial
{
namespace ping_handler
{

/**
 * \brief Message handler that handles ping messages.
 */
//* MessageHandler
/**
 * Responds to ping message types.  A ping is a simple message that is meant to
 * test communications channels.  A ping simply responds with a copy of the data
 * it was sent.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class PingHandler : public industrial::message_handler::MessageHandler
{

public:
  /**
* \brief Class initializer
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

  /**
* \brief Class initializer (Direct call to base class with the same name)
* I couldn't get the "using" form to work/
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection)
{ return MessageHandler::init(msg_type, connection);};


private:



 /**
  * \brief Callback executed upon receiving a ping message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage & in);
};

}//ping_handler
}//industrial


#endif /* PING_HANDLER_H_ */
