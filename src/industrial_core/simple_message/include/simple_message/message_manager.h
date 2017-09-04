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

#ifndef MESSAGE_MANAGER_H
#define MESSAGE_MANAGER_H

#ifndef FLATHEADERS
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/message_handler.h"
#include "simple_message/ping_handler.h"
#include "simple_message/comms_fault_handler.h"
#include "simple_message/simple_comms_fault_handler.h"
#else
#include "smpl_msg_connection.h"
#include "message_handler.h"
#include "ping_handler.h"
#include "comms_fault_handler.h"
#include "simple_comms_fault_handler.h"
#endif


namespace industrial
{
namespace message_manager
{

/**
 * \brief The message manager handles communications for a simple server.
 */
//* MessageManager
/**
 * The message manager receives simple messages via it's communications connection.
 * It then performs function callbacks based on the message type.  Callbacks perform
 * the desired operation and send a reply, if required.
 *
 * The message manager can be run in two ways.  spin(), similar to ROS this executes
 * a blocking execution indefinitely or spinOnce(), where a single execution of the
 * manager is performed.  In spinOnce mode, other server operations can be made, but
 * the server application must make certain to execute the spinOnce() function at a
 * minimum rate so as not to loose connection data.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MessageManager
{

public:

  /**
   * \brief Constructor
   */
  MessageManager();

  /**
   * \brief Destructor
   */
  ~MessageManager();

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be managed.
   * This connection must be properly initialized and connected before
   * it is passed to the manager.
   *
   * \return true on success, false otherwise
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

  /**
   * \brief Class initializer
   *
   * \param simple message connection that will be managed.
   * This connection must be properly initialized and connected before
   * it is passed to the manager.
   *
   * \param connection fault handler to be used in case of a connection
   * fault.
   *
   * \return true on success, false otherwise
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
            industrial::comms_fault_handler::CommsFaultHandler* fault_handler);

  /**
   * \brief Perform an single execution of the message manager (a single
   * receive and send (if required)
   */
  void spinOnce();

  /**
   * \brief Perform a indefinite execution of the message manager
   */
  void spin();

  /**
   * \brief Adds a message handler to the manager
   *
   * \param handler handler to add
   * \param replace existing handler (of same msg-type), if exists
   *
   * \return true if successful, otherwise false (max # of handlers reached)
   */
  bool add(industrial::message_handler::MessageHandler* handler, bool allow_replace = false);

  /**
   * \brief Gets number of handlers
   *
   * \return connection reference
   */
  unsigned int getNumHandlers()
  {
    return this->num_handlers_;
  }

  /**
   * \brief Gets maximumn number of handlers
   *
   * \return connection reference
   */
  unsigned int getMaxNumHandlers()
  {
    return this->MAX_NUM_HANDLERS;
  }


  /**
   * \brief Gets communications fault handler
   *
   * \return reference to message handler or NULL if one doesn't exist
   */
  industrial::comms_fault_handler::CommsFaultHandler* getCommsFaultHandler()
  {
    return this->comms_hndlr_;
  }

  /**
   * \brief Gets communications fault handler
   *
   * \param Pointer to message handler
   */
  void setCommsFaultHandler(industrial::comms_fault_handler::CommsFaultHandler* handler)
  {
    this->comms_hndlr_ = handler;
  }


private:

  /**
   * \brief Maximum number of handlers
   *
   * The number of handlers is limited in order to avoid dynamic memory allocation
   * on robot controllers.
   */
  static const unsigned int MAX_NUM_HANDLERS = 64;

  /**
   * \brief buffer of handlers
   */
  industrial::message_handler::MessageHandler* handlers_[MAX_NUM_HANDLERS];

  /**
   * \brief Reference to reply connection (called if incoming message requires a reply)
   */
  industrial::smpl_msg_connection::SmplMsgConnection* connection_;

  /**
   * \brief Internal ping handle (by default every message manager can handle pings)
   */
  industrial::ping_handler::PingHandler ping_hndlr_;

  /**
   * \brief Internal default comms handler (this is used if a communications fault handler is
   * not specified as part of the class init.
   */
  industrial::simple_comms_fault_handler::SimpleCommsFaultHandler def_comms_hndlr_;

  /**
   * \brief Reference to comms handler
   */
  industrial::comms_fault_handler::CommsFaultHandler* comms_hndlr_;

  /**
   * \brief Number of handlers
   */
  unsigned int num_handlers_;

  /**
   * \brief Gets message handler for specific message type
   *
   * \param msg_type message type to handle
   *
   * \return reference to message handler or NULL if one doesn't exist
   */
  industrial::message_handler::MessageHandler* getHandler(int msg_type);

  /**
   * \brief Gets index of message handler for specific message type
   *
   * \param message type to handle
   *
   * \return index of matching handler or -1 if one doesn't exist
   */
  int getHandlerIdx(int msg_type);

  /**
   * \brief Gets default communications fault handler
   *
   * \return reference to message handler or NULL if one doesn't exist
   */
  industrial::simple_comms_fault_handler::SimpleCommsFaultHandler& getDefaultCommsFaultHandler()
  {
    return this->def_comms_hndlr_;
  }
  /**
   * \brief Gets ping handler
   *
   * \return connection reference
   */
  industrial::ping_handler::PingHandler& getPingHandler()
  {
    return this->ping_hndlr_;
  }
  ;

  /**
   * \brief Sets connection manager
   *
   * \param connection connection reference
   */
  void setConnection(industrial::smpl_msg_connection::SmplMsgConnection* connection)
  {
    this->connection_ = connection;
  }
  ;

  /**
   * \brief Gets connection for manager
   *
   * \return connection reference
   */
  industrial::smpl_msg_connection::SmplMsgConnection* getConnection()
  {
    return this->connection_;
  }
  ;

  /**
   * \brief Sets message type that callback expects
   *
   * \param msg_type message type
   */
  void setNumHandlers(unsigned int num_handlers)
  {
    this->num_handlers_ = num_handlers;
  }
  ;

};

} // namespace industrial
} // namespace message_manager

#endif //MESSAGE_MANAGER
