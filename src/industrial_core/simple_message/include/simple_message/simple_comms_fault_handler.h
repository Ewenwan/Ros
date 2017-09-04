/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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

#ifndef DEFAULT_COMMS_FAULT_HANDLER_H
#define DEFAULT_COMMS_FAULT_HANDLER_H

#ifndef FLATHEADERS
#include "simple_message/comms_fault_handler.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/log_wrapper.h"
#else
#include "comms_fault_handler.h"
#include "smpl_msg_connection.h"
#include "log_wrapper.h"
#endif

namespace industrial
{
namespace simple_comms_fault_handler
{

/**
 * \brief Default implementation of comms fault handler.  This class attempts
 * to reconnect if the connection is lost.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class SimpleCommsFaultHandler : public industrial::comms_fault_handler::CommsFaultHandler

{
public:

  /**
      * \brief Default constructor
      *
      */
  SimpleCommsFaultHandler();

  /**
       * \brief Destructor
       *
       */
  ~SimpleCommsFaultHandler();
  /**
    * \brief Initializes default communications fault handler
    *
    * \param message connection to use for reconnecting
    *
    * \return true on success, false otherwise
    */
   bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);


   /**
      * \brief Send failure callback method: Nothing is performed
      *
      */
     void sendFailCB() {LOG_WARN("Send failure, no callback support");};

     /**
      * \brief Receive failure callback method: Nothing is performed
      *
      */
     void receiveFailCB() {LOG_WARN("Receive failure, no callback support");};

     /**
      * \brief Connection failure callback method: On a connection failure
      * a blocking reconnection is attempted.
      *
      */
     void connectionFailCB();

private:


/**
 * \brief Reference to reply connection (called if incoming message requires a reply)
 */
industrial::smpl_msg_connection::SmplMsgConnection* connection_;

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
};

} //namespace default_comms_fault_handler
} //namespace industrial

#endif /* DEFAULT_COMMS_FAULT_HANDLER_H */
