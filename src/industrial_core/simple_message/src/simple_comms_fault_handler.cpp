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

#ifndef FLATHEADERS
#include "simple_message/simple_comms_fault_handler.h"
#include "simple_message/log_wrapper.h"
#else
#include "simple_comms_fault_handler.h"
#include "log_wrapper.h"
#endif

namespace industrial
{
namespace simple_comms_fault_handler
{

SimpleCommsFaultHandler::SimpleCommsFaultHandler()
{
  this->connection_ = NULL;
}


SimpleCommsFaultHandler::~SimpleCommsFaultHandler()
{
}

bool SimpleCommsFaultHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  bool rtn = false;

  if (NULL != connection)
  {
    this->setConnection(connection);
    LOG_INFO("Default communications fault handler successfully initialized");
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to initialize default communications fault handler");
    rtn = false;
  }
  return rtn;
}

void SimpleCommsFaultHandler::connectionFailCB()
{

  if (!(this->getConnection()->isConnected()))
  {
    LOG_INFO("Connection failed, attempting reconnect");
    this->getConnection()->makeConnect();
  }
  else
  {
    LOG_WARN("Connection fail callback called while still connected (Possible bug)");
  }
}



}//namespace default_comms_fault_handler
}//namespace industrial




