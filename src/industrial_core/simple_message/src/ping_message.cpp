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
#include "simple_message/ping_message.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/byte_array.h"
#else
#include "ping_message.h"
#include "log_wrapper.h"
#include "byte_array.h"
#endif

using namespace industrial::simple_message;
using namespace industrial::byte_array;

namespace industrial
{
namespace ping_message
{


PingMessage::PingMessage(void)
{
  this->init();
}

PingMessage::~PingMessage(void)
{

}


bool PingMessage::init(SimpleMessage & msg)
{
  bool rtn = false;

  if (this->getMessageType() == msg.getMessageType())
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to initialize message, wrong type: %d, expected %d",
              msg.getMessageType(), this->getMessageType());
    rtn = false;
  }

  return rtn;
}


void PingMessage::init()
{
  this->setMessageType(StandardMsgTypes::PING);
}


}// ping_message
}// industrial




