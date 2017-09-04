/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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
#include "simple_message/messages/robot_status_message.h"
#include "simple_message/robot_status.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "robot_status_message.h"
#include "robot_status.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::robot_status;

namespace industrial
{
namespace robot_status_message
{

RobotStatusMessage::RobotStatusMessage(void)
{
  this->init();
}

RobotStatusMessage::~RobotStatusMessage(void)
{

}

bool RobotStatusMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->status_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload robot status data");
  }
  return rtn;
}

void RobotStatusMessage::init(industrial::robot_status::RobotStatus & status)
{
  this->init();
  this->status_.copyFrom(status);
}

void RobotStatusMessage::init()
{
  this->setMessageType(StandardMsgTypes::STATUS);
  this->status_.init();
}

bool RobotStatusMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot status message load");
  if (buffer->load(this->status_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load robot status data");
  }
  return rtn;
}

bool RobotStatusMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing robot status message unload");

  if (buffer->unload(this->status_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload robot status data");
  }
  return rtn;
}

}
}

