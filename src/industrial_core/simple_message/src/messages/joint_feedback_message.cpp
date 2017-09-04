/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
#include "simple_message/messages/joint_feedback_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_feedback_message.h"
#include "joint_data.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::joint_feedback;

namespace industrial
{
namespace joint_feedback_message
{

JointFeedbackMessage::JointFeedbackMessage(void)
{
  this->init();
}

JointFeedbackMessage::~JointFeedbackMessage(void)
{

}

bool JointFeedbackMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload joint feedback message data");
  }
  return rtn;
}

void JointFeedbackMessage::init(industrial::joint_feedback::JointFeedback & data)
{
  this->init();
  this->data_.copyFrom(data);
}

void JointFeedbackMessage::init()
{
  this->setMessageType(StandardMsgTypes::JOINT_FEEDBACK);
  this->data_.init();
}

bool JointFeedbackMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint feedback message load");
  if (buffer->load(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint feedback message data");
  }
  return rtn;
}

bool JointFeedbackMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint feedback message unload");

  if (buffer->unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint feedback message data");
  }
  return rtn;
}

}
}

