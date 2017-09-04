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

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/log_wrapper.h"
#else
#include "simple_message.h"
#include "log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoPlus.h"
#endif


using namespace industrial::byte_array;

namespace industrial
{

namespace simple_message
{

SimpleMessage::SimpleMessage(void)
{
}

SimpleMessage::~SimpleMessage(void)
{
}



bool SimpleMessage::init(int msgType, int commType, int replyCode)
{
  ByteArray data;
  data.init();
  return this->init(msgType, commType, replyCode, data);
}

bool SimpleMessage::init(int msgType, int commType, int replyCode, ByteArray & data )
{
  LOG_COMM("SimpleMessage::init(type: %d, comm: %d, reply: %d, data[%d]...)",
            msgType, commType, replyCode, data.getBufferSize());
  this->setMessageType(msgType);
  this->setCommType(commType);
  this->setReplyCode(replyCode);
  this->data_.copyFrom(data);

  return this->validateMessage();
}

bool SimpleMessage::init(ByteArray & msg)
{
  int dataSize = 0;
  bool rtn = false;

  if (msg.getBufferSize() >= this->getHeaderSize())
  {
    // Check to see if the message is larger than the standard header
    // If so, copy out the data portion.
    if (msg.getBufferSize() > this->getHeaderSize())
    {
      dataSize = msg.getBufferSize() - this->getHeaderSize();
      LOG_COMM("Unloading data portion of message: %d bytes", dataSize);
      msg.unload(this->data_, dataSize);
    }
    LOG_COMM("Unloading header data");
    msg.unload(this->reply_code_);
    msg.unload(this->comm_type_);
    msg.unload(this->message_type_);
    LOG_COMM("SimpleMessage::init(type: %d, comm: %d, reply: %d, data[%d]...)",
              this->message_type_, this->comm_type_, this->reply_code_, this->data_.getBufferSize());
    rtn = this->validateMessage();
  }
  else
  {
    LOG_ERROR("Failed to init message, buffer size too small: %u", msg.getBufferSize());
    rtn = false;
  }
  return rtn;
}

void SimpleMessage::toByteArray(ByteArray & msg)
{
  msg.init();

  msg.load(this->getMessageType());
  msg.load(this->getCommType());
  msg.load(this->getReplyCode());
  if (this->data_.getBufferSize() > 0 )
  {
    msg.load(this->data_);
  }

}


void SimpleMessage::setData( ByteArray & data)
{
  this->data_.copyFrom(data);
}


bool SimpleMessage::validateMessage()
{

  if ( StandardMsgTypes::INVALID == this->getMessageType())
  {
    LOG_WARN("Invalid message type: %u", this->getMessageType());
    return false;
  }

  if ( CommTypes::INVALID == this->getCommType())
  {
    LOG_WARN("Invalid comms. type: %u", this->getCommType());
    return false;
  }

  if (
      (CommTypes::SERVICE_REPLY ==  this->getCommType() &&
          ReplyTypes::INVALID == this->getReplyCode()) ||
          ((CommTypes::SERVICE_REPLY !=  this->getCommType() &&
              ReplyTypes::INVALID != this->getReplyCode()))
  )
  {
    LOG_WARN("Invalid reply. Comm type: %u, Reply type: %u",
             this->getCommType(), this->getReplyCode());
    return false;
  }

  return true;
}



	
} // namespace simple_message
} // namespace industrial
