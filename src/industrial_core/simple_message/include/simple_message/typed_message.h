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

#ifndef TYPED_MESSAGE_H
#define TYPED_MESSAGE_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/byte_array.h"
#else
#include "simple_message.h"
#include "byte_array.h"
#endif


namespace industrial
{
namespace typed_message
{

/**
 * \brief Message interface for typed messages built from SimpleMessage.
 *
 * This is an interface for a helper class that when implemented is used
 * to create simple messages of the various types (i.e. as defined by the
 * message type enumeration).  It also has constructors and initializers 
 * that can be used to create a typed message from a simple message.
 *
 * If the typed message does not support a particular simple message type
 * the "to" method should be overridden to return false.  For exmaple, a
 * ping message cannot be a topic, it is always expected to be a request/
 * reply.  A joint trajectory point on the other hand may either be a topic
 * (i.e. asynchronously sent) or a request/reply (i.e. syncrounously sent)
 *
 * Classes that implement this interface shall include data members for
 * the data payload of the typed message.
 *
 * \deprecated The base function implementations in the class will be removed
 * in a later release.  This will force classes that inherit from this
 * class to implement them.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class TypedMessage : public industrial::simple_serialize::SimpleSerialize
{

public:
  /**
   * \brief Initializes message from a simple message
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool init(industrial::simple_message::SimpleMessage & msg)=0;

  /**
   * \brief Initializes a new empty message
   *
   */
  virtual void init()=0;

  /**
   * \brief creates a simple_message request
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toRequest(industrial::simple_message::SimpleMessage & msg)
  {
	  industrial::byte_array::ByteArray data;
	  data.load(*this);
	  return msg.init(this->getMessageType(),
			  industrial::simple_message::CommTypes::SERVICE_REQUEST,
			  industrial::simple_message::ReplyTypes::INVALID, data);
  }

  /**
   * \brief creates a simple_message reply
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toReply(industrial::simple_message::SimpleMessage & msg,
		  industrial::simple_message::ReplyType reply)
  {
	  industrial::byte_array::ByteArray data;
	data.load(*this);
	return msg.init(this->getMessageType(),
			industrial::simple_message::CommTypes::SERVICE_REPLY,
			reply, data);
  }
  /**
   * \brief creates a simple_message topic
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toTopic(industrial::simple_message::SimpleMessage & msg)
  {
	  industrial::byte_array::ByteArray data;
    data.load(*this);
    return msg.init(this->getMessageType(),
    		industrial::simple_message::CommTypes::TOPIC,
    		industrial::simple_message::ReplyTypes::INVALID, data);
  }
  /**
   * \brief gets message type (enumeration)
   *
   * \return message type
   */
  int getMessageType() const
  {
    return message_type_;
  }
  
  /**
   * \brief Gets the communication type of the message
   * 
   * \return the value of the comm_type parameter (refer to simple_message::CommTypes::CommType)
   */
  int getCommType() const
  {
    return comm_type_;
  }

protected:

  /**
   * \brief sets message type
   *
   * \return message type
   */
  void setMessageType(int message_type = industrial::simple_message::StandardMsgTypes::INVALID)
  {
    this->message_type_ = message_type;
  }
  
  /**
   * \brief Sets the communication type of the message
   *
   * \param comm_type: value of the comm_type parameter (refer to simple_message::CommTypes::CommType)
   */
  void setCommType(int comm_type = industrial::simple_message::CommTypes::INVALID)
  {
    this->comm_type_ = comm_type;
  }

private:

  /**
   * \brief Message type expected by callback
   */

  int message_type_;
    
  /**
   * \brief Communications type (see simple_message::CommTypes::CommType)
   */
  int comm_type_;

};

}
}

#endif /* TYPED_MESSAGE_H */
