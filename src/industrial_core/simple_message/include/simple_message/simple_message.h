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

#ifndef SIMPLE_MSG_H
#define SIMPLE_MSG_H

#ifndef FLATHEADERS
#include "simple_message/simple_serialize.h"
#include "simple_message/byte_array.h"
#include "simple_message/shared_types.h"
#else
#include "simple_serialize.h"
#include "byte_array.h"
#include "shared_types.h"
#endif


namespace industrial
{

namespace simple_message
{

/**
 * \brief Enumeration of standard message types (supported by all platforms).
 * In addition, each robot interface will support it's own message types.
 */
namespace StandardMsgTypes
{
  enum StandardMsgType
  {
 INVALID = 0,
 PING = 1,

 //TODO: Keeping these message type for the time being.  Refactoring
 // the messages should remove the need for this message.
 JOINT_POSITION = 10,
 JOINT = 10, 
 READ_INPUT = 20,
 WRITE_OUTPUT = 21,

 JOINT_TRAJ_PT = 11,  //Joint trajectory point message (typically for streaming)
 JOINT_TRAJ = 12,	  //Joint trajectory message (typically for trajectory downloading)
 STATUS = 13,         //Robot status message (for reporting the robot state)
 JOINT_TRAJ_PT_FULL = 14,  // Joint trajectory point message (all message fields)
 JOINT_FEEDBACK = 15,      // Feedback of joint pos/vel/accel

 // Begin vendor specific message types (only define the beginning enum value,
 // specific enum values should be defined locally, within in the range reserved
 // here.  Each vendor can reserve up 100 types

 SWRI_MSG_BEGIN     = 1000,
 UR_MSG_BEGIN       = 1100,
 ADEPT_MSG_BEGIN    = 1200,
 ABB_MSG_BEGIN      = 1300,
 FANUC_MSG_BEGIN    = 1400,
 MOTOMAN_MSG_BEGIN  = 2000
  };
}
typedef StandardMsgTypes::StandardMsgType StandardMsgType;

/**
 * \brief Enumeration of communications types (supported by all platforms).
 */
namespace CommTypes
{
  enum CommType
  {
 INVALID = 0,
 TOPIC = 1,
 SERVICE_REQUEST = 2,
 SERVICE_REPLY = 3
  };
}
typedef CommTypes::CommType CommType;

/**
 * \brief Enumeration of reply types (supported by all platforms).  In cases of
 * success or failure, the return data should include the relevant return info.
 */
namespace ReplyTypes
{
  enum ReplyType
  {
 INVALID = 0,
 SUCCESS = 1,
 FAILURE = 2
  };
}
typedef ReplyTypes::ReplyType ReplyType;



/**
* \brief This class defines a simple messaging protocol for communicating with an
* industrial robot controller.  
*
* The protocol meets the following requirements:
*
* 1. Format should be simple enough that code can be shared between ROS and
* the controller (for those controllers that support C/C++).  For those controllers
* that do not support C/C++, the protocol must be simple enough to be decoded with
* the limited capabilities of the typical robot programming language.  A corollary
* to this requirement is that the protocol should not be so onerous as to overwhelm
* the limited resources of the robot controller
*
* 2. Format should allow for data streaming (ROS topic like)
*
* 3. Format should allow for data reply (ROS service like)
*
* 4. The protocol is not intended to encapsulate version information  It is up to
* individual developers to ensure that code developed for communicating platforms
* does not have any version conflicts (this includes message type identifiers).
*
* Message Structure
*
* - <PREFIX> Not considered part of the message
*   - int LENGTH (HEADER + DATA) in bytes
*
*
*
* - <HEADER>
*   - int MSG_TYPE identifies type of message (standard (see StandardMsgTypes::StandardMsgType)
*     and robot specific values)
*   - int COMM_TYPE identified communications type (see CommTypes::CommType)
*   - int REPLY CODE (service reply only) reply code (see ReplyTypes::ReplyType)
* 
* - <BODY>
*   - ByteArray DATA variable length data determined by message
*     type and and communications type.
*
*
* THIS CLASS IS NOT THREAD-SAFE
*
*/
class SimpleMessage
{

  
public:
  /**
   * \brief Constructs an empty message
   */
	SimpleMessage();

  /**
   * \brief Destructs a message
   */
	~SimpleMessage(void);
  /**
   * \brief Initializes a fully populated simple message
   *
   * \param message type. Globally unique message ID (see StandardMsgType)
   * \param communications types (see CommType)
   * \param reply code(see ReplyType), only valide if comms type is a reply
   * \param data payload for the message
   *
   * \return true if valid message created
   */
	bool init(int msgType, int commType, int replyCode,
	          industrial::byte_array::ByteArray &data );
            
  /**
   * \brief Initializes a simple message with an emtpy data payload
   *
   * \param message type. Globally unique message ID (see StandardMsgType)
   * \param communications types (see CommType)
   * \param reply code(see ReplyType), only valide if comms type is a reply
   *
   * \return true if valid message created
   */
  bool init(int msgType, int commType, int replyCode);
  
  /**
   * \brief Initializes a simple message from a generic byte array.  The byte
   * array is assumed to hold a valid message with a HEADER and data payload
   *
   * \param valid message (as bytes)
   *
   * \return true if valid message created
   */
  bool init(industrial::byte_array::ByteArray & msg);

   /**
   * \brief Populates a raw byte array with the message.  Any data stored in
   * the passed in byte array is deleted
   * 
   * \param byte array to be populated
   */
  void toByteArray(industrial::byte_array::ByteArray & msg);

  /**
   * \brief Gets size of message header in bytes(fixed)
   *
   * \return message header size
   */
  static unsigned int getHeaderSize() { return SimpleMessage::HEADER_SIZE; };
  
   /**
   * \brief Gets size of message length member in bytes (fixed)
   *
   * \return message header size
   */
  static unsigned int getLengthSize() { return SimpleMessage::LENGTH_SIZE; };

  /**
   * \brief Gets message type(see StandardMsgType)
   *
   * \return message type
   */
	int getMessageType() {return this->message_type_;};
  
  /**
   * \brief Gets message type(see CommType)
   *
   * \return communications type
   */
	int getCommType() {return this->comm_type_;};
  
    /**
   * \brief Gets reply code(see ReplyType)
   *
   * \return reply code
   */
  int getReplyCode() {return this->reply_code_;};
  
   /**
   * \brief Gets message length (total size, HEADER + data)
   *
   * \return message length
   */
	int getMsgLength() {return this->getHeaderSize() + this->data_.getBufferSize();};
  
   /**
   * \brief Gets length of message data portion.
   *
   * \return message data length
   */
	int getDataLength() {return this->data_.getBufferSize();};
  
   /**
   * \brief Returns a reference to the internal data member
   *
   * \return reference to message data portion.
   */
  industrial::byte_array::ByteArray & getData() {return this->data_;};
	
  /**
   * \brief performs logical checks to ensure that the message is fully
   * defined and adheres to the message conventions.
   *
   * \return true if message valid, false otherwise
   */
  bool validateMessage();
	


private:

  /**
   * \brief Message type(see StandardMsgType)
   */
  industrial::shared_types::shared_int message_type_;
  
   /**
   * \brief Communications type(see CommType)
   */
  industrial::shared_types::shared_int comm_type_;
  
  /**
   * \brief Reply code(see ReplyType)
   */
  industrial::shared_types::shared_int reply_code_;
  
  /**
   * \brief Message data portion
   */
	industrial::byte_array::ByteArray data_;

  /**
   * \brief Size(in bytes) of message header (fixed)
   */
	static const unsigned int HEADER_SIZE = sizeof(industrial::shared_types::shared_int) +
	    sizeof(industrial::shared_types::shared_int) +
	    sizeof(industrial::shared_types::shared_int);
      
  /**
   * \brief Size (in bytes) of message length parameter (fixed)
   */
	static const unsigned int LENGTH_SIZE = sizeof(industrial::shared_types::shared_int);

  /**
   * \brief Sets message type
   *
   * \param message type
   */
  void setMessageType(int msgType) {this->message_type_ = msgType;};
  
  /**
   * \brief Sets communications type
   *
   * \param communications type
   */
  void setCommType(int commType) {this->comm_type_ = commType;};
  
  /**
   * \brief Sets reply code
   *
   * \param reply code
   */
  void setReplyCode(int replyCode) {this->reply_code_ = replyCode;};
  
  /**
   * \brief Sets data portion
   *
   * \param data portion
   */
  void setData(industrial::byte_array::ByteArray & data);
};

}//namespace simple_message
}//namespace industrial

#endif //SIMPLE_MSG_
