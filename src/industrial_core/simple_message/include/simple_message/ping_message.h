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

#ifndef PING_MESSAGE_H
#define PING_MESSAGE_H

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#endif


namespace industrial
{
namespace ping_message
{

/**
 * \brief Class encapsulated ping message generation methods (either to or
 * from a SimpleMessage type.
 */
//* PingMessage
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class PingMessage : public industrial::typed_message::TypedMessage
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates an empty byte ping message.
   *
   */
  PingMessage(void);

  /**
   * \brief Destructor
   *
   */
  ~PingMessage(void);

  /**
   * \brief Initializes message from a simple message
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes a new ping message
   *
   */
  void init();

  /**
     * \brief The ping message overrides the base method toTopic to always
     * return false.  A ping cannot be sent as a topic.
     *
     */
  bool toTopic(industrial::simple_message::SimpleMessage & msg)
    {
  	  return false;
    }

  // Overrides - SimpleSerialize
    bool load(industrial::byte_array::ByteArray *buffer){return true;}
    bool unload(industrial::byte_array::ByteArray *buffer){return true;}
    unsigned int byteLength(){return 0;}

private:


};

}
}

#endif /* PING_MESSAGE_H */
