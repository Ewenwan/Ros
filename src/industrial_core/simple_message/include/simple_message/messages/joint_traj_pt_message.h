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

#ifndef JOINT_TRAJ_PT_MESSAGE_H
#define JOINT_TRAJ_PT_MESSAGE_H

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_traj_pt.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "joint_traj_pt.h"
#endif

namespace industrial
{
namespace joint_traj_pt_message
{


/**
 * \brief Class encapsulated joint trajectory point message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::joint_traj_pt::JointTrajPt data type.
 * The data portion of this typed message matches JointTrajPt.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointTrajPtMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint trajectory point structure
   *
   * \param joint trajectory point data structure
   *
   */
  void init(industrial::joint_traj_pt::JointTrajPt & point);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->point_.byteLength();
  }

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence) { point_.setSequence(sequence); }

  industrial::joint_traj_pt::JointTrajPt point_;

private:


};

}
}

#endif /* JOINT_TRAJ_PT_MESSAGE_H */
