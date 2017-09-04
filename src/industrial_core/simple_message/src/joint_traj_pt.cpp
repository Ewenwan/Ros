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
#include "simple_message/joint_traj_pt.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_traj_pt.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;
using namespace industrial::shared_types;

namespace industrial
{
namespace joint_traj_pt
{

JointTrajPt::JointTrajPt(void)
{
  this->init();
}
JointTrajPt::~JointTrajPt(void)
{

}

void JointTrajPt::init()
{
  this->joint_position_.init();
  this->sequence_ = 0;
  this->velocity_ = 0.0;
  this->duration_ = 0.0;
}

void JointTrajPt::init(shared_int sequence, JointData & position, shared_real velocity, shared_real duration)
{
  this->setJointPosition(position);
  this->setSequence(sequence);
  this->setVelocity(velocity);
  this->setDuration(duration);
}

void JointTrajPt::copyFrom(JointTrajPt &src)
{
  this->setSequence(src.getSequence());
  src.getJointPosition(this->joint_position_);
  this->setVelocity(src.getVelocity());
  this->setDuration(src.getDuration());
}

bool JointTrajPt::operator==(JointTrajPt &rhs)
{
  return this->joint_position_ == rhs.joint_position_ && this->sequence_ == rhs.sequence_
      && this->velocity_ == rhs.velocity_ && this->duration_ == rhs.duration_;

}

bool JointTrajPt::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing joint trajectory point load");

  if (buffer->load(this->sequence_))
  {
    if (this->joint_position_.load(buffer))
    {
      if (buffer->load(this->velocity_))
      {
        if (buffer->load(this->duration_))
        {
          LOG_COMM("Trajectory point successfully loaded");
          rtn = true;
        }
        else
        {
          rtn = false;
          LOG_ERROR("Failed to load joint traj pt. duration");
        }
        rtn = true;
      }
      else
      {
        rtn = false;
        LOG_ERROR("Failed to load joint traj pt. velocity");
      }

    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to load joint traj. pt.  position data");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt. sequence number");
  }

  return rtn;
}

bool JointTrajPt::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing joint traj. pt. unload");
  if (buffer->unload(this->duration_))
  {
    if (buffer->unload(this->velocity_))
    {
      if (this->joint_position_.unload(buffer))
      {
        if (buffer->unload(this->sequence_))
        {
          rtn = true;
          LOG_COMM("Joint traj. pt successfully unloaded");
        }
        else
        {
          LOG_ERROR("Failed to unload joint traj. pt. sequence number");
          rtn = false;
        }
      }
      else
      {
        LOG_ERROR("Failed to unload joint traj. pt.  position data");
        rtn = false;
      }

    }
    else
    {
      LOG_ERROR("Failed to unload joint traj. pt. velocity");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Failed to unload joint traj. pt. duration");
    rtn = false;
  }

  return rtn;
}

}
}

