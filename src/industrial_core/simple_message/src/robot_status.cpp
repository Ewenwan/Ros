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
#include "simple_message/robot_status.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "robot_status.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

#ifdef ROS
// Files below used to translate between ROS messages enums and
// enums defined in this file
#include "industrial_msgs/RobotMode.h"
#include "industrial_msgs/TriState.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace robot_status
{

namespace RobotModes
{

#ifdef ROS

int toROSMsgEnum(RobotModes::RobotMode mode)
{

  switch (mode)
  {
    case RobotModes::AUTO:
      return industrial_msgs::RobotMode::AUTO;
      break;
    case RobotModes::MANUAL:
      return industrial_msgs::RobotMode::MANUAL;
      break;
    case RobotModes::UNKNOWN:
      return industrial_msgs::RobotMode::UNKNOWN;
  }
  return industrial_msgs::RobotMode::UNKNOWN;

}
;

#endif

}

namespace TriStates
{

#ifdef ROS

int toROSMsgEnum(TriStates::TriState state)
{

  switch (state)
  {
    case TriStates::TS_UNKNOWN:
      return industrial_msgs::TriState::UNKNOWN;
      break;
    case TriStates::TS_TRUE:
      return industrial_msgs::TriState::TRUE;
      break;
    case TriStates::TS_FALSE:
      return industrial_msgs::TriState::FALSE;
      break;
  }
  return industrial_msgs::TriState::UNKNOWN;

}
;

#endif

}

RobotStatus::RobotStatus(void)
{
  this->init();
}
RobotStatus::~RobotStatus(void)
{

}

void RobotStatus::init()
{
  this->init(TriStates::TS_UNKNOWN, TriStates::TS_UNKNOWN, 0, TriStates::TS_UNKNOWN,
             TriStates::TS_UNKNOWN, RobotModes::UNKNOWN, TriStates::TS_UNKNOWN);
}

void RobotStatus::init(TriState drivesPowered, TriState eStopped, industrial::shared_types::shared_int errorCode,
                       TriState inError, TriState inMotion, RobotMode mode, TriState motionPossible)
{
  this->setDrivesPowered(drivesPowered);
  this->setEStopped(eStopped);
  this->setErrorCode(errorCode);
  this->setInError(inError);
  this->setInMotion(inMotion);
  this->setMode(mode);
  this->setMotionPossible(motionPossible);
}

void RobotStatus::copyFrom(RobotStatus &src)
{
  this->setDrivesPowered(src.getDrivesPowered());
  this->setEStopped(src.getEStopped());
  this->setErrorCode(src.getErrorCode());
  this->setInError(src.getInError());
  this->setInMotion(src.getInMotion());
  this->setMode(src.getMode());
  this->setMotionPossible(src.getMotionPossible());
}

bool RobotStatus::operator==(RobotStatus &rhs)
{
  return this->drives_powered_ == rhs.drives_powered_ && this->e_stopped_ == rhs.e_stopped_
      && this->error_code_ == rhs.error_code_ && this->in_error_ == rhs.in_error_ && this->in_motion_ == rhs.in_motion_
      && this->mode_ == rhs.mode_ && this->motion_possible_ == rhs.motion_possible_;
}

bool RobotStatus::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing robot status load");

  if (buffer->load(this->drives_powered_) && buffer->load(this->e_stopped_) && buffer->load(this->error_code_)
      && buffer->load(this->in_error_) && buffer->load(this->in_motion_) && buffer->load(this->mode_)
      && buffer->load(this->motion_possible_))
  {

    LOG_COMM("Robot status successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("Robot status not loaded");
    rtn = false;
  }

  return rtn;
}

bool RobotStatus::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing robot status unload");
  if (buffer->unload(this->motion_possible_) && buffer->unload(this->mode_) && buffer->unload(this->in_motion_)
      && buffer->unload(this->in_error_) && buffer->unload(this->error_code_) && buffer->unload(this->e_stopped_)
      && buffer->unload(this->drives_powered_))
  {

    rtn = true;
    LOG_COMM("Robot status successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload robot status");
    rtn = false;
  }

  return rtn;
}

}
}

