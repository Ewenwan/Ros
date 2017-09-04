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
#include "simple_message/joint_data.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_data.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace joint_data
{

JointData::JointData(void)
{
  this->init();
}
JointData::~JointData(void)
{

}

void JointData::init()
{
  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->setJoint(i, 0.0);
  }
}

bool JointData::setJoint(shared_int index, shared_real value)
{
  bool rtn = false;

  if (index < this->getMaxNumJoints())
  {
    this->joints_[index] = value;
    rtn = true;
  }
  else
  {
    LOG_ERROR("Joint index: %d, is greater than size: %d", index, this->getMaxNumJoints());
    rtn = false;
  }
  return rtn;
}

bool JointData::getJoint(shared_int index, shared_real & value) const
{
  bool rtn = false;

  if (index < this->getMaxNumJoints())
  {
    value = this->joints_[index];
    rtn = true;
  }
  else
  {
    LOG_ERROR("Joint index: %d, is greater than size: %d", index, this->getMaxNumJoints());
    rtn = false;
  }
  return rtn;
}

shared_real JointData::getJoint(shared_int index) const
{
  shared_real rtn = 0.0;
  this->getJoint(index, rtn);
  return rtn;
}
  

void JointData::copyFrom(JointData &src)
{
  shared_real value = 0.0;

  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    src.getJoint(i, value);
    this->setJoint(i, value);
  }
}

bool JointData::operator==(JointData &rhs)
{
  bool rtn = true;

  shared_real lhsvalue, rhsvalue;

  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->getJoint(i, lhsvalue);
    rhs.getJoint(i, rhsvalue);
    if (lhsvalue != rhsvalue)
    {
      rtn = false;
      break;
    }
  }
  return rtn;

}

bool JointData::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  LOG_COMM("Executing joint position load");
  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->getJoint(i, value);
    rtn = buffer->load(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to load joint position data");
      break;
    }
  }
  return rtn;
}

bool JointData::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  LOG_COMM("Executing joint position unload");
  for (int i = this->getMaxNumJoints() - 1; i >= 0; i--)
  {
    rtn = buffer->unload(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to unload message joint: %d from data[%d]", i, buffer->getBufferSize());
      break;
    }
    this->setJoint(i, value);
  }
  return rtn;
}

}
}

