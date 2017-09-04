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
#include "simple_message/joint_traj.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_traj.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::joint_traj_pt;

namespace industrial
{
namespace joint_traj
{

JointTraj::JointTraj(void)
{
	this->init();
}
JointTraj::~JointTraj(void)
{

}

void JointTraj::init()
{
	JointTrajPt empty;

	this->size_ = 0;
	for (shared_int i = 0; i < this->getMaxNumPoints(); i++)
	{
		this->points_[i].copyFrom(empty);
	}
}

bool JointTraj::addPoint(JointTrajPt & point)
{
	bool rtn = false;

	if (!this->isFull())
	{
		this->points_[this->size()].copyFrom(point);
		this->size_++;
		rtn = true;
	}
	else
	{
		rtn = false;
		LOG_ERROR("Failed to add point, buffer is full");
	}

	return rtn;
}

bool JointTraj::getPoint(shared_int index, JointTrajPt & point)
{
	bool rtn = false;

	if (index < this->size())
	{
		point.copyFrom(this->points_[index]);
		rtn = true;
	}
	else
	{
		LOG_ERROR("Point index: %d, is greater than size: %d", index, this->size());
		rtn = false;
	}
	return rtn;
}

void JointTraj::copyFrom(JointTraj &src)
{
	JointTrajPt value;

	this->size_ = src.size();
	for (shared_int i = 0; i < this->size(); i++)
	{
		src.getPoint(i, value);
		this->points_[i].copyFrom(value);
	}
}

bool JointTraj::operator==(JointTraj &rhs)
{
	bool rtn = true;

	if(this->size() == rhs.size())
	{
		for(shared_int i = 0; i < this->size(); i++)
		{
			JointTrajPt value;
			rhs.getPoint(i, value);
			if(!(this->points_[i] == value))
			{
				LOG_DEBUG("Joint trajectory point different");
				rtn = false;
				break;
			}
			else
			{
				rtn = true;
			}
		}
	}
	else
	{
		LOG_DEBUG("Joint trajectory compare failed, size mismatch");
		rtn = false;
	}

	return rtn;
}


bool JointTraj::load(industrial::byte_array::ByteArray *buffer)
{
	bool rtn = false;
	JointTrajPt value;

	LOG_COMM("Executing joint trajectory load");
	for (shared_int i = 0; i < this->size(); i++)
	{
		this->getPoint(i, value);
		rtn = buffer->load(value);
		if (!rtn)
		{
			LOG_ERROR("Failed to load joint traj.pt. data");
			rtn = false;
			break;
		}
		rtn = true;
	}

	if (rtn)
	{
		rtn = buffer->load(this->size());
	}
	return rtn;
}

bool JointTraj::unload(industrial::byte_array::ByteArray *buffer)
{
	bool rtn = false;
	JointTrajPt value;

	LOG_COMM("Executing joint trajectory unload");

	rtn = buffer->unload(this->size_);

	if(rtn)
	{
		for (int i = this->size() - 1; i >= 0; i--)
		{
			rtn = buffer->unload(value);
			if (!rtn)
			{
				LOG_ERROR("Failed to unload message point: %d from data[%d]", i, buffer->getBufferSize());
				break;
			}
			this->points_[i].copyFrom(value);
		}
	}
	else
	{
		LOG_ERROR("Failed to unload trajectory size");
	}
	return rtn;
}

}
}

