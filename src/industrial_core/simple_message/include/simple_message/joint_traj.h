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

#ifndef JOINT_TRAJ_H
#define JOINT_TRAJ_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_traj_pt.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "joint_traj_pt.h"
#endif



namespace industrial
{
namespace joint_traj
{

/**
 * \brief Class encapsulated joint trajectory.  A joint trajectory includes
 * an array of JointTrajPt data types.  The intention for this class is to
 * be loaded into a single message for communication over a simple connection.
 *
 * For simplicity and cross platform compliance, this is implemented as a
 * fixed size array.  The size of the trajectory cannot exceed the max size
 * of the array.
 */
//* JointTraj
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTraj : public industrial::simple_serialize::SimpleSerialize
{
public:
	/**
	 * \brief Default constructor
	 *
	 * This method creates empty data.
	 *
	 */
	JointTraj(void);
	/**
	 * \brief Destructor
	 *
	 */
	~JointTraj(void);

	/**
	 * \brief Initializes a empty joint data
	 *
	 */
	void init();

	/**
	 * \brief Adds a point value to the end of the buffer
	 *
	 * \param point value
	 *
	 * \return true if value set, otherwise false (buffer is full)
	 */
	bool addPoint(industrial::joint_traj_pt::JointTrajPt & point);

	/**
	 * \brief Gets a point value within the buffer
	 *
	 * \param point index
	 * \param point value
	 *
	 * \return true if value set, otherwise false (index greater than size)
	 */
	bool getPoint(industrial::shared_types::shared_int index,
			industrial::joint_traj_pt::JointTrajPt & point);
	/**
	 * \brief Gets a size of trajectory
	 *
	 * \return trajectory size
	 */
	industrial::shared_types::shared_int size()
	{
		return this->size_;
	}

	/**
	 * \brief returns True if buffer is full
	 *
	 * \return true if buffer is full
	 */
	bool isFull()
	{
		return this->size_ >= this->getMaxNumPoints();
	}

	/**
	 * \brief returns the maximum number of points the message holds
	 *
	 * \return max number of points
	 */
	int getMaxNumPoints() const
	{
		return MAX_NUM_POINTS;
	}

	/**
	 * \brief Copies the passed in value
	 *
	 * \param src (value to copy)
	 */
	void copyFrom(JointTraj &src);

	/**
	 * \brief == operator implementation
	 *
	 * \return true if equal
	 */
	bool operator==(JointTraj &rhs);

	// Overrides - SimpleSerialize
	bool load(industrial::byte_array::ByteArray *buffer);
	bool unload(industrial::byte_array::ByteArray *buffer);
	unsigned int byteLength()
	{
		industrial::joint_traj_pt::JointTrajPt pt;
		return this->size() * pt.byteLength();
	}

private:

	/**
	 * \brief maximum number of joints positions that can be held in the message.
	 */
	static const industrial::shared_types::shared_int MAX_NUM_POINTS = 200;
	/**
	 * \brief internal data buffer
	 */
	industrial::joint_traj_pt::JointTrajPt points_[MAX_NUM_POINTS];
	/**
	 * \brief size of trajectory
	 */
	industrial::shared_types::shared_int size_;

};

}
}

#endif /* JOINT_TRAJ_H */
