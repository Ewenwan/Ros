/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef JOINT_TRAJ_PT_FULL_H
#define JOINT_TRAJ_PT_FULL_H

#ifndef FLATHEADERS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

namespace industrial
{
namespace joint_traj_pt_full
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD  = -1, ///< Downloading drivers only: signal start of trajectory
  START_TRAJECOTRY_STREAMING = -2, ///< deprecated, please use START_TRAJECTORY_STREAMING instead
  START_TRAJECTORY_STREAMING = -2, ///< Streaming drivers only: signal start of trajectory
  END_TRAJECTORY  = -3, ///< Downloading drivers only: signal end of trajectory
  STOP_TRAJECTORY = -4  ///< Server should stop the current motion (if any) as soon as possible
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

namespace ValidFieldTypes
{
enum ValidFieldType
{
  TIME = 0x01, POSITION = 0x02, VELOCITY = 0x04, ACCELERATION = 0x08
};
}
typedef ValidFieldTypes::ValidFieldType ValidFieldType;

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt class, but this
 * class provides the full message contents directly to the robot controller,
 * rather than simplifying the velocity duration.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   robot_id            (industrial::shared_types::shared_int)    4  bytes
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   valid_fields        (industrial::shared_types::shared_int)    4  bytes
 *   time                (industrial::shared_types::shared_real)   4  bytes
 *   positions           (industrial::joint_data)                  40 bytes
 *   velocities          (industrial::joint_data)                  40 bytes
 *   accelerations       (industrial::joint_data)                  40 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtFull : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointTrajPtFull(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtFull(void);

  /**
   * \brief Initializes a empty joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory point
   *
   */
  void init(industrial::shared_types::shared_int robot_id,
            industrial::shared_types::shared_int sequence,
            industrial::shared_types::shared_int valid_fields,
            industrial::shared_types::shared_real time,
            industrial::joint_data::JointData & positions,
            industrial::joint_data::JointData & velocities,
            industrial::joint_data::JointData & accelerations);

  /**
   * \brief Sets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * \param robot_id new robot_id value
   */
  void setRobotID(industrial::shared_types::shared_int robot_id)
  {
    this->robot_id_ = robot_id;
  }

  /**
   * \brief Gets robot_id.
   *        Robot group # (0-based), for controllers with multiple axis-groups.
   *
   * @return robot_id value
   */
  industrial::shared_types::shared_int getRobotID()
  {
    return this->robot_id_;
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

  /**
   * \brief Sets joint trajectory point timestamp
   *
   * \param time new time value
   */
  void setTime(industrial::shared_types::shared_real time)
  {
    this->time_ = time;
    this->valid_fields_ |= ValidFieldTypes::TIME;  // set the bit
  }

  /**
   * \brief Returns joint trajectory point timestamp
   *
   * \param time returned time value
   * \return true if this field contains valid data
   */
  bool getTime(industrial::shared_types::shared_real & time)
  {
    time = this->time_;
    return is_valid(ValidFieldTypes::TIME);
  }

  /**
   * \brief Clears the joint trajectory point timestamp
   */
  void clearTime()
  {
    this->time_ = 0;
    this->valid_fields_ &= ~ValidFieldTypes::TIME;  // clear the bit
  }

  /**
   * \brief Sets joint position data
   *
   * \param positions new joint position data
   */
  void setPositions(industrial::joint_data::JointData &positions)
  {
    this->positions_.copyFrom(positions);
    this->valid_fields_ |= ValidFieldTypes::POSITION;  // set the bit
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param dest returned joint position
   * \return true if this field contains valid data
   */
  bool getPositions(industrial::joint_data::JointData &dest)
  {
    dest.copyFrom(this->positions_);
    return is_valid(ValidFieldTypes::POSITION);
  }

  /**
   * \brief Clears the position data
   */
  void clearPositions()
  {
    this->positions_.init();
    this->valid_fields_ &= ~ValidFieldTypes::POSITION;  // clear the bit
  }

  /**
   * \brief Sets joint velocity data
   *
   * \param velocities new joint velocity data
   */
  void setVelocities(industrial::joint_data::JointData &velocities)
  {
    this->velocities_.copyFrom(velocities);
    this->valid_fields_ |= ValidFieldTypes::VELOCITY;  // set the bit
  }

  /**
   * \brief Returns a copy of the velocity data
   *
   * \param dest returned joint velocity
   * \return true if this field contains valid data
   */
  bool getVelocities(industrial::joint_data::JointData &dest)
  {
    dest.copyFrom(this->velocities_);
    return is_valid(ValidFieldTypes::VELOCITY);
  }

  /**
   * \brief Clears the velocity data
   */
  void clearVelocities()
  {
    this->velocities_.init();
    this->valid_fields_ &= ~ValidFieldTypes::VELOCITY;  // clear the bit
  }
  /**
   * \brief Sets joint acceleration data
   *
   * \param accelerations new joint acceleration data
   */
  void setAccelerations(industrial::joint_data::JointData &accelerations)
  {
    this->accelerations_.copyFrom(accelerations);
    this->valid_fields_ |= ValidFieldTypes::ACCELERATION;  // set the bit
  }

  /**
   * \brief Returns a copy of the acceleration data
   *
   * \param dest returned joint acceleration
   * \return true if this field contains valid data
   */
  bool getAccelerations(industrial::joint_data::JointData &dest)
  {
    dest.copyFrom(this->accelerations_);
    return is_valid(ValidFieldTypes::ACCELERATION);
  }

  /**
   * \brief Clears the acceleration data
   */
  void clearAccelerations()
  {
    this->accelerations_.init();
    this->valid_fields_ &= ~ValidFieldTypes::ACCELERATION;  // clear the bit
  }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPtFull &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointTrajPtFull &rhs);

  /**
   * \brief check the validity state for a given field
   * @param field field to check
   * @return true if specified field contains valid data
   */
  bool is_valid(ValidFieldType field)
  {
    return valid_fields_ & field;
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 3*sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_real)
        + 3*this->positions_.byteLength();
  }

private:

  /**
   * \brief robot group # (0-based) for controllers that support multiple axis-groups
   */
  industrial::shared_types::shared_int robot_id_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;
  /**
   * \brief bit-mask of (optional) fields that have been initialized with valid data
   * \see enum ValidFieldTypes
   */
  industrial::shared_types::shared_int valid_fields_;
  /**
   * \brief joint trajectory point timestamp
   *        Typically, time_from_start of this trajectory (in seconds)
   */
  industrial::shared_types::shared_real time_;

  /**
   * \brief joint trajectory point positional data
   */
  industrial::joint_data::JointData positions_;
  /**
   * \brief joint trajectory point velocity data
   */
  industrial::joint_data::JointData velocities_;  /**
   * \brief joint trajectory point acceleration data
   */
  industrial::joint_data::JointData accelerations_;

};

}
}

#endif /* JOINT_TRAJ_PT_FULL_H */
