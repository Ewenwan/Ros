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

#ifndef JOINT_TRAJECTORY_STREAMER_H
#define JOINT_TRAJECTORY_STREAMER_H

#include <boost/thread/thread.hpp>
#include "industrial_robot_client/joint_trajectory_interface.h"

namespace industrial_robot_client
{
namespace joint_trajectory_streamer
{

using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};
}
typedef TransferStates::TransferState TransferState;

/**
 * \brief Message handler that streams joint trajectories to the robot controller
 */

//* JointTrajectoryStreamer
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryStreamer : public JointTrajectoryInterface
{

public:

  // since this class defines a different init(), this helps find the base-class init()
  using JointTrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  JointTrajectoryStreamer(int min_buffer_size = 1) : min_buffer_size_(min_buffer_size) {};

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  ~JointTrajectoryStreamer();

  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<JointTrajPtMessage>* msgs);

  void streamingThread();

  bool send_to_robot(const std::vector<JointTrajPtMessage>& messages);

protected:

  void trajectoryStop();

  boost::thread* streaming_thread_;
  boost::mutex mutex_;
  int current_point_;
  std::vector<JointTrajPtMessage> current_traj_;
  TransferState state_;
  ros::Time streaming_start_;
  int min_buffer_size_;
};

} //joint_trajectory_streamer
} //industrial_robot_client

#endif /* JOINT_TRAJECTORY_STREAMER_H */
