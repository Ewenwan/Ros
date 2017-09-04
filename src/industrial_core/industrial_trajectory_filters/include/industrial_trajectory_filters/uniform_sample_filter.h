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

#ifndef UNIFORM_SAMPLE_FILTER_H_
#define UNIFORM_SAMPLE_FILTER_H_

#include <industrial_trajectory_filters/filter_base.h>

/*
 * These headers were part of the trajectory filter interface from the
 * arm navigation system.  These can no longer be included in a catkin based
 * package.
 #include <filters/filter_base.h>
 #include <pluginlib/class_list_macros.h>
 */

namespace industrial_trajectory_filters
{

/**
 * \brief This is a simple filter which performs a uniforming sampling of
 * a trajectory using linear interpolation.
 *
 */
template<typename T>

  class UniformSampleFilter : public industrial_trajectory_filters::FilterBase<T>
  {
  public:
    /**
     * @brief Default constructor
     */
    UniformSampleFilter();
    /**
     * @brief Default destructor
     */
    ~UniformSampleFilter();

    virtual bool configure();

    /**
     * Uniformly samples(in terms of time) the input trajectory based upon the
     * sample duration.  Sampling is performed via interpolation ensuring smooth
     * velocity and acceleration.  NOTE: For this reason trajectories must be
     * fully defined before using this filter.
     * @param trajectory_in non uniform trajectory
     * @param trajectory_out uniform(in terms of time) trajectory
     * @return
     */
    bool update(const T& trajectory_in, T& trajectory_out);

    /**
     * @brief Perform interpolation between p1 and p2.  Time from start must be
     * in between p1 and p2 times.
     * @param p1 prior trajectory point
     * @param p2 subsequent trajectory point
     * @param time_from_start time from start of trajectory (i.e. p0).
     * @param interp_pt resulting interpolated point
     * @return true if successful, otherwise false.
     */
    bool interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1, trajectory_msgs::JointTrajectoryPoint & p2,
                       double time_from_start, trajectory_msgs::JointTrajectoryPoint & interp_pt);

  private:
    /**
     * @brief uniform sample duration (sec)
     */
    double sample_duration_;
  };

/**
 * @brief Specializing trajectory filter implementation
 */
typedef UniformSampleFilter<MessageAdapter> UniformSampleFilterAdapter;

}

#endif
