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

#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include <kdl/velocityprofile_spline.hpp>
#include <ros/ros.h>

using namespace industrial_trajectory_filters;

const double DEFAULT_SAMPLE_DURATION = 0.050; //seconds

template<typename T>
  UniformSampleFilter<T>::UniformSampleFilter() :
      industrial_trajectory_filters::FilterBase<T>()
  {
    ROS_INFO_STREAM("Constructing N point filter");
    sample_duration_ = DEFAULT_SAMPLE_DURATION;
    this->filter_name_ = "UniformSampleFilter";
    this->filter_type_ = "UniformSampleFilter";
  }

template<typename T>
  UniformSampleFilter<T>::~UniformSampleFilter()
  {
  }

template<typename T>
  bool UniformSampleFilter<T>::configure()
  {
    if (!this->nh_.getParam("sample_duration", sample_duration_))
    {
      ROS_WARN_STREAM( "UniformSampleFilter, params has no attribute sample_duration.");
    }
    ROS_INFO_STREAM("Using a sample_duration value of " << sample_duration_);

    return true;
  }

template<typename T>
  bool UniformSampleFilter<T>::update(const T& trajectory_in, T& trajectory_out)
  {
    bool success = false;
    size_t size_in = trajectory_in.request.trajectory.points.size();
    double duration_in = trajectory_in.request.trajectory.points.back().time_from_start.toSec();
    double interpolated_time = 0.0;
    size_t index_in = 0;

    trajectory_msgs::JointTrajectoryPoint p1, p2, interp_pt;

    trajectory_out = trajectory_in;

    // Clear out the trajectory points
    trajectory_out.request.trajectory.points.clear();

    while (interpolated_time < duration_in)
    {
      ROS_DEBUG_STREAM("Interpolated time: " << interpolated_time);
      // Increment index until the interpolated time is past the start time.
      while (interpolated_time > trajectory_in.request.trajectory.points[index_in + 1].time_from_start.toSec())
      {
        ROS_DEBUG_STREAM(
            "Interpolated time: " << interpolated_time << ", next point time: " << (trajectory_in.request.trajectory.points[index_in + 1].time_from_start.toSec()));
        ROS_DEBUG_STREAM("Incrementing index");
        index_in++;
        if (index_in >= size_in)
        {
          ROS_ERROR_STREAM(
              "Programming error, index: " << index_in << ", greater(or equal) to size: " << size_in << " input duration: " << duration_in << " interpolated time:)" << interpolated_time);
          return false;
        }
      }
      p1 = trajectory_in.request.trajectory.points[index_in];
      p2 = trajectory_in.request.trajectory.points[index_in + 1];
      if (!interpolatePt(p1, p2, interpolated_time, interp_pt))
      {
        ROS_ERROR_STREAM("Failed to interpolate point");
        return false;
      }
      trajectory_out.request.trajectory.points.push_back(interp_pt);
      interpolated_time += sample_duration_;

    }

    ROS_INFO_STREAM(
        "Interpolated time exceeds original trajectory (quitting), original: " << duration_in << " final interpolated time: " << interpolated_time);
    p2 = trajectory_in.request.trajectory.points.back();
    p2.time_from_start = ros::Duration(interpolated_time);
    // TODO: Really should check that appending the last point doesn't result in
    // really slow motion at the end.  This could happen if the sample duration is a
    // large percentage of the trajectory duration (not likely).
    trajectory_out.request.trajectory.points.push_back(p2);

    ROS_INFO_STREAM(
        "Uniform sampling, resample duraction: " << sample_duration_ << " input traj. size: " << trajectory_in.request.trajectory.points.size() << " output traj. size: " << trajectory_out.request.trajectory.points.size());

    success = true;
    return success;
  }

template<typename T>
  bool UniformSampleFilter<T>::interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1,
                                             trajectory_msgs::JointTrajectoryPoint & p2, double time_from_start,
                                             trajectory_msgs::JointTrajectoryPoint & interp_pt)
  {
    bool rtn = false;
    double p1_time_from_start = p1.time_from_start.toSec();
    double p2_time_from_start = p2.time_from_start.toSec();

    ROS_DEBUG_STREAM("time from start: " << time_from_start);

    if (time_from_start >= p1_time_from_start && time_from_start <= p2_time_from_start)
    {
      if (p1.positions.size() == p1.velocities.size() && p1.positions.size() == p1.accelerations.size())
      {
        if (p1.positions.size() == p2.positions.size() && p1.velocities.size() == p2.velocities.size()
            && p1.accelerations.size() == p2.accelerations.size())
        {
          // Copy p1 to ensure the interp_pt has the correct size vectors
          interp_pt = p1;
          // TODO: Creating a new spline calculator in this function means that
          // it may be created multiple times for the same points (assuming the
          // resample duration is less that the actual duration, which it might
          // be sometimes)
          KDL::VelocityProfile_Spline spline_calc;
          ROS_DEBUG_STREAM( "---------------Begin interpolating joint point---------------");

          for (size_t i = 0; i < p1.positions.size(); ++i)
          {
            // Calculated relative times for spline calculation
            double time_from_p1 = time_from_start - p1.time_from_start.toSec();
            double time_from_p1_to_p2 = p2_time_from_start - p1_time_from_start;

            ROS_DEBUG_STREAM("time from p1: " << time_from_p1);
            ROS_DEBUG_STREAM( "time_from_p1_to_p2: " << time_from_p1_to_p2);

            spline_calc.SetProfileDuration(p1.positions[i], p1.velocities[i], p1.accelerations[i], p2.positions[i],
                                           p2.velocities[i], p2.accelerations[i], time_from_p1_to_p2);

            ros::Duration time_from_start_dur(time_from_start);
            ROS_DEBUG_STREAM( "time from start_dur: " << time_from_start_dur);

            interp_pt.time_from_start = time_from_start_dur;
            interp_pt.positions[i] = spline_calc.Pos(time_from_p1);
            interp_pt.velocities[i] = spline_calc.Vel(time_from_p1);
            interp_pt.accelerations[i] = spline_calc.Acc(time_from_p1);

            ROS_DEBUG_STREAM(
                "p1.pos: " << p1.positions[i] << ", vel: " << p1.velocities[i] << ", acc: " << p1.accelerations[i] << ", tfs: " << p1.time_from_start);

            ROS_DEBUG_STREAM(
                "p2.pos: " << p2.positions[i] << ", vel: " << p2.velocities[i] << ", acc: " << p2.accelerations[i] << ", tfs: " << p2.time_from_start);

            ROS_DEBUG_STREAM(
                "interp_pt.pos: " << interp_pt.positions[i] << ", vel: " << interp_pt.velocities[i] << ", acc: " << interp_pt.accelerations[i] << ", tfs: " << interp_pt.time_from_start);
          }
          ROS_DEBUG_STREAM( "---------------End interpolating joint point---------------");
          rtn = true;
        }
        else
        {
          ROS_ERROR_STREAM("Trajectory point size mismatch");
          ROS_ERROR_STREAM(
              "Trajectory point 1, pos: " << p1.positions.size() << " vel: " << p1.velocities.size() << " acc: " << p1.accelerations.size());
          ROS_ERROR_STREAM(
              "Trajectory point 2, pos: " << p2.positions.size() << " vel: " << p2.velocities.size() << " acc: " << p2.accelerations.size());
          rtn = false;
        }

      }
      else
      {
        ROS_ERROR_STREAM(
            "Trajectory point not fully defined, pos: " << p1.positions.size() << " vel: " << p1.velocities.size() << " acc: " << p1.accelerations.size());
        rtn = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM(
          "Time: " << time_from_start << " not between interpolation point times[" << p1.time_from_start.toSec() << "," << p2.time_from_start.toSec() << "]");
      rtn = false;
    }

    return rtn;
  }

// registering planner adapter
CLASS_LOADER_REGISTER_CLASS( industrial_trajectory_filters::UniformSampleFilterAdapter,
                            planning_request_adapter::PlanningRequestAdapter);

/*
 * Old plugin declaration for arm navigation trajectory filters
 PLUGINLIB_DECLARE_CLASS(industrial_trajectory_filters,
 IndustrialNPointFilterJointTrajectoryWithConstraints,
 industrial_trajectory_filters::NPointFilter<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
 filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>);

 */

