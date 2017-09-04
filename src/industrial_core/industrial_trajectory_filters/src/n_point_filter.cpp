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

//#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <industrial_trajectory_filters/n_point_filter.h>

using namespace industrial_trajectory_filters;

const int DEFAULT_N = 2;

template<typename T>
  NPointFilter<T>::NPointFilter() :
      FilterBase<T>()
  {
    ROS_INFO_STREAM("Constructing N point filter");
    n_points_ = DEFAULT_N;
    this->filter_name_ = "NPointFilter";
    this->filter_type_ = "NPointFilter";
  }

template<typename T>
  NPointFilter<T>::~NPointFilter()
  {
  }

template<typename T>
  bool NPointFilter<T>::configure()
  {
    //if (!filters::FilterBase<T>::getParam("n_points", n_points_))
    if (!this->nh_.getParam("n_points", n_points_))
    {
      ROS_WARN_STREAM("NPointFilter, params has no attribute n_points.");
    }
    if (n_points_ < 2)
    {
      ROS_WARN_STREAM( "n_points attribute less than min(2), setting to minimum");
      n_points_ = 2;
    }
    ROS_INFO_STREAM("Using a n_points value of " << n_points_);

    return true;
  }

template<typename T>
  bool NPointFilter<T>::update(const T& trajectory_in, T& trajectory_out)
  {
    bool success = false;
    int size_in = trajectory_in.request.trajectory.points.size();

    // Copy non point related data
    trajectory_out.request.trajectory = trajectory_in.request.trajectory;
    // Clear out the trajectory points
    trajectory_out.request.trajectory.points.clear();

    if (size_in > n_points_)
    {
      //Add first point to output trajectory
      trajectory_out.request.trajectory.points.push_back(trajectory_in.request.trajectory.points.front());

      int intermediate_points = n_points_ - 2; //subtract the first and last elements
      double int_point_increment = double(size_in) / double(intermediate_points + 1.0);
      ROS_INFO_STREAM(
          "Number of intermediate points: " << intermediate_points << ", increment: " << int_point_increment);

      // The intermediate point index is determined by the following equation:
      //     int_point_index = i * int_point_increment
      // Ex: n_points_ = 4, size_in = 11, intermediate_points = 2 ->
      //     int_point_increment = 3.66667,
      //     i = 1: int_point_index = 3
      //		 i = 2: int_point_index = 7
      for (int i = 1; i <= intermediate_points; i++)
      {
        int int_point_index = int(double(i) * int_point_increment);
        ROS_INFO_STREAM("Intermediate point index: " << int_point_index);
        trajectory_out.request.trajectory.points.push_back(trajectory_in.request.trajectory.points[int_point_index]);
      }

      //Add last point to output trajectory
      trajectory_out.request.trajectory.points.push_back(trajectory_in.request.trajectory.points.back());

      ROS_INFO_STREAM(
          "Filtered trajectory from: " << trajectory_in.request.trajectory.points.size() << " to: " << trajectory_out.request.trajectory.points.size());

      success = true;
    }
    else
    {
      ROS_WARN_STREAM( "Trajectory size less than n: " << n_points_ << ", pass through");
      trajectory_out.request.trajectory = trajectory_in.request.trajectory;
      success = true;
    }

    return success;
  }

// registering planner adapter
CLASS_LOADER_REGISTER_CLASS(industrial_trajectory_filters::NPointFilterAdapter,
                            planning_request_adapter::PlanningRequestAdapter);

/*
 * Old plugin declaration for arm navigation trajectory filters
 PLUGINLIB_DECLARE_CLASS(industrial_trajectory_filters,
 IndustrialNPointFilterJointTrajectoryWithConstraints,
 industrial_trajectory_filters::NPointFilter<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
 filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>);

 */
