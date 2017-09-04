/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Southwest Research Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Southwest Research Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Chris Lewis */

#include <industrial_trajectory_filters/smoothing_trajectory_filter.h>
#include <console_bridge/console.h>
#include <moveit/robot_state/conversions.h>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>

namespace industrial_trajectory_filters
{


  SmoothingTrajectoryFilter::SmoothingTrajectoryFilter()
  {
      initialized_ = false;
  }

  bool SmoothingTrajectoryFilter::init(std::vector<double> &coef)
  {
    if(coef.size()%2 == 1) {		// smoothing filters must have an odd number of coefficients
      initialized_ = true;
      num_coef_ = coef.size();
      double sum =0;
      for(int i=0; i<num_coef_; i++) {
	coef_.push_back(coef[i]);
	sum += coef[i];
      }
      gain_ = sum;		// set gain to be the sum of the coefficients because we need unity gain
      return(true);
    }
    else{
      initialized_ = false;
      return(false);
    }
  }

  SmoothingTrajectoryFilter::~SmoothingTrajectoryFilter()
  {
    coef_.clear();
  }

 bool SmoothingTrajectoryFilter::applyFilter(robot_trajectory::RobotTrajectory& rob_trajectory) const
  {
    if(!initialized_) return(false);

    const int num_points = rob_trajectory.getWayPointCount(); 
    if(num_points <=2) return(false); // nothing to do here, can't change either first or last point
    const int num_states = rob_trajectory.getWayPoint(0).getVariableCount();
    std::vector<double> xv;
    
    // filter each variable independently
    for(int i=0; i<num_states; i++){ 
      double start_value     = rob_trajectory.getWayPoint(0).getVariablePosition(i);
      double start_slope     = rob_trajectory.getWayPoint(1).getVariablePosition(i) - start_value; // slope at start
      double end_value      = rob_trajectory.getWayPoint(num_points-1).getVariablePosition(i);
      double end_slope      = end_value - rob_trajectory.getWayPoint(num_points-2).getVariablePosition(i); // slope at end
 
      // initialize the filter to have initial slope
      xv.clear();
      double value = start_value - (num_coef_/2)*start_slope;
      for(int j=0; j<num_coef_; j++) { 
	xv.push_back(value);
	value += start_slope;
      }
      
      // cycle through every waypoint, and apply the filter, NOTE, 1st and last waypoints should not be changed
      for(int j=1; j<num_points-1; j++){
	// shift backwards
	for(int k=0; k<num_coef_-1; k++){
	  xv[k] = xv[k+1];  
	}
	
	// get next input to filter which is num_coef/2 in front of current point being smoothed
	if(j+num_coef_/2 < num_points){	
	  xv[num_coef_ - 1] = rob_trajectory.getWayPoint(j+num_coef_/2).getVariablePosition(i); // i'th state of j'th waypoint
	}
	else{
	  end_value += end_slope;
	  xv[num_coef_-1] = end_value; // fill by continuing with final slope
	}
	// apply the filter
	double sum = 0.0;
	for(int k=0; k<num_coef_; k++){ 
	  sum += xv[k]*coef_[k];
	}

	// save the results
	rob_trajectory.getWayPointPtr(j)->setVariablePosition(i,sum/gain_); // j'th waypoint, i'th variable set to output value

      }// end for every waypoint

    } // end for every state

    return(true);

}// end SmoothingTrajectoryFilter::applyfilter()

}  // end namespace 
