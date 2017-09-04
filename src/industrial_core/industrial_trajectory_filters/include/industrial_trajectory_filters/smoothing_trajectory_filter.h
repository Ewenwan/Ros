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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef MOVEIT_TRAJECTORY_PROCESSING_SMOOTHING_TRAJECTORY_FILTER_
#define MOVEIT_TRAJECTORY_PROCESSING_SMOOTHING_TRAJECTORY_FILTER_

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace industrial_trajectory_filters
{

  /*! \brief This class  filters the trajectory using a Finite Impluse Response filter */
class SmoothingTrajectoryFilter
{
public:
  /*!  \brief Constructor */
  SmoothingTrajectoryFilter();

  /*! \brief Destructor */
  ~SmoothingTrajectoryFilter();
  
  /*!  \brief Constructor
   *    @param coef   a vector of Smoothing coeficients with an odd number of values
   *    @return  true if the number of coeficients is odd, otherwise, false. All smoothing filters have odd number of coef
   */
  bool init(std::vector<double> &coef);

  /* \brief action of filter depends on the coefficients, intended to be a low pass filter
   *  @param rob_trajectory   A robot_trajectory::RobotTrajectory to be filtered
   */
  bool applyFilter(robot_trajectory::RobotTrajectory& rob_trajectory) const;  

private:
  double gain_; /*!< gain_ is the sum of the coeficients to achieve unity gain overall */
  int num_coef_; /*< the number of coefficients  */
  std::vector<double> coef_; /*!< Vector of coefficients  */ 
  bool initialized_; /*!< was the init() function called sucessfully? */
};

}
#endif
