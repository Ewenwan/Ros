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

#ifndef N_POINT_FILTER_H_
#define N_POINT_FILTER_H_

#include <industrial_trajectory_filters/filter_base.h>

namespace industrial_trajectory_filters
{

/**
 * @brief This is a simple filter which reduces a trajectory to N points or less
 */
template<typename T>

  class NPointFilter : public industrial_trajectory_filters::FilterBase<T>
  {
  public:
    /**
     * @brief Default constructor
     */
    NPointFilter();
    /**
     * @brief Default destructor
     */
    ~NPointFilter();
    ;

    virtual bool configure();

    // \brief Reduces a trajectory to N points or less.  The resulting trajectory
    // contains only point within the original trajectory (no interpolation is done
    // between points).

    /**
     * \brief Reduces a trajectory to N points or less.  The resulting trajectory
     * contains only point within the original trajectory (no interpolation is done
     *  between points).
     * @param trajectory_in input trajectory
     * @param trajectory_out filtered trajectory (N points or less
     * @return true if successful
     */
    bool update(const T& trajectory_in, T& trajectory_out);

  private:
    /**
     * @brief number of points to reduce trajectory to
     */
    int n_points_;

  };

/**
 * @brief Specializing trajectory filter implementation
 */
typedef NPointFilter<MessageAdapter> NPointFilterAdapter;

}

#endif
