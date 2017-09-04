/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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


#ifndef PARAM_UTILS_H_
#define PARAM_UTILS_H_

#include <map>
#include <vector>
#include <string>

namespace industrial_utils
{
namespace param
{

/**
 * \brief Gets parameter list as vector of strings
 *
 * \param param_name name of list parameter
 * \param list_param populated with parameter value(s)
 *
 * \return true if parameter
 */
bool getListParam(const std::string param_name, std::vector<std::string> & list_param);

/**
 * \brief Tries to get a set of joint names using several fallback methods:
 *          1) check parameter for an explicit list
 *          2) try to parse from given URDF data
 *          3) use default joint names: ["joint_1", "joint_2", ..., "joint_6"]
 *
 * \param[in] joint_list_param name of joint-names-list parameter to check
 * \param[in] urdf_param name of URDF description parameter to check
 * \param[out] joint_names list of joint names
 *
 * \return true if parameter found, false if defaults used
 */
bool getJointNames(const std::string joint_list_param, const std::string urdf_param,
		           std::vector<std::string> & joint_names);

/**
 * \brief Tries to read joint velocity limits from the specified URDF parameter
 *
 * \param[in] urdf_param_name name of URDF parameter
 * \param[out] velocity_limits map of velocity limits for each URDF joint
 *
 * \return true if parameter found, false if not found
 */
bool getJointVelocityLimits(const std::string urdf_param_name, std::map<std::string, double> &velocity_limits);

} //industrial_utils::param
} //industrial_utils
#endif /* PARAM_UTILS_H_ */
