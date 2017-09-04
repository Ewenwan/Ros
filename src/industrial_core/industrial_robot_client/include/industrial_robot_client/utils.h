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

#ifndef INDUSTRIAL_ROBOT_CLIENT_UTILS_H_
#define INDUSTRIAL_ROBOT_CLIENT_UTILS_H_

#include <vector>
#include <string>
#include <map>

namespace industrial_robot_client
{
namespace utils
{


/**
 * \brief Checks to see if members are within the same range.  Specifically,
 * Each item in abs(lhs[i] - rhs[i]) < abs(range/2.0).  Empty vectors always
 * return true.
 * The behavior of this function around large values (near double limits) is
 * uncertain.
 * This function is not optimized, do not use for large vectors or loops.
 *
 * \param lhs first vector
 * \param rhs second vector
 * \param full_range
 *
 * \return true set are similar (same members, same order)
 */
bool isWithinRange(const std::vector<double> & lhs, const std::vector<double> & rhs,
                   double full_range);

/**
 * \brief Convenience function for inserting a value into an std::map, assuming
 * a string key and a double value
 *
 * \param key
 * \param value
 * \param map
 *
 * \return true if insertion successful (or if key already exists)
 */
bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings);


/**
 * \brief Convenience function for taking two separate key,value vectors and creating
 * the more convenient map.  This is helpful for some ROS message types where maps are
 * defined as two separate vectors (ROS messages do not support maps).
 *
 * \param keys vector of key values
 * \param values vector of values
 * \param mappings of key values.
 *
 * \return true if conversion successful
 */
bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings);


/**
 * \brief Map version of @see isWithinRange
 *
 * \param keys vector of key values
 * \param lhs first map
 * \param rhs second map
 * \param full_range
 *
 * \return true if values are within range
 */
bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range);


/**
 * \brief Key/Values vector version of @see isWithinRange
 *
 * \param lhs_keys vector of lhs keys
 * \param lhs_values vector of lhs_values
 * \param rhs_keys vector of rhs keys
 * \param rhs_values vector of rhs_values
 * \param full_range
 *
 * \return true values within range
 */
bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values,
                   double full_range);

} //utils
} //industrial_robot_client

#endif /* INDUSTRIAL_ROBOT_CLIENT_UTILS_H_ */
