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

#include "ros/console.h"

#include "industrial_utils/utils.h"
#include "industrial_robot_client/utils.h"

#include <cmath>
#include <iostream>

namespace industrial_robot_client
{
namespace utils
{

bool isWithinRange(const std::vector<double> & lhs, const std::vector<double> & rhs, double full_range)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::lhs size: " << lhs.size() << " does not match rhs size: " << rhs.size());
    rtn = false;
  }
  else
  {
    // Calculating the half range causes some precision loss, but it's good enough
    double half_range = full_range / 2.0;
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < lhs.size(); ++i)
    {
      if (fabs(lhs[i] - rhs[i]) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}

bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings)
{
  bool rtn = false;

  std::pair<std::map<std::string, double>::iterator, bool> insert_rtn;

  insert_rtn = mappings.insert(std::make_pair(key, value));

  // The second value returned form insert is a boolean (true for success)
  if (!insert_rtn.second)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Failed to insert item into map with key: " << key);
    rtn = false;
  }
  else
  {
    rtn = true;
  }
  return rtn;

}

bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings)
{
  bool rtn;

  mappings.clear();

  if (keys.size() == values.size())
  {
    rtn = true;

    for (size_t i = 0; i < keys.size(); ++i)
    {
      rtn = mapInsert(keys[i], values[i], mappings);
      if (!rtn)
      {
        break;
      }
    }

  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::keys size: " << keys.size()
                     << " does not match values size: " << values.size());

    rtn = false;
  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range)
{
  bool rtn = false;

  if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Size mistmatch ::lhs size: " << lhs.size() <<
                     " rhs size: " << rhs.size() << " key size: " << keys.size());

    rtn = false;
  }
  else
  {
    // Calculating the half range causes some precision loss, but it's good enough
    double half_range = full_range / 2.0;
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < keys.size(); ++i)
    {
      if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, double full_range)
{
  bool rtn = false;
  std::map<std::string, double> lhs_map;
  std::map<std::string, double> rhs_map;
  if (industrial_utils::isSimilar(lhs_keys, rhs_keys))
  {
    if (toMap(lhs_keys, lhs_values, lhs_map) && toMap(rhs_keys, rhs_values, rhs_map))
    {
      rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_range);
    }
  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Key vectors are not similar");
    rtn = false;
  }
  return rtn;
}

} //utils
} //industrial_robot_client
