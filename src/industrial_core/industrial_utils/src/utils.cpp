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

#include "industrial_utils/utils.h"
#include "ros/ros.h"
#include <algorithm>

namespace industrial_utils
{

bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    rtn = false;
  }
  else
  {
    std::sort(lhs.begin(), lhs.end());
    std::sort(rhs.begin(), rhs.end());
    rtn = isSame(lhs, rhs);
  }

  return rtn;
}

bool isSame(const std::vector<std::string> & lhs, const std::vector<std::string> & rhs)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    rtn = false;
  }
  else
  {
    rtn = std::equal(lhs.begin(), lhs.end(), rhs.begin());
  }
  return rtn;
}

bool findChainJointNames(const urdf::LinkConstSharedPtr &link, bool ignore_fixed,
		                 std::vector<std::string> &joint_names)
{
  typedef std::vector<urdf::JointSharedPtr > joint_list;
  typedef std::vector<urdf::LinkSharedPtr > link_list;
  std::string found_joint, found_link;

  // check for joints directly connected to this link
  const joint_list &joints = link->child_joints;
  ROS_DEBUG("Found %lu child joints:", joints.size());
  for (joint_list::const_iterator it=joints.begin(); it!=joints.end(); ++it)
  {
    ROS_DEBUG_STREAM("  " << (*it)->name << ": type " <<  (*it)->type);
    if (ignore_fixed && (*it)->type == urdf::Joint::FIXED)
      continue;

    if (found_joint.empty())
    {
      found_joint = (*it)->name;
      joint_names.push_back(found_joint);
    }
    else
    {
      ROS_WARN_STREAM("Unable to find chain in URDF.  Branching joints: " << found_joint << " and " << (*it)->name);
      return false;  // branching tree (multiple valid child-joints)
    }
  }

  // check for joints connected to children of this link
  const link_list &links = link->child_links;
  std::vector<std::string> sub_joints;
  ROS_DEBUG("Found %lu child links:", links.size());
  for (link_list::const_iterator it=links.begin(); it!=links.end(); ++it)
  {
    ROS_DEBUG_STREAM("  " << (*it)->name);
    if (!findChainJointNames(*it, ignore_fixed, sub_joints))   // NOTE: recursive call
      return false;

    if (sub_joints.empty())
      continue;

    if (found_link.empty())
    {
      found_link = (*it)->name;
      joint_names.insert(joint_names.end(), sub_joints.begin(), sub_joints.end());  // append sub_joints
    }
    else
    {
      ROS_WARN_STREAM("Unable to find chain in URDF.  Branching links: " << found_link << " and " << (*it)->name);
      return false;  // branching tree (multiple valid child-joints)
    }
  }

  return true;
}

} //industrial_utils
