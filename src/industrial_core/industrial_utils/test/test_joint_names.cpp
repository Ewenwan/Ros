/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

using namespace industrial_utils;

#define ROS_ERROR_EXIT(...) do {ROS_ERROR(__VA_ARGS__); exit(-1); } while (0)

// Quick program to test joint-name extraction from URDF
int main(int argc, char **argv)
{
  if (argc != 2)
    ROS_ERROR_EXIT("Usage: test_joint_names my_robot.urdf");

  urdf::Model model;
  if (!model.initFile(argv[1]))
    ROS_ERROR_EXIT("Unable to open URDF file: %s", argv[1]);

  std::vector<std::string> joint_names;
  if (!findChainJointNames(model.getRoot(), true, joint_names))
    ROS_ERROR_EXIT("Unable to read Joint Names from URDF");

  printf("Joint names: ");
  std::ostringstream s;
  std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(s, ", "));
  std::string out = s.str();
  std::cout << "[" << out.erase(out.size()-2) << "]" << std::endl;

  return 1;
}
