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

#ifndef INDUSTRIAL_UTILS_H_
#define INDUSTRIAL_UTILS_H_

#include <vector>
#include <string>
#include "urdf/model.h"

namespace industrial_utils
{

/**
 * \brief Checks to see if sets are similar(same members, any order)
 * NOTE: Vectors are passed by copy because they are modified by the
 * function(i.e. ordered).
 * This function should not be used for large vectors or in loops (it
 * is slow!)
 *
 * \param lhs first vector
 * \param rhs second vector
 *
 * \return true set are similar (same members, any order)
 */
bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs);

/**
 * \brief Checks to see if sets are the same(same members, same order)
 * Wraps std::equal method.
 *
 * \param lhs first vector
 * \param rhs second vector
 *
 * \return true set are similar (same members, same order)
 */
bool isSame(const std::vector<std::string> & lhs, const std::vector<std::string> & rhs);

/*
 * \brief Returns joint names for a simple serial chain from a URDF tree
 *          - returns an error if branching tree is found
 *          - assumes chain runs root->leaf.  leaf->root->leaf chains not allowed.
 *
 * \param[in] link URDF model link to search from (e.g. model.getRoot())
 * \param[in] ignore_fixed flag to ignore fixed joints
 * \param[in,out] joint_names vector of joint names
 *
 * \return true if successful, false if error occurred (e.g. branching tree)
 */
bool findChainJointNames(const urdf::LinkConstSharedPtr &link, bool ignore_fixed,
		                 std::vector<std::string> &joint_names);

} //industrial_utils

#endif /* INDUSTRIAL_UTILS_H_ */
