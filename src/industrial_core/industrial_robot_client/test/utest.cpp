/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include "industrial_robot_client/utils.h"
#include <iostream>
#include <gtest/gtest.h>

using namespace industrial_robot_client::utils;


TEST(IndustrialUtilsSuite, vector_within_range)
{

  std::vector<double> v1;
  std::vector<double> v2;
  const double range = 0.01;
  const double half_range = range / 2.0;
  const double outside_range = 1.1 * half_range;
  const double inside_range = 0.9 * half_range;
  const double a = 9.65;
  const double b = 1324.239;
  const double c = 90204987243.1433223;

  //NOTE: Asserts are used because each step builds upon the the prevoius step
  EXPECT_TRUE(isWithinRange(v1, v2, range));

  v1.push_back(a);
  ASSERT_FALSE(isWithinRange(v1, v2, range));

  v2.push_back(a);
  ASSERT_TRUE(isWithinRange(v1, v2, range));
  ASSERT_TRUE(isWithinRange(v1, v2, -range));

  v1.push_back(b);
  ASSERT_FALSE(isWithinRange(v1, v2, range));

  v2.push_back(b + inside_range);
  ASSERT_TRUE(isWithinRange(v1, v2, range));

  v1.push_back(b);
  v2.push_back(b + outside_range);
  ASSERT_FALSE(isWithinRange(v1, v2, range));

  v1.pop_back();
  v2.pop_back();

  v1.push_back(c);
  v2.push_back(c + inside_range);
  ASSERT_TRUE(isWithinRange(v1, v2, range));

  v1.push_back(c);
  v2.push_back(c + outside_range);
  ASSERT_FALSE(isWithinRange(v1, v2, range));

}

TEST(IndustrialUtilsSuite, map_within_range)
{
  std::vector<double> v1;
  std::vector<std::string> v1_keys;
  std::vector<double> v2;
  std::vector<std::string> v2_keys;
  std::vector<double> v3;
  std::vector<std::string> v3_keys;
  const double range = 0.01;
  const double half_range = range / 2.0;
  const double outside_range = 1.1 * half_range;
  const double inside_range = 0.9 * half_range;
  const double a = 9.65;
  const double b = 1324.239;
  const double c = 90204987243.1433223;


  // Empty vectors should return true
  EXPECT_TRUE(isWithinRange(v1_keys, v1, v2_keys, v2, range));


  v1.push_back(a);
  v1_keys.push_back("a");
  v1.push_back(b);
  v1_keys.push_back("b");
  v1.push_back(c);
  v1_keys.push_back("c");

  ASSERT_FALSE(isWithinRange(v1_keys, v1, v2_keys, v2, range));
  EXPECT_TRUE(isWithinRange(v1_keys, v1, v1_keys, v1, range));


  v2.push_back(inside_range + c);
  v2_keys.push_back("c");
  v2.push_back(inside_range + b);
  v2_keys.push_back("b");
  v2.push_back(inside_range + a);
  v2_keys.push_back("a");

  ASSERT_TRUE(isWithinRange(v1_keys, v1, v2_keys, v2, range));

  v3.push_back(outside_range + a);
  v3_keys.push_back("a");
  v3.push_back(outside_range + b);
  v3_keys.push_back("b");
  v3.push_back(outside_range + c);
  v3_keys.push_back("c");


  ASSERT_FALSE(isWithinRange(v1_keys, v1, v3_keys, v3, range));


}

// Run all the tests that were declared with TEST()
  int main(int argc, char **argv)
  {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
