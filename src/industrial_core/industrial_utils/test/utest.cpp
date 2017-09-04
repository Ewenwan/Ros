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

#include "industrial_utils/utils.h"

#include <gtest/gtest.h>

using namespace industrial_utils;

TEST(IndustrialUtilsSuite, vector_compare)
{

  std::vector<std::string> v1;
  std::vector<std::string> v2;
  std::vector<std::string> v3;

  // Testing empty vectors
  EXPECT_TRUE(isSimilar(v1, v2));
  EXPECT_TRUE(isSame(v1, v2));

  std::string i1 = "item_1";
  std::string i2 = "item_2";
  std::string i3 = "item_3";

  v1.push_back(i1);
  v1.push_back(i2);
  v1.push_back(i3);

  // Testing one empty, one populated
  EXPECT_FALSE(isSimilar(v1, v2));
  EXPECT_FALSE(isSame(v1, v2));

  v2.push_back(i1);
  v2.push_back(i2);

  // Testing one partially filled, one full
  EXPECT_FALSE(isSimilar(v1, v2));
  EXPECT_FALSE(isSame(v1, v2));

  v2.push_back(i3);

  // Testing same vectors
  EXPECT_TRUE(isSimilar(v1, v2));
  EXPECT_TRUE(isSame(v1, v2));

  v3.push_back(i3);
  v3.push_back(i2);
  v3.push_back(i1);

  // Testing similar but not same
  EXPECT_TRUE(isSimilar(v1, v3));
  EXPECT_FALSE(isSame(v1, v3));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
