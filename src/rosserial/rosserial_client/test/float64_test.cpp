#include "ros_lib/ros/msg.h"
#include <gtest/gtest.h>


class TestFloat64 : public ::testing::Test
{
public:
  union
  {
    double val;
    unsigned char buffer[8];
  };
  
  static const double cases[];
  static const int num_cases;
};
 
const double TestFloat64::cases[] = {
  0.0, 10.0, 15642.1, -50.2, 0.0001, -0.321,
  123456.789, -987.654321, 3.4e38, -3.4e38,
};
const int TestFloat64::num_cases = sizeof(TestFloat64::cases) / sizeof(double);


TEST_F(TestFloat64, testRoundTrip)
{
  for (int i = 0; i < num_cases; i++)
  {
    memset(buffer, 0, sizeof(buffer));
    ros::Msg::serializeAvrFloat64(buffer, cases[i]);
    EXPECT_FLOAT_EQ(cases[i], val);
    
    float ret = 0;
    ros::Msg::deserializeAvrFloat64(buffer, &ret);
    EXPECT_FLOAT_EQ(cases[i], ret);
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
