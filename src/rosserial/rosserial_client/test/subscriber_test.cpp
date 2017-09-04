#include "ros_lib/ros/subscriber.h"
#include <gtest/gtest.h>


bool callbackCalled;
unsigned char buffer[1];

class DummyMsg
{
public:
  int serialize(unsigned char *outbuffer) const { return 0; }
  int deserialize(unsigned char *inbuffer) { return 0; }
  const char * getType() { return ""; }
  const char * getMD5() { return ""; }
};

class DummyClass
{
public:
  static void staticCallback(const DummyMsg& msg)
  {
    callbackCalled = true;
  }
  void memberCallback(const DummyMsg& msg)
  {
    callbackCalled = true;
  }
};


TEST(TestSubscriber, testStaticCallback)
{
  ros::Subscriber<DummyMsg> sub("topic_name", &DummyClass::staticCallback);

  callbackCalled = false;
  sub.callback(buffer);
  ASSERT_TRUE(callbackCalled);
}

TEST(TestSubscriber, testMemberCallback)
{
  DummyClass cl;
  ros::Subscriber<DummyMsg, DummyClass> sub("topic_name", &DummyClass::memberCallback, &cl);

  callbackCalled = false;
  sub.callback(buffer);
  ASSERT_TRUE(callbackCalled);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
