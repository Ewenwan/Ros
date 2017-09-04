#include "ros/ros.h"
#include "std_msgs/String.h"

namespace rosserial {
#include "rosserial_test/ros.h"
#include "rosserial/std_msgs/String.h"
}

#include <gtest/gtest.h>
#include "rosserial_test/fixture.h"
#include "rosserial_test/helpers.h"

/**
 * Single message published from a rosserial-connected client,
 * verified from a roscpp Subscriber.
 */
TEST_F(SingleClientFixture, single_publish) {
  // Rosserial client set up to publish simple message.
  rosserial::std_msgs::String string_msg;
  rosserial::ros::Publisher client_pub("chatter", &string_msg);
  client_nh.advertise(client_pub);
  client_nh.initNode();
  char s[] = "from-rosserial-client";
  string_msg.data = s;

  // Roscpp subscriber to receive the message from the client.
  StringCallback str_callback;
  ros::Subscriber check_sub = nh.subscribe("chatter", 1, &StringCallback::callback, &str_callback);

  for(int attempt = 0; attempt < 50; attempt++) {
    client_pub.publish(&string_msg);
    client_nh.spinOnce();
    ros::spinOnce();
    if (str_callback.times_called > 0) break;
    ros::Duration(0.1).sleep();
  }
  EXPECT_GT(str_callback.times_called, 0);
  EXPECT_STREQ(s, str_callback.last_msg.data.c_str());
}

int rosserial_string_cb_count = 0;
std::string last_string;

static void rosserial_string_cb(const rosserial::std_msgs::String& msg)
{
  rosserial_string_cb_count++;
  last_string = std::string(msg.data);
}

/**
 * Single message sent from a roscpp Publisher, received
 * by a rosserial client subscriber.
 */
TEST_F(SingleClientFixture, single_subscribe) {
  rosserial::ros::Subscriber<rosserial::std_msgs::String> client_sub("chatter", rosserial_string_cb);
  client_nh.subscribe(client_sub);
  client_nh.initNode();

  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1);

  std_msgs::String string_msg;
  string_msg.data = "to-rosserial-client";
  for(int attempt = 0; attempt < 50; attempt++) {
    pub.publish(string_msg);
    ros::spinOnce();
    client_nh.spinOnce();
    if (rosserial_string_cb_count > 0) break;
    ros::Duration(0.1).sleep();
  }
  EXPECT_GT(rosserial_string_cb_count, 0);
  EXPECT_EQ(string_msg.data, last_string);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_publish_subscribe");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
