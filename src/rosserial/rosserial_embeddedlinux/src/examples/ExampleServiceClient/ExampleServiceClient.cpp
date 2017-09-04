/*
 * rosserial_embeddedlinux service client example
 *
 * Calls a service offered by a ROS service server and publishes what it receives from
 * the service server to the chatter topic. It also prints the received service response.
 *
 * You can run a suitable service on ROS with:
 * $ rosrun rosserial_embeddedlinux client.py
 *
 * When you run this program on the embedded linux system, client.py on the ROS workstation
 * will ask you to respond with a string. The string you give it will be passed back to this
 * service client, which will print and publish it.
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_embeddedlinux/Test.h>
#include <stdio.h>

ros::NodeHandle  nh;
using rosserial_embeddedlinux::Test;
#define ROSSRVR_IP "192.168.15.149"

ros::ServiceClient<Test::Request, Test::Response> client("test_srv");

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

int main()
{
	  nh.initNode(ROSSRVR_IP);
	  nh.serviceClient(client);
	  nh.advertise(chatter);
	  while(!nh.connected()) nh.spinOnce();
	  printf("Startup complete\n");
	  while (1) {
		  Test::Request req;
		  Test::Response res;
		  req.input = hello;
		  client.call(req, res);
		  printf("Service responded with \"%s\"\n", res.output);
		  str_msg.data = res.output;
		  chatter.publish( &str_msg );
		  nh.spinOnce();
		  sleep(1);
	  }
}
