/*
 * rosserial_embeddedlinux service server example
 *
 * Advertises a service it offers. Prints the string sent to the service
 * and responds with an alternating string.
 * The service request can be sent from the ROS command line with e.g.
 * $ xxx
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_examples_msgs/Test.h>
#include <stdio.h>

ros::NodeHandle  nh;
using rosserial_examples::Test;
#define ROSSRVR_IP "192.168.15.122"

int i=0;
void svcCallback(const Test::Request & req, Test::Response & res){
	if((i++)%2)
		res.output = "hello";
	else
		res.output = "ros";
	printf("Service request message: \"%s\" received, responding with: %s", res.output);
}
ros::ServiceServer<Test::Request, Test::Response> server("test_srv",&svcCallback);

int main()
{
	nh.initNode(ROSSRVR_IP);
	nh.advertiseService(server);

	while(1) {
		nh.spinOnce();
		sleep(1);
	}
}
