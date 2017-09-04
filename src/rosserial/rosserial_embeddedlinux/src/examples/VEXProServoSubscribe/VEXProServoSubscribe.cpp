/*
 * VEXProServoSubscribe.cpp
 *
 *  Created on: Jul 12, 2012
 *      Author: bouchier
 *  Drives a servo or motor on VEXPro motor1 connection to the requested value: 0 - 255
 *  that is received on subscribed topic servo1
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <qeservo.h>
using namespace std;

ros::NodeHandle  nh;
CQEServo &servo = CQEServo::GetRef();
char *rosSrvrIp = "192.168.15.149";

void messageCb(const std_msgs::Int32& servo1_msg){
	int position = servo1_msg.data;
	printf("Received subscribed servo position %d\n", position);
	servo.SetCommand(0, position);
}
ros::Subscriber<std_msgs::Int32> sub("servo1", messageCb );

int main() {
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);

	while(1) {
		  sleep(1);
		  nh.spinOnce();
	}
}
