/*
 * VEXProMotor13Subscribe.cpp
 * Control motor 13 speed by publishing the desired speed on a ros topic with e.g.
 * $ rostopic pub my_topic std_msgs/Int32 120
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include "qemotoruser.h"

/*
 * Control motor 13 speed by publishing the desired speed on a ros topic with e.g.
 * $ rostopic pub my_topic std_msgs/Int32 120
 * The range of speeds is -255 to +255 (corresponding to full reverse to full forward).
 * Publish negative speeds using the syntax below:
 * $ rostopic pub my_topic std_msgs/Int32 -- -120
 * (This construct tells the shell to feed everything after -- directly to rostopic.)
 */

ros::NodeHandle  nh;
CQEMotorUser &motor = CQEMotorUser::GetRef();
char *rosSrvrIp = "192.168.11.9";

void messageCb(const std_msgs::Int32& motor13_msg){
	int speed = motor13_msg.data;
	printf("Received subscribed motor speed %d\n", speed);
    motor.SetPWM(0, speed);
}
ros::Subscriber<std_msgs::Int32> sub("motor13", messageCb );


int main()
{
	//nh.initNode();
	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);

	while(1) {
		  sleep(1);
		  nh.spinOnce();
	}
}
