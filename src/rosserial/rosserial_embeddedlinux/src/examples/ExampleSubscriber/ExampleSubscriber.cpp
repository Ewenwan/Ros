/*
 * rosserial_embeddedlinux subscriber example
 *
 * Prints a string sent on a subscribed ros topic.
 * The string can be sent with e.g.
 * $ rostopic pub chatter std_msgs/String -- "Hello Embedded Linux"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

ros::NodeHandle  nh;
char *rosSrvrIp = "192.168.15.149";

void messageCb(const std_msgs::String& received_msg){
	printf("Received subscribed chatter message: %s\n", received_msg.data);
}
ros::Subscriber<std_msgs::String> sub("chatter", messageCb );

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
