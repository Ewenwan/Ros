/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
DigitalOut myled(LED1);

void messageCb( const std_msgs::Empty& toggle_msg) {
    myled = !myled;   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

int main() {
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);

    while (1) {
        str_msg.data = hello;
        chatter.publish( &str_msg );
        nh.spinOnce();
        wait_ms(500);
    }
}

