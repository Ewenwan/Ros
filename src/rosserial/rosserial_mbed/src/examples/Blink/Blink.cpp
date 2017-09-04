/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;
DigitalOut myled(LED1);

void messageCb(const std_msgs::Empty& toggle_msg){
    myled = !myled;   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

int main() {
    nh.initNode();
    nh.subscribe(sub);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}

