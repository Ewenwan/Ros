//#define COMPLIE_FLOAT64_CODE_ROSSERIAL
#ifdef  COMPILE_FLOAT64_CODE_ROSSERIAL

/*
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float64.h>


ros::NodeHandle nh;

float x;
DigitalOut myled(LED1);

void messageCb( const std_msgs::Float64& msg) {
    x = msg.data - 1.0;
    myled = !myled; // blink the led
}

std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> s("your_topic", &messageCb);
ros::Publisher p("my_topic", &test);

int main() {
    nh.initNode();
    nh.advertise(p);
    nh.subscribe(s);
    while (1) {
        test.data = x;
        p.publish( &test );
        nh.spinOnce();
        wait_ms(10);
    }
}
#endif