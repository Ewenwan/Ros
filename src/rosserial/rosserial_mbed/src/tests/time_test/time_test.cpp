//#define COMPILE_TIME_CODE_ROSSERIAL
#ifdef COMPILE_TIME_CODE_ROSSERIAL

/*
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */

#include "mbed.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>


ros::NodeHandle  nh;

std_msgs::Time test;
ros::Publisher p("my_topic", &test);

int main() {
    nh.initNode();
    nh.advertise(p);

    while (1) {
        test.data = nh.now();
        p.publish( &test );
        nh.spinOnce();
        wait_ms(10);
    }
}
#endif