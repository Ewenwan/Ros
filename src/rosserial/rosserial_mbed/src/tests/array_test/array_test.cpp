//#define COMPILE_ARRAY_CODE_RSOSSERIAL
#ifdef COMPILE_ARRAY_CODE_RSOSSERIAL

/*
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum
 */
#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


ros::NodeHandle nh;

bool set_;
DigitalOut myled(LED1);

geometry_msgs::Pose sum_msg;
ros::Publisher p("sum", &sum_msg);

void messageCb(const geometry_msgs::PoseArray& msg) {
    sum_msg.position.x = 0;
    sum_msg.position.y = 0;
    sum_msg.position.z = 0;
    for (int i = 0; i < msg.poses_length; i++) {
        sum_msg.position.x += msg.poses[i].position.x;
        sum_msg.position.y += msg.poses[i].position.y;
        sum_msg.position.z += msg.poses[i].position.z;
    }
    myled = !myled;   // blink the led
}

ros::Subscriber<geometry_msgs::PoseArray> s("poses",messageCb);

int main() {
    nh.initNode();
    nh.subscribe(s);
    nh.advertise(p);

    while (1) {
        p.publish(&sum_msg);
        nh.spinOnce();
        wait_ms(10);
    }
}
#endif