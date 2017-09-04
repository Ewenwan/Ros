/*
 * rosserial PIR Motion Sensor Example
 *
 * This tutorial demonstrates the usage of the
 * Seeedstudio PIR Motion Sensor Grove module
 * http://www.seeedstudio.com/depot/Grove-PIR-Motion-Sensor-p-802.html
 *
 * Source Code Based on:
 * https://developer.mbed.org/teams/Seeed/code/Seeed_Grove_PIR_Motion_Sensor_Example/
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Bool motion_msg;
ros::Publisher pub_motion("motion", &motion_msg);

Timer t;
#ifdef TARGET_LPC1768
DigitalIn sig1(p9);
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
DigitalIn sig1(D6);
#else
#error "You need to specify a pin for the sensor"
#endif

int main()
{
  t.start();

  nh.initNode();
  nh.advertise(pub_motion);

  long publisher_timer = 0;

  while (1)
  {

    if (t.read_ms() > publisher_timer)
    {
      motion_msg.data = sig1;
      pub_motion.publish(&motion_msg);
      publisher_timer = t.read_ms() + 1000;
    }
    nh.spinOnce();
  }
}

