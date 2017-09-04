/*
 * rosserial Collission Sensor Example
 *
 * This tutorial demonstrates the usage of the
 * Seeedstudio Collision Grove module
 * http://www.seeedstudio.com/wiki/Grove_-_Collision_Sensor
 *
 * Source Code Based on:
 * https://developer.mbed.org/components/Grove-Collision-Sensor/
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Bool collision_msg;
ros::Publisher pub_collision("collision", &collision_msg);


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
  nh.advertise(pub_collision);

  long publisher_timer = 0;

  while (1)
  {

    if (t.read_ms() > publisher_timer)
    {
      collision_msg.data = !sig1;
      pub_collision.publish(&collision_msg);
      publisher_timer = t.read_ms() + 1000;
    }
    nh.spinOnce();
  }
}

