/*
 * rosserial Buzzer Module Example
 *
 * This tutorial demonstrates the usage of the
 * Seeedstudio Buzzer Grove module
 * http://www.seeedstudio.com/depot/Grove-Buzzer-p-768.html
 *
 * Source Code Based on:
 * https://developer.mbed.org/teams/Seeed/code/Seeed_Grove_Buzzer/
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

#ifdef TARGET_LPC1768
PwmOut buzzer(p21);
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
PwmOut buzzer(D2);
#else
#error "You need to specify a pin for the sensor"
#endif

Timeout toff;
bool playing = false;
DigitalOut led(LED1);

void nobeep()
{
  buzzer.write(0.0);
  led = 1;
  playing = false;
}
void beep(float freq, float time)
{
  buzzer.period(1.0 / freq);
  buzzer.write(0.5);
  toff.attach(nobeep, time);
  led = 0;
}

void messageCb(const std_msgs::Float32MultiArray& msg)
{
  if (!playing)
  {
    playing = true;
    // msg.data[0] - Note
    // msg.data[1] - duration in seconds
    beep(msg.data[0], msg.data[1]);
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("buzzer", &messageCb);

int main()
{
  buzzer = 0;
  led = 1;
  nh.initNode();
  nh.subscribe(sub);
  while (1)
  {
    nh.spinOnce();
    wait_ms(1);
  }
}
