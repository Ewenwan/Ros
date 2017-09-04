/*
 * rosserial Motor Shield Example
 *
 * This tutorial demonstrates the usage of the
 * Seeedstudio Motor Shield
 * http://www.seeedstudio.com/depot/motor-shield-v20-p-1377.html
 *
 * Source Code Based on:
 * https://developer.mbed.org/teams/shields/code/Seeed_Motor_Shield/
 */

#include "mbed.h"
#include "MotorDriver.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#ifdef TARGET_LPC1768
#define MOTORSHIELD_IN1     p21
#define MOTORSHIELD_IN2     p22
#define MOTORSHIELD_IN3     p23
#define MOTORSHIELD_IN4     p24
#define SPEEDPIN_A          p25
#define SPEEDPIN_B          p26
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
#define MOTORSHIELD_IN1     D8
#define MOTORSHIELD_IN2     D11
#define MOTORSHIELD_IN3     D12
#define MOTORSHIELD_IN4     D13
#define SPEEDPIN_A          D9
#define SPEEDPIN_B          D10
#else
#error "You need to specify a pin for the sensor"
#endif

MotorDriver motorDriver(MOTORSHIELD_IN1, MOTORSHIELD_IN2, MOTORSHIELD_IN3, MOTORSHIELD_IN4, SPEEDPIN_A, SPEEDPIN_B);
ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist& msg)
{
  if (msg.angular.z == 0 && msg.linear.x == 0)
  {
    motorDriver.stop();
  }
  else
  {
    if (msg.angular.z < 0)
    {
      int speed = (int)(msg.angular.z * -100);
      motorDriver.setSpeed(speed, MOTORA);
      motorDriver.setSpeed(speed, MOTORB);
      motorDriver.goRight();
    }
    else if (msg.angular.z > 0)
    {
      int speed = (int)(msg.angular.z * 100);
      motorDriver.setSpeed(speed, MOTORA);
      motorDriver.setSpeed(speed, MOTORB);
      motorDriver.goLeft();
    }
    else if (msg.linear.x < 0)
    {
      int speed = (int)(msg.linear.x * -100);
      motorDriver.setSpeed(speed, MOTORA);
      motorDriver.setSpeed(speed, MOTORB);
      motorDriver.goBackward();
    }
    else if (msg.linear.x > 0)
    {
      int speed = (int)(msg.linear.x * 100);
      motorDriver.setSpeed(speed, MOTORA);
      motorDriver.setSpeed(speed, MOTORB);
      motorDriver.goForward();
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);
Timer t;

int main()
{
  t.start();
  long vel_timer = 0;
  nh.initNode();
  nh.subscribe(sub);
  motorDriver.init();
  motorDriver.setSpeed(90, MOTORB);
  motorDriver.setSpeed(90, MOTORA);
  while (1)
  {
    if (t.read_ms() > vel_timer)
    {
      motorDriver.stop();
      vel_timer = t.read_ms() + 500;
    }
    nh.spinOnce();
    wait_ms(1);
  }
}
