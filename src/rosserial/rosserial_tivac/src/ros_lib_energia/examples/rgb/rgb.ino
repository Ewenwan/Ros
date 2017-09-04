/*
 * rosserial subscriber Example
 */
 
#include <ros.h>
#include <std_msgs/ColorRGBA.h>

void color_cb( const std_msgs::ColorRGBA& msg)
{
  analogWrite(RED_LED,msg.r*255);
  analogWrite(GREEN_LED,msg.g*255);
  analogWrite(BLUE_LED,msg.b*255);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::ColorRGBA> color_sub("led", &color_cb );

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  analogWrite(RED_LED,255);
  analogWrite(GREEN_LED,255);
  analogWrite(BLUE_LED,255);
  
  nh.initNode();
  nh.subscribe(color_sub);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}

