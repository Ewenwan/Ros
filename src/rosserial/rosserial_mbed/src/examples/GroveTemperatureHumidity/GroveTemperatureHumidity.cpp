/*
 * rosserial Temperature and Humidity Sensor Example
 *
 * This tutorial demonstrates the usage of the
 * Seeedstudio Temperature and Humidity Grove module
 * http://www.seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor
 *
 * Source Code Based of:
 * https://developer.mbed.org/teams/Seeed/code/Seeed_Grove_Temp_Humidity_Example/
 */

#include "mbed.h"
#include "DHT.h"
#include <ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

ros::NodeHandle  nh;

sensor_msgs::Temperature temp_msg;
sensor_msgs::RelativeHumidity humidity_msg;
ros::Publisher pub_temp("temperature", &temp_msg);
ros::Publisher pub_humidity("humidity", &humidity_msg);

Timer t;
#ifdef TARGET_LPC1768
DHT sensor(p9, AM2302);
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
DHT sensor(D6, AM2302);
#elif defined(TARGET_NUCLEO_F401RE)
DHT sensor(A6, AM2302);
#else
#error "You need to specify a pin for the sensor"
#endif

int main()
{
  int error = 0;
  t.start();

  nh.initNode();
  nh.advertise(pub_temp);
  nh.advertise(pub_humidity);

  long publisher_timer = 0;
  temp_msg.header.frame_id = "/base_link";
  humidity_msg.header.frame_id = "/base_link";

  while (1)
  {

    if (t.read_ms() > publisher_timer)
    {
      error = sensor.readData();
      if (0 == error)
      {
        temp_msg.temperature = sensor.ReadTemperature(CELCIUS);
        temp_msg.header.stamp = nh.now();
        pub_temp.publish(&temp_msg);

        humidity_msg.relative_humidity = sensor.ReadHumidity();
        humidity_msg.header.stamp = nh.now();
        pub_humidity.publish(&humidity_msg);
      }
      publisher_timer = t.read_ms() + 1000;
    }
    nh.spinOnce();
  }
}

