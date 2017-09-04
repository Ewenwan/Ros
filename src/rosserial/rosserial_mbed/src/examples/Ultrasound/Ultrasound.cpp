/*
 * rosserial Ultrasound Example
 *
 * This example is for the Maxbotix Ultrasound rangers.
 */

#include "mbed.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

#if defined(TARGET_LPC1768)
const PinName adc_pin = p20;
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
const PinName adc_pin = A0;
#else
#error "You need to specify a pin for the sensor"
#endif

char frameid[] = "/ultrasound";

float getRange_Ultrasound(PinName pin_num) {
    int val = 0;
    for (int i=0; i<4; i++) val += AnalogIn(pin_num).read_u16();
    float range =  val;
    return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

Timer t;
int main() {
    t.start();
    nh.initNode();
    nh.advertise(pub_range);

    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id =  frameid;
    range_msg.field_of_view = 0.1;  // fake
    range_msg.min_range = 0.0;
    range_msg.max_range = 6.47;

    //pinMode(8,OUTPUT);
    //digitalWrite(8, LOW);

    long range_time=0;

    while (1) {

        //publish the adc value every 50 milliseconds
        //since it takes that long for the sensor to stablize
        if ( t.read_ms() >= range_time ) {
            range_msg.range = getRange_Ultrasound(adc_pin);
            range_msg.header.stamp = nh.now();
            pub_range.publish(&range_msg);
            range_time =  t.read_ms() + 50;
        }

        nh.spinOnce();
    }
}

