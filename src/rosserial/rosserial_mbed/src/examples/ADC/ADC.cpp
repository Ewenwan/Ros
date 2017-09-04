/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#include "mbed.h"
#include <ros.h>
#include <rosserial_mbed/Adc.h>

#if defined(TARGET_LPC1768)
PinName adc0 = p15;
PinName adc1 = p16;
PinName adc2 = p17;
PinName adc3 = p18;
PinName adc4 = p19;
PinName adc5 = p20;
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
PinName adc0 = A0;
PinName adc1 = A1;
PinName adc2 = A2;
PinName adc3 = A3;
PinName adc4 = A4;
PinName adc5 = A5;
#else
#error "You need to specify the pins for the adcs"
#endif

ros::NodeHandle nh;

rosserial_mbed::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);


//We average the analog reading to elminate some of the noise
int averageAnalog(PinName pin) {
    int v=0;
    for (int i=0; i<4; i++) v+= AnalogIn(pin).read_u16();
    return v/4;
}

long adc_timer;

int main() {
    nh.initNode();

    nh.advertise(p);

    while (1) {
        adc_msg.adc0 = averageAnalog(adc0);
        adc_msg.adc1 = averageAnalog(adc1);
        adc_msg.adc2 = averageAnalog(adc2);
        adc_msg.adc3 = averageAnalog(adc3);
        adc_msg.adc4 = averageAnalog(adc4);
        adc_msg.adc5 = averageAnalog(adc5);

        p.publish(&adc_msg);

        nh.spinOnce();
    }
}

