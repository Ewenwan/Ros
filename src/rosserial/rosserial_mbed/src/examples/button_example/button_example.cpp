/*
 * Button Example for Rosserial
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Bool.h>

#ifdef TARGET_LPC1768
PinName button = p20;
#elif defined(TARGET_KL25Z)
PinName button = D10;
#elif defined(TARGET_NUCLEO_F401RE)
PinName button = USER_BUTTON;
#else
#error "You need to specify a pin for the button"
#endif

ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

DigitalIn button_pin(button);
DigitalOut led_pin(LED1);

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

Timer t;
int main() {
    t.start();
    nh.initNode();
    nh.advertise(pub_button);

    //Enable the pullup resistor on the button
    button_pin.mode(PullUp);

    //The button is a normally button
    last_reading = ! button_pin;

    while (1) {
        bool reading = ! button_pin;

        if (last_reading!= reading) {
            last_debounce_time = t.read_ms();
            published = false;
        }

        //if the button value has not changed for the debounce delay, we know its stable
        if ( !published && (t.read_ms() - last_debounce_time)  > debounce_delay) {
            led_pin = reading;
            pushed_msg.data = reading;
            pub_button.publish(&pushed_msg);
            published = true;
        }

        last_reading = reading;

        nh.spinOnce();
    }
}

