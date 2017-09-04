/*
 * rosserial Clapper Example
 *
 * This code is a very simple example of the kinds of
 * custom sensors that you can easily set up with rosserial
 * and Mbed.  This code uses a microphone attached to
 * analog pin p20 detect two claps (2 loud sounds).
 * You can use this clapper, for example, to command a robot
 * in the area to come do your bidding.
 */

#include <ros.h>
#include <std_msgs/Empty.h>

#if defined(TARGET_LPC1768)
PinName mic = p20;
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
PinName mic = A0;
#else
#error "You need to specify a pin for the mic"
#endif

ros::NodeHandle  nh;

std_msgs::Empty clap_msg;
ros::Publisher p("clap", &clap_msg);

enum clapper_state { clap1, clap_one_waiting,  pause, clap2};
clapper_state clap;

int volume_thresh = 200;  //a clap sound needs to be:
//abs(clap_volume) > average noise + volume_thresh
AnalogIn mic_pin(mic);
int adc_ave=0;

Timer t;
int main() {
    t.start();
    nh.initNode();

    nh.advertise(p);

    //measure the average volume of the noise in the area
    for (int i =0; i<10; i++) adc_ave += mic_pin.read_u16();
    adc_ave /= 10;

    long event_timer = 0;

    while (1) {
        int mic_val = 0;
        for (int i=0; i<4; i++) mic_val += mic_pin.read_u16();

        mic_val = mic_val/4-adc_ave;

        switch (clap) {
            case clap1:
                if (abs(mic_val) > volume_thresh) {
                    clap = clap_one_waiting;
                    event_timer = t.read_ms();
                }
                break;
            case clap_one_waiting:
                if ( (abs(mic_val) < 30) && ( (t.read_ms()- event_timer) > 20 ) ) {
                    clap= pause;
                    event_timer = t.read_ms();

                }
                break;
            case pause: // make sure there is a pause between
                // the loud sounds
                if ( mic_val > volume_thresh) {
                    clap = clap1;

                } else if ( (t.read_ms()-event_timer)> 60)  {
                    clap = clap2;
                    event_timer = t.read_ms();

                }
                break;
            case clap2:
                if (abs(mic_val) > volume_thresh) { //we have got a double clap!
                    clap = clap1;
                    p.publish(&clap_msg);
                } else if ( (t.read_ms()-event_timer)> 200) {
                    clap= clap1; // no clap detected, reset state machine
                }

                break;
        }
        nh.spinOnce();
    }
}

