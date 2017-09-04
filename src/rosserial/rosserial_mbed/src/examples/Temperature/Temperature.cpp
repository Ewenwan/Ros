/*
 * rosserial Temperature Sensor Example
 *
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 *
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */

#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);


// From the datasheet the BMP module address LSB distinguishes
// between read (1) and write (0) operations, corresponding to
// address 0x91 (read) and 0x90 (write).
// shift the address 1 bit right (0x91 or 0x90), the Wire library only needs the 7
// most significant bits for the address 0x91 >> 1 = 0x48
// 0x90 >> 1 = 0x48 (72)

int sensorAddress = 0x91 >>1;  // From datasheet sensor address is 0x91
                               // shift the address 1 bit right, the Wire library only needs the 7
                               // most significant bits for the address

Timer t;
#ifdef TARGET_LPC1768
I2C i2c(p9, p10);        // sda, scl
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
I2C i2c(D14, D15);       // sda, scl
#else
#error "You need to specify a pin for the sensor"
#endif

int main() {
    t.start();

    nh.initNode();
    nh.advertise(pub_temp);

    long publisher_timer =0;

    while (1) {

        if (t.read_ms() > publisher_timer) {
            // step 1: request reading from sensor
            //Wire.requestFrom(sensorAddress,2);
            char cmd = 2;
            i2c.write(sensorAddress, &cmd, 1);

            wait_ms(50);

            char msb;
            char lsb;
            int temperature;
            i2c.read(sensorAddress, &msb, 1); // receive high byte (full degrees)
            i2c.read(sensorAddress, &lsb, 1); // receive low byte (fraction degrees)

            temperature = ((msb) << 4);  // MSB
            temperature |= (lsb >> 4);   // LSB

            temp_msg.data = temperature*0.0625;
            pub_temp.publish(&temp_msg);

            publisher_timer = t.read_ms() + 1000;
        }

        nh.spinOnce();
    }
}

