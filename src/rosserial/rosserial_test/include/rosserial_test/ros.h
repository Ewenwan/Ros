#ifndef ROSSERIAL_ROS_H
#define ROSSERIAL_ROS_H

#include "rosserial/ros/node_handle.h"
#include "rosserial/duration.cpp"
#include "rosserial/time.cpp"
#include <iostream>

class ClientComms {
public:
  // Can smuggle in an fd representing either the back end of 
  // a socket or serial pty, and run the same tests over both.
  static int fd;

  // Accessible to be manipulated by tests, for test behaviours
  // dependent on the passage of time.
  static unsigned long millis;

  void init() {
  }
  int read() {
    unsigned char ch;
    ssize_t ret = ::read(fd, &ch, 1);
    return ret == 1 ? ch : -1;
  }
  void write(uint8_t* data, int length) {
    ::write(fd, data, length);
  }
  unsigned long time() {
    return millis;
  } 
};

int ClientComms::fd = -1;
unsigned long ClientComms::millis = 0;

namespace ros {
typedef NodeHandle_<ClientComms, 5, 5, 200, 200> NodeHandle;
}

#endif  // ROSSERIAL_ROS_H
