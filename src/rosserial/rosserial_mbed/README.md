## rosserial_mbed

This is a rosserial client implementation for the mbed platform.

Note: This has been tested and currently supports building using the [gcc4mbed](https://github.com/adamgreen/gcc4mbed) ofline compiler for the LPC1768, KL25Z and NUCLEO F401RE.

### Issues

* No support for the mbed online compiler (WIP)

### Usage/Workflow

* Make sure you have the gcc4mbed installed on your system
* Install the library somewhere in your system
    * `rosrun rosserial_mbed make_libraries.py <ros-lib-dir>`
* Move to the project's folder and set the env var for the gcc4mbed and the ros-lib paths
    * Example: `$ export GCC4MBED_DIR=/home/gary/devel/mbed/gcc4mbed`
    * Example: `$ export ROS_LIB_DIR=/home/gary/devel/ros-lib`
* `make all && make deploy`

Please see the [rosserial Tutorials on the ROS wiki](http://wiki.ros.org/rosserial_mbed/Tutorials) to get started.
