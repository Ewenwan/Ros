^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_arduino
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2017-03-01)
------------------
* Fixed issue with CMake CMP0054 (`#273 <https://github.com/ros-drivers/rosserial/issues/273>`_)
* Add Teensy LC support (`#270 <https://github.com/ros-drivers/rosserial/issues/270>`_)
* Support Teensy 3.5, 3.6. (`#259 <https://github.com/ros-drivers/rosserial/issues/259>`_)
* Contributors: Brent Yi, FirefoxMetzger, Mike Purvis

0.7.5 (2016-11-22)
------------------
* Missing 'h' inside constructor ArduinoHardware(ArduinoHardware& h) (`#251 <https://github.com/ros-drivers/rosserial/issues/251>`_)
* Contributors: MalcolmReynlods

0.7.4 (2016-09-21)
------------------

0.7.3 (2016-08-05)
------------------

0.7.2 (2016-07-15)
------------------
* Add ros.h include to transform_broadcaster
* Add environment variable for arduino location
* Add support for HW Serial ports on the Teensy
* Contributors: David Lavoie-Boutin, Gary Servin

0.7.1 (2015-07-06)
------------------

0.7.0 (2015-04-23)
------------------

0.6.3 (2014-11-05)
------------------
* Fix for Arduino upload path issue.
* Contributors: Mike Purvis

0.6.2 (2014-09-10)
------------------
* Clean up rosserial_arduino/package.xml
* Generic CMake helpers.
* Contributors: Mike Purvis

0.6.1 (2014-06-30)
------------------

0.6.0 (2014-06-11)
------------------

0.5.6 (2014-06-11)
------------------
* Add Mike Purvis as maintainer
* Updated examples for Arduino 1.+
* Added Teensy 3.1 support (MK20DX256)
* Updated ArduinoHardware.h to add Teensy 3.0 support
* Contributors: Michael Ferguson, Mike Purvis, Moju Zhao, Tony Baltovski, kjanesch

0.5.5 (2014-01-14)
------------------
* Leonardo: Use the USB serial port for ROS messages option


0.5.3 (2013-09-21)
------------------
* add support for leonardo and due

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------

0.4.5 (2013-07-02)
------------------
* Fixed a bug in ros_lib install logic which took an exception because it copied files to themselves
  Added execute permission to make_libraries.py in rosserial_embeddedlinux
  Moved examples under src in rosserial_embeddedlinux
* fix package name

0.4.4 (2013-03-20)
------------------

0.4.3 (2013-03-13 14:08)
------------------------
* forgot to remove install directives

0.4.2 (2013-03-13 01:15)
------------------------
* fix build issues when in isolation by moving more stuff into make_library

0.4.1 (2013-03-09)
------------------

0.4.0 (2013-03-08)
------------------
* initial catkin version on github
