^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosserial_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2017-03-01)
------------------
* Fix typo in serial error message (`#253 <https://github.com/ros-drivers/rosserial/issues/253>`_)
* Contributors: Jonathan Binney

0.7.5 (2016-11-22)
------------------

0.7.4 (2016-09-21)
------------------
* Try to read more serial bytes in a loop (`#248 <https://github.com/ros-drivers/rosserial/issues/248>`_)
* Add missing "import errno" to rosserial_python
* Integration tests for rosserial (`#243 <https://github.com/ros-drivers/rosserial/issues/243>`_)
* rosserial_python tcp server allowing socket address reuse (`#242 <https://github.com/ros-drivers/rosserial/issues/242>`_)
* Contributors: Mike Purvis, Vitor Matos, davidshin172

0.7.3 (2016-08-05)
------------------

0.7.2 (2016-07-15)
------------------

0.7.1 (2015-07-06)
------------------

0.7.0 (2015-04-23)
------------------
* Adds default queue_size of 10 for rosserial_python publisher.
* Fixed queue size warning with diagnostics publisher.
* We don't need roslib.load_manifest any more under catkin.
* Contributors: Basheer Subei, David Lavoie-Boutin, Mike Purvis, eisoku9618

0.6.3 (2014-11-05)
------------------

0.6.2 (2014-09-10)
------------------
* Added MD5 verification for request and response messags upon ServiceClient registration.
* Enabled registration of service clients
* Contributors: Jonathan Jekir

0.6.1 (2014-06-30)
------------------

0.6.0 (2014-06-11)
------------------

0.5.6 (2014-06-11)
------------------
* Add Mike Purvis as maintainer to all but xbee.
* Added the missing inWaiting() to RosSerialServer
* improvement: inform user of mismatched checksum for topic_id and msg
* Fix indent on if-block.
* Check for data in buffer before attempting to tryRead. Insert a 1ms sleep to avoid pegging the processor.
* Better warning message for tryRead.
* fix two points: 1. the number of bytes to read for chk_byte, 2. the wrong indentation about the defination of sendDiagnostics()
* Try-block to handle IOErrors thrown from tryRead
* Merge from hydro-devel.
* fix the dupilcated registration problem of subscriber
* remove ID_TX_STOP from rosserial_msgs/msg/TopicInfo.msg, using hardcode modification. fix the dupilcated registration problem of subscriber
* modified rosserial
* modified rosserial
* Contributors: Girts Linde, Mike Purvis, Moju Zhao, bakui, denis

0.5.5 (2014-01-14)
------------------

0.5.4 (2013-10-17)
------------------

0.5.3 (2013-09-21)
------------------
* De-register subscribers and service clients upon disconnect.
  This prevents callbacks being called after a client program
  terminates a connection.
* Fill out package.xml properly, include docstring in helper Python node.
* Add message info helper script that supports rosserial_server

0.5.2 (2013-07-17)
------------------

* Fix release version

0.5.1 (2013-07-15)
------------------
* Merge branch 'rosserial_bakui' of git://github.com/tongtybj/rosserial into tongtybj-rosserial_bakui
  * Modified the frame structure for serial communication, particularly add the checksum for msg_len
* Incorporate protocol version in message. Try to detect protocol version mismatch and give appropriate hints.

0.4.5 (2013-07-02)
------------------
* Fix SeviceServer member names in error message
  'm' prefix was omitted, causing an exception while trying to print
  an error about md5 mismatches. Fix this to allow the error to be
  presented to the user.
* Allow service calls with empty requests
  std_srvs::Empty has a request message of size zero. SerialClient.send
  returns the size of the sent message, which is checked to ensure
  data crossed the serial line. Accommodate services with empty requests
  by modifying the check to acknowledge all transmissions of zero or
  more bytes as valid.
* revert name of node, add a few comments/spacing
* fix private parameters - temporary fix breaks fork_server for tcp
* Fix `#35 <https://github.com/ros-drivers/rosserial/issues/35>`_

0.4.4 (2013-03-20)
------------------
* Fixed "Lost sync" message at initial connection that happens on both Arduino &
  embeddedLinux. Problem was last_sync initialized to epoch and compared against
  Time.now() always times out on first compare.

0.4.3 (2013-03-13 14:08)
------------------------

0.4.2 (2013-03-13 01:15)
------------------------

0.4.1 (2013-03-09)
------------------

0.4.0 (2013-03-08)
------------------
* initial catkin version on github
