#!/usr/bin/env python

""" move_fake_pi_arm_start.py - Version 1.1 2013-12-20

    Position the fake Pi Robot's arm to a more natural starting position.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('move_fake_pi_arm_start')

    lift = rospy.Publisher('arm_shoulder_lift_joint/command', Float64, queue_size=5)
    elbow = rospy.Publisher('arm_elbow_flex_joint/command', Float64, queue_size=5)
    
    rospy.wait_for_message('joint_states', JointState)
    
    for i in range(3):
        lift.publish(Float64(2.62))
        elbow.publish(Float64(0.45))
        rospy.sleep(0.5)
