#!/usr/bin/env python

""" odom_ekf.py - Version 1.1 2013-12-20

    Republish the /robot_pose_ekf/odom_combined topic which is of type 
    geometry_msgs/PoseWithCovarianceStamped as an equivalent message of
    type nav_msgs/Odometry so we can view it in RViz.

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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF():
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher('output', Odometry, queue_size=5)
        
        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('input', PoseWithCovarianceStamped)
        
        # Subscribe to the /odom_combined topic
        rospy.Subscriber('input', PoseWithCovarianceStamped, self.pub_ekf_odom)
        
        rospy.loginfo("Publishing combined odometry on /odom_ekf")
        
    def pub_ekf_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = '/odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose
        
        self.ekf_pub.publish(odom)
        
if __name__ == '__main__':
    try:
        OdomEKF()
        rospy.spin()
    except:
        pass
        

        
