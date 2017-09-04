#!/usr/bin/env python

"""
    follower2.py - Version 1.1 2013-12-20
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Relies on PCL ROS nodelets in the launch file to pre-filter the
    cloud on the x, y and z dimensions.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight x-displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.5)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        
        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)
        
        # Initialize the movement command
        self.move_cmd = Twist()
    
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Subscribe to the point cloud
        self.depth_subscriber = rospy.Subscriber('point_cloud', PointCloud2, self.set_cmd_vel, queue_size=1)

        rospy.loginfo("Subscribing to point cloud...")
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)

        rospy.loginfo("Ready to follow!")
        
    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0
        
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            x += pt_x
            y += pt_y
            z += pt_z
            n += 1
        
        # If we have points, compute the centroid coordinates
        if n:
            x /= n 
            y /= n 
            z /= n
            
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold):
                # Compute the angular component of the movement
                linear_speed = (z - self.goal_z) * self.z_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(linear_speed))), linear_speed)
            else:
                self.move_cmd.linear.x *= self.slow_down_factor
                
            if (abs(x) > self.x_threshold):     
                # Compute the linear component of the movement
                angular_speed = -x * self.x_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(angular_speed))), angular_speed)
            else:
                # Stop the rotation smoothly
                self.move_cmd.angular.z *= self.slow_down_factor
                
        else:
            # Stop the robot smoothly
            self.move_cmd.linear.x *= self.slow_down_factor
            self.move_cmd.angular.z *= self.slow_down_factor
            
        # Publish the movement command
        self.cmd_vel_pub.publish(self.move_cmd)

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)      
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")
