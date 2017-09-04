#!/usr/bin/env python

""" nav_square.py - Version 1.1 2013-12-20

    A basic demo of the using odometry data to move the robot
    along a square trajectory.

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

import rospy #ros system depends
from geometry_msgs.msg import Twist, Point, Quaternion
# Twist:linear.x linear.y linear.z    Point: x,y,z   Quaternion: x,y,z,w 
import tf  # transform betwween two axis
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
# Quaternion to eulerian angle and constarin the angle to -180-180
from math import radians, copysign, sqrt, pow, pi  # mathematical function and parameter

# defining it as a Python class
class NavSquare():
    # standard class initialization
    def __init__(self): 
        # Creat a node named out_and_back
        rospy.init_node('nav_square', anonymous=False)
        # Set rospy to execute a shutdown function when exiting 
        rospy.on_shutdown(self.shutdown)
        
        # variable definition
        rate = 20             # moving friquency ,How fast will we update the robot's movement?      
        r = rospy.Rate(rate)  # friquency variable,   Set the equivalent ROS rate variable       
        # Set the parameters for the target square  ~ private paramters
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
        linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
        angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 1)) # degrees to radians
        
        # Creat a Publisher, to control the robot's speed  topic  message_type  buffer_size
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)   
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()       
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom' # world-fixed frame
        
        # Find the reference frame
        # Find out if the robot uses /base_link or /base_footprint
        # /base_footprint frame used by the TurtleBot
        # /base_link frame      used by Pi Robot and Maxwell
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0)) #wait for tramsrform
            self.base_frame = '/base_footprint' # the robot uses /base_footprint frame   
        except (tf.Exception, tf.ConnectivityException, tf.LookupException): # Exception = error state 
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
                
        # Initialize the position variable as a Point type(comes from geometry_msgs.msg which has x,y,z)
        position = Point()

        # Cycle through the four sides of the square
        for i in range(4):
        # Straight forward
            # initialize the movement command with linear velocity
            move_cmd = Twist() # Initialize the movement command to zero , Twist:linear.x linear.y linear.z    Point: x,y,z         
            move_cmd.linear.x = linear_speed            # Set the movement command to forward motion with desired speed
            #record the starting position
            (position, rotation) = self.get_odom()      # Get the starting position values                        
            x_start = position.x  
            y_start = position.y
            distance = 0                                # traveled  distance            
            # travel with a linear  speed
            while distance < goal_distance and not rospy.is_shutdown(): # hasno't reached the desired distance       
                self.cmd_vel.publish(move_cmd)                          # Publish the Twist message velocity comannd  until reached the destination               
                r.sleep()
                (position, rotation) = self.get_odom()                  # Get the current position               
                # Compute the Euclidean distance from the start         
                distance = sqrt(pow((position.x - x_start), 2) +        # update the traveled  distance 
                                pow((position.y - y_start), 2))
                
        # Stop the robot before rotating
            # stop()
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
        # rotate
            move_cmd.angular.z = angular_speed      # Set the movement command to a rotation        
            last_angle = rotation                   # starting angle, odometry record  the last angle measured(starting angle)        
            turn_angle = 0                          # traveled angle           
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():# hasno't reached the desired goal_angle     
                self.cmd_vel.publish(move_cmd)                 # Publish the Twist message velocity comannd angular_speed until reached the desired goal_angle                   
                r.sleep()                                      # sleep with rate
                (position, rotation) = self.get_odom()         # Get the current rotation angle        
                delta_angle = normalize_angle(rotation - last_angle) # compute the the delt_angel  constraint with in -pi to pi       
                turn_angle += delta_angle                      # update the traveled turn_angle
                last_angle = rotation                          # update the last_angle
                
        # Stop the robot before the next leg
            # stop()
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
        # Stop the robot when we are done
        self.cmd_vel.publish(Twist())
   
    # get the location ang angel information from odometry    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        # used * is Python's a notion for passing a list of numbers to a function
        # trans is a list of x, y, and z coordinates
        # rot is a list of x, y, z and w quaternion components.
        return (Point(*trans), quat_to_angle(Quaternion(*rot))) #current location Point and the current angel     
          
    # stop and close automatically when shutting down the node
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    # stop the robot    
    def stop(self):
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        
if __name__ == '__main__':
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

