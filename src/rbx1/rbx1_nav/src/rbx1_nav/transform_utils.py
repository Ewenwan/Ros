#!/usr/bin/env python

""" A couple of handy conversion utilities taken from the turtlebot_node.py script found in the
    turtlebot_node ROS package at:
    
    http://www.ros.org/wiki/turtlebot_node
    
"""

import PyKDL
from math import pi

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res
