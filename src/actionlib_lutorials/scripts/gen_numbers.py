#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Float32
import random

def gen_number():
    pub = rospy.Publisher('random_number', Float32)
    rospy.init_node('random_number_generator', log_level=rospy.INFO)
    rospy.loginfo("Generating random numbers")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish(Float32(random.normalvariate(5, 1)))
        rate.sleep()
        
if __name__ == '__main__':
    try:
        gen_number()
    except KeyboardInterrupt:
        print ("done")