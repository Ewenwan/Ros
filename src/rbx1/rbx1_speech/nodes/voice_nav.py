#!/usr/bin/env python

"""
  voice_nav.py - Version 1.1 2013-12-20
  
  Allows controlling a mobile base using simple speech commands.
  
  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class VoiceNav:
    def __init__(self):
        rospy.init_node('voice_nav')
        
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = rospy.get_param("~max_speed", 0.8)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        
        # A flag to determine whether or not voice control is paused
        self.paused = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me'],
                                    'slower': ['slow down', 'slower'],
                                    'faster': ['speed up', 'faster'],
                                    'forward': ['forward', 'ahead', 'straight', 'move', 'come'],
                                    'backward': ['back', 'backward', 'back up'],
                                    'rotate left': ['rotate left', 'left'],
                                    'rotate right': ['rotate right', 'right'],
                                    'turn left': ['turn left'],
                                    'turn right': ['turn right'],
                                    'quarter': ['quarter speed'],
                                    'half': ['half speed', 'half'],
                                    'full': ['full speed', 'full'],
                                    'pause': ['pause speech', 'pause'],
                                    'continue': ['continue speech']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                       
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
        
        # If the user has asked to pause/continue voice control,
        # set the flag accordingly 
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
        
        # If voice control is paused, simply return without
        # performing any action
        if self.paused:
            return       
        
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'forward':    
            self.cmd_vel.linear.x = self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == 'rotate left':
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'rotate right':  
            self.cmd_vel.linear.x = 0      
            self.cmd_vel.angular.z = -self.angular_speed
            
        elif command == 'turn left':
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z += self.angular_increment
            else:        
                self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z -= self.angular_increment
            else:        
                self.cmd_vel.angular.z = -self.angular_speed
                
        elif command == 'backward':
            self.cmd_vel.linear.x = -self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel = Twist()
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x += copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z += copysign(self.angular_increment, self.cmd_vel.angular.z)
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x -= copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z -= copysign(self.angular_increment, self.cmd_vel.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
                
        else:
            return

        self.cmd_vel.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel.linear.x))
        self.cmd_vel.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")

